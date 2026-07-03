#!/bin/bash
# ============================================================
# G1D 主控脚本 V8.0
# - 引用 g1d_utils.sh 工具函数库
# - 合并 rotate/move/height 到 step_executor.py CLI
# - 全局步骤超时 (J)
# - 原子写入 task_progress.json (K)
# - 任务前健康检查 (N)
# - 偏移校准引导
# - 远程通知 (webhook)
# - Dry-run 支持
# ============================================================
# 注意：不使用 set -e，因为交互菜单中的 curl/read 等命令可能返回非零
# 关键执行点通过 execute_step() 的返回值自行检查
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/g1d_utils.sh"

# 断点续跑开关：true 则任务失败后继续执行后续任务
CONTINUE_ON_TASK_ERROR=true

# ROS2 环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    set +u; source /opt/ros/humble/setup.bash; set -u
    echo "✅ ROS2 环境已加载" | tee -a "$LOG_FILE"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    set +u; source /opt/ros/foxy/setup.bash; set -u
    echo "✅ ROS2 环境已加载" | tee -a "$LOG_FILE"
fi

# 检查监控服务
echo "🔍 检查监控服务状态..."
if systemctl is-active --quiet g1d-monitor 2>/dev/null; then
    echo "✅ 监控服务运行中，仪表盘: http://$(hostname -I | awk '{print $1}'):28087/"
else
    echo "⚠️ 监控服务未运行，可执行: sudo systemctl start g1d-monitor"
fi

trap 'printf "\n⚠️ 中断...\n"; cleanup_all; exit 0' INT

# ==================== 清理 ====================

cleanup_all() {
    echo "🧹 清理进程..." | tee -a "$LOG_FILE"
    [ -f "$SCRIPT_HOME_DIR/stop_our_robot_control.sh" ] && "$SCRIPT_HOME_DIR/stop_our_robot_control.sh" >> "$LOG_FILE" 2>&1 || true
    pkill -9 -f "robot_nav_arm_flow.py" 2>/dev/null || true
    pkill -9 -f "gripper_control.py" 2>/dev/null || true
    pkill -f "camera_stream.py" 2>/dev/null || true
    python3 -c "
import json, urllib.request
try:
    payload = {'task_id':'emergency_stop_$(date +%s)','task_command_info':[{'command_id':'s1','command_code':'emergency_stop','command_param':{}}]}
    urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:8080/api/s1-agent/v1/task/submit', data=json.dumps(payload).encode(), headers={'Content-Type':'application/json'}, method='POST'), timeout=3)
except: pass
" >> "$LOG_FILE" 2>&1 || true
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear:{x:0,y:0,z:0},angular:{x:0,y:0,z:0}}" 2>/dev/null || true
    # 任务中断时标记当前步骤为失败
    if [ -f "$TASK_PROGRESS_FILE" ]; then
        python3 -c "
import json, tempfile, os
try:
    with open('$TASK_PROGRESS_FILE', 'r') as f:
        data = json.load(f)
    for act in data.get('actions', []):
        if act.get('status') == 'running':
            act['status'] = 'failed'
            act['error'] = '任务被中断'
            break
    tmp_fd, tmp_path = tempfile.mkstemp(dir=os.path.dirname('$TASK_PROGRESS_FILE'), suffix='.json')
    with os.fdopen(tmp_fd, 'w') as f:
        json.dump(data, f, ensure_ascii=False)
    os.replace(tmp_path, '$TASK_PROGRESS_FILE')
except: pass
"
    fi
}

# ==================== 紧急复位 ====================

function_emergency_reset() {
    echo "🛑 执行紧急复位..." | tee -a "$LOG_FILE"
    python3 -c "
import json, urllib.request
try:
    payload = {'task_id':'emergency_stop_$(date +%s)','task_command_info':[{'command_id':'s1','command_code':'emergency_stop','command_param':{}}]}
    urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:8080/api/s1-agent/v1/task/submit', data=json.dumps(payload).encode(), headers={'Content-Type':'application/json'}, method='POST'), timeout=3)
except: pass
" | tee -a "$LOG_FILE"
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear:{x:0,y:0,z:0},angular:{x:0,y:0,z:0}}" 2>/dev/null || true

    if curl --max-time 5 -s -X POST "http://192.168.123.164:18083/api/actions/execute" \
       -H "Content-Type: application/json" \
       -d '{"type":"arm_task","phase":"RESET","target_object":"","timeout_sec":10}' > /dev/null 2>&1; then
        echo "机械臂 RESET 已发送 (HTTP)" | tee -a "$LOG_FILE"
    else
        echo "⚠️ HTTP 复位失败，尝试 ROS 话题..." | tee -a "$LOG_FILE"
        ros2 topic pub --once /arm_control_refactor/task_command std_msgs/msg/String \
          "{\"data\":\"{\\\"task_id\\\":\\\"emergency_$(date +%s)\\\",\\\"phase\\\":\\\"RESET\\\",\\\"target_object\\\":\\\"\\\"}\"}" \
          && echo "机械臂 RESET 已发送 (ROS)" | tee -a "$LOG_FILE" \
          || echo "❌ 机械臂 RESET 失败" | tee -a "$LOG_FILE"
    fi

    curl -s -X POST "${GRIPPER_URL}/api/v1/command" -H "Content-Type: application/json" -d '{"command":"release","wait":false}' && echo "吸盘 release 已发送" || echo "吸盘 release 失败" | tee -a "$LOG_FILE"
    echo "✅ 紧急复位完成" | tee -a "$LOG_FILE"
    read -p "按回车返回..."
}

# ==================== 点位库建立 ====================

function_change_data(){
    cd /opt/data/nav_map
    printf "地图文件夹:\n"; ls -lrt
    read -p "输入地图名: " CHOICE_MAP
    cd "$CHOICE_MAP/maincenter"
    printf "版本:\n"; ls -lrt
    read -p "版本: " CHOICE_VER
    cd "$CHOICE_VER"
    [ ! -f nav_points.txt ] && { echo "❌ 找不到 nav_points.txt"; return 1; }
    cat nav_points.txt

    > "$POINT_INF_FILE"
    while true; do
        read -p "点位名称 (q退出): " pname
        [ "$pname" == "q" ] && break
        read -p "香烟种类 (如 XiongMao): " ctype
        local LINE_DATA=$(awk -v name="$pname" '$11==name{print $11,$1,$2,$3,$4,$5,$6,$7; exit}' nav_points.txt)
        if [ -z "$LINE_DATA" ]; then
            echo "❌ 地图中无此点"
        else
            echo "$pname $LINE_DATA $ctype" >> "$POINT_INF_FILE"
            echo "✅ 已缓存: $pname ($ctype)"
        fi
    done
    local END_DATA=$(awk '$11=="end_point"{print $11,$1,$2,$3,$4,$5,$6,$7; exit}' nav_points.txt)
    if [ -n "$END_DATA" ]; then
        echo "end_point $END_DATA end" >> "$POINT_INF_FILE"
        echo "✅ end_point 已加入"
    fi
    echo "点位库建立完毕" | tee -a "$LOG_FILE"
    cat "$POINT_INF_FILE"
}

# ==================== 步骤执行（全局超时）====================

execute_step() {
    local step_cmd="$1" step_name="$2" step_idx="$3"
    printf -- "\n\033[1;34m[执行步骤]\033[0m %s (索引: %d)\n" "$step_cmd" "$step_idx" | tee -a "$LOG_FILE"

    # 标记为 running
    atomic_write_progress "running" "$step_idx"

    # 读取全局步骤超时
    local step_timeout=$(get_sdk_param "step_timeout_sec" "180")
    local timed_out=0

    if [ "$step_timeout" -gt 0 ] 2>/dev/null; then
        # 带超时执行
        timeout "$step_timeout" bash -c "eval \"$step_cmd\"" >> "$LOG_FILE" 2>&1 || true
        local ret=$?
        # timeout 返回 124 表示超时
        if [ $ret -eq 124 ]; then
            timed_out=1
        elif [ $ret -ne 0 ]; then
            :  # 普通失败
        fi
    else
        # 无超时
        eval "$step_cmd" >> "$LOG_FILE" 2>&1 || true
        local ret=$?
    fi

    if [ $timed_out -eq 1 ]; then
        printf -- "\033[1;31m❌ 步骤超时 (%ds)，中断循环\033[0m\n" "$step_timeout" | tee -a "$LOG_FILE"
        atomic_write_progress "failed" "$step_idx" "步骤超时(${step_timeout}s)"
        send_notify "步骤超时: $step_name (${step_timeout}s)" "ERROR"
        return 1
    elif [ $ret -ne 0 ]; then
        printf -- "\033[1;31m❌ 步骤失败，中断循环\033[0m\n" | tee -a "$LOG_FILE"
        atomic_write_progress "failed" "$step_idx" "退出码=$ret"
        send_notify "步骤失败: $step_name (退出码=$ret)" "ERROR"
        return 1
    else
        printf -- "\033[1;32m✅ 步骤完成\033[0m\n" | tee -a "$LOG_FILE"
        atomic_write_progress "success" "$step_idx"
        return 0
    fi
}

# ==================== 任务执行 ====================

function_run_ini_tasks(){
    [ ! -f "$POINT_INF_FILE" ] && { echo "请先建立点位库"; return 1; }
    [ ! -f "$TASK_INI_FILE" ] && { echo "找不到 $TASK_INI_FILE"; return 1; }

    # N: 任务前健康检查
    if ! function_health_check; then
        read -p "健康检查未通过，是否仍然继续？(y/n): " cf
        [ "$cf" == "y" ] || { echo "❌ 任务取消" | tee -a "$LOG_FILE"; return 1; }
    fi

    local sections=$(tr -d '\r' < "$TASK_INI_FILE" | awk '/^\[/{gsub(/[\[\]]/,""); print}')
    for section in $sections; do
        local item_name=$(get_ini_value "$TASK_INI_FILE" "$section" "name")
        local item_count=$(get_ini_value "$TASK_INI_FILE" "$section" "count")
        local point_name=$(get_ini_value "$TASK_INI_FILE" "$section" "point")
        [ "$point_name" == "NOT_FOUND" ] && point_name=""
        [ -z "$item_name" ] || [ "$item_name" == "NOT_FOUND" ] || [ -z "$item_count" ] || [ "$item_count" == "NOT_FOUND" ] && continue
        [[ "$item_count" =~ ^[0-9]+$ ]] || continue

        local point_valid=0
        if [ -n "$point_name" ] && [[ "$point_name" =~ ^[A-Za-z0-9_-]+$ ]]; then
            point_valid=1
        else
            [ -n "$point_name" ] && echo "⚠️ point 字段无效: '$point_name'，将用香烟种类查找" | tee -a "$LOG_FILE"
            point_name=""
        fi

        get_point_data() {
            local line=""
            if [ $point_valid -eq 1 ]; then
                line=$(grep "^$point_name " "$POINT_INF_FILE" | head -1)
            fi
            if [ -z "$line" ]; then
                line=$(awk -v c="$item_name" '$9 == c {print; exit}' "$POINT_INF_FILE")
                [ -z "$line" ] && line=$(grep "^$item_name " "$POINT_INF_FILE" | head -1)
            fi
            [ -z "$line" ] && { echo "❌ 点位未找到: ${point_name:-$item_name}"; exit 1; }
            echo "$line"
        }

        local PICK_LINE=$(get_point_data "${point_name:-$item_name}")
        local CIG_TYPE=$(echo "$PICK_LINE" | awk '{print $9}')
        item_name="$CIG_TYPE"

        local px=$(echo "$PICK_LINE" | awk '{print $2}')
        local py=$(echo "$PICK_LINE" | awk '{print $3}')
        local pz=$(echo "$PICK_LINE" | awk '{print $4}')
        local qx=$(echo "$PICK_LINE" | awk '{print $5}')
        local qy=$(echo "$PICK_LINE" | awk '{print $6}')
        local qz=$(echo "$PICK_LINE" | awk '{print $7}')
        local qw=$(echo "$PICK_LINE" | awk '{print $8}')
        local START_POSE=$(python3 -c "import json; print(json.dumps({'position':{'x':float($px),'y':float($py),'z':float($pz)},'orientation':{'x':float($qx),'y':float($qy),'z':float($qz),'w':float($qw)},'look_at':True}))")

        local END_LINE=$(grep "^end_point " "$POINT_INF_FILE" | head -1)
        local ex=$(echo "$END_LINE" | awk '{print $2}')
        local ey=$(echo "$END_LINE" | awk '{print $3}')
        local ez=$(echo "$END_LINE" | awk '{print $4}')
        local eqx=$(echo "$END_LINE" | awk '{print $5}')
        local eqy=$(echo "$END_LINE" | awk '{print $6}')
        local eqz=$(echo "$END_LINE" | awk '{print $7}')
        local eqw=$(echo "$END_LINE" | awk '{print $8}')
        local END_POSE=$(python3 -c "import json; print(json.dumps({'position':{'x':float($ex),'y':float($ey),'z':float($ez)},'orientation':{'x':float($eqx),'y':float($eqy),'z':float($eqz),'w':float($eqw)},'look_at':True}))")

        local actions_raw=$(get_ini_value "$TASK_INI_FILE" "$section" "actions")
        local use_custom=0
        [ "$actions_raw" != "NOT_FOUND" ] && [ -n "$actions_raw" ] && use_custom=1

        # ----- 步骤拆分与进度初始化 -----
        local steps_file=$(mktemp /tmp/steps_array.XXXXXX)
        export SPLIT_STEPS_FILE="$steps_file"
        export SPLIT_PROG_FILE="$TASK_PROGRESS_FILE"
        export SPLIT_INI_FILE="$TASK_INI_FILE"
        export SPLIT_SECTION="$section"
        export SPLIT_ITEM="$item_name"
        export SPLIT_POINT="${point_name:-$item_name}"
        export SPLIT_COUNT="$item_count"
        export SPLIT_PX="$px"
        export SPLIT_PY="$py"
        export SPLIT_PZ="$pz"
        export SPLIT_EX="$ex"
        export SPLIT_EY="$ey"
        export SPLIT_EZ="$ez"

        python3 -c "
import os, re, json, sys, tempfile
steps_file = os.environ['SPLIT_STEPS_FILE']
prog_file = os.environ['SPLIT_PROG_FILE']
ini_file = os.environ['SPLIT_INI_FILE']
section = os.environ['SPLIT_SECTION']
item_name = os.environ['SPLIT_ITEM']
point_name = os.environ['SPLIT_POINT']
item_count = int(os.environ['SPLIT_COUNT'])
px = float(os.environ['SPLIT_PX'])
py = float(os.environ['SPLIT_PY'])
pz = float(os.environ['SPLIT_PZ'])
ex = float(os.environ['SPLIT_EX'])
ey = float(os.environ['SPLIT_EY'])
ez = float(os.environ['SPLIT_EZ'])

actions_raw = ''
with open(ini_file, 'r') as f:
    in_section = False
    for line in f:
        stripped = line.strip()
        if stripped == '[$section]':
            in_section = True
            continue
        if in_section and stripped.startswith('['):
            break
        if in_section and stripped.startswith('actions'):
            parts = stripped.split('=', 1)
            if len(parts) > 1:
                actions_raw = parts[1].split(';')[0].strip()
                break

if actions_raw:
    steps = [s.strip() for s in re.split(r',(?![^()]*\))', actions_raw) if s.strip()]
else:
    steps = ['nav_start', 'weitiao', 'pick', 'backup', 'nav_end', 'place', 'reset']

with open(steps_file, 'w') as f:
    for s in steps:
        f.write(s + '\n')

data = {
    'item': item_name,
    'point': point_name,
    'total': item_count,
    'current': 0,
    'step': '准备中',
    'status': 'idle',
    'actions': [{'name': s, 'status': 'pending'} for s in steps],
    'start_point': {'name': point_name, 'x': px, 'y': py, 'z': pz},
    'end_point': {'name': 'end_point', 'x': ex, 'y': ey, 'z': ez}
}
# 原子写入
tmp_fd, tmp_path = tempfile.mkstemp(dir=os.path.dirname(prog_file), suffix='.json')
with os.fdopen(tmp_fd, 'w') as f:
    json.dump(data, f, ensure_ascii=False)
os.replace(tmp_path, prog_file)
"

        unset SPLIT_STEPS_FILE SPLIT_PROG_FILE SPLIT_INI_FILE SPLIT_SECTION SPLIT_ITEM SPLIT_POINT SPLIT_COUNT SPLIT_PX SPLIT_PY SPLIT_PZ SPLIT_EX SPLIT_EY SPLIT_EZ

        if [ -s "$steps_file" ]; then
            echo "📋 任务步骤列表:" | tee -a "$LOG_FILE"
            cat "$steps_file" | tee -a "$LOG_FILE"
            local total_steps=$(wc -l < "$steps_file")
            echo "🔢 步骤总数: $total_steps" | tee -a "$LOG_FILE"
        else
            echo "❌ 步骤文件为空，跳过此任务" | tee -a "$LOG_FILE"
            rm -f "$steps_file"
            continue
        fi

        printf -- "\n\033[1;33m========== %s (共 %d 次) ==========\033[0m\n" "$item_name" $item_count | tee -a "$LOG_FILE"
        send_notify "开始任务: $item_name (共 $item_count 次)"

        local task_failed=0
        for ((c=1; c<=$item_count; c++)); do
            printf -- "\033[1;36m--- 第 %d/%d 次 ---\033[0m\n" $c $item_count | tee -a "$LOG_FILE"

            # 更新 current 和 status
            python3 -c "
import json, os, tempfile
prog_file = '$TASK_PROGRESS_FILE'
with open(prog_file, 'r') as f:
    data = json.load(f)
data['current'] = $c
data['status'] = 'running'
for act in data.get('actions', []):
    act['status'] = 'pending'
tmp_fd, tmp_path = tempfile.mkstemp(dir=os.path.dirname(prog_file), suffix='.json')
with os.fdopen(tmp_fd, 'w') as f:
    json.dump(data, f, ensure_ascii=False)
os.replace(tmp_path, prog_file)
"

            local step_idx=0
            echo "🚀 开始执行步骤循环..." | tee -a "$LOG_FILE"
            while IFS= read -r step_raw; do
                [ -z "$step_raw" ] && continue
                echo "⏳ 准备执行步骤: $step_raw (索引 $step_idx)" | tee -a "$LOG_FILE"
                local cmd=$(build_step_cmd "$step_raw" "$START_POSE" "$END_POSE" "$item_name")
                [ -z "$cmd" ] && continue
                if ! execute_step "$cmd" "$step_raw" "$step_idx"; then
                    if [ "$CONTINUE_ON_TASK_ERROR" = "true" ]; then
                        echo "⚠️ 步骤失败，但继续执行下一个任务" | tee -a "$LOG_FILE"
                        task_failed=1
                        break
                    else
                        cleanup_all
                        printf -- "\033[1;32m🎉 任务中断，后续任务已跳过\033[0m\n" | tee -a "$LOG_FILE"
                        send_notify "任务中断: $item_name" "ERROR"
                        rm -f "$steps_file"
                        return 1
                    fi
                fi
                step_idx=$((step_idx + 1))
            done < "$steps_file"
            [ $task_failed -eq 1 ] && break
            echo "🏁 次数 $c 步骤循环结束" | tee -a "$LOG_FILE"
            cleanup_all
        done

        # 标记完成
        python3 -c "
import json, os, tempfile
prog_file = '$TASK_PROGRESS_FILE'
with open(prog_file, 'r') as f:
    data = json.load(f)
data['status'] = 'done'
data['step'] = '已完成'
tmp_fd, tmp_path = tempfile.mkstemp(dir=os.path.dirname(prog_file), suffix='.json')
with os.fdopen(tmp_fd, 'w') as f:
    json.dump(data, f, ensure_ascii=False)
os.replace(tmp_path, prog_file)
"
        rm -f "$steps_file"

        if [ $task_failed -eq 1 ]; then
            send_notify "任务失败: $item_name" "ERROR"
            continue
        else
            send_notify "任务完成: $item_name"
        fi
    done
    printf -- "\033[1;32m🎉 所有任务清单执行完毕！\033[0m\n" | tee -a "$LOG_FILE"
    rm -f "$TASK_PROGRESS_FILE"
}

# ==================== 显示状态 ====================

show_current_status() {
    if curl -s --connect-timeout 2 "$STATUS_API_URL" > /dev/null 2>&1; then
        echo "仪表盘: http://$(hostname -I | awk '{print $1}'):28087/"
    else
        echo "❌ 监控服务未运行"
    fi
}

# ==================== 夹爪菜单 ====================

function_gripper_menu() {
    while true; do
        clear
        echo "===== 夹爪/吸盘 ====="
        echo "[1] 吸合 [2] 张开 [3] 状态 [q] 返回"
        read -p "选择: " c
        case $c in
            1) curl -s -X POST "$GRIPPER_URL/api/v1/command" -H "Content-Type: application/json" -d '{"command":"suck","wait":false}' && echo "✅" || echo "❌"; read -p "回车..." ;;
            2) curl -s -X POST "$GRIPPER_URL/api/v1/command" -H "Content-Type: application/json" -d '{"command":"release","wait":false}' && echo "✅" || echo "❌"; read -p "回车..." ;;
            3) curl -s "$GRIPPER_URL/api/v1/status"; read -p "回车..." ;;
            q|Q) return ;;
        esac
    done
}

# ==================== SDK 菜单（通过控制API:28091）====================

CONTROL_API="http://127.0.0.1:28091"

function_sdk_menu() {
    while true; do
        clear
        echo "===== SDK 底盘控制 (via API:28091) ====="
        echo "[1] 旋转"
        echo "[2] 平移 (forward/backward)"
        echo "[3] 升降至目标高度"
        echo "[4] 相对升降 (up/down)"
        echo "[5] 夹爪控制"
        echo "[6] 紧急停止"
        echo "[7] 查看控制器状态"
        echo "[8] 仪表盘 (浏览器打开)"
        echo "[9] 并行: 旋转+升降"
        echo "[0] 并行: 移动+升降"
        echo "[a] 并行: 导航+升降"
        echo "[q] 返回"
        read -p "选择: " c
        case $c in
            1) read -p "角度(度): " ang
               curl -s -X POST "$CONTROL_API/api/control/rotate" -H "Content-Type: application/json" -d "{\"angle\":$ang}" | python3 -m json.tool
               read -p "回车..." ;;
            2) read -p "方向 (forward/backward): " dir
               [[ "$dir" =~ ^(forward|backward)$ ]] || { echo "无效"; continue; }
               read -p "距离(m): " dist
               curl -s -X POST "$CONTROL_API/api/control/move" -H "Content-Type: application/json" -d "{\"direction\":\"$dir\",\"distance\":$dist}" | python3 -m json.tool
               read -p "回车..." ;;
            3) read -p "目标物理高度(米): " h
               curl -s -X POST "$CONTROL_API/api/control/lift_to" -H "Content-Type: application/json" -d "{\"height\":$h}" | python3 -m json.tool
               read -p "回车..." ;;
            4) read -p "方向 (up/down): " dir; read -p "距离(米): " dist
               curl -s -X POST "$CONTROL_API/api/control/lift_rel" -H "Content-Type: application/json" -d "{\"direction\":\"$dir\",\"distance\":$dist}" | python3 -m json.tool
               read -p "回车..." ;;
            5) function_gripper_menu ;;
            6) curl -s -X POST "$CONTROL_API/api/control/stop" -H "Content-Type: application/json" -d '{}' | python3 -m json.tool; read -p "回车..." ;;
            7) curl -s "$CONTROL_API/api/status" | python3 -m json.tool; read -p "回车..." ;;
            8) echo "仪表盘: http://$(hostname -I | awk '{print $1}'):28087/"; read -p "回车..." ;;
            9) read -p "角度(度): " ang; read -p "目标高度(米): " h
               curl -s -X POST "$CONTROL_API/api/control/rotate_lift" -H "Content-Type: application/json" -d "{\"angle\":$ang,\"height\":$h}" | python3 -m json.tool
               read -p "回车..." ;;
            0) read -p "方向 (forward/backward): " dir; read -p "距离(m): " dist; read -p "目标高度(米): " h
               curl -s -X POST "$CONTROL_API/api/control/move_lift" -H "Content-Type: application/json" -d "{\"direction\":\"$dir\",\"distance\":$dist,\"height\":$h}" | python3 -m json.tool
               read -p "回车..." ;;
            a|A) read -p "目标高度(米): " h
                 echo "请输入导航位姿 JSON (如 {\"x\":1.0,\"y\":2.0,\"theta\":0.5}):"
                 read -p "pose: " pose_json
                 curl -s -X POST "$CONTROL_API/api/control/nav_start_lift" -H "Content-Type: application/json" -d "{\"pose\":$pose_json,\"height\":$h}" | python3 -m json.tool
                 read -p "回车..." ;;
            q|Q) return ;;
        esac
    done
}

# ==================== 摄像头流服务 ====================

function_camera_menu() {
    while true; do
        clear
        echo "===== 摄像头流服务 ====="
        if pgrep -f "camera_stream.py" > /dev/null 2>&1; then
            echo "状态: ✅ 运行中 (端口 28088)"
        else
            echo "状态: ❌ 未运行"
        fi
        echo "[1] 启动 [2] 停止 [q] 返回"
        read -p "选择: " c
        case $c in
            1)
                nohup python3 "$PROJECT_HOME_DIR/camera_stream.py" > /tmp/camera_stream.log 2>&1 &
                echo "启动中，等待发现相机话题..."
                sleep 3
                if pgrep -f "camera_stream.py" > /dev/null 2>&1; then
                    echo "✅ 摄像头流服务已启动"
                    echo "快照: http://$(hostname -I | awk '{print $1}'):28088/snap/left"
                    echo "MJPEG: http://$(hostname -I | awk '{print $1}'):28088/mjpeg/left"
                else
                    echo "❌ 启动失败，查看 /tmp/camera_stream.log"
                fi
                read -p "回车..."
                ;;
            2)
                pkill -f "camera_stream.py" 2>/dev/null || true
                rm -f /tmp/camera_left.jpg /tmp/camera_right.jpg
                echo "✅ 已停止"
                read -p "回车..."
                ;;
            q|Q) return ;;
        esac
    done
}

# ==================== 主菜单 ====================

while true; do
    clear
    printf "\n🤖 温州烟草G1D V8.0\n"
    echo "[1] 建立点位库"
    echo "[2] 执行任务清单"
    echo "[3] SDK 底盘控制"
    echo "[4] 夹爪/吸盘"
    echo "[5] 紧急复位"
    echo "[6] 查看仪表盘地址"
    echo "[7] 健康检查"
    echo "[8] 立柱偏移校准"
    echo "[9] 摄像头流服务"
    echo "[0] 退出"
    read -p "选项: " CHOICE
    case $CHOICE in
        1) function_change_data; read -p "回车..." ;;
        2) read -p "确认执行？(y/n): " cf; [ "$cf" == "y" ] && function_run_ini_tasks; read -p "回车..." ;;
        3) function_sdk_menu ;;
        4) function_gripper_menu ;;
        5) function_emergency_reset ;;
        6) show_current_status; read -p "回车..." ;;
        7) function_health_check; read -p "回车..." ;;
        8) function_calibrate_offset; read -p "回车..." ;;
        9) function_camera_menu ;;
        0) exit 0 ;;
        *) echo "无效"; sleep 1 ;;
    esac
done
