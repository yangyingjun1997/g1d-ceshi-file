#!/bin/bash
# ============================================================
# G1D Shell 工具函数库 V1.0
# 被 g1d_project.sh source 引用
# 包含：INI 解析、SDK 参数读取、原子写入、健康检查、
#       偏移校准、远程通知、步骤构建
# ============================================================

HOST_NAME=$(whoami)
PROJECT_HOME_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_HOME_DIR="/home/$HOST_NAME"
LOG_FILE="$PROJECT_HOME_DIR/g1d_project.log"
TASK_INI_FILE="$PROJECT_HOME_DIR/conf/task_list.ini"
POINT_INF_FILE="$PROJECT_HOME_DIR/conf/point_inf"
SDK_INI_FILE="$PROJECT_HOME_DIR/conf/params.ini"
GRIPPER_URL="http://192.168.123.164:18080"
STATUS_API_URL="http://localhost:28087/api/status"
TASK_PROGRESS_FILE="/tmp/current_task.json"
LIFT_OFFSET_FILE="$PROJECT_HOME_DIR/conf/lift_offset.json"

# Webhook 通知 URL（环境变量优先，否则为空不发送）
G1D_WEBHOOK_URL="${G1D_WEBHOOK_URL:-}"

# ==================== INI 解析 ====================

get_ini_value() {
    local file="$1" section="$2" key="$3"
    tr -d '\r' < "$file" | awk -F '=' -v sect="[$section]" -v k="$key" '
        BEGIN{found=0}
        $0==sect{found=1; next}
        /^\[/ && found{exit}
        found {
            raw_line = $0
            sub(/[ \t]*;.*/, "", raw_line)
            split(raw_line, parts, "=")
            gsub(/^[ \t]+|[ \t]+$/,"", parts[1])
            if(parts[1] == k) {
                val = parts[2]
                for(i=3;i<=length(parts);i++) val = val "=" parts[i]
                gsub(/^[ \t]+|[ \t]+$/,"", val)
                sub(/[ \t]*;.*/, "", val)
                print val
                exit
            }
        }'
}

get_sdk_param() {
    local key="$1" default="$2"
    if [ -f "$SDK_INI_FILE" ]; then
        local val=$(get_ini_value "$SDK_INI_FILE" "sdk_params" "$key")
        [ "$val" != "NOT_FOUND" ] && echo "$val" || echo "$default"
    else
        echo "$default"
    fi
}

# ==================== 原子写入 JSON 进度文件 ====================

atomic_write_progress() {
    local status="$1"
    local step_idx="${2:-}"
    local error_msg="${3:-}"

    python3 -c "
import json, os, tempfile

PROG_FILE = '$TASK_PROGRESS_FILE'
status = '$status'
step_idx = $step_idx if '$step_idx'.isdigit() else None
error_msg = '$error_msg'

try:
    data = None
    if os.path.exists(PROG_FILE):
        with open(PROG_FILE, 'r') as f:
            data = json.load(f)

    if data is None or 'actions' not in data:
        raise ValueError('Invalid progress data')

    if step_idx is not None and step_idx < len(data['actions']):
        data['actions'][step_idx]['status'] = status
        if status == 'running':
            data['step'] = data['actions'][step_idx]['name']
        if error_msg and status == 'failed':
            data['actions'][step_idx]['error'] = error_msg

    tmp_fd, tmp_path = tempfile.mkstemp(
        dir=os.path.dirname(PROG_FILE),
        suffix='.json'
    )
    with os.fdopen(tmp_fd, 'w') as f:
        json.dump(data, f, ensure_ascii=False)
    os.replace(tmp_path, PROG_FILE)
except Exception as e:
    try:
        if data is not None:
            with open(PROG_FILE, 'w') as f:
                json.dump(data, f, ensure_ascii=False)
    except Exception:
        pass
"
}

# ==================== 远程通知 (webhook) ====================

send_notify() {
    local message="$1"
    local level="${2:-INFO}"
    if [ -z "$G1D_WEBHOOK_URL" ]; then
        return
    fi
    python3 -c "
import json, requests, sys
url = '$G1D_WEBHOOK_URL'
msg = '''$message'''
level = '$level'
try:
    payload = {'msgtype': 'text', 'text': {'content': f'[G1D {level}] {msg}'}}
    requests.post(url, json=payload, timeout=5)
except Exception:
    pass
" 2>/dev/null || true
}

# ==================== 健康检查 ====================

function_health_check() {
    local failed=0
    echo ""
    echo "🏥 ========== 任务前健康检查 =========="

    # 1. YOLO 服务
    echo -n "  YOLO 服务 (18081) ... "
    if curl -s --connect-timeout 3 --max-time 5 "http://192.168.123.164:18081/health" > /dev/null 2>&1; then
        echo "✅ 正常"
    else
        echo "❌ 不可达"
        failed=$((failed + 1))
    fi

    # 2. 微调服务
    echo -n "  微调服务 (18084) ... "
    if curl -s --connect-timeout 3 --max-time 5 "http://192.168.123.164:18084/health" > /dev/null 2>&1; then
        echo "✅ 正常"
    else
        echo "❌ 不可达"
        failed=$((failed + 1))
    fi

    # 3. 机械臂 API
    echo -n "  机械臂API (18083) ... "
    if curl -s --connect-timeout 3 --max-time 5 "http://192.168.123.164:18083/api/health" > /dev/null 2>&1; then
        echo "✅ 正常"
    else
        echo "❌ 不可达"
        failed=$((failed + 1))
    fi

    # 4. 夹爪
    echo -n "  夹爪 (18080) ... "
    if curl -s --connect-timeout 3 --max-time 5 "${GRIPPER_URL}/api/v1/status" > /dev/null 2>&1; then
        echo "✅ 正常"
    else
        echo "❌ 不可达"
        failed=$((failed + 1))
    fi

    # 5. 机械臂进程
    echo -n "  机械臂进程 (arm_task_node.py) ... "
    if pgrep -f "arm_task_node.py" > /dev/null 2>&1; then
        echo "✅ 运行中"
    else
        echo "❌ 未运行"
        failed=$((failed + 1))
    fi

    # 6. 立柱 offset
    echo -n "  立柱 offset ... "
    if [ -f "$LIFT_OFFSET_FILE" ]; then
        local offset_val=$(python3 -c "import json; print(json.load(open('$LIFT_OFFSET_FILE')).get('offset','N/A'))" 2>/dev/null || echo "N/A")
        echo "✅ 已检测 (${offset_val}m)"
    else
        echo "⚠️ 未检测（status_monitor 可能尚未完成开机检测）"
    fi

    # 7. ROS2 odom 话题
    echo -n "  ROS2 /agv/odom ... "
    local odom_check=$(ros2 topic list 2>/dev/null | grep -c "/agv/odom" || echo "0")
    if [ "$odom_check" -ge 1 ]; then
        echo "✅ 存在"
    else
        echo "❌ 未发现"
        failed=$((failed + 1))
    fi

    echo "========================================"
    if [ $failed -gt 0 ]; then
        echo "⚠️  有 $failed 项检查未通过！"
        return 1
    else
        echo "✅ 所有核心检查通过"
        return 0
    fi
}

# ==================== 偏移校准引导 ====================

function_calibrate_offset() {
    echo ""
    echo "🔧 ========== 立柱偏移校准 =========="
    echo "此功能将："
    echo "  1. 让立柱下降到物理最低点"
    echo "  2. 读取 /hispeed_state 的 y 值作为 offset"
    echo "  3. 写入 $LIFT_OFFSET_FILE"
    echo ""

    read -p "确认开始校准？(y/n): " cf
    [ "$cf" == "y" ] || { echo "已取消"; return; }

    # 检查 status_monitor 是否在运行
    if ! systemctl is-active --quiet g1d-monitor 2>/dev/null; then
        echo "⚠️  status_monitor 未运行，尝试直接读取 /hispeed_state"
    fi

    echo "⏳ 等待立柱下降到最低点（最多30秒）..."

    # 等待并采样 hispeed_state
    local offset_value=$(python3 -c "
import rclpy, time, json, os, tempfile
from rclpy.node import Node
try:
    from geometry_msgs.msg import Vector3 as HispeedMsg
except ImportError:
    HispeedMsg = None

if HispeedMsg is None:
    print('ERROR: 无法导入 HispeedMsg')
    exit(1)

rclpy.init()
node = Node('offset_calibrate')

min_height = 999.0
stable_count = 0
start = time.time()

def cb(msg):
    global min_height, stable_count
    h = msg.y
    if h < min_height:
        min_height = h
        stable_count = 0
    if abs(h - min_height) < 0.002:
        stable_count += 1
    else:
        stable_count = 0

node.create_subscription(HispeedMsg, '/hispeed_state', cb, 10)

while time.time() - start < 30 and stable_count < 3:
    rclpy.spin_once(node, timeout_sec=0.5)

node.destroy_node()
rclpy.shutdown()

if min_height < 1.0:
    # 原子写入
    data = {'offset': min_height, 'timestamp': time.time()}
    tmp_fd, tmp_path = tempfile.mkstemp(dir=os.path.dirname('$LIFT_OFFSET_FILE'), suffix='.json')
    with os.fdopen(tmp_fd, 'w') as f:
        json.dump(data, f)
    os.replace(tmp_path, '$LIFT_OFFSET_FILE')
    print(f'{min_height:.4f}')
else:
    print('ERROR: 未检测到有效高度')
" 2>&1)

    if [[ "$offset_value" == ERROR* ]]; then
        echo "❌ 校准失败: $offset_value"
    else
        echo "✅ 校准完成！offset = ${offset_value}m"
        echo "   物理范围: [0, 0.427]m"
        echo "   SDK 范围: [${offset_value}, $(python3 -c "print(f'{0.427 + float(\"${offset_value}\"):.4f}')")]m"
        send_notify "立柱偏移校准完成: offset=${offset_value}m"
    fi
}

# ==================== 步骤命令构建 ====================

build_step_cmd() {
    local step="$1" start_pose="$2" end_pose="$3" item_name="$4"
    step=$(echo "$step" | xargs)
    case "$step" in
        nav_start) echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py nav_start --pose '$start_pose'" ;;
        nav_end)   echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py nav_end --pose '$end_pose'" ;;
        nav_start_lift\(*\))
            local h=$(echo "$step" | grep -oP '(?<=\().*(?=\))')
            echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py nav_start_lift --pose '$start_pose' --height $h" ;;
        nav_end_lift\(*\))
            local h=$(echo "$step" | grep -oP '(?<=\().*(?=\))')
            echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py nav_end_lift --pose '$end_pose' --height $h" ;;
        rotate_lift\(*\))
            local p=$(echo "$step" | grep -oP '(?<=\().*(?=\))')
            echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py rotate_lift --params '$p'" ;;
        pick)      echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py pick --target '$item_name'" ;;
        place)     echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py place" ;;
        reset)     echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py reset" ;;
        weitiao)   echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py weitiao --target '$item_name'" ;;
        yolo_pick) echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py yolo_pick --target '$item_name'" ;;
        verify_distance) echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py verify_distance" ;;
        adjust_retry) echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py adjust_retry --target '$item_name'" ;;
        lift_to\(*\))
            local h=$(echo "$step" | grep -oP '(?<=\().*(?=\))')
            echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py lift_to --height $h" ;;
        backup\(*\))
            local d=$(echo "$step" | grep -oP '(?<=\().*(?=\))')
            echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py backup --distance $d" ;;
        rotate\(*\))
            local a=$(echo "$step" | grep -oP '(?<=\().*(?=\))')
            echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py rotate --angle $a" ;;
        move\(*\))
            local p=$(echo "$step" | grep -oP '(?<=\().*(?=\))')
            echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py move --params '$p'" ;;
        gripper\(*\))
            local a=$(echo "$step" | grep -oP '(?<=\().*(?=\))')
            echo "python3 $PROJECT_HOME_DIR/bin/step_executor.py gripper --action $a" ;;
        *) echo "" ;;
    esac
}
