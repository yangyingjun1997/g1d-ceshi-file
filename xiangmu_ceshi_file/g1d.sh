#!/bin/bash
# G1D 机器人控制系统 - 主入口脚本
# 用法: ./g1d.sh [命令]
#   无参数: 显示交互菜单
#   install: 安装本机 systemd 服务
#   status: 查看本机所有 g1d 服务状态
#   config: 编辑配置文件
#   control|monitor|dashboard|offset|order: 直接启动对应脚本

G1D_HOME="$(cd "$(dirname "$0")" && pwd)"
CONF="$G1D_HOME/conf/params.ini"

# 自动检测当前设备
detect_device() {
    # 多策略获取本机IP
    local ip=""
    # 策略1: hostname -I (Linux 标准)
    ip=$(hostname -I 2>/dev/null | awk '{print $1}')
    # 策略2: ip addr show (回退)
    [ -z "$ip" ] && ip=$(ip addr show 2>/dev/null | grep -Eo 'inet (192\.168\.[0-9]+\.[0-9]+)' | awk '{print $2}' | head -1)
    # 策略3: ifconfig (老旧系统回退)
    [ -z "$ip" ] && ip=$(ifconfig 2>/dev/null | grep -Eo 'inet (addr:)?192\.168\.[0-9]+\.[0-9]+' | head -1 | awk '{print $2}' | sed 's/addr://')
    # 策略4: hostname 回退（无法获取IP时）
    if [ -z "$ip" ]; then
        case "$(hostname)" in
            *unitree*|*Unitree*) echo "164"; return ;;
            *nuc*|*NUC*|*robot*) echo "5"; return ;;
            *4090*) echo "4090"; return ;;
        esac
    fi

    case "$ip" in
        192.168.123.5) echo "5" ;;
        192.168.123.164) echo "164" ;;
        192.168.100.100) echo "4090" ;;
        *) echo "unknown" ;;
    esac
}

DEVICE=$(detect_device)

show_menu() {
    echo "========================================"
    echo "  G1D 机器人控制系统"
    echo "========================================"
    echo "  项目目录: $G1D_HOME"
    echo "  当前设备: ${DEVICE}端"
    echo ""
    
    case "$DEVICE" in
        5)
            echo "  [1] 启动控制API     (port 28091)"
            echo "  [2] 启动状态监控     (port 28087)"
            echo "  [3] 启动服务面板     (port 28092)"
            echo "  [4] SDK交互菜单"
            ;;
        164)
            echo "  [1] 启动Offset检测   (port 28089)"
            ;;
        4090)
            echo "  [1] 启动订单看板     (port 28090)"
            ;;
        *)
            echo "  [1] 控制API (5端)"
            echo "  [2] 状态监控 (5端)"
            echo "  [3] 服务面板 (5端)"
            echo "  [4] 订单看板 (4090端)"
            echo "  [5] Offset检测 (164端)"
            echo "  [6] SDK交互菜单"
            ;;
    esac
    
    echo ""
    echo "  [i] 安装本机服务 (systemd)"
    echo "  [s] 查看服务状态"
    echo "  [c] 编辑配置文件"
    echo "  [0] 退出"
    echo "========================================"
}

install_services() {
    echo ">>> 安装 ${DEVICE}端 systemd 服务..."
    local svc_dir="$G1D_HOME/services/$DEVICE"
    if [ ! -d "$svc_dir" ]; then
        echo "未找到 ${DEVICE}端的服务文件目录"
        return 1
    fi

    # 根据设备确定用户名和 home 目录
    local user home_dir
    case "$DEVICE" in
        164) user="unitree"; home_dir="/home/unitree" ;;
        5)   user="robot"; home_dir="/home/robot" ;;
        4090) user="ubuntu"; home_dir="/home/ubuntu" ;;
        *)   user="$(whoami)"; home_dir="$HOME" ;;
    esac

    for svc in "$svc_dir"/*.service; do
        [ -f "$svc" ] || continue
        local name=$(basename "$svc")
        # 动态替换路径并安装
        sed -e "s|__G1D_HOME__|$G1D_HOME|g" \
            -e "s|__G1D_USER__|$user|g" \
            "$svc" | sudo tee "/etc/systemd/system/$name" > /dev/null
        sudo systemctl daemon-reload
        sudo systemctl enable "$name"
        echo "  ✓ $name 已安装并启用 (路径: $G1D_HOME)"
    done
    echo ">>> 安装完成"
}

show_status() {
    echo ">>> G1D 服务状态 (${DEVICE}端)..."
    for svc in "$G1D_HOME/services/$DEVICE"/*.service; do
        [ -f "$svc" ] || continue
        local name=$(basename "$svc")
        sudo systemctl status "$name" --no-pager -l 2>/dev/null | head -5
        echo ""
    done
}

edit_config() {
    ${EDITOR:-nano} "$CONF"
}

# 快捷命令
case "${1:-}" in
    install) install_services; exit 0 ;;
    status) show_status; exit 0 ;;
    config) edit_config; exit 0 ;;
    control) python3 "$G1D_HOME/bin/g1d_control_api.py"; exit 0 ;;
    monitor) python3 "$G1D_HOME/bin/status_monitor.py"; exit 0 ;;
    dashboard) python3 "$G1D_HOME/bin/service_dashboard.py"; exit 0 ;;
    offset) python3 "$G1D_HOME/bin/g1d_offset_detector.py"; exit 0 ;;
    order) python3 "$G1D_HOME/bin/order_dashboard.py"; exit 0 ;;
    menu|'') ;;
    *) echo "未知命令: $1"; exit 1 ;;
esac

# 交互菜单
while true; do
    show_menu
    read -p "请选择: " choice
    case "$choice" in
        1)
            case "$DEVICE" in
                5) python3 "$G1D_HOME/bin/g1d_control_api.py" ;;
                164) python3 "$G1D_HOME/bin/g1d_offset_detector.py" ;;
                4090) python3 "$G1D_HOME/bin/order_dashboard.py" ;;
                *) python3 "$G1D_HOME/bin/g1d_control_api.py" ;;
            esac
            ;;
        2)
            case "$DEVICE" in
                5) python3 "$G1D_HOME/bin/status_monitor.py" ;;
                *) python3 "$G1D_HOME/bin/status_monitor.py" ;;
            esac
            ;;
        3)
            case "$DEVICE" in
                5) python3 "$G1D_HOME/bin/service_dashboard.py" ;;
                *) python3 "$G1D_HOME/bin/service_dashboard.py" ;;
            esac
            ;;
        4)
            case "$DEVICE" in
                5) bash "$G1D_HOME/bin/mainbash.sh" ;;
                *) python3 "$G1D_HOME/bin/order_dashboard.py" ;;
            esac
            ;;
        5) python3 "$G1D_HOME/bin/g1d_offset_detector.py" ;;
        6) bash "$G1D_HOME/bin/mainbash.sh" ;;
        i|I) install_services ;;
        s|S) show_status ;;
        c|C) edit_config ;;
        0) exit 0 ;;
        *) echo "无效选择" ;;
    esac
done
