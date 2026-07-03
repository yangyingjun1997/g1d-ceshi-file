#!/usr/bin/env python3
"""
G1D 立柱偏移检测服务 V1.2
运行在机器人本体 (192.168.123.164)

功能：
  1. 开机后自动订阅 /hispeed_state，检测立柱下降到最低点时的 y 值作为 offset
  2. 提供 HTTP 接口供外部设备 (192.168.123.5) 读取 offset 和实时高度
  3. 【新增】提供 /api/basic_status 极简兜底端点，供调用方在5端故障时降级使用
  4. offset 文件持久化，本体重启自动重新检测
  5. ROS2 订阅 + shell 后备双模式，确保总能获取 hispeed 数据

HTTP 端口：28089
  GET /api/offset        → 完整状态（含检测状态、缓存信息等）
  GET /api/basic_status  → 极简兜底数据（<20ms响应，零ROS2依赖）
  GET /health            → 健康检查

部署：
  sudo cp g1d_offset_detector.py /home/unitree/
  sudo cp g1d-offset-detector.service /etc/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable g1d-offset-detector
  sudo systemctl restart g1d-offset-detector
"""
import json, os, time, threading, subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler
from datetime import datetime

# ---- 常量 ----
HTTP_PORT = 28089
OFFSET_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lift_offset.json")
FULL_TRAVEL = 0.427
DETECT_TIMEOUT = 60      # 检测超时（秒）
STABLE_THRESHOLD = 0.002  # 高度稳定阈值（米）
STABLE_COUNT = 3          # 连续稳定次数
HISPEED_POLL_INTERVAL = 2.0  # shell 后备轮询间隔（秒）

# ---- 全局状态 ----
_state = {
    "offset": None,
    "uptime_sec": 0.0,
    "boot_id": "",
    "detecting": True,
    "column_height": -1.0,
    "physical_height": -1.0,
    "timestamp": "",
}
_lock = threading.Lock()
_running = True
_hispeed_ever_received = False
_hispeed_last_update = 0.0  # 【新增】记录最后一次收到hispeed数据的时间戳


def get_boot_id():
    """获取本次启动的唯一标识（/proc/sys/kernel/random/boot_id）"""
    try:
        with open("/proc/sys/kernel/random/boot_id", "r") as f:
            return f.read().strip()
    except Exception:
        return ""


def get_uptime():
    """获取系统运行时间（秒）"""
    try:
        with open("/proc/uptime", "r") as f:
            return float(f.read().split()[0])
    except Exception:
        return 0.0


def save_offset(offset_value):
    """原子写入 offset 文件"""
    import tempfile
    data = {
        "offset": offset_value,
        "uptime_sec": get_uptime(),
        "boot_id": get_boot_id(),
        "timestamp": datetime.now().isoformat(),
        "full_travel_m": FULL_TRAVEL,
    }
    try:
        dir_name = os.path.dirname(OFFSET_FILE) or "."
        os.makedirs(dir_name, exist_ok=True)
        tmp_fd, tmp_path = tempfile.mkstemp(dir=dir_name, suffix=".json")
        with os.fdopen(tmp_fd, "w") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp_path, OFFSET_FILE)
    except Exception as e:
        print(f"[offset_detector] 保存 offset 文件失败: {e}")


def load_offset_if_current():
    """如果 offset 文件属于本次启动则加载，否则返回 None"""
    if not os.path.exists(OFFSET_FILE):
        return None
    try:
        with open(OFFSET_FILE, "r") as f:
            data = json.load(f)
        if data.get("boot_id") == get_boot_id():
            return data.get("offset")
        return None
    except Exception:
        return None


def update_physical_height():
    """根据当前 column_height 和 offset 计算物理高度"""
    with _lock:
        h = _state["column_height"]
        offset = _state["offset"]
        if offset is not None and h > -1.0:
            _state["physical_height"] = round(h - offset, 4)
        else:
            _state["physical_height"] = -1.0


def _mark_hispeed_received():
    """【新增】标记hispeed数据已更新，需在锁外调用以避免死锁"""
    global _hispeed_last_update, _hispeed_ever_received
    _hispeed_last_update = time.time()
    _hispeed_ever_received = True


def shell_read_hispeed():
    """shell 后备：读取 /hispeed_state 的 y 值（兼容 Foxy/Humble）"""
    # 本体是 Foxy，不支持 --once，用 timeout + head 替代
    # 同时显式指定消息类型 Vector3 避免多类型话题报错
    try:
        res = subprocess.run(
            "source /opt/ros/foxy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null; "
            "timeout 3 ros2 topic echo /hispeed_state geometry_msgs/msg/Vector3 2>/dev/null | head -n 4 | grep 'y:' | awk '{print $2}'",
            shell=True, capture_output=True, text=True, timeout=8,
            executable="/bin/bash"
        )
        if res.returncode == 0 and res.stdout.strip():
            h = float(res.stdout.strip())
            with _lock:
                _state["column_height"] = h
            _mark_hispeed_received()  # 【修改】使用独立函数更新时间戳
            update_physical_height()
            return h
    except Exception:
        pass
    return None


# ==================== ROS2 偏移检测 ====================

def ros2_detect_and_monitor():
    """使用 ROS2 订阅检测偏移并持续监控，失败返回 False"""
    try:
        import rclpy
        from rclpy.node import Node
        try:
            from geometry_msgs.msg import Vector3 as HispeedMsg
        except ImportError:
            HispeedMsg = None

        if HispeedMsg is None:
            print("[offset_detector] 无法导入 HispeedMsg，使用 shell 后备")
            return False

        rclpy.init()
        node = rclpy.create_node("offset_detector")

        min_height = 999.0
        stable_count = 0
        offset_detected = False
        last_cb_time = 0.0

        def hispeed_cb(msg):
            nonlocal min_height, stable_count, offset_detected, last_cb_time
            h = msg.y
            last_cb_time = time.time()
            with _lock:
                _state["column_height"] = h
            _mark_hispeed_received()  # 【修改】使用独立函数更新时间戳
            update_physical_height()

            if not offset_detected and -1.0 <= h <= 0.5:
                if h < min_height:
                    min_height = h
                    stable_count = 0
                if abs(h - min_height) < STABLE_THRESHOLD:
                    stable_count += 1
                else:
                    stable_count = 0

        # Foxy 上多类型话题需要指定 QoS，使用兼容的 best_effort
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(HispeedMsg, "/hispeed_state", hispeed_cb, qos)
        print("[offset_detector] ROS2 已订阅 /hispeed_state (Vector3, best_effort)")

        # 阶段1：检测 offset
        start = time.time()
        while time.time() - start < DETECT_TIMEOUT and not offset_detected:
            rclpy.spin_once(node, timeout_sec=0.5)
            if stable_count >= STABLE_COUNT and (time.time() - start) > 2.0:
                offset_detected = True
                with _lock:
                    _state["offset"] = min_height
                    _state["detecting"] = False
                    _state["timestamp"] = datetime.now().isoformat()
                save_offset(min_height)
                print(f"[offset_detector] ✅ 偏移检测完成: offset = {min_height:.4f}m")
                update_physical_height()

        if not offset_detected and min_height < 1.0:
            with _lock:
                _state["offset"] = min_height
                _state["detecting"] = False
                _state["timestamp"] = datetime.now().isoformat()
            save_offset(min_height)
            print(f"[offset_detector] ⚠️ 超时，使用当前最低值: offset = {min_height:.4f}m")
            update_physical_height()

        # 阶段2：持续监控，ROS2 断流时 shell 后备
        print("[offset_detector] 进入持续监控模式...")
        while _running:
            rclpy.spin_once(node, timeout_sec=0.5)
            # ROS2 订阅超过 5 秒未更新 → 启动 shell 后备
            if last_cb_time > 0 and time.time() - last_cb_time > 5.0:
                shell_read_hispeed()

        node.destroy_node()
        rclpy.shutdown()
        return True

    except Exception as e:
        print(f"[offset_detector] ROS2 异常: {e}")
        return False


def shell_detect_and_monitor():
    """纯 shell 后备模式：检测偏移并持续监控"""
    print("[offset_detector] 使用 shell 后备模式...")

    min_height = 999.0
    stable_count = 0
    start = time.time()
    offset_detected = False

    # 阶段1：检测 offset
    while time.time() - start < DETECT_TIMEOUT and not offset_detected:
        h = shell_read_hispeed()
        if h is not None and -1.0 <= h <= 0.5:
            if h < min_height:
                min_height = h
                stable_count = 0
            if abs(h - min_height) < STABLE_THRESHOLD:
                stable_count += 1
            else:
                stable_count = 0
            if stable_count >= STABLE_COUNT and (time.time() - start) > 2.0:
                offset_detected = True
        time.sleep(HISPEED_POLL_INTERVAL)

    if min_height < 1.0:
        with _lock:
            _state["offset"] = min_height
            _state["detecting"] = False
            _state["timestamp"] = datetime.now().isoformat()
        save_offset(min_height)
        print(f"[offset_detector] offset = {min_height:.4f}m (shell 模式)")
        update_physical_height()
    else:
        print("[offset_detector] ❌ shell 模式也未检测到有效高度")

    # 阶段2：持续 shell 监控
    print("[offset_detector] 持续监控中...")
    while _running:
        shell_read_hispeed()
        time.sleep(HISPEED_POLL_INTERVAL)


# ==================== HTTP 服务 ====================

class OffsetHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _json_response(self, data, code=200):
        """【新增】统一JSON响应方法"""
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode())

    def do_GET(self):
        if self.path == "/api/offset":
            with _lock:
                data = dict(_state)
                data["uptime_sec"] = get_uptime()
                data["boot_id"] = get_boot_id()
                data["full_travel_m"] = FULL_TRAVEL
            self._json_response(data)

        # 【新增】极简兜底端点：供5端挂掉时调用方降级使用
        elif self.path == "/api/basic_status":
            try:
                with _lock:
                    st = dict(_state)
                basic_data = {
                    "hispeed_y_m": st.get("column_height", -1.0),
                    "lift_offset_m": st.get("offset"),
                    "physical_height_m": st.get("physical_height", -1.0),
                    "full_travel_m": FULL_TRAVEL,
                    "boot_id": st.get("boot_id", "") or get_boot_id(),
                    "uptime_sec": round(get_uptime(), 2),
                    "detecting": st.get("detecting", True),
                    "data_age_sec": round(time.time() - _hispeed_last_update, 3) if _hispeed_last_update > 0 else None,
                    "source": "164_basic",
                    "timestamp": time.time()
                }
                self._json_response(basic_data)
            except Exception as e:
                self._json_response({"error": str(e), "source": "164_basic"}, 500)

        elif self.path == "/health":
            self._json_response({"ok": True})

        else:
            self.send_response(404)
            self.end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        self.end_headers()


# ==================== 主入口 ====================

def main():
    global _running
    print("=" * 50)
    print("G1D 立柱偏移检测服务 V1.3")
    print("=" * 50)

    # 检查是否已有本次启动的 offset
    cached = load_offset_if_current()
    if cached is not None:
        print(f"[offset_detector] 本次启动已有 offset: {cached:.4f}m，跳过检测")
        with _lock:
            _state["offset"] = cached
            _state["detecting"] = False
            _state["boot_id"] = get_boot_id()
            _state["timestamp"] = datetime.now().isoformat()
    else:
        print("[offset_detector] 本次启动尚未检测 offset，开始检测...")

    # 启动 HTTP 服务（先于检测，确保外部能立即访问）
    class ReuseHTTPServer(HTTPServer):
        allow_reuse_address = True
    server = ReuseHTTPServer(("0.0.0.0", HTTP_PORT), OffsetHandler)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    print(f"[offset_detector] HTTP 服务已启动，端口 {HTTP_PORT}")

    # 立即尝试读取一次 hispeed，确保 column_height 不为 -1
    shell_read_hispeed()

    # ROS2 检测+监控在独立线程运行，不阻塞 HTTP 主线程
    def detect_and_monitor_thread():
        nonlocal cached
        if cached is None:
            if not ros2_detect_and_monitor():
                shell_detect_and_monitor()
        else:
            # 已有 offset，直接进入监控模式
            if not ros2_detect_and_monitor():
                shell_detect_and_monitor()

    monitor_thread = threading.Thread(target=detect_and_monitor_thread, daemon=True)
    monitor_thread.start()

    # 主线程保持 HTTP 响应
    try:
        while _running:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        _running = False
        server.shutdown()


if __name__ == "__main__":
    main()