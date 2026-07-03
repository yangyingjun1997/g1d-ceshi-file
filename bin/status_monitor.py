#!/usr/bin/env python3
"""
G1D 状态监控 V9.3 (异步并发 + 双机冗余版)
- 启动提速: DDS保护期3s + Shell异步线程 + 10s订阅重建
- 数据完整: 保留Vector3/Point32双订阅 + SDK降级
- 冗余支持: 配合164端 /api/basic_status 实现调用方自动降级
"""
import json, os, socket, subprocess, sys, threading, time, collections, glob, shutil, gzip
from datetime import datetime, timedelta
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point32
from std_msgs.msg import String

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PROJECT_ROOT, "lib"))
sys.path.insert(0, os.path.join(_PROJECT_ROOT, "web"))

from g1d_common import (
    log, HispeedMsg,
    LIFT_OFFSET_FILE, TASK_PROGRESS_FILE, LAST_ARM_STATUS_FILE,
    FULL_TRAVEL, GRIPPER_URL, ARM_API_URL, ARM_HEALTH_URL,
    YOLO_XYZ_URL, YOLO_HEALTH_URL, ADJUST_HEALTH_URL, ADJUST_URL,
    POINT_INF_FILE, SDK_PARAMS_FILE, PROJECT_HOME, SDK_HEIGHT_BIN, SDK_SIMPLE_BIN,
    SSH_USER, SSH_HOST, ROBOT_IF,
    read_lift_offset, save_lift_offset, get_physical_height,
    is_chassis_power_cycled,
    yaw_from_odom, check_arm_process, atomic_write_json,
    CHASSIS_BOOT_MAX_UPTIME,
)
from dashboard_template import DASHBOARD_HTML

HTTP_PORT = 28087
OFFSET_DETECTOR_URL = "http://192.168.123.164:28089"
LOG_DIR = Path(PROJECT_HOME)
LOG_DIR.mkdir(parents=True, exist_ok=True)
STATUS_LOG_FILE = LOG_DIR / "status_log.jsonl"
ARM_STALE_TIMEOUT = 5.0
ARM_BACKUP_FILE = LOG_DIR / "last_arm_status.json"
MAX_LOG_SIZE = 10 * 1024 * 1024
MAX_LOG_DAYS = 3
HISTORY_LENGTH = 300
HISTORY_KEYS = ["column_height", "physical_height", "odom.x", "odom.y", "odom.yaw",
                "velocity.linear_x", "velocity.angular_z"]
OFFSET_STABLE_THRESHOLD = 3
OFFSET_EPSILON = 0.002
OFFSET_MAX_WAIT = 30.0

# 【优化】启动时序参数
DDS_DISCOVERY_GRACE_PERIOD = 3.0   # 从8s降至3s
OFFSET_FETCH_INTERVAL = 5.0
RESUBSCRIBE_TRIGGER_TIME = 10.0    # 从30s降至10s


class StatusMonitor(Node):
    def __init__(self):
        super().__init__('status_monitor')
        self._start_time = time.time()
        self._startup_log_done = False

        self.state = {
            "timestamp": "",
            "odom": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            "velocity": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
            "cmd_vel": {"linear_x": 0.0, "linear_y": 0.0, "angular_z": 0.0},
            "column_height": -1.0,
            "physical_height": -1.0,
            "lift_offset": None,
            "arm_status": {"phase": "", "status_text": "", "exec_status": -1},
            "yolo": {
                "range_mm": -1, "selected_label": "", "forward_distance_m": -1,
                "turn_first_yaw_deg": -1, "lateral_error_m": 0.0, "height_down_m": 0.0,
                "box_parallel_yaw_deg": -1, "confidence": 0.0, "reproj_error_px": 0.0,
                "depth_delta_mm": 0.0, "orientation": "unknown",
                "near_edge_forward_mm": 0, "center_forward_mm": 0,
                "center_vertical_mm": 0, "lateral_mm": 0,
                "yolo_health": "unknown", "adjust_health": "unknown",
                "arm_api_health": "unknown", "alignment_status": "unknown"
            },
            "gripper": {"state": "unknown"},
            "task_progress": {},
            "point_inf": [],
            "sdk_params": {},
            "arm_process_running": False,
            "camera_stream_running": False,
            "_last_ros_arm_time": 0.0,
            "_last_odom_time": 0.0,
            "last_command_result": None,
        }
        self.lock = threading.Lock()

        self._history = {k: collections.deque(maxlen=HISTORY_LENGTH) for k in HISTORY_KEYS}
        self._history_ts = collections.deque(maxlen=HISTORY_LENGTH)
        self._events = collections.deque(maxlen=50)
        # 追踪 control_api 异步任务状态
        self._pending_tasks = {}  # task_id -> {"cmd": "rotate", "start_time": ...}
        self._pending_lock = threading.Lock()
        self._prev_task_status = None

        # ROS2 订阅
        self.create_subscription(Odometry, '/agv/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(String, '/arm_control_refactor/task_status', self.arm_status_cb, 10)

        # 降级模式的 cmd_vel publisher（避免 control_api 不可达时二开 ROS2 节点冲突）
        self._fallback_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        hispeed_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        if HispeedMsg is not None:
            self.create_subscription(HispeedMsg, '/hispeed_state', self.hispeed_cb, hispeed_qos)
            self.get_logger().info("已订阅 /hispeed_state (Vector3)")
        try:
            self.create_subscription(Point32, '/hispeed_state', self.hispeed_cb_point32, hispeed_qos)
            self.get_logger().info("已订阅 /hispeed_state (Point32)")
        except Exception as e:
            self.get_logger().warn(f"无法订阅 Point32: {e}")

        self._hispeed_last_update = 0.0
        self._hispeed_ever_received = False

        # Offset 状态
        self._offset_min_height = 999.0
        self._offset_stable_count = 0
        self._offset_detected = False
        self._offset_start_time = time.time()
        self._last_offset_fetch_time = 0.0
        self._last_robot_uptime = None
        self._last_remote_offset_data = {}
        self._last_remote_fetch_ts = 0.0

        # HTTP连接池
        self._img_session = requests.Session()
        adapter = requests.adapters.HTTPAdapter(pool_connections=4, pool_maxsize=8)
        self._img_session.mount('http://', adapter)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.start_http_server()
        # 启动任务追踪线程（轮询 control_api 任务状态，发 command_done/error 事件）
        threading.Thread(target=self._task_tracker_loop, daemon=True).start()
        self.get_logger().info(f"[STARTUP] t=0.0s StatusMonitor V9.3 初始化完成")

    # ==================== ROS2 回调 ====================
    def odom_cb(self, msg):
        try:
            with self.lock:
                self.state["odom"].update({"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y,
                                           "z": msg.pose.pose.position.z, "yaw": yaw_from_odom(msg)})
                self.state["velocity"].update({"linear_x": msg.twist.twist.linear.x,
                                               "linear_y": msg.twist.twist.linear.y,
                                               "angular_z": msg.twist.twist.angular.z})
                self.state["_last_odom_time"] = time.time()
        except Exception as e:
            self.get_logger().error(f"odom 回调异常: {e}")

    def cmd_vel_cb(self, msg):
        with self.lock:
            self.state["cmd_vel"].update({"linear_x": msg.linear.x, "linear_y": msg.linear.y, "angular_z": msg.angular.z})

    def arm_status_cb(self, msg):
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.state["arm_status"].update({"phase": data.get("phase", ""), "status_text": data.get("status_text", ""),
                                                 "exec_status": int(data.get("exec_status", -1))})
                self.state["_last_ros_arm_time"] = time.time()
            atomic_write_json(str(ARM_BACKUP_FILE), self.state["arm_status"])
        except Exception as e:
            self.get_logger().warn(f"arm_status 回调异常: {e}")

    def _update_hispeed(self, y_val, source_tag):
        with self.lock:
            self.state["column_height"] = y_val
            self._hispeed_last_update = time.time()
            self._hispeed_ever_received = True
        offset = self.state.get("lift_offset")
        if offset is not None:
            self.state["physical_height"] = get_physical_height(y_val, offset)
        log_attr = f'_hispeed_{source_tag}_logged'
        if not hasattr(self, log_attr):
            setattr(self, log_attr, True)
            elapsed = time.time() - self._start_time
            self.get_logger().info(f"[STARTUP] t={elapsed:.1f}s hispeed ({source_tag}) 收到数据: y={y_val:.4f}")

    def hispeed_cb(self, msg):
        self._update_hispeed(msg.y, "Vector3")

    def hispeed_cb_point32(self, msg):
        self._update_hispeed(msg.y, "Point32")

    # ==================== Offset 获取 ====================
    def _fetch_offset_from_robot(self):
        now = time.time()
        if now - self._last_offset_fetch_time < OFFSET_FETCH_INTERVAL:
            return self._last_remote_offset_data or None
        self._last_offset_fetch_time = now
        try:
            resp = requests.get(f"{OFFSET_DETECTOR_URL}/api/offset", timeout=(1, 2))
            if resp.status_code != 200: return None
            data = resp.json()
            self._last_remote_offset_data = data
            self._last_remote_fetch_ts = now

            remote_height = data.get("column_height", -1.0)
            if remote_height > -1.0 and not self._hispeed_ever_received:
                self._update_hispeed(remote_height, "Remote164")

            robot_uptime = data.get("uptime_sec", 0)
            self._last_robot_uptime = robot_uptime
            my_uptime = time.time() - self._start_time

            if is_chassis_power_cycled(my_uptime, robot_uptime):
                if self._offset_detected:
                    self.get_logger().warn("检测到底盘断电重启，清除旧offset")
                self._offset_detected = False
                self.state["lift_offset"] = None
                self._offset_min_height = 999.0
                self._offset_stable_count = 0
                self._offset_start_time = time.time()
                try: os.remove(LIFT_OFFSET_FILE)
                except: pass
                return data

            if self.state.get("lift_offset") is None:
                offset = data.get("offset")
                if offset is not None:
                    self.state["lift_offset"] = offset
                    self._offset_detected = True

            cur_height = self.state.get("column_height", -1.0)
            cur_offset = self.state.get("lift_offset")
            if cur_offset is not None and cur_height > -1.0:
                self.state["physical_height"] = get_physical_height(cur_height, cur_offset)
            return data
        except requests.exceptions.Timeout:
            pass
        except Exception:
            pass
        return None

    def _read_local_offset_cache(self):
        try:
            if os.path.exists(LIFT_OFFSET_FILE):
                with open(LIFT_OFFSET_FILE, 'r') as f:
                    data = json.load(f)
                return {"value": data.get("offset"), "boot_id": data.get("boot_id"),
                        "timestamp": data.get("timestamp"), "file_path": str(LIFT_OFFSET_FILE)}
        except: pass
        return None

    def _build_offset_backup_data(self):
        remote_data = self._fetch_offset_from_robot()
        local_cache = self._read_local_offset_cache()
        current_offset = self.state.get("lift_offset")
        source = "remote_164" if (remote_data and remote_data.get("offset") is not None) else \
                 "local_cache" if (local_cache and local_cache.get("value") is not None) else \
                 "runtime_detected" if current_offset is not None else "unavailable"
        result = {"current_offset_m": current_offset, "offset_source": source,
                  "offset_valid": current_offset is not None, "full_travel_m": FULL_TRAVEL, "timestamp": time.time()}
        if remote_data:
            result["offset_remote"] = {"value": remote_data.get("offset"), "boot_id": remote_data.get("boot_id"),
                                       "detecting": remote_data.get("detecting", False), "uptime_sec": remote_data.get("uptime_sec"),
                                       "age_sec": round(time.time() - self._last_remote_fetch_ts, 1) if self._last_remote_fetch_ts > 0 else None}
        else: result["offset_remote"] = None
        if local_cache: result["offset_local_cache"] = local_cache
        else: result["offset_local_cache"] = None
        return result

    # ==================== Offset 本地检测 ====================
    def _detect_lift_offset(self, current_height):
        if self._offset_detected: return
        elapsed = time.time() - self._offset_start_time
        if current_height > 0.5 or current_height < -1.0: return
        if current_height < self._offset_min_height:
            self._offset_min_height = current_height
            self._offset_stable_count = 0
        if abs(current_height - self._offset_min_height) < OFFSET_EPSILON: self._offset_stable_count += 1
        else: self._offset_stable_count = 0
        if self._offset_stable_count >= OFFSET_STABLE_THRESHOLD: self._save_offset(self._offset_min_height)
        if elapsed > OFFSET_MAX_WAIT: self._save_offset(self._offset_min_height)

    def _save_offset(self, offset_value):
        self._offset_detected = True
        self.state["lift_offset"] = offset_value
        save_lift_offset(offset_value)
        self.get_logger().info(f"立柱 offset 检测完成: {offset_value:.4f}m")

    # ==================== 健康检查 & 历史 & 事件 ====================
    def _check_health(self, url, host=None, port=None):
        try:
            resp = requests.get(url, timeout=2)
            return "OK" if resp.status_code == 200 else f"ERR {resp.status_code}"
        except: pass
        if host and port:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2)
                result = sock.connect_ex((host, port))
                sock.close()
                return "OK" if result == 0 else "DOWN"
            except: return "DOWN"
        return "DOWN"

    def _record_history(self):
        ts = time.time()
        self._history_ts.append(ts)
        for k in HISTORY_KEYS:
            if k == "column_height": self._history[k].append(self.state.get("column_height", -1.0))
            elif k == "physical_height": self._history[k].append(self.state.get("physical_height", -1.0))
            elif k.startswith("odom."): self._history[k].append(self.state["odom"][k.split('.')[1]])
            elif k.startswith("velocity."): self._history[k].append(self.state["velocity"][k.split('.')[1]])

    def get_history(self):
        return {"timestamps": list(self._history_ts), **{k: list(v) for k, v in self._history.items()}}

    def _detect_events(self):
        tp = self.state.get("task_progress", {})
        current_status = tp.get("status") if tp else None
        if self._prev_task_status is not None and current_status != self._prev_task_status:
            item = tp.get("item", "unknown")
            if current_status == "done":
                self._events.append({"type": "task_done", "message": f"任务完成: {item}", "level": "success", "ts": time.time()})
            elif current_status == "running" and self._prev_task_status == "idle":
                self._events.append({"type": "task_start", "message": f"任务开始: {item}", "level": "info", "ts": time.time()})
        self._prev_task_status = current_status
        if tp and "actions" in tp:
            for act in tp["actions"]:
                if act.get("status") == "failed" and act.get("_notified") is not True:
                    act["_notified"] = True
                    self._events.append({"type": "step_failed", "message": f"步骤失败: {act['name']}", "level": "error", "ts": time.time()})

    def get_events(self, since=0): return [e for e in self._events if e["ts"] > since]

    # ==================== 远程命令（代理到 g1d_control_api:28091）====================
    CONTROL_API_URL = "http://127.0.0.1:28091"

    # 命令映射: status_monitor cmd → control_api endpoint
    _CMD_MAP = {
        "rotate":      ("/api/control/rotate",    {"angle": "angle"}),
        "move":        ("/api/control/move",      {"direction": "direction", "distance": "distance"}),
        "lift_to":     ("/api/control/lift_to",   {"height": "height"}),
        "lift_rel":    ("/api/control/lift_rel",  {"direction": "direction", "distance": "distance"}),
        "gripper":     ("/api/control/gripper",   {"action": "action"}),
        "arm":         ("/api/control/arm",       {"phase": "phase", "target": "target", "timeout": "timeout"}),
        "stop":        ("/api/control/stop",      {}),
        "nav":         ("/api/control/nav",       {"pose": "pose", "task_id": "task_id", "timeout": "timeout"}),
        "weitiao":     ("/api/control/weitiao",   {"target": "target", "mode": "mode"}),
        "yolo_pick":   ("/api/control/yolo_pick", {"target": "target"}),
        "task":        ("/api/control/task",      {"steps": "steps", "pose_start": "pose_start", "pose_end": "pose_end", "target": "target"}),
        # 并行合成动作
        "nav_start_lift": ("/api/control/nav_start_lift", {"pose": "pose", "height": "height"}),
        "rotate_lift":    ("/api/control/rotate_lift",    {"angle": "angle", "height": "height"}),
        "move_lift":      ("/api/control/move_lift",      {"direction": "direction", "distance": "distance", "height": "height"}),
        "arm_lift":       ("/api/control/arm_lift",       {"phase": "phase", "target": "target", "height": "height", "timeout": "timeout"}),
    }

    def execute_command(self, cmd_data):
        cmd = cmd_data.get("cmd", "")
        try:
            # 摄像头启动不走控制API（本地进程管理）
            if cmd == "start_camera":
                try:
                    subprocess.Popen(["python3", os.path.join(PROJECT_HOME, "camera_stream.py")],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                     env={**os.environ, "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID","0")})
                    return {"ok": True, "msg": "摄像头启动中"}
                except Exception as e:
                    return {"ok": False, "msg": str(e)}

            # 代理到控制API
            mapping = self._CMD_MAP.get(cmd)
            if mapping is None:
                return {"ok": False, "msg": f"未知命令: {cmd}"}
            endpoint, field_map = mapping
            payload = {}
            for src_key, dst_key in field_map.items():
                val = cmd_data.get(src_key)
                if val is not None:
                    payload[dst_key] = val

            resp = requests.post(f"{self.CONTROL_API_URL}{endpoint}",
                                 json=payload, timeout=10)
            result = resp.json()
            # 记录事件
            if cmd == "stop":
                self._events.append({"type": "emergency", "message": "紧急停止！", "level": "error", "ts": time.time()})
            else:
                task_id = result.get("task_id", "")
                self._events.append({"type": "command_start", "message": f"执行: {cmd} (task:{task_id})", "level": "info", "ts": time.time()})
                # 注册异步任务追踪，后台线程轮询完成后发 command_done/error 事件
                if task_id:
                    with self._pending_lock:
                        self._pending_tasks[task_id] = {"cmd": cmd, "start_time": time.time()}
            return result
        except requests.ConnectionError:
            # 控制API不可达，降级到旧方式
            self.get_logger().warning(f"控制API不可达，使用降级模式执行: {cmd}")
            return self._execute_command_fallback(cmd_data)
        except Exception as e:
            return {"ok": False, "msg": f"命令执行失败: {e}"}

    def _task_tracker_loop(self):
        """后台线程：轮询 control_api 任务状态，完成后发 command_done/error 事件"""
        while rclpy.ok():
            time.sleep(0.3)
            with self._pending_lock:
                task_ids = list(self._pending_tasks.keys())
            for tid in task_ids:
                try:
                    resp = requests.get(f"{self.CONTROL_API_URL}/api/task/{tid}", timeout=3)
                    data = resp.json()
                    status = data.get("status", "unknown")
                    if status in ("completed", "interrupted", "failed", "cancelled"):
                        with self._pending_lock:
                            task_info = self._pending_tasks.pop(tid, {})
                        cmd = task_info.get("cmd", "unknown")
                        result = data.get("result") or {}
                        msg = result.get("msg", status) if isinstance(result, dict) else str(status)
                        if status == "completed":
                            self._events.append({"type": "command_done", "message": f"{cmd} 完成: {msg}", "level": "success", "ts": time.time()})
                        else:
                            self._events.append({"type": "command_error", "message": f"{cmd} 失败: {msg}", "level": "error", "ts": time.time()})
                except Exception:
                    pass
            # 清理超时的 pending task（5分钟未完成视为超时）
            now = time.time()
            with self._pending_lock:
                expired = [tid for tid, info in self._pending_tasks.items()
                           if now - info.get("start_time", 0) > 300]
                for tid in expired:
                    task_info = self._pending_tasks.pop(tid, {})
                    self._events.append({"type": "command_error",
                                         "message": f"{task_info.get('cmd','?')} 超时",
                                         "level": "error", "ts": time.time()})

    def _execute_command_fallback(self, cmd_data):
        """控制API不可达时的降级执行（使用本地 cmd_vel publisher，避免二开 ROS2 节点冲突）"""
        cmd = cmd_data.get("cmd", "")
        try:
            if cmd == "rotate":
                angle = float(cmd_data.get("angle", 0))
                threading.Thread(target=self._fallback_rotate, args=(angle,), daemon=True).start()
                return {"ok": True, "msg": f"旋转已发送(降级)"}
            elif cmd == "move":
                d, dist = cmd_data.get("direction", "forward"), float(cmd_data.get("distance", 0.3))
                threading.Thread(target=self._fallback_move, args=(d, dist), daemon=True).start()
                return {"ok": True, "msg": f"移动已发送(降级)"}
            elif cmd == "lift_to":
                threading.Thread(target=self._run_step_executor_fallback,
                                 args=("lift_to", {"height": float(cmd_data.get("height", 0.0))}), daemon=True).start()
                return {"ok": True, "msg": f"升降已发送(降级)"}
            elif cmd == "lift_rel":
                threading.Thread(target=self._run_step_executor_fallback,
                                 args=("lift_rel", {"direction": cmd_data.get("direction","up"),
                                 "distance": float(cmd_data.get("distance",0.1))}), daemon=True).start()
                return {"ok": True, "msg": f"相对升降已发送(降级)"}
            elif cmd == "gripper":
                resp = requests.post(f"{GRIPPER_URL}/api/v1/command", json={"command": cmd_data.get("action","suck"), "wait": False}, timeout=5)
                return {"ok": resp.status_code == 200, "msg": f"夹爪已发送(降级)"}
            elif cmd == "arm":
                resp = requests.post(ARM_API_URL, json={"type": "arm_task", "phase": cmd_data.get("phase","RESET"), "target_object": "", "timeout_sec": 30}, timeout=35)
                return {"ok": resp.status_code == 200, "msg": f"机械臂已发送(降级)"}
            elif cmd == "stop":
                self._publish_fallback_cmd_vel(0.0, 0.0)
                self._events.append({"type": "emergency", "message": "紧急停止！", "level": "error", "ts": time.time()})
                return {"ok": True, "msg": "紧急停止已发送(降级)"}
            else:
                return {"ok": False, "msg": f"未知命令: {cmd}"}
        except Exception as e:
            return {"ok": False, "msg": f"降级执行失败: {e}"}

    def _publish_fallback_cmd_vel(self, linear_x, angular_z):
        """通过本地 publisher 发送 cmd_vel（降级模式）"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self._fallback_cmd_pub.publish(twist)

    def _fallback_rotate(self, angle_deg):
        """降级旋转：开环定时发送角速度"""
        import math
        speed = 0.5
        rad = math.radians(float(angle_deg))
        dur = abs(rad) / speed
        angular_z = speed if rad > 0 else -speed
        end_time = time.time() + dur
        self.get_logger().warning(f"降级旋转: {angle_deg:.1f}° 持续 {dur:.1f}s")
        while time.time() < end_time:
            self._publish_fallback_cmd_vel(0.0, angular_z)
            time.sleep(0.05)
        self._publish_fallback_cmd_vel(0.0, 0.0)

    def _fallback_move(self, direction, distance):
        """降级移动：开环定时发送线速度"""
        if direction not in ("forward", "backward"):
            return
        speed = 0.2
        linear_x = speed if direction == "forward" else -speed
        dur = abs(float(distance)) / speed
        end_time = time.time() + dur
        self.get_logger().warning(f"降级移动: {direction} {distance}m 持续 {dur:.1f}s")
        while time.time() < end_time:
            self._publish_fallback_cmd_vel(linear_x, 0.0)
            time.sleep(0.05)
        self._publish_fallback_cmd_vel(0.0, 0.0)

    def _run_step_executor_fallback(self, step, kwargs):
        """降级模式：直接spawn step_executor子进程"""
        import subprocess as sp
        cmd = [sys.executable, os.path.join(PROJECT_HOME, "step_executor.py"), step]
        for k, v in kwargs.items(): cmd.extend([f"--{k}", str(v)])
        env = os.environ.copy()
        setup_bash = "/opt/ros/humble/setup.bash"
        if os.path.exists(setup_bash):
            try:
                out = sp.run(f"source {setup_bash} && env", shell=True, capture_output=True, text=True, timeout=10, executable="/bin/bash")
                if out.returncode == 0:
                    for line in out.stdout.splitlines():
                        if '=' in line:
                            k2, _, v2 = line.partition('=')
                            if k2.startswith(('ROS','AMENT','PYTHONPATH','LD_LIBRARY','PATH','COLCON')): env[k2] = v2
            except: pass
        self._events.append({"type": "command_start", "message": f"执行(降级): {step}", "level": "info", "ts": time.time()})
        try:
            result = sp.run(cmd, capture_output=True, text=True, timeout=120, env=env)
            self.state["last_command_result"] = {"step": step, "returncode": result.returncode, "ts": time.time()}
            level = "success" if result.returncode == 0 else "error"
            self._events.append({"type": "command_done" if result.returncode==0 else "command_error", "message": f"{step} {'完成' if result.returncode==0 else '失败'}", "level": level, "ts": time.time()})
        except Exception as e:
            self._events.append({"type": "command_error", "message": f"{step} 异常: {e}", "level": "error", "ts": time.time()})

    # ==================== 日志轮转（原子写入，防数据丢失）====================
    def _rotate_logs(self):
        try:
            if STATUS_LOG_FILE.exists() and STATUS_LOG_FILE.stat().st_size > MAX_LOG_SIZE:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                archive = LOG_DIR / f"status_log_{ts}.gz"
                tmp_archive = LOG_DIR / f".status_log_{ts}.gz.tmp"
                # 先写入临时文件，再 rename（防崩溃丢数据）
                with open(STATUS_LOG_FILE, 'rb') as f_in:
                    with gzip.open(tmp_archive, 'wb') as f_out:
                        shutil.copyfileobj(f_in, f_out)
                os.replace(tmp_archive, archive)
                # 清空现有日志文件
                with open(STATUS_LOG_FILE, 'w') as f:
                    f.write("")
                # 清理过期归档
                cutoff = datetime.now() - timedelta(days=MAX_LOG_DAYS)
                for old in sorted(glob.glob(str(LOG_DIR / "status_log_*.gz"))):
                    try:
                        if datetime.strptime(old.split("status_log_")[1].split(".")[0], "%Y%m%d_%H%M%S") < cutoff:
                            os.remove(old)
                    except Exception:
                        pass
        except Exception as e:
            self.get_logger().error(f"日志轮转失败: {e}")

    # ==================== 后台HTTP轮询 ====================
    def _background_poll(self):
        def _poll_health():
            self.state["yolo"]["yolo_health"] = self._check_health(YOLO_HEALTH_URL)
            self.state["yolo"]["adjust_health"] = self._check_health(ADJUST_HEALTH_URL)
            self.state["yolo"]["arm_api_health"] = self._check_health(ARM_HEALTH_URL)
        def _poll_yolo():
            try:
                resp = requests.get(YOLO_XYZ_URL, timeout=2)
                if resp.status_code == 200:
                    data = resp.json()
                    with self.lock:
                        self.state["yolo"].update({
                            "range_mm": data.get("range_from_left_camera_mm", -1), "selected_label": data.get("selected_yolo_label",""),
                            "confidence": data.get("selected_yolo_confidence",0.0), "orientation": data.get("selected_orientation","unknown")
                        })
                        hint = data.get("robot_alignment",{}).get("control_hint",{})
                        self.state["yolo"].update({"forward_distance_m": hint.get("forward_distance_m",-1), "height_down_m": hint.get("height_down_m",0.0)})
            except: self.state["yolo"]["range_mm"] = -1
        def _poll_gripper():
            try:
                resp = requests.get(f"{GRIPPER_URL}/api/v1/status", timeout=2)
                if resp.status_code == 200:
                    inner = resp.json().get("data",{})
                    self.state["gripper"]["state"] = inner.get("object_state","") or ("吸合" if inner.get("is_sucked") else "unknown")
            except: self.state["gripper"]["state"] = "DOWN"
        threads = [threading.Thread(target=f, daemon=True) for f in [_poll_health, _poll_yolo, _poll_gripper]]
        for t in threads: t.start()
        for t in threads: t.join(timeout=5)

    # ==================== Shell读取(完整保留V8.0逻辑) ====================
    def _shell_read_height(self):
        ros_env = dict(os.environ)
        ros_bin = "/opt/ros/humble/bin"
        if ros_bin not in ros_env.get("PATH", ""): ros_env["PATH"] = ros_bin + ":" + ros_env.get("PATH", "/usr/bin:/bin")
        for msg_type in ['geometry_msgs/msg/Vector3', 'geometry_msgs/msg/Point32']:
            try:
                cmd = f"timeout 5 ros2 topic echo /hispeed_state {msg_type} --once 2>&1 | grep -m1 'y:' | awk '{{print $2}}'"
                res = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=8, executable="/bin/bash", env=ros_env)
                if res.stdout and res.stdout.strip():
                    self._update_hispeed(float(res.stdout.strip()), f"Shell_{msg_type.split('/')[-1]}")
                    return
            except: pass
        try:
            res2 = subprocess.run(f"{SDK_HEIGHT_BIN} {ROBOT_IF}", shell=True, capture_output=True, text=True, timeout=3)
            if res2.returncode == 0 and res2.stdout.strip():
                self._update_hispeed(float(res2.stdout.strip()), "SDK_Binary")
                return
        except: pass

    # ==================== 【核心优化】定时回调 & 后台工作 ====================
    def timer_callback(self):
        self._rotate_logs()
        if self.state["_last_odom_time"] > 0 and time.time() - self.state["_last_odom_time"] > 5.0:
            self.get_logger().warn("odom 数据超过 5 秒未更新")

        # 【优化】10s无数据即重建订阅
        if not self._hispeed_ever_received and time.time() - self._start_time > RESUBSCRIBE_TRIGGER_TIME:
            if not hasattr(self, '_hispeed_resub_count'): self._hispeed_resub_count = 0
            if self._hispeed_resub_count < 3:
                self._hispeed_resub_count += 1
                self.get_logger().warn(f"hispeed {RESUBSCRIBE_TRIGGER_TIME}s 无数据，重建订阅 (第{self._hispeed_resub_count}次)")
                try:
                    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
                    if HispeedMsg is not None: self.create_subscription(HispeedMsg, '/hispeed_state', self.hispeed_cb, qos)
                    self.create_subscription(Point32, '/hispeed_state', self.hispeed_cb_point32, qos)
                except Exception as e: self.get_logger().warn(f"重建订阅失败: {e}")

        if not hasattr(self, '_bg_thread') or not self._bg_thread.is_alive():
            self._bg_thread = threading.Thread(target=self._background_work, daemon=True)
            self._bg_thread.start()

    def _background_work(self):
        uptime = time.time() - self._start_time

        # 【优化】保护期缩短至3s，期间并行获取远程offset
        if uptime < DDS_DISCOVERY_GRACE_PERIOD:
            self._fetch_offset_from_robot()
            time.sleep(0.3)
            self._do_non_shell_background_tasks()
            return

        # 【优化】Shell读取彻底异步化，绝不阻塞主循环
        if not self._hispeed_ever_received or time.time() - self._hispeed_last_update > 5.0:
            if not hasattr(self, '_shell_thread') or not self._shell_thread.is_alive():
                self._shell_thread = threading.Thread(target=self._shell_read_height, daemon=True)
                self._shell_thread.start()

        current_height = self.state["column_height"]
        self._fetch_offset_from_robot()

        if self.state.get("lift_offset") is None:
            if -0.5 < current_height < 1.0: self._detect_lift_offset(current_height)
            if not self._offset_detected and os.path.exists(LIFT_OFFSET_FILE):
                try:
                    offset = read_lift_offset()
                    self.state["lift_offset"] = offset
                    self._offset_detected = True
                except: pass

        offset = self.state.get("lift_offset")
        if offset is not None and current_height > -1.0:
            self.state["physical_height"] = get_physical_height(current_height, offset)

        self._do_non_shell_background_tasks()

        # 启动完成标记
        if not self._startup_log_done and self._hispeed_ever_received:
            self._startup_log_done = True
            self.get_logger().info(f"[STARTUP] t={uptime:.1f}s data_ready=true, height={self.state['column_height']:.4f}")

    def _do_non_shell_background_tasks(self):
        self.state["arm_process_running"] = check_arm_process()
        self.state["camera_stream_running"] = os.path.exists("/tmp/camera_left.jpg") or os.path.exists("/tmp/camera_right.jpg")
        self._background_poll()
        with self.lock:
            if time.time() - self.state["_last_ros_arm_time"] > ARM_STALE_TIMEOUT:
                try:
                    if os.path.exists(ARM_BACKUP_FILE):
                        with open(ARM_BACKUP_FILE, "r") as f: last_arm = json.load(f)
                        self.state["arm_status"].update({k: last_arm.get(k,"") for k in ["phase","status_text","exec_status"]})
                except: pass
        try:
            with open(TASK_PROGRESS_FILE, "r") as f: self.state["task_progress"] = json.load(f)
        except: self.state["task_progress"] = {}
        if not hasattr(self, '_point_refresh_ts') or time.time() - self._point_refresh_ts > 10:
            self._point_refresh_ts = time.time()
            try:
                points = []
                with open(POINT_INF_FILE, "r") as f:
                    for line in f:
                        parts = line.strip().split()
                        if len(parts) >= 9 and not line.startswith('#'):
                            points.append({"name": parts[0], "x": float(parts[1]), "y": float(parts[2]), "z": float(parts[3]), "cigarette": parts[8]})
                self.state["point_inf"] = points
            except: self.state["point_inf"] = []
        if not hasattr(self, '_sdk_refresh_ts') or time.time() - self._sdk_refresh_ts > 30:
            self._sdk_refresh_ts = time.time()
            try:
                params = {}
                with open(SDK_PARAMS_FILE, "r") as f:
                    for line in f:
                        if '=' in line and not line.startswith('#'):
                            k, v = line.split('=', 1)
                            params[k.strip()] = v.strip()
                self.state["sdk_params"] = params
            except: self.state["sdk_params"] = {}
        self._record_history()
        self._detect_events()
        self.state["timestamp"] = datetime.now().isoformat()
        with self.lock: state_copy = json.dumps(self.state, ensure_ascii=False)
        with open(STATUS_LOG_FILE, 'a') as f: f.write(state_copy + "\n")

    def get_full_state(self):
        with self.lock: return dict(self.state)

    # ==================== HTTP 服务 ====================
    def start_http_server(self):
        monitor = self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                parsed = urlparse(self.path)
                path, params = parsed.path, parse_qs(parsed.query)
                if path == '/api/status': self._json_response(monitor.get_full_state())
                elif path == '/api/history': self._json_response(monitor.get_history())
                elif path == '/api/logs': self._json_response(monitor._read_recent_logs(int(params.get('count',[50])[0])))
                elif path == '/api/lift_height':
                    st = monitor.state
                    offset = st.get("lift_offset")
                    backup = monitor._build_offset_backup_data()
                    self._json_response({
                        "lift_offset_m": offset, "hispeed_y_m": st.get("column_height",-1.0),
                        "physical_height_m": st.get("physical_height",-1.0), "full_travel_m": FULL_TRAVEL,
                        "sdk_min_m": offset, "sdk_max_m": round(offset+FULL_TRAVEL,4) if offset else None,
                        "physical_min_m": 0.0, "physical_max_m": FULL_TRAVEL, "unit": "meter",
                        "my_uptime_sec": round(time.time()-monitor._start_time,1),
                        "robot_uptime_sec": monitor._last_robot_uptime,
                        "offset_valid": offset is not None, "offset_source": backup["offset_source"],
                        "offset_remote": backup["offset_remote"], "offset_local_cache": backup["offset_local_cache"],
                        "data_age_sec": round(time.time()-monitor._hispeed_last_update,3) if monitor._hispeed_ever_received else None,
                        "timestamp": time.time()
                    })
                elif path == '/api/offset_backup': self._json_response(monitor._build_offset_backup_data())
                elif path == '/api/ready':
                    ready = monitor._hispeed_ever_received and monitor._offset_detected
                    code = 200 if ready else 503
                    self._json_response({"ready": ready, "uptime_sec": round(time.time()-monitor._start_time,1)}, code)
                elif path == '/api/events': self._json_response(monitor.get_events(float(params.get('since',[0])[0])))
                elif path.startswith('/api/task/'):
                    # 代理查询 control_api 任务状态
                    tid = path.replace('/api/task/', '').split('/')[0]
                    try:
                        resp = requests.get(f"{monitor.CONTROL_API_URL}/api/task/{tid}", timeout=3)
                        self._json_response(resp.json())
                    except Exception as e:
                        self._json_response({"status": "unknown", "error": str(e)})
                elif path == '/api/yolo_detect':
                    try:
                        resp = requests.post("http://192.168.123.164:18081/xyz", timeout=15)
                        self._json_response({'ok': resp.json().get('ok',False) if resp.status_code==200 else False})
                    except Exception as e: self._json_response({'ok': False, 'error': str(e)})
                elif path.startswith('/api/yolo_img/'):
                    img_name = path.replace('/api/yolo_img/', '')
                    allowed = ['left_input.jpg','left_points.jpg','left_projected.jpg','left_projected_zoom.jpg','left_candidates.jpg','right_input.jpg','right_points.jpg','right_candidates.jpg']
                    if img_name not in allowed: self.send_response(403); self.end_headers(); return
                    try:
                        resp = monitor._img_session.get(f"http://192.168.123.164:18081/latest/{img_name}", timeout=(1,2))
                        if resp.status_code==200 and len(resp.content)>100:
                            self.send_response(200); self.send_header('Content-Type', resp.headers.get('Content-Type','image/jpeg'))
                            self.send_header('Cache-Control','no-cache,no-store'); self.end_headers(); self.wfile.write(resp.content)
                        else: self.send_response(404); self.end_headers()
                    except: self.send_response(502); self.end_headers()
                else:
                    self.send_response(200); self.send_header('Content-Type','text/html; charset=utf-8')
                    self.end_headers(); self.wfile.write(DASHBOARD_HTML.encode('utf-8'))
            def do_OPTIONS(self):
                self.send_response(204)
                for h,v in [('Access-Control-Allow-Origin','*'),('Access-Control-Allow-Methods','GET,POST,OPTIONS'),('Access-Control-Allow-Headers','Content-Type'),('Access-Control-Max-Age','86400')]:
                    self.send_header(h,v)
                self.end_headers()
            def do_POST(self):
                if urlparse(self.path).path == '/api/command':
                    try: self._json_response(monitor.execute_command(json.loads(self.rfile.read(int(self.headers.get('Content-Length',0))))))
                    except: self._json_response({"ok":False,"msg":"无效JSON"},400)
                else: self.send_response(404); self.end_headers()
            def _json_response(self, data, code=200):
                self.send_response(code); self.send_header('Content-Type','application/json')
                self.send_header('Access-Control-Allow-Origin','*'); self.end_headers()
                self.wfile.write(json.dumps(data,ensure_ascii=False).encode())
            def log_message(self, format, *args): pass

        class ReuseHTTPServer(HTTPServer): allow_reuse_address = True
        server = ReuseHTTPServer(('0.0.0.0', HTTP_PORT), Handler)
        self.get_logger().info(f"仪表盘: http://<IP>:{HTTP_PORT}")
        threading.Thread(target=server.serve_forever, daemon=True).start()

    def _read_recent_logs(self, count=50, level_filter=None):
        try:
            if not STATUS_LOG_FILE.exists(): return []
            lines = []
            with open(STATUS_LOG_FILE, 'r') as f:
                for line in f: lines.append(line.strip())
            result = []
            for line in lines[-count:]:
                if not line: continue
                try:
                    e = json.loads(line)
                    result.append({"ts":e.get("timestamp",""),"height":e.get("column_height",-1),"physical_height":e.get("physical_height",-1),
                                   "arm_phase":e.get("arm_status",{}).get("phase",""),"x":e.get("odom",{}).get("x",0),"y":e.get("odom",{}).get("y",0)})
                except: continue
            return result
        except: return []


def main():
    rclpy.init()
    node = StatusMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()