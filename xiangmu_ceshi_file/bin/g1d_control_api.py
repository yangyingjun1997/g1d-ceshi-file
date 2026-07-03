#!/usr/bin/env python3
"""
G1D 统一控制 API V1.0
- 部署在 192.168.123.5
- HTTP 端口 28091
- 常驻 ROS2 Node，复用 cmd_vel publisher / odom subscriber / hispeed subscriber
- 所有动作在后台线程执行，立即返回 task_id
- 提供 task 状态查询和取消

API 列表:
  POST /api/control/rotate     {"angle": 200}
  POST /api/control/move       {"direction": "forward", "distance": 0.5}
  POST /api/control/lift_to    {"height": 0.3}
  POST /api/control/lift_rel   {"direction": "up", "distance": 0.1}
  POST /api/control/gripper    {"action": "suck"}       (suck/release/status)
  POST /api/control/arm        {"phase": "PICK", "target": "XiongMao", "timeout": 130}
  POST /api/control/nav        {"pose": {...}, "task_id": "xxx", "timeout": 180}
  POST /api/control/weitiao    {"target": "XiongMao", "mode": "enhanced"}
  POST /api/control/yolo_pick  {"target": "XiongMao"}
  POST /api/control/verify_distance  {}
  POST /api/control/stop       {}  (紧急停止)
  POST /api/control/task       {"steps": [...], "pose_start": {...}, "pose_end": {...}, "target": "XiongMao"}
  GET  /api/task/{task_id}     查询任务状态
  POST /api/task/{task_id}/cancel  取消任务
  GET  /api/status             控制器状态（offset/height/odom等）
  GET  /health                 健康检查
"""
import json, math, os, signal, sys, subprocess, threading, time, uuid, tempfile
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

# ---- 自动安装依赖 ----
try:
    import requests
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "requests", "-q"])
    import requests

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lib"))
from g1d_common import (
    log, HispeedMsg, DRY_RUN, dry_run_skip,
    LIFT_OFFSET_FILE, FULL_TRAVEL, SSH_USER, SSH_HOST, ROBOT_IF,
    SDK_HEIGHT_BIN, SDK_SIMPLE_BIN,
    ARM_API_URL, YOLO_XYZ_URL, ADJUST_URL, GRIPPER_URL, NAV_API_URL,
    LAST_ARM_STATUS_FILE, TASK_PROGRESS_FILE,
    read_lift_offset, save_lift_offset, get_physical_height,
    yaw_from_odom, position_from_odom, check_arm_process,
    load_sdk_params, http_get, http_post, run_remote_ssh,
    atomic_write_json, notify,
)
from robot_nav_arm_flow import NavigationClient

HTTP_PORT = 28091

# 线程局部存储：每个任务线程记录自己的 control_gen 代际
_task_context = threading.local()
# 全局启动时间
_start_time = time.time()


class ControlNode(Node):
    """常驻 ROS2 Node，持有订阅和发布者"""

    def __init__(self):
        super().__init__('g1d_control_api')
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._odom_data = None
        self._odom_lock = threading.Lock()
        self._column_height = None
        self._hispeed_lock = threading.Lock()
        # spin 锁：确保 rclpy.spin_once 不会在多个线程中并发调用
        self._spin_lock = threading.Lock()
        # 控制代际计数器：每次新指令递增，旧任务的 while 循环检测到代际不匹配自动退出
        self._control_gen = 0
        # 活跃代际集合：解决新指令覆盖旧任务线程局部变量的竞态问题
        self._active_gens = set()
        self._gens_lock = threading.Lock()

        self.create_subscription(Odometry, '/agv/odom', self._odom_cb, 10)
        hispeed_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )
        # 双类型订阅 hispeed（Vector3 和 Point32，发布者可能是任意一种）
        if HispeedMsg is not None:
            self.create_subscription(HispeedMsg, '/hispeed_state', self._hispeed_cb, hispeed_qos)
            self.get_logger().info("已订阅 /hispeed_state (Vector3)")
        try:
            from geometry_msgs.msg import Point32
            self.create_subscription(Point32, '/hispeed_state', self._hispeed_cb, hispeed_qos)
            self.get_logger().info("已订阅 /hispeed_state (Point32)")
        except Exception:
            self.get_logger().warn("无法订阅 Point32 类型 hispeed")

        # 等待 odom 就绪
        self._odom_ready = self._wait_for_odom()
        # 读取 offset
        self._lift_offset = read_lift_offset()
        # 加载参数
        self._params = load_sdk_params()
        for k, v in self._params.items():
            setattr(self, k, v)
        # 校验关键参数
        required_keys = ["lift_timeout", "step_timeout", "rotate_speed", "move_speed",
                         "cam_to_base_t", "full_travel"]
        missing = [k for k in required_keys if not hasattr(self, k) or getattr(self, k) is None]
        if missing:
            self.get_logger().warning(f"缺少配置项: {missing}，将使用默认值")

        self._nav_client = NavigationClient(NAV_API_URL)
        self.get_logger().info("ControlNode 初始化完成")

    def _odom_cb(self, msg):
        with self._odom_lock:
            self._odom_data = msg

    def _hispeed_cb(self, msg):
        with self._hispeed_lock:
            self._column_height = msg.y

    def _wait_for_odom(self, timeout=5.0):
        start = time.time()
        while self._odom_data is None and time.time() - start < timeout:
            self.safe_spin_once(0.1)
        return self._odom_data is not None

    def get_yaw(self):
        with self._odom_lock:
            return yaw_from_odom(self._odom_data)

    def get_position(self):
        with self._odom_lock:
            return position_from_odom(self._odom_data)

    def get_column_height(self):
        with self._hispeed_lock:
            return self._column_height

    def safe_spin_once(self, timeout=0.05):
        """线程安全的 spin_once，用锁防止并发调用"""
        with self._spin_lock:
            rclpy.spin_once(self, timeout_sec=timeout)

    def _check_stop(self):
        """检查当前线程的控制代是否已过期（被新指令取代）
        双重检查：1) 全局 control_gen 递增（快速检查）
                   2) active_gens 成员检查（防御竞态条件）"""
        my_gen = getattr(_task_context, 'control_gen', -1)
        if my_gen < 1:
            return  # 未初始化，放行
        if my_gen != self._control_gen:
            with self._gens_lock:
                if my_gen not in self._active_gens:
                    self.publish_cmd_vel(0.0, 0.0)
                    raise InterruptedError("控制被新指令中断")

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """紧急停止"""
        self.publish_cmd_vel(0.0, 0.0)
        self.get_logger().info("紧急停止已发送")

    # ==================== 旋转（odom 闭环）====================

    def rotate(self, angle_deg):
        """闭环旋转，正角度为逆时针"""
        if dry_run_skip(f"旋转 {angle_deg:.1f}°"):
            return {"ok": True, "msg": f"[DRY-RUN] 旋转 {angle_deg:.1f}°"}

        if self.rotate_segments > 1:
            seg_angle = float(angle_deg) / self.rotate_segments
            for i in range(self.rotate_segments):
                self._rotate_single(seg_angle)
                if i < self.rotate_segments - 1:
                    time.sleep(0.3)
        else:
            self._rotate_single(float(angle_deg))
        return {"ok": True, "msg": f"旋转 {angle_deg:.1f}° 完成"}

    def _rotate_single(self, angle_deg):
        target_rad = math.radians(angle_deg)
        if not self._odom_ready:
            self._rotate_openloop(angle_deg)
            return

        speed = 0.5
        angular_z = speed if angle_deg > 0 else -speed
        initial_yaw = self.get_yaw()
        if initial_yaw is None:
            self._rotate_openloop(angle_deg)
            return

        prev_yaw = initial_yaw
        cumulative = 0.0
        while abs(cumulative) < abs(target_rad):
            self._check_stop()
            # spin 线程持续更新 odom，这里只需要读缓存
            time.sleep(0.02)
            current_yaw = self.get_yaw()
            if current_yaw is None:
                continue
            delta = current_yaw - prev_yaw
            if delta > math.pi: delta -= 2 * math.pi
            elif delta < -math.pi: delta += 2 * math.pi
            cumulative += delta
            prev_yaw = current_yaw
            remaining = abs(target_rad) - abs(cumulative)
            if remaining < math.radians(5):
                angular_z = (0.5 if angle_deg > 0 else -0.5) * 0.3
            elif remaining < math.radians(15):
                angular_z = (0.5 if angle_deg > 0 else -0.5) * 0.6
            self.publish_cmd_vel(0.0, angular_z)
        self.publish_cmd_vel(0.0, 0.0)
        self.get_logger().info(f"旋转完成，实际 {math.degrees(cumulative):.1f}°")

    def _rotate_openloop(self, angle_deg):
        speed = 0.5
        rad = math.radians(float(angle_deg))
        dur = abs(rad) / speed
        angular_z = speed if rad > 0 else -speed
        self.get_logger().info(f"开环旋转: {angle_deg:.1f}°")
        self._cmd_vel_timed(0.0, angular_z, dur)

    # ==================== 移动（odom 闭环）====================

    def move(self, direction, distance):
        """方向 forward/backward，距离(米)"""
        if direction not in ("forward", "backward"):
            return {"ok": False, "msg": "方向仅支持 forward/backward"}
        linear_x = 0.2 if direction == "forward" else -0.2
        if dry_run_skip(f"移动 {direction} {distance:.3f}m"):
            return {"ok": True, "msg": f"[DRY-RUN] 移动 {direction} {distance:.3f}m"}
        self._cmd_vel_move(linear_x, distance)
        return {"ok": True, "msg": f"移动 {direction} {distance:.3f}m 完成"}

    def _cmd_vel_move(self, linear_x, distance):
        speed = abs(linear_x)
        if speed <= 0 or abs(distance) < 0.05:
            return
        if self._odom_ready:
            self._cmd_vel_move_odom(linear_x, distance)
        else:
            dur = abs(distance) / speed
            self._cmd_vel_timed(linear_x, 0.0, dur)

    def _cmd_vel_move_odom(self, linear_x, distance):
        ix, iy = self.get_position()
        iyaw = self.get_yaw()
        if ix is None or iyaw is None:
            self._cmd_vel_timed(linear_x, 0.0, abs(distance) / abs(linear_x))
            return
        dir_x = math.cos(iyaw)
        dir_y = math.sin(iyaw)
        while True:
            self._check_stop()
            # spin 线程持续更新 odom，这里只需要读缓存
            time.sleep(0.02)
            cx, cy = self.get_position()
            if cx is None: continue
            dx, dy = cx - ix, cy - iy
            dist_moved = dx * dir_x + dy * dir_y
            if abs(dist_moved) >= abs(distance): break
            remaining = abs(distance) - abs(dist_moved)
            cur_speed = linear_x
            if remaining < 0.05: cur_speed = linear_x * 0.3
            elif remaining < 0.15: cur_speed = linear_x * 0.6
            self.publish_cmd_vel(cur_speed, 0.0)
        self.publish_cmd_vel(0.0, 0.0)

    def _cmd_vel_timed(self, linear_x, angular_z, duration):
        """开环定时发送 cmd_vel"""
        end_time = time.time() + duration
        while time.time() < end_time:
            self._check_stop()
            self.publish_cmd_vel(linear_x, angular_z)
            time.sleep(0.05)
        self.publish_cmd_vel(0.0, 0.0)

    def backup(self, distance):
        """后退"""
        if distance <= 0: return {"ok": True, "msg": "距离为0，忽略"}
        self._cmd_vel_move(-0.2, distance)
        return {"ok": True, "msg": f"后退 {distance:.3f}m 完成"}

    # ==================== 升降 ====================

    def lift_to(self, height_m):
        """升降至目标物理高度"""
        self._check_stop()
        if height_m < 0 or height_m > self.full_travel:
            return {"ok": False, "msg": f"高度 {height_m:.3f}m 超出 [0, {self.full_travel}]m"}
        if dry_run_skip(f"升降到 {height_m:.3f}m"):
            return {"ok": True, "msg": f"[DRY-RUN] 升降到 {height_m:.3f}m"}
        sdk_target = height_m + self._lift_offset
        cmd = (f"ssh -n -o ConnectTimeout=10 -o ServerAliveInterval=5 "
               f"-o ServerAliveCountMax=2 {SSH_USER}@{SSH_HOST} "
               f"'{SDK_HEIGHT_BIN} {ROBOT_IF} {sdk_target:.4f}' </dev/null")
        try:
            res = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                                 timeout=self.lift_timeout)
            if res.returncode != 0:
                self.get_logger().warning(f"升降返回非零: {res.stderr.strip()}")
        except subprocess.TimeoutExpired:
            self.get_logger().warning("升降命令超时")
        self._check_stop()
        self._verify_lift_height(height_m)
        return {"ok": True, "msg": f"升降到 {height_m:.3f}m 完成 (SDK: {sdk_target:.4f}m)"}

    def lift_rel(self, direction, distance_m):
        """相对升降 up/down"""
        self._check_stop()
        if direction not in ("up", "down"):
            return {"ok": False, "msg": "方向仅支持 up/down"}
        speed = 0.1
        duration = distance_m / speed
        if dry_run_skip(f"相对升降: {direction} {distance_m:.3f}m"):
            return {"ok": True, "msg": f"[DRY-RUN] 相对升降 {direction} {distance_m:.3f}m"}
        cmd = (f"ssh -n -o ConnectTimeout=10 -o ServerAliveInterval=5 "
               f"-o ServerAliveCountMax=2 {SSH_USER}@{SSH_HOST} "
               f"'{SDK_SIMPLE_BIN} {ROBOT_IF} {direction} {speed} {duration}' </dev/null")
        try:
            res = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                                 timeout=duration + 10)
            if res.returncode != 0:
                self.get_logger().warning(f"相对升降返回非零: {res.stderr.strip()}")
        except subprocess.TimeoutExpired:
            self.get_logger().warning("相对升降命令超时")
        return {"ok": True, "msg": f"相对升降 {direction} {distance_m:.3f}m 完成"}

    def _verify_lift_height(self, target_m, tolerance=0.01):
        # 等待 spin 线程更新 hispeed 数据
        time.sleep(1.5)
        h = self.get_column_height()
        if h is None: return
        physical = get_physical_height(h, self._lift_offset)
        error = abs(physical - target_m)
        if error > tolerance:
            self.get_logger().warning(f"高度偏差: 期望 {target_m:.3f}m, 实际 {physical:.3f}m")
        else:
            self.get_logger().info(f"高度验证通过: {physical:.3f}m")

    # ==================== 导航 ====================

    def nav(self, pose, task_id=None, timeout=180):
        self._check_stop()
        if task_id is None:
            task_id = f"nav-{int(time.time())}"
        self._nav_client.submit_navigation(pose, task_id)
        self._nav_client.wait_navigation(task_id, timeout, 1.0)
        return {"ok": True, "msg": f"导航完成 {task_id}"}

    # ==================== 夹爪 ====================

    def gripper(self, action):
        if action == "status":
            try:
                resp = http_get(f"{GRIPPER_URL}/api/v1/status", timeout=5,
                                max_retries=self.http_max_retries,
                                retry_delay=self.http_retry_delay)
                return {"ok": True, "msg": resp.text}
            except Exception as e:
                return {"ok": False, "msg": f"夹爪状态查询失败: {e}"}
        elif action in ("suck", "release"):
            try:
                resp = http_post(f"{GRIPPER_URL}/api/v1/command",
                                 {"command": action, "wait": False}, timeout=5,
                                 max_retries=self.http_max_retries,
                                 retry_delay=self.http_retry_delay)
                return {"ok": resp.status_code == 200, "msg": f"夹爪 {action} 已发送"}
            except Exception as e:
                return {"ok": False, "msg": f"夹爪控制失败: {e}"}
        return {"ok": False, "msg": f"未知夹爪动作: {action}"}

    # ==================== 微调 ====================

    def weitiao(self, target, mode=None):
        if mode is None:
            mode = self.weitiao_mode
        if mode == "classic":
            return self._weitiao_classic(target)
        return self._weitiao_enhanced(target)

    def _weitiao_classic(self, target):
        try:
            resp = http_get(ADJUST_URL, params={"label": target}, timeout=30,
                            max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
            if resp.status_code != 200:
                return {"ok": False, "msg": f"微调请求失败: HTTP {resp.status_code}"}
            data = resp.json()
            hint = data.get("robot_alignment", {}).get("control_hint", {})
            height_down_mm = hint.get("height_down_m", 0.0)
            if abs(height_down_mm) > self.height_threshold_mm:
                direction = "down" if height_down_mm > 0 else "up"
                distance_m = abs(height_down_mm) / 1000.0
                self.lift_rel(direction, distance_m)
            return {"ok": True, "msg": "经典微调完成"}
        except Exception as e:
            return {"ok": False, "msg": f"经典微调异常: {e}"}

    def _weitiao_enhanced(self, target):
        loop_count = 0
        for search_attempt in range(self.max_search + 1):
            self._check_stop()
            loop_count += 1
            if loop_count > self.enhanced_max_loops:
                return {"ok": False, "msg": "增强微调达到最大循环次数"}
            try:
                resp = http_get(ADJUST_URL, params={"label": target}, timeout=30,
                                max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
            except Exception as e:
                return {"ok": False, "msg": f"微调请求异常: {e}"}
            if resp.status_code != 200:
                return {"ok": False, "msg": "微调请求失败"}

            data = resp.json()
            align = data.get("robot_alignment", {})
            near_edge = align.get("near_edge_robot_alignment", {}).get("target", {}).get("ground_forward_mm", None)
            if near_edge is None:
                near_edge = align.get("robot_alignment", {}).get("target", {}).get("ground_forward_mm", 0)
            hint = align.get("control_hint", {})
            lateral = hint.get("lateral_error_m", 0.0)
            confidence = data.get("selected_yolo_confidence", 0.0)

            if near_edge <= 0 or confidence <= 0:
                if search_attempt < self.max_search:
                    self.lift_rel("up", self.search_step_height)
                    self.backup(self.search_backup)
                    continue
                else:
                    return {"ok": False, "msg": "未检测到目标"}

            if self.near_edge_min <= near_edge <= self.near_edge_max:
                height_down_mm = hint.get("height_down_m", 0.0)
                if abs(height_down_mm) > self.height_threshold_mm:
                    direction = "down" if height_down_mm > 0 else "up"
                    distance_m = abs(height_down_mm) / 1000.0
                    self.lift_rel(direction, distance_m)
                return {"ok": True, "msg": f"增强微调完成, near_edge={near_edge:.0f}mm"}

            deviation = near_edge - (self.near_edge_min + self.near_edge_max) / 2
            if abs(deviation) < self.small_deviation_mm:
                distance_m = -deviation / 1000.0
                linear_x = 0.1 if distance_m > 0 else -0.1
                self._cmd_vel_move(linear_x, abs(distance_m))
                continue

            for retry in range(self.max_adjust_retries + 1):
                loop_count += 1
                if loop_count > self.enhanced_max_loops:
                    return {"ok": False, "msg": "增强微调达到最大循环次数"}
                try:
                    resp = http_get(ADJUST_URL, params={"label": target}, timeout=30,
                                    max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
                except Exception:
                    continue
                if resp.status_code == 200:
                    data = resp.json()
                    align = data.get("robot_alignment", {})
                    near_edge = align.get("near_edge_robot_alignment", {}).get("target", {}).get("ground_forward_mm", 0)
                    if self.near_edge_min <= near_edge <= self.near_edge_max:
                        return {"ok": True, "msg": f"增强微调完成, near_edge={near_edge:.0f}mm"}
                break  # 跳出重试循环，回到搜索循环
        return {"ok": False, "msg": "增强微调未完全成功"}

    # ==================== YOLO 抓取 ====================

    def yolo_pick(self, target):
        try:
            resp = http_get(f"{YOLO_XYZ_URL}?label={target}", timeout=10,
                            max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
            if resp.status_code != 200:
                return {"ok": False, "msg": "YOLO请求失败"}
            data = resp.json()
            if "XiongMao" in target:
                point = data["box_head_point_above_xyz_mm"]
                offset = [20.0, 0.0, 0.0]
            else:
                point = data["center_above_xyz_mm"]
                offset = [0.0, 0.0, 0.0]
            pick_mm = [p + o for p, o in zip(point, offset)]
            cam_to_base_t = self.cam_to_base_t
            pick_base = [
                pick_mm[2]/1000.0 + cam_to_base_t[0],
                -pick_mm[0]/1000.0 + cam_to_base_t[1],
                -pick_mm[1]/1000.0 + cam_to_base_t[2]
            ]
            return self.arm("PICK", json.dumps({"position": pick_base}))
        except Exception as e:
            return {"ok": False, "msg": f"YOLO抓取异常: {e}"}

    # ==================== 距离验证 ====================

    def verify_distance(self):
        try:
            resp = http_get(YOLO_XYZ_URL, timeout=10,
                            max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
            if resp.status_code != 200:
                return {"ok": False, "msg": "YOLO服务不可用"}
            dist_mm = resp.json().get("range_from_left_camera_mm", 9999)
            if dist_mm > 600:
                return {"ok": False, "msg": f"距离过远({dist_mm}mm)"}
            return {"ok": True, "msg": f"距离正常({dist_mm}mm)"}
        except Exception as e:
            return {"ok": False, "msg": f"距离验证异常: {e}"}

    # ==================== 多步骤任务 ====================

    def run_task(self, steps, pose_start=None, pose_end=None, target="XiongMao"):
        """执行多步骤任务"""
        self._params = load_sdk_params()
        for k, v in self._params.items():
            setattr(self, k, v)
        self._lift_offset = read_lift_offset()

        results = []
        for i, step in enumerate(steps):
            self._check_stop()
            step = step.strip()
            if not step:
                continue
            result = self._execute_step(step, pose_start, pose_end, target, i, len(steps))
            results.append({"step": step, "index": i, **result})
            if not result.get("ok", False):
                return {"ok": False, "msg": f"步骤 {step} 失败", "results": results,
                        "failed_at": i}
        return {"ok": True, "msg": "任务完成", "results": results}

    def _execute_step(self, step, pose_start, pose_end, target, idx, total):
        """执行单个步骤"""
        log.info(f"[任务 {idx+1}/{total}] 执行: {step}")

        # 解析带参数的步骤，如 lift_to(0.3), rotate(200), backup(0.5), move(forward,0.5)
        import re
        m = re.match(r'(\w+)\((.+)\)', step)
        if m:
            step_name = m.group(1)
            step_param = m.group(2)
        else:
            step_name = step
            step_param = None

        try:
            if step_name == "nav_start":
                return self.nav(pose_start or {})
            elif step_name == "nav_end":
                return self.nav(pose_end or {})
            elif step_name == "nav_start_lift":
                h = float(step_param or 0)
                t1 = threading.Thread(target=self.nav, args=(pose_start or {},), daemon=True)
                t1.start()
                t2 = threading.Thread(target=self.lift_to, args=(h,), daemon=True)
                t2.start()
                t1.join(timeout=180)
                t2.join(timeout=60)
                return {"ok": True, "msg": "导航+升降完成"}
            elif step_name == "nav_end_lift":
                h = float(step_param or 0)
                t1 = threading.Thread(target=self.nav, args=(pose_end or {},), daemon=True)
                t1.start()
                t2 = threading.Thread(target=self.lift_to, args=(h,), daemon=True)
                t2.start()
                t1.join(timeout=180)
                t2.join(timeout=60)
                return {"ok": True, "msg": "导航+升降完成"}
            elif step_name == "rotate_lift":
                parts = [p.strip() for p in step_param.split(",")]
                angle, h = float(parts[0]), float(parts[1])
                t1 = threading.Thread(target=self.rotate, args=(angle,), daemon=True)
                t1.start()
                t2 = threading.Thread(target=self.lift_to, args=(h,), daemon=True)
                t2.start()
                t1.join(timeout=60)
                t2.join(timeout=60)
                return {"ok": True, "msg": "旋转+升降完成"}
            elif step_name == "pick":
                if not check_arm_process():
                    log.warning("机械臂进程 arm_task_node.py 未运行！")
                return self.arm("PICK", target, extra_wait=0.5)
            elif step_name == "place":
                return self.arm("PLACE", "")
            elif step_name == "reset":
                return self.arm("RESET", "")
            elif step_name == "weitiao":
                return self.weitiao(target)
            elif step_name == "yolo_pick":
                return self.yolo_pick(target)
            elif step_name == "verify_distance":
                return self.verify_distance()
            elif step_name == "lift_to":
                return self.lift_to(float(step_param or 0))
            elif step_name == "lift_rel":
                parts = [p.strip() for p in step_param.split(",")]
                return self.lift_rel(parts[0], float(parts[1]) if len(parts) > 1 else 0.1)
            elif step_name == "rotate":
                return self.rotate(float(step_param or 0))
            elif step_name == "move":
                return self.move(*[p.strip() for p in step_param.split(",", 1)])
            elif step_name == "backup":
                return self.backup(float(step_param or 0.5))
            elif step_name == "gripper":
                return self.gripper(step_param or "status")
            else:
                return {"ok": False, "msg": f"未知步骤: {step_name}"}
        except Exception as e:
            return {"ok": False, "msg": f"步骤异常: {e}"}

    def arm(self, phase, target="", timeout_sec=None, extra_wait=0.0):
        """机械臂控制（带 extra_wait 参数版本）"""
        self._check_stop()
        if timeout_sec is None:
            timeout_sec = self.arm_timeout
        if dry_run_skip(f"机械臂 {phase}: {target}"):
            return {"ok": True, "msg": f"[DRY-RUN] 机械臂 {phase}"}
        last_error = ""
        for attempt in range(self.arm_max_retries):
            if attempt > 0:
                time.sleep(self.arm_retry_delay)
            payload = {
                "type": "arm_task",
                "phase": phase,
                "target_object": target,
                "timeout_sec": timeout_sec
            }
            try:
                resp = http_post(ARM_API_URL, payload, timeout=timeout_sec + 10,
                                 max_retries=self.http_max_retries,
                                 retry_delay=self.http_retry_delay)
            except Exception as e:
                last_error = f"请求异常: {e}"
                continue
            if resp.status_code != 200:
                last_error = f"HTTP {resp.status_code}"
                continue
            result = resp.json()
            status_text = result.get("final_status", {}).get("status_text", "UNKNOWN")
            if status_text == "DONE":
                atomic_write_json(LAST_ARM_STATUS_FILE, {
                    "phase": phase, "target_object": target,
                    "status": status_text, "timestamp": time.time()
                })
                if extra_wait > 0:
                    time.sleep(extra_wait)
                return {"ok": True, "msg": f"机械臂 {phase} 完成"}
            last_error = f"状态={status_text}"
        return {"ok": False, "msg": f"机械臂 {phase} 失败: {last_error}"}

    # ==================== 并行合成动作 ====================

    def _parallel_exec(self, func_a, args_a, name_a, func_b, args_b, name_b):
        """通用并行执行：两个子任务同时运行，等待全部完成"""
        results = {"a": None, "b": None, "errors": []}
        errors_lock = threading.Lock()
        # 继承当前线程的控制代际到子线程
        parent_gen = getattr(_task_context, 'control_gen', -1)

        def run_side(side, fn, args, label):
            _task_context.control_gen = parent_gen
            try:
                r = fn(*args)
                with errors_lock:
                    results[side] = r
            except InterruptedError as e:
                with errors_lock:
                    results[side] = {"ok": False, "msg": f"{label} 被中断: {e}"}
                    results["errors"].append(f"{label} 中断")
            except Exception as e:
                with errors_lock:
                    results["side"] = {"ok": False, "msg": f"{label} 异常: {e}"}
                    results["errors"].append(f"{label}: {e}")

        ta = threading.Thread(target=run_side, args=("a", func_a, args_a, name_a), daemon=True)
        tb = threading.Thread(target=run_side, args=("b", func_b, args_b, name_b), daemon=True)
        ta.start(); tb.start()
        ta.join(); tb.join()

        ok = all(r.get("ok", False) for r in [results.get("a"), results.get("b")] if isinstance(r, dict))
        err_msg = "; ".join(results["errors"]) if results["errors"] else ""
        msgs = []
        if isinstance(results.get("a"), dict) and results["a"].get("msg"):
            msgs.append(f"{name_a}: {results['a']['msg']}")
        if isinstance(results.get("b"), dict) and results["b"].get("msg"):
            msgs.append(f"{name_b}: {results['b']['msg']}")
        summary = " | ".join(msgs)
        if not ok:
            return {"ok": False, "msg": summary or (err_msg or f"并行动作失败")}
        return {"ok": True, "msg": summary or f"{name_a} + {name_b} 完成"}

    def nav_start_lift(self, pose, height_m):
        """【并行】导航到抓取点 + 立柱升至指定高度"""
        self._check_stop()
        nav_task_id = f"nav-{int(time.time())}"
        return self._parallel_exec(
            self.nav, (pose, nav_task_id, 180), "导航",
            self.lift_to, (height_m,), "升降"
        )

    def rotate_lift(self, angle_deg, height_m):
        """【并行】旋转指定角度 + 立柱升降"""
        self._check_stop()
        return self._parallel_exec(
            self.rotate, (angle_deg,), "旋转",
            self.lift_to, (height_m,), "升降"
        )

    def move_lift(self, direction, distance_m, height_m):
        """【并行】移动 + 立柱升降"""
        self._check_stop()
        return self._parallel_exec(
            self.move, (direction, distance_m), "移动",
            self.lift_to, (height_m,), "升降"
        )

    def arm_lift(self, phase, target, height_m, timeout_sec=None):
        """【并行】机械臂操作 + 立柱升降"""
        self._check_stop()
        return self._parallel_exec(
            self.arm, (phase, target, timeout_sec if timeout_sec else 130), "机械臂",
            self.lift_to, (height_m,), "升降"
        )


# ==================== 任务管理器 ====================

class TaskManager:
    """管理异步任务的生命周期"""

    def __init__(self, control_node=None):
        self._tasks = {}  # task_id → {status, result, thread, cancel_flag}
        self._lock = threading.Lock()
        self._control_node = control_node

    def submit(self, func, *args, **kwargs):
        task_id = str(uuid.uuid4())[:8]
        cancel_flag = threading.Event()

        def wrapper():
            # 记录当前控制代际到线程局部变量，_check_stop 会与之比对
            my_gen = -1
            if self._control_node is not None:
                my_gen = self._control_node._control_gen
                _task_context.control_gen = my_gen
                # 注册活跃代际（防御竞态条件）
                with self._control_node._gens_lock:
                    self._control_node._active_gens.add(my_gen)
            gen = my_gen
            print(f"[task:{task_id}] gen={gen} 开始执行", flush=True)

            with self._lock:
                self._tasks[task_id]["status"] = "running"

            # 获取超时配置（秒）
            timeout_sec = getattr(self._control_node, 'step_timeout', 180) if self._control_node else 180
            timed_out = threading.Event()

            # 超时监控线程
            def timeout_watcher():
                if timed_out.wait(timeout=timeout_sec):
                    return  # 正常完成，watcher 退出
                # 超时，强制中断
                print(f"[task:{task_id}] gen={gen} 超时 ({timeout_sec}s)，强制中断", flush=True)
                if self._control_node:
                    self._control_node._control_gen += 1
                    with self._control_node._gens_lock:
                        self._control_node._active_gens.clear()
                    self._control_node.stop_robot()
                with self._lock:
                    if self._tasks.get(task_id, {}).get("status") == "running":
                        self._tasks[task_id]["status"] = "failed"
                        self._tasks[task_id]["result"] = {"ok": False, "msg": f"任务超时 ({timeout_sec}s)"}

            watcher = threading.Thread(target=timeout_watcher, daemon=True)
            watcher.start()

            try:
                result = func(*args, **kwargs)
                timed_out.set()  # 通知 watcher 正常完成
                print(f"[task:{task_id}] gen={gen} 执行完成", flush=True)
                with self._lock:
                    if self._tasks.get(task_id, {}).get("status") == "running":
                        self._tasks[task_id]["status"] = "completed"
                        self._tasks[task_id]["result"] = result
            except InterruptedError as e:
                timed_out.set()
                print(f"[task:{task_id}] gen={gen} 被新指令中断", flush=True)
                with self._lock:
                    if self._tasks.get(task_id, {}).get("status") == "running":
                        self._tasks[task_id]["status"] = "interrupted"
                        self._tasks[task_id]["result"] = {"ok": False, "msg": str(e)}
            except Exception as e:
                timed_out.set()
                print(f"[task:{task_id}] gen={gen} 异常: {e}", flush=True)
                with self._lock:
                    if self._tasks.get(task_id, {}).get("status") == "running":
                        self._tasks[task_id]["status"] = "failed"
                        self._tasks[task_id]["result"] = {"ok": False, "msg": str(e)}
            finally:
                # 清理活跃代际
                if self._control_node is not None and my_gen > 0:
                    with self._control_node._gens_lock:
                        self._control_node._active_gens.discard(my_gen)

        with self._lock:
            self._tasks[task_id] = {
                "status": "pending",
                "result": None,
                "thread": None,
                "cancel_flag": cancel_flag,
                "created_at": time.time(),
            }
            t = threading.Thread(target=wrapper, daemon=True)
            self._tasks[task_id]["thread"] = t
            t.start()
        return task_id

    def get_status(self, task_id):
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return {"status": "not_found"}
            return {
                "status": task["status"],
                "result": task["result"],
                "created_at": task["created_at"],
            }

    def cancel(self, task_id):
        with self._lock:
            task = self._tasks.get(task_id)
            if task is None:
                return False
            task["cancel_flag"].set()
            task["status"] = "cancelled"
        return True

    def cleanup_old(self, max_age=3600):
        """清理超过1小时的已完成任务"""
        now = time.time()
        with self._lock:
            to_delete = []
            for tid, task in self._tasks.items():
                if task["status"] in ("completed", "failed", "cancelled"):
                    if now - task["created_at"] > max_age:
                        to_delete.append(tid)
            for tid in to_delete:
                del self._tasks[tid]


# ==================== HTTP 服务 ====================

class ControlHandler(BaseHTTPRequestHandler):
    control_node = None
    task_manager = None

    def log_message(self, fmt, *args):
        pass

    def _send(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        if isinstance(body, str):
            body = body.encode()
        self.wfile.write(body)

    def _json_response(self, data, code=200):
        self._send(code, "application/json; charset=utf-8",
                   json.dumps(data, ensure_ascii=False))

    def _read_body(self):
        length = int(self.headers.get('Content-Length', 0))
        if length == 0:
            return {}
        body = self.rfile.read(length)
        try:
            return json.loads(body)
        except Exception:
            return {}

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/health":
            node = self.control_node
            self._json_response({
                "ok": True,
                "service": "g1d_control_api",
                "version": "V1.3",
                "port": HTTP_PORT,
                "rclpy_ok": rclpy.ok(),
                "odom_ready": node._odom_ready if node else False,
                "hispeed_ready": node._column_height is not None if node else False,
                "control_gen": node._control_gen if node else 0,
                "uptime_sec": round(time.time() - _start_time, 1),
            })
        elif path == "/api/status":
            node = self.control_node
            self._json_response({
                "odom_ready": node._odom_ready,
                "lift_offset": node._lift_offset,
                "column_height": node.get_column_height(),
                "physical_height": get_physical_height(
                    node.get_column_height() or -1.0, node._lift_offset
                ) if node.get_column_height() is not None else -1.0,
                "yaw": node.get_yaw(),
                "position": list(node.get_position() or (None, None)),
                "arm_process": check_arm_process(),
            })
        elif path.startswith("/api/task/"):
            task_id = path.split("/api/task/")[1].split("/")[0]
            self._json_response(self.task_manager.get_status(task_id))
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        data = self._read_body()
        node = self.control_node
        tm = self.task_manager

        try:
            # ---- 紧急停止（同步执行，不进任务队列）----
            if path == "/api/control/stop":
                node._control_gen += 1  # 使所有正在运行的任务自动失效
                with node._gens_lock:
                    node._active_gens.clear()  # 清空所有活跃代际
                node.stop_robot()
                # 同时尝试停止机械臂
                try:
                    requests.post(ARM_API_URL, json={
                        "type": "arm_task", "phase": "RESET",
                        "target_object": "", "timeout_sec": 5
                    }, timeout=3)
                except Exception:
                    pass
                self._json_response({"ok": True, "msg": "紧急停止已发送"})

            # ---- 单步控制（异步执行，返回 task_id）----
            elif path == "/api/control/rotate":
                angle = float(data.get("angle", 0))
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.rotate, angle)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"旋转 {angle}° 已提交"})

            elif path == "/api/control/move":
                direction = data.get("direction", "forward")
                distance = float(data.get("distance", 0.3))
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.move, direction, distance)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"移动 {direction} {distance}m 已提交"})

            elif path == "/api/control/lift_to":
                height = float(data.get("height", 0.0))
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.lift_to, height)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"升降至 {height}m 已提交"})

            elif path == "/api/control/lift_rel":
                direction = data.get("direction", "up")
                distance = float(data.get("distance", 0.1))
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.lift_rel, direction, distance)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"相对升降 {direction} {distance}m 已提交"})

            elif path == "/api/control/gripper":
                action = data.get("action", "status")
                # 夹爪操作很快，同步执行
                result = node.gripper(action)
                self._json_response(result)

            elif path == "/api/control/arm":
                phase = data.get("phase", "RESET")
                target = data.get("target", "")
                timeout = int(data.get("timeout", 130))
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.arm, phase, target, timeout)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"机械臂 {phase} 已提交"})

            elif path == "/api/control/nav":
                pose = data.get("pose", {})
                task_id_nav = data.get("task_id", f"nav-{int(time.time())}")
                timeout = int(data.get("timeout", 180))
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.nav, pose, task_id_nav, timeout)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"导航已提交 {task_id_nav}"})

            elif path == "/api/control/weitiao":
                target = data.get("target", "XiongMao")
                mode = data.get("mode", None)
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.weitiao, target, mode)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"微调已提交"})

            elif path == "/api/control/yolo_pick":
                target = data.get("target", "XiongMao")
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.yolo_pick, target)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"YOLO抓取已提交"})

            elif path == "/api/control/verify_distance":
                node._control_gen += 1
                task_id = tm.submit(node.verify_distance)
                self._json_response({"ok": True, "task_id": task_id, "msg": "距离验证已提交"})

            # ---- 多步骤任务 ----
            elif path == "/api/control/task":
                steps = data.get("steps", [])
                pose_start = data.get("pose_start", None)
                pose_end = data.get("pose_end", None)
                target = data.get("target", "XiongMao")
                if isinstance(pose_start, str):
                    pose_start = json.loads(pose_start)
                if isinstance(pose_end, str):
                    pose_end = json.loads(pose_end)
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.run_task, steps, pose_start, pose_end, target)
                self._json_response({"ok": True, "task_id": task_id,
                                     "msg": f"多步骤任务已提交 ({len(steps)}步)"})

            # ---- 并行合成动作 ----
            elif path == "/api/control/nav_start_lift":
                pose = data.get("pose", {})
                height = float(data.get("height", 0.0))
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.nav_start_lift, pose, height)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"并行: 导航+升至{height}m 已提交"})

            elif path == "/api/control/rotate_lift":
                angle = float(data.get("angle", 0))
                height = float(data.get("height", 0.0))
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.rotate_lift, angle, height)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"并行: 旋转{angle}°+升至{height}m 已提交"})

            elif path == "/api/control/move_lift":
                direction = data.get("direction", "forward")
                distance = float(data.get("distance", 0.3))
                height = float(data.get("height", 0.0))
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.move_lift, direction, distance, height)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"并行: 移动+升至{height}m 已提交"})

            elif path == "/api/control/arm_lift":
                phase = data.get("phase", "RESET")
                target = data.get("target", "")
                height = float(data.get("height", 0.0))
                timeout = int(data.get("timeout", 130))
                node._control_gen += 1; node.stop_robot()
                task_id = tm.submit(node.arm_lift, phase, target, height, timeout)
                self._json_response({"ok": True, "task_id": task_id, "msg": f"并行: 机械臂{phase}+升至{height}m 已提交"})

            # ---- 取消任务 ----
            elif path.startswith("/api/task/") and path.endswith("/cancel"):
                task_id = path.split("/api/task/")[1].split("/")[0]
                ok = tm.cancel(task_id)
                self._json_response({"ok": ok, "msg": "任务已取消" if ok else "任务不存在"})

            else:
                self._send(404, "text/plain", b"not found")

        except Exception as e:
            self._json_response({"ok": False, "msg": f"请求处理失败: {e}"}, 500)


class ReuseThreadingHTTPServer(HTTPServer):
    """支持端口复用的多线程 HTTP 服务器，每个请求独立线程处理"""
    allow_reuse_address = True

    def process_request(self, request, client_address):
        import socketserver
        t = threading.Thread(target=self.process_request_thread,
                             args=(request, client_address), daemon=True)
        t.start()

    def process_request_thread(self, request, client_address):
        try:
            self.finish_request(request, client_address)
        except Exception:
            self.handle_error(request, client_address)
        finally:
            self.shutdown_request(request)


def main():
    global _start_time
    _start_time = time.time()
    print("=" * 50, flush=True)
    print("G1D 统一控制 API V1.3", flush=True)
    print(f"端口: {HTTP_PORT}", flush=True)
    print("=" * 50, flush=True)

    # 初始化 ROS2
    if not rclpy.ok():
        rclpy.init()

    # 创建控制节点
    control_node = ControlNode()

    # 创建任务管理器
    task_manager = TaskManager(control_node)

    # 设置 handler 的类变量
    ControlHandler.control_node = control_node
    ControlHandler.task_manager = task_manager

    # 启动专用 ROS2 spin 线程（唯一调用 spin_once 的线程）
    _running = True
    def spin_loop():
        while _running and rclpy.ok():
            try:
                control_node.safe_spin_once(0.1)
            except Exception as e:
                # ExternalShutdownException 等 → ROS2 已关闭，必须退出
                if not rclpy.ok() or 'Shutdown' in type(e).__name__:
                    print(f"[control_api] ROS2 已关闭，spin 线程退出", flush=True)
                    break
                print(f"[control_api] spin_once 异常: {e}", flush=True)
                time.sleep(0.5)
    spin_thread = threading.Thread(target=spin_loop, daemon=True)
    spin_thread.start()
    print("[control_api] ROS2 spin 线程已启动", flush=True)

    # DDS 数据接收监控：长时间无 odom/hispeed 数据时告警
    def dds_watchdog_loop():
        last_odom_warn = 0
        last_hispeed_warn = 0
        while _running and rclpy.ok():
            time.sleep(30)
            now = time.time()
            if not control_node._odom_ready and now - last_odom_warn > 300:
                print(f"[control_api] 警告: 无 odom 数据", flush=True)
                last_odom_warn = now
            if control_node._column_height is None and now - last_hispeed_warn > 300:
                print(f"[control_api] 警告: 无 hispeed 数据 (升降高度不可用)", flush=True)
                last_hispeed_warn = now
    threading.Thread(target=dds_watchdog_loop, daemon=True).start()

    # 启动 HTTP 服务（多线程版，不会因单个请求阻塞）
    server = ReuseThreadingHTTPServer(("0.0.0.0", HTTP_PORT), ControlHandler)
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    http_thread.start()
    print(f"[control_api] HTTP 服务已启动: http://0.0.0.0:{HTTP_PORT}/", flush=True)

    # 定期清理旧任务
    def cleanup_loop():
        while _running:
            time.sleep(60)
            task_manager.cleanup_old()
    threading.Thread(target=cleanup_loop, daemon=True).start()

    # 主线程保持存活（不在此 spin，避免与控制方法争抢）
    # 注册 SIGTERM 处理（systemd stop 发 SIGTERM）
    def _sigterm_handler(signum, frame):
        nonlocal _running
        print("[control_api] 收到 SIGTERM，正在停止...", flush=True)
        _running = False
    signal.signal(signal.SIGTERM, _sigterm_handler)

    try:
        while _running:
            # 短间隔 sleep，使 SIGTERM 信号能被及时处理
            for _ in range(10):
                if not _running:
                    break
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        _running = False
        server.shutdown()
        control_node.destroy_node()
        rclpy.shutdown()
        print("[control_api] 服务已停止")


if __name__ == "__main__":
    main()
