#!/usr/bin/env python3
"""
步骤执行器 V6.0
- 使用 g1d_common 共享模块
- 合并 rotate/move/height CLI 子命令（替代独立脚本）
- odom 闭环控制
- offset 补偿 + 高度验证
- HTTP 重试
- Dry-run 模式（G1D_DRY_RUN=1）
- 结构化日志
- Webhook 通知
"""
import sys, argparse, json, time, math, threading, os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lib"))
from g1d_common import (
    log, DRY_RUN, dry_run_skip, HispeedMsg,
    LIFT_OFFSET_FILE, FULL_TRAVEL, SSH_USER, SSH_HOST, ROBOT_IF,
    SDK_HEIGHT_BIN, SDK_SIMPLE_BIN,
    ARM_API_URL, YOLO_XYZ_URL, ADJUST_URL, GRIPPER_URL, NAV_API_URL,
    LAST_ARM_STATUS_FILE,
    read_lift_offset, save_lift_offset, get_physical_height,
    yaw_from_odom, position_from_odom, check_arm_process,
    load_sdk_params, http_get, http_post, run_remote_ssh,
    atomic_write_json, notify,
)
from robot_nav_arm_flow import NavigationClient


class StepExecutor(Node):
    def __init__(self, base_url=None):
        super().__init__('step_executor')
        if base_url is None:
            base_url = NAV_API_URL
        self.nav_client = NavigationClient(base_url)
        self.start_pose = {}
        self.end_pose = {}
        self.pick_instruction = "XiongMao"
        self.place_instruction = "place"
        self.nav_timeout = 180
        self.arm_timeout = 130
        self.nav_query_interval = 1.0
        self.backup_distance = 0.5

        # ROS2 通信
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._odom_data = None
        self._odom_lock = threading.Lock()
        self._column_height = None
        self.create_subscription(Odometry, '/agv/odom', self._odom_cb, 10)
        if HispeedMsg is not None:
            self.create_subscription(HispeedMsg, '/hispeed_state', self._hispeed_cb, 10)

        # 等待 odom 就绪
        self._odom_ready = self._wait_for_odom()

        # 读取 offset
        self._lift_offset = read_lift_offset()
        if self._lift_offset != 0.0:
            log.info(f"立柱 offset: {self._lift_offset:.4f}m", extra={"offset": self._lift_offset})

        # 加载参数
        self._params = load_sdk_params()
        # 将参数展开为实例属性（兼容现有代码）
        for k, v in self._params.items():
            setattr(self, k, v)
        # 特殊映射
        self.http_max_retries = self._params["http_max_retries"]
        self.http_retry_delay = self._params["http_retry_delay"]

    # ==================== ROS2 回调 ====================

    def _odom_cb(self, msg):
        with self._odom_lock:
            self._odom_data = msg

    def _hispeed_cb(self, msg):
        self._column_height = msg.y

    def _wait_for_odom(self, timeout=5.0):
        log.info("等待 odom 数据...")
        start = time.time()
        while self._odom_data is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._odom_data is None:
            log.warning("未收到 odom 数据，旋转/移动将使用开环控制")
            return False
        return True

    def _get_yaw(self):
        with self._odom_lock:
            return yaw_from_odom(self._odom_data)

    def _get_position(self):
        with self._odom_lock:
            return position_from_odom(self._odom_data)

    # ==================== 并行执行 ====================

    def _run_parallel(self, *tasks):
        exceptions = []
        def wrapper(method, args):
            try:
                method(*args)
            except Exception as e:
                exceptions.append(e)
        threads = []
        for method, args in tasks:
            t = threading.Thread(target=wrapper, args=(method, args))
            t.daemon = True
            t.start()
            threads.append(t)
        for t in threads:
            t.join()
        if exceptions:
            raise RuntimeError(f"并行步骤失败: {exceptions[0]}")

    # ==================== 步骤分发 ====================

    def run_step(self, step_name, **kwargs):
        if step_name == "nav_start":
            self._do_nav("Start", self.start_pose)
        elif step_name == "nav_end":
            self._do_nav("End", self.end_pose)
        elif step_name == "nav_start_lift":
            height = float(kwargs.get("height", 0.0))
            self._run_parallel(
                (self._do_nav, ("Start", self.start_pose)),
                (self._lift_to, (height,))
            )
        elif step_name == "nav_end_lift":
            height = float(kwargs.get("height", 0.0))
            self._run_parallel(
                (self._do_nav, ("End", self.end_pose)),
                (self._lift_to, (height,))
            )
        elif step_name == "rotate_lift":
            params = kwargs.get("params", "")
            parts = [p.strip() for p in params.split(",")]
            if len(parts) < 2:
                raise RuntimeError("rotate_lift 需要角度和高度，如 200,0.30")
            angle = float(parts[0])
            height = float(parts[1])
            self._run_parallel(
                (self._rotate, (angle,)),
                (self._lift_to, (height,))
            )
        elif step_name == "pick":
            if not check_arm_process():
                log.warning("机械臂进程 arm_task_node.py 未运行！")
            self._arm_http("PICK", self.pick_instruction, extra_wait=0.5)
        elif step_name == "place":
            self._arm_http("PLACE", "")
        elif step_name == "reset":
            self._arm_http("RESET", "")
        elif step_name == "weitiao":
            log.info(f"当前微调模式: {self.weitiao_mode}")
            if self.weitiao_mode == "classic":
                self._weitiao_classic()
            else:
                self._weitiao_enhanced()
        elif step_name == "yolo_pick":
            self._yolo_pick()
        elif step_name == "verify_distance":
            self._verify_distance()
        elif step_name == "adjust_retry":
            self._adjust_retry()
        elif step_name == "lift_to":
            self._lift_to(float(kwargs.get("height", 0.0)))
        elif step_name == "lift_rel":
            self._relative_lift(kwargs["direction"], float(kwargs.get("distance", 0.1)))
        elif step_name == "lift_offset":
            self._show_offset()
        elif step_name == "backup":
            self._backup(kwargs.get("distance", self.backup_distance))
        elif step_name == "rotate":
            self._rotate(kwargs["angle"])
        elif step_name == "move":
            self._move(kwargs["params"])
        elif step_name == "gripper":
            self._gripper(kwargs["action"])
        else:
            log.warning(f"未知步骤: {step_name}")

    # ==================== 导航 ====================

    def _do_nav(self, target_name, pose):
        task_id = f"nav-{target_name.lower()}-{int(time.time())}"
        log.info(f"导航到 {target_name}，task_id={task_id}", extra={"task": task_id})
        if dry_run_skip(f"导航到 {target_name}"):
            return
        self.nav_client.submit_navigation(pose, task_id)
        self.nav_client.wait_navigation(task_id, self.nav_timeout, self.nav_query_interval)
        log.info(f"已到达 {target_name}")

    # ==================== 机械臂 HTTP ====================

    def _arm_http(self, phase, target_object, extra_wait=0.0):
        if dry_run_skip(f"机械臂 {phase}: target_object='{target_object}'"):
            return

        last_error = ""
        for attempt in range(self.arm_max_retries):
            if attempt > 0:
                log.info(f"机械臂等待 {self.arm_retry_delay}s 后重试 ({attempt+1}/{self.arm_max_retries})")
                time.sleep(self.arm_retry_delay)
            payload = {
                "type": "arm_task",
                "phase": phase,
                "target_object": target_object,
                "timeout_sec": self.arm_timeout
            }
            log.info(f"机械臂 {phase}: target_object='{target_object}'")
            try:
                resp = http_post(ARM_API_URL, payload, timeout=self.arm_timeout + 10,
                                 max_retries=self.http_max_retries,
                                 retry_delay=self.http_retry_delay)
            except Exception as e:
                last_error = f"请求异常: {e}"
                log.warning(last_error, extra={"error": str(e)})
                continue
            if resp.status_code != 200:
                last_error = f"HTTP {resp.status_code}: {resp.text[:200]}"
                log.warning(f"机械臂错误: {last_error}")
                continue
            result = resp.json()
            final = result.get("final_status", {})
            status_text = final.get("status_text", "UNKNOWN")
            if status_text == "DONE":
                log.info(f"机械臂完成: {status_text}")
                atomic_write_json(LAST_ARM_STATUS_FILE, {
                    "phase": phase, "target_object": target_object,
                    "status": status_text, "exec_status": final.get("exec_status", -1),
                    "timestamp": time.time()
                })
                if extra_wait > 0:
                    time.sleep(extra_wait)
                return
            else:
                last_error = f"状态={status_text}"
                log.warning(f"机械臂失败: {last_error}")
        if phase == "PICK":
            if not check_arm_process():
                log.warning("机械臂进程 arm_task_node.py 未运行！")
        notify(f"机械臂 {phase} 失败: {last_error}", level="ERROR")
        raise RuntimeError(f"机械臂 {phase} 最终失败: {last_error}")

    # ==================== 经典微调 ====================

    def _weitiao_classic(self):
        log.info("执行经典微调...")
        try:
            resp = http_get(ADJUST_URL, params={"label": self.pick_instruction}, timeout=30,
                            max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
            if resp.status_code != 200:
                log.warning(f"微调请求失败，状态码: {resp.status_code}")
                return
            data = resp.json()
            hint = data.get("robot_alignment", {}).get("control_hint", {})
            forward_err = abs(hint.get("forward_distance_m", 0))
            turn_err = abs(hint.get("turn_first_yaw_deg", 0))
            log.info(f"微调完成，前向误差: {forward_err:.3f}m，角度误差: {turn_err:.1f}°")
            height_down_mm = hint.get("height_down_m", 0.0)
            if abs(height_down_mm) > self.height_threshold_mm:
                direction = "down" if height_down_mm > 0 else "up"
                distance_m = abs(height_down_mm) / 1000.0
                log.info(f"高度偏差 {height_down_mm:.1f}mm，执行相对升降: {direction} {distance_m:.3f}m",
                         extra={"height": distance_m})
                self._relative_lift(direction, distance_m, 0.1)
            else:
                log.info(f"高度偏差 {height_down_mm:.1f}mm 在阈值内，无需调整")
        except Exception as e:
            log.warning(f"经典微调异常: {e}", extra={"error": str(e)})

    # ==================== 增强微调 ====================

    def _weitiao_enhanced(self):
        loop_count = 0
        for search_attempt in range(self.max_search + 1):
            loop_count += 1
            if loop_count > self.enhanced_max_loops:
                log.warning(f"增强微调达到最大循环次数 {self.enhanced_max_loops}，退出")
                break
            log.info(f"视觉对齐 搜索第{search_attempt+1}/{self.max_search+1} 次 (总循环 {loop_count}/{self.enhanced_max_loops})")
            try:
                resp = http_get(ADJUST_URL, params={"label": self.pick_instruction}, timeout=30,
                                max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
            except Exception as e:
                log.warning(f"微调请求异常: {e}", extra={"error": str(e)})
                break
            if resp.status_code != 200:
                log.warning("微调请求失败")
                break
            data = resp.json()
            align = data.get("robot_alignment", {})
            near_edge = align.get("near_edge_robot_alignment", {}).get("target", {}).get("ground_forward_mm", None)
            if near_edge is None:
                near_edge = align.get("robot_alignment", {}).get("target", {}).get("ground_forward_mm", 0)
            hint = align.get("control_hint", {})
            lateral = hint.get("lateral_error_m", 0.0)
            confidence = data.get("selected_yolo_confidence", 0.0)
            reproj = data.get("left_reprojection_error_px", 0.0)
            depth_delta = data.get("depth_delta_mm", 0.0)

            log.info(f"近端边前向: {near_edge:.0f}mm, 横向偏差: {lateral:.3f}m, 角度偏差: {abs(hint.get('turn_first_yaw_deg',0)):.1f}°")
            if abs(lateral) > 0.3:
                log.warning(f"横向偏差较大 ({lateral:.3f}m)，暂不自动纠正")

            if near_edge <= 0 or confidence <= 0:
                log.warning(f"未检测到目标 (near_edge={near_edge:.0f}mm, conf={confidence:.2f})")
                if search_attempt < self.max_search:
                    log.info("执行搜索：升高 + 后退")
                    self._relative_lift("up", self.search_step_height, 0.1)
                    self._backup(self.search_backup)
                    continue
                else:
                    log.warning("已达最大搜索次数，但仍未检测到目标，任务继续")
                    return

            low_quality = (confidence < 0.15) or (reproj > 3.0) or (depth_delta > 100.0)
            if low_quality and search_attempt < self.max_search:
                log.warning("检测质量差，执行搜索：升高 + 后退")
                self._relative_lift("up", self.search_step_height, 0.1)
                self._backup(self.search_backup)
                continue

            if self.near_edge_min <= near_edge <= self.near_edge_max:
                log.info(f"近端边前向 {near_edge:.0f}mm 在理想区间内")
                height_down_mm = hint.get("height_down_m", 0.0)
                if abs(height_down_mm) > self.height_threshold_mm:
                    direction = "down" if height_down_mm > 0 else "up"
                    distance_m = abs(height_down_mm) / 1000.0
                    log.info(f"高度偏差 {height_down_mm:.1f}mm，执行相对升降: {direction} {distance_m:.3f}m",
                             extra={"height": distance_m})
                    self._relative_lift(direction, distance_m, 0.1)
                else:
                    log.info(f"高度偏差 {height_down_mm:.1f}mm 在阈值内，无需调整")
                return

            deviation = near_edge - (self.near_edge_min + self.near_edge_max) / 2
            if abs(deviation) < self.small_deviation_mm:
                distance_m = -deviation / 1000.0
                linear_x = 0.1 if distance_m > 0 else -0.1
                abs_dist = abs(distance_m)
                log.info(f"偏差 {deviation:.0f}mm < {self.small_deviation_mm}mm，直接底盘移动: {'前进' if distance_m>0 else '后退'} {abs_dist:.3f}m")
                self._cmd_vel_move(linear_x, abs_dist)
                continue

            for retry in range(self.max_adjust_retries + 1):
                loop_count += 1
                if loop_count > self.enhanced_max_loops:
                    log.warning(f"增强微调达到最大循环次数 {self.enhanced_max_loops}，退出")
                    return
                log.info(f"重新微调 第{retry+1}/{self.max_adjust_retries+1} 次")
                try:
                    resp = http_get(ADJUST_URL, params={"label": self.pick_instruction}, timeout=30,
                                    max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
                except Exception:
                    log.warning("微调请求异常")
                    continue
                if resp.status_code == 200:
                    data = resp.json()
                    align = data.get("robot_alignment", {})
                    near_edge = align.get("near_edge_robot_alignment", {}).get("target", {}).get("ground_forward_mm", 0)
                    if self.near_edge_min <= near_edge <= self.near_edge_max:
                        log.info(f"微调后近端边前向 {near_edge:.0f}mm 达到理想区间")
                        return
                    else:
                        log.info(f"微调后近端边前向 {near_edge:.0f}mm，仍不在区间")
                else:
                    log.warning("微调请求失败")
            break

        log.warning("视觉对齐未完全成功，但任务继续")

    # ==================== 相对升降（带边界检查）====================

    def _relative_lift(self, direction, distance_m, speed=0.1, duration=None):
        if duration is None:
            duration = distance_m / speed

        if dry_run_skip(f"相对升降: {direction} {distance_m:.3f}m @ {speed}m/s"):
            return

        cmd = f"ssh -n -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 {SSH_USER}@{SSH_HOST} '{SDK_SIMPLE_BIN} {ROBOT_IF} {direction} {speed} {duration}' </dev/null"
        log.info(f"相对升降: {direction} {distance_m:.3f}m", extra={"height": distance_m})
        try:
            res = __import__('subprocess').run(cmd, shell=True, capture_output=True, text=True, timeout=duration+10)
            if res.returncode != 0:
                log.warning(f"相对升降返回非零: {res.stderr.strip()}")
            else:
                log.info("相对升降完成")
        except __import__('subprocess').TimeoutExpired:
            log.warning("相对升降命令超时")

        self._check_physical_height_boundary()

    def _check_physical_height_boundary(self):
        """检查当前物理高度是否在有效范围内"""
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._column_height is None:
            return
        physical_height = get_physical_height(self._column_height, self._lift_offset)
        if physical_height < -0.01:
            log.warning(f"物理高度低于最低点: {physical_height:.3f}m", extra={"height": physical_height})
        elif physical_height > self.full_travel + 0.01:
            log.warning(f"物理高度超过最高点: {physical_height:.3f}m", extra={"height": physical_height})

    # ==================== YOLO 抓取 ====================

    def _yolo_pick(self):
        label = self.pick_instruction
        log.info(f"获取 {label} 的YOLO抓取点...")
        resp = http_get(f"{YOLO_XYZ_URL}?label={label}", timeout=10,
                        max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
        if resp.status_code != 200:
            raise RuntimeError("YOLO请求失败")
        data = resp.json()
        if "XiongMao" in label:
            point = data["box_head_point_above_xyz_mm"]
            offset = [20.0, 0.0, 0.0]
        else:
            point = data["center_above_xyz_mm"]
            offset = [0.0, 0.0, 0.0]
        pick_mm = [p + o for p, o in zip(point, offset)]
        log.info(f"YOLO抓取点(mm): {pick_mm}")
        cam_to_base_t = self.cam_to_base_t
        pick_base = [
            pick_mm[2]/1000.0 + cam_to_base_t[0],
            -pick_mm[0]/1000.0 + cam_to_base_t[1],
            -pick_mm[1]/1000.0 + cam_to_base_t[2]
        ]
        self._arm_http("PICK", json.dumps({"position": pick_base}))

    # ==================== 距离验证 ====================

    def _verify_distance(self):
        resp = http_get(YOLO_XYZ_URL, timeout=10,
                        max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
        if resp.status_code != 200:
            raise RuntimeError("YOLO服务不可用")
        dist_mm = resp.json().get("range_from_left_camera_mm", 9999)
        log.info(f"当前烟盒距离: {dist_mm}mm")
        if dist_mm > 600:
            raise RuntimeError(f"距离过远({dist_mm}mm)")

    # ==================== 微调重试 ====================

    def _adjust_retry(self):
        for i in range(3):
            self._weitiao_enhanced()
            log.info(f"微调尝试 {i+1}/3 完成")

    # ==================== 绝对升降（offset 补偿 + 高度验证）====================

    def _lift_to(self, height_m):
        if height_m < 0 or height_m > self.full_travel:
            log.warning(f"目标物理高度 {height_m:.3f}m 超出 [0, {self.full_travel}]m，放弃升降",
                        extra={"height": height_m})
            return

        sdk_target = height_m + self._lift_offset
        log.info(f"升降: 物理目标 {height_m:.3f}m | offset {self._lift_offset:.4f}m | SDK 下发 {sdk_target:.4f}m",
                 extra={"height": height_m, "offset": self._lift_offset})

        if dry_run_skip(f"升降到物理高度 {height_m:.3f}m (SDK: {sdk_target:.4f}m)"):
            return

        cmd = f"ssh -n -o ConnectTimeout=10 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 {SSH_USER}@{SSH_HOST} '{SDK_HEIGHT_BIN} {ROBOT_IF} {sdk_target:.4f}' </dev/null"
        try:
            res = __import__('subprocess').run(cmd, shell=True, capture_output=True, text=True, timeout=self.lift_timeout)
            if res.returncode != 0:
                log.warning(f"升降命令返回非零: {res.stderr.strip()}")
            else:
                log.info("升降完成")
        except __import__('subprocess').TimeoutExpired:
            log.warning(f"升降命令超时（{self.lift_timeout}秒），但任务将继续")

        self._verify_lift_height(height_m)

    def _verify_lift_height(self, target_physical_m, tolerance=0.01):
        """升降完成后验证实际高度"""
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._column_height is not None:
                break
        if self._column_height is None:
            log.warning("无法获取当前高度，跳过验证")
            return
        physical_height = get_physical_height(self._column_height, self._lift_offset)
        error = abs(physical_height - target_physical_m)
        if error > tolerance:
            log.warning(f"高度验证偏差较大: 期望 {target_physical_m:.3f}m, 实际 {physical_height:.3f}m, 偏差 {error:.4f}m",
                        extra={"height": physical_height})
        else:
            log.info(f"高度验证通过: {physical_height:.3f}m (偏差 {error:.4f}m)",
                     extra={"height": physical_height})

    # ==================== 显示 offset ====================

    def _show_offset(self):
        offset = self._lift_offset
        print(f"当前 offset: {offset:.4f}m")
        print(f"物理范围: [0, {self.full_travel}]m")
        print(f"SDK 范围: [{offset:.4f}, {self.full_travel + offset:.4f}]m")

    # ==================== 后退 ====================

    def _backup(self, distance):
        if distance <= 0:
            return
        log.info(f"后退 {distance}m", extra={"distance": distance})
        self._cmd_vel_move(-0.2, distance)

    # ==================== 旋转（odom 闭环）====================

    def _rotate(self, angle):
        if self.rotate_segments > 1:
            seg_angle = float(angle) / self.rotate_segments
            for i in range(self.rotate_segments):
                self._rotate_single(seg_angle)
                if i < self.rotate_segments - 1:
                    time.sleep(0.3)
        else:
            self._rotate_single(float(angle))

    def _rotate_single(self, angle_deg):
        """闭环旋转，正角度为逆时针"""
        target_rad = math.radians(angle_deg)

        if dry_run_skip(f"旋转 {angle_deg:.1f}°"):
            return

        if not self._odom_ready:
            self._rotate_openloop(angle_deg)
            return

        speed = 0.5
        angular_z = speed if angle_deg > 0 else -speed

        initial_yaw = self._get_yaw()
        if initial_yaw is None:
            self._rotate_openloop(angle_deg)
            return

        prev_yaw = initial_yaw
        cumulative = 0.0
        twist = Twist()
        twist.angular.z = angular_z

        log.info(f"闭环旋转: {angle_deg:.1f}°", extra={"angle": angle_deg})
        while abs(cumulative) < abs(target_rad):
            rclpy.spin_once(self, timeout_sec=0.05)
            current_yaw = self._get_yaw()
            if current_yaw is None:
                continue

            delta = current_yaw - prev_yaw
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi
            cumulative += delta
            prev_yaw = current_yaw

            remaining = abs(target_rad) - abs(cumulative)
            if remaining < math.radians(5):
                twist.angular.z = angular_z * 0.3
            elif remaining < math.radians(15):
                twist.angular.z = angular_z * 0.6

            self._cmd_vel_pub.publish(twist)
            time.sleep(0.02)

        twist.angular.z = 0.0
        self._cmd_vel_pub.publish(twist)
        log.info(f"旋转完成，实际 {math.degrees(cumulative):.1f}°",
                 extra={"angle": math.degrees(cumulative)})

    def _rotate_openloop(self, angle_deg):
        """开环旋转后备"""
        speed = 0.5
        rad = math.radians(float(angle_deg))
        dur = abs(rad) / speed
        angular_z = speed if rad > 0 else -speed
        log.info(f"开环旋转: {angle_deg:.1f}° 持续 {dur:.1f}s", extra={"angle": angle_deg})
        self._cmd_vel_twist(linear_x=0.0, angular_z=angular_z, duration=dur)

    # ==================== 移动（odom 闭环）====================

    def _move(self, params):
        parts = [p.strip() for p in params.split(",")]
        if len(parts) < 2:
            log.warning("move 参数格式: 方向,距离")
            return
        direction = parts[0].lower()
        if direction not in ("forward", "backward"):
            log.warning("方向仅支持 forward 或 backward")
            return
        distance = float(parts[1])
        linear_x = 0.2 if direction == "forward" else -0.2
        log.info(f"移动: {direction} {distance}m", extra={"distance": distance})
        self._cmd_vel_move(linear_x, distance)

    def _cmd_vel_move(self, linear_x, distance):
        speed = abs(linear_x)
        if speed <= 0 or abs(distance) < 0.05:
            log.warning("距离过小，忽略")
            return

        if dry_run_skip(f"移动 {'前进' if linear_x > 0 else '后退'} {distance:.3f}m"):
            return

        if self._odom_ready:
            self._cmd_vel_move_odom(linear_x, distance)
        else:
            dur = abs(distance) / speed
            if dur < 0.4:
                dur = 0.4
            self._cmd_vel_twist(linear_x=linear_x, angular_z=0.0, duration=dur)

    def _cmd_vel_move_odom(self, linear_x, distance):
        """前后闭环移动"""
        ix, iy = self._get_position()
        if ix is None:
            self._cmd_vel_twist(linear_x=linear_x, angular_z=0.0, duration=abs(distance)/abs(linear_x))
            return
        iyaw = self._get_yaw()
        if iyaw is None:
            self._cmd_vel_twist(linear_x=linear_x, angular_z=0.0, duration=abs(distance)/abs(linear_x))
            return

        dir_x = math.cos(iyaw)
        dir_y = math.sin(iyaw)
        twist = Twist()
        twist.linear.x = linear_x

        log.info(f"闭环移动: {'前进' if linear_x > 0 else '后退'} {distance:.3f}m")
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            cx, cy = self._get_position()
            if cx is None:
                continue
            dx = cx - ix
            dy = cy - iy
            dist_moved = dx * dir_x + dy * dir_y

            if abs(dist_moved) >= abs(distance):
                break

            remaining = abs(distance) - abs(dist_moved)
            if remaining < 0.05:
                twist.linear.x = linear_x * 0.3
            elif remaining < 0.15:
                twist.linear.x = linear_x * 0.6

            self._cmd_vel_pub.publish(twist)
            time.sleep(0.02)

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        self._cmd_vel_pub.publish(twist)
        log.info("移动完成")

    # ==================== 底盘底层控制 ====================

    def _cmd_vel_twist(self, linear_x, angular_z, duration):
        """开环定时发送 cmd_vel（使用本地 Publisher，不依赖 shell ros2 topic pub）"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        rate = 20  # Hz
        total_steps = int(duration * rate)
        period = 1.0 / rate
        log.info(f"[底盘] 开环 cmd_vel: linear_x={linear_x}, angular_z={angular_z}, duration={duration}s, rate={rate}Hz")
        for _ in range(total_steps):
            self._cmd_vel_pub.publish(twist)
            time.sleep(period)
        # 发送停转指令
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self._cmd_vel_pub.publish(twist)

    # ==================== 夹爪 ====================

    def _gripper(self, action):
        if action == "suck":
            http_post(f"{GRIPPER_URL}/api/v1/command", payload={"command": "suck", "wait": False},
                      max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
        elif action == "release":
            http_post(f"{GRIPPER_URL}/api/v1/command", payload={"command": "release", "wait": False},
                      max_retries=self.http_max_retries, retry_delay=self.http_retry_delay)
        elif action == "status":
            print(http_get(f"{GRIPPER_URL}/api/v1/status",
                           max_retries=self.http_max_retries, retry_delay=self.http_retry_delay).text)


def main():
    parser = argparse.ArgumentParser(description='G1D 步骤执行器 V6.0')
    parser.add_argument("step", help="步骤名称")
    parser.add_argument("--pose", help="导航目标 (JSON)")
    parser.add_argument("--target", help="目标物体名称")
    parser.add_argument("--distance", type=float, help="距离(米)")
    parser.add_argument("--angle", type=float, help="角度(度)")
    parser.add_argument("--params", help="复合参数 (如 forward,0.5 或 190,0.02)")
    parser.add_argument("--action", help="夹爪动作 (suck/release/status)")
    parser.add_argument("--height", type=float, help="高度(米)")
    parser.add_argument("--direction", help="升降方向 (up/down)")
    parser.add_argument("--dry-run", action="store_true", help="Dry-run 模式（仅打印不执行）")
    args = parser.parse_args()

    if args.dry_run:
        import os
        os.environ["G1D_DRY_RUN"] = "1"
        # 需要重新导入以使 DRY_RUN 生效
        import g1d_common
        g1d_common.DRY_RUN = True

    _ros2_initialized_here = False
    try:
        if not rclpy.ok():
            rclpy.init()
            _ros2_initialized_here = True
    except Exception:
        pass

    executor = StepExecutor()
    try:
        if args.pose:
            pose = json.loads(args.pose)
            if args.step in ("nav_start", "nav_start_lift"):
                executor.start_pose = pose
            elif args.step in ("nav_end", "nav_end_lift"):
                executor.end_pose = pose
        if args.target:
            executor.pick_instruction = args.target

        kwargs = {}
        for attr in ["distance", "angle", "params", "action", "height", "direction"]:
            val = getattr(args, attr, None)
            if val is not None:
                kwargs[attr] = val

        notify(f"步骤开始: {args.step} {kwargs}")
        executor.run_step(args.step, **kwargs)
        notify(f"步骤完成: {args.step}")
    except Exception as e:
        notify(f"步骤失败: {args.step} - {e}", level="ERROR")
        raise
    finally:
        executor.destroy_node()
        if _ros2_initialized_here:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
