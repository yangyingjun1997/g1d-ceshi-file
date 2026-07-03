#!/usr/bin/env python3
"""
G1D 共享模块
- 集中定义常量（URL、路径、物理参数）
- 共享函数（offset 读写、yaw 计算、机械臂进程检查、参数加载）
- 共享导入（HispeedMsg）
- 结构化日志（logging 模块）
- 远程通知（webhook）
- Dry-run 支持
"""
import json, math, os, sys, subprocess, time, tempfile, configparser, logging

# 自动安装缺失的第三方依赖
try:
    import requests
except ImportError:
    print("[g1d_common] requests 未安装，正在自动安装...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "requests", "-q"])
    import requests

# ==================== 常量 ====================

# 自动检测项目根目录：g1d_common.py 在 lib/ 下，上两级就是项目根
_G1D_HOME = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_HOME = _G1D_HOME

# 文件路径
LIFT_OFFSET_FILE = os.path.join(PROJECT_HOME, "conf", "lift_offset.json")
TASK_PROGRESS_FILE = "/tmp/current_task.json"
LAST_ARM_STATUS_FILE = "/tmp/last_arm_status.json"

# 立柱物理参数
FULL_TRAVEL = 0.427

# 远程主机（模块加载时从配置读取，后续可通过 load_ssh_config() 刷新）
def _init_ssh_defaults():
    """初始化 SSH 默认值（从 params.ini 读取）"""
    net = load_network_config()
    ssh_cfg = load_ssh_config()
    if_name = net.get("robot_if", "eth0")
    # SSH_HOST 指向机器人本体（164端），供升降控制等使用
    host_164 = net.get("host_164", "192.168.123.164")
    user_164 = ssh_cfg.get("ssh_user_164", net.get("ssh_user", "unitree"))
    return user_164, host_164, if_name

SSH_USER, SSH_HOST, ROBOT_IF = _init_ssh_defaults()

# SDK 二进制路径
SDK_PATH = "~/unitree_sdk2/build/bin"
SDK_HEIGHT_BIN = f"{SDK_PATH}/g1d_height_control"
SDK_SIMPLE_BIN = f"{SDK_PATH}/g1d_simple_control"

# HTTP 服务 URL
GRIPPER_URL = "http://192.168.123.164:18080"
ARM_API_URL = "http://192.168.123.164:18083/api/actions/execute"
ARM_HEALTH_URL = "http://192.168.123.164:18083/api/health"
YOLO_XYZ_URL = "http://192.168.123.164:18081/xyz"
YOLO_HEALTH_URL = "http://192.168.123.164:18081/health"
ADJUST_URL = "http://192.168.123.164:18084/adjust"
ADJUST_HEALTH_URL = "http://192.168.123.164:18084/health"
NAV_API_URL = "http://127.0.0.1:8080"

SDK_PARAMS_FILE = os.path.join(PROJECT_HOME, "conf", "params.ini")
POINT_INF_FILE = os.path.join(PROJECT_HOME, "conf", "point_inf")
TASK_INI_FILE = os.path.join(PROJECT_HOME, "conf", "task_list.ini")

# 默认参数
DEFAULT_NEAR_EDGE_MIN = 150.0
DEFAULT_NEAR_EDGE_MAX = 250.0
DEFAULT_HEIGHT_THRESHOLD_MM = 50.0
DEFAULT_SEARCH_STEP_HEIGHT = 0.05
DEFAULT_SEARCH_BACKUP = 0.2
DEFAULT_MAX_SEARCH = 3
DEFAULT_MAX_ADJUST_RETRIES = 2
DEFAULT_SMALL_DEVIATION_MM = 50.0
DEFAULT_ENHANCED_MAX_LOOPS = 5
DEFAULT_LIFT_TIMEOUT = 30
DEFAULT_ARM_MAX_RETRIES = 2
DEFAULT_ARM_RETRY_DELAY = 1.5
DEFAULT_ROTATE_SEGMENTS = 1
DEFAULT_HTTP_MAX_RETRIES = 2
DEFAULT_HTTP_RETRY_DELAY = 1.0
DEFAULT_CAM_TO_BASE = [0.3, 0.0, 0.2]
DEFAULT_STEP_TIMEOUT = 180

# Webhook 通知 URL（为空则不发送）
WEBHOOK_URL = os.environ.get("G1D_WEBHOOK_URL", "")

# ==================== HispeedMsg 共享导入 ====================

try:
    from geometry_msgs.msg import Vector3 as HispeedMsg
except ImportError:
    HispeedMsg = None

# ==================== 结构化日志 ====================

def setup_logger(name, level=logging.INFO):
    """创建结构化 JSON 日志 logger"""
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger
    logger.setLevel(level)
    handler = logging.StreamHandler()
    handler.setLevel(level)

    class JsonFormatter(logging.Formatter):
        def format(self, record):
            import json as _json
            log_entry = {
                "ts": self.formatTime(record, self.datefmt),
                "level": record.levelname,
                "module": record.name,
                "msg": record.getMessage()
            }
            # 附加自定义字段
            for key in ("height", "offset", "angle", "distance", "step",
                        "task", "duration", "retcode", "error", "webhook_status"):
                val = getattr(record, key, None)
                if val is not None:
                    log_entry[key] = val
            return _json.dumps(log_entry, ensure_ascii=False)

    formatter = JsonFormatter(datefmt="%Y-%m-%dT%H:%M:%S")
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    return logger

# 全局 logger
log = setup_logger("g1d")

# ==================== Dry-run 支持 ====================

DRY_RUN = os.environ.get("G1D_DRY_RUN", "").lower() in ("1", "true", "yes")

def dry_run_skip(action_desc):
    """如果是 dry-run 模式，打印动作描述并返回 True（跳过实际执行）"""
    if DRY_RUN:
        log.info(f"[DRY-RUN] {action_desc}")
        return True
    return False

# ==================== offset 读写 ====================

# 底盘断电判定：两台机器启动时间差在此范围内视为同一次上电
CHASSIS_BOOT_TIME_DIFF_SEC = 30
# 两台机器 uptime 都小于此值时才做判定（避免两台都运行很久但偶然接近的误判）
CHASSIS_BOOT_MAX_UPTIME = 300  # 5分钟

def read_lift_offset():
    """读取开机时检测到的立柱 offset"""
    try:
        with open(LIFT_OFFSET_FILE, 'r') as f:
            data = json.load(f)
        return float(data.get('offset', 0.0))
    except Exception:
        return 0.0

def is_chassis_power_cycled(my_uptime_sec, robot_uptime_sec):
    """判断底盘是否断电重启：两台机器同源供电，启动时间接近说明同一次上电

    条件：两台 uptime 都 < 300s 且启动时间差 < 30s → 底盘刚上电，需要重新检测offset
    """
    if my_uptime_sec > CHASSIS_BOOT_MAX_UPTIME or robot_uptime_sec > CHASSIS_BOOT_MAX_UPTIME:
        return False  # 至少一台已运行很久，不是刚上电
    my_boot_time = time.time() - my_uptime_sec
    robot_boot_time = time.time() - robot_uptime_sec
    return abs(my_boot_time - robot_boot_time) < CHASSIS_BOOT_TIME_DIFF_SEC

def save_lift_offset(offset_value):
    """原子写入 offset 到文件"""
    try:
        data = {"offset": offset_value, "timestamp": time.time()}
        tmp_fd, tmp_path = tempfile.mkstemp(
            dir=os.path.dirname(LIFT_OFFSET_FILE), suffix='.json'
        )
        with os.fdopen(tmp_fd, 'w') as f:
            json.dump(data, f)
        os.replace(tmp_path, LIFT_OFFSET_FILE)
        log.info(f"立柱 offset 已保存: {offset_value:.4f}m", extra={"offset": offset_value})
    except Exception as e:
        log.error(f"写入 offset 文件失败: {e}", extra={"error": str(e)})

def get_physical_height(hispeed_y, offset=None):
    """计算物理高度"""
    if offset is None:
        offset = read_lift_offset()
    return hispeed_y - offset

# ==================== ROS2 工具函数 ====================

def yaw_from_odom(odom_data):
    """从 Odometry 消息计算 yaw 角"""
    if odom_data is None:
        return None
    q = odom_data.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def position_from_odom(odom_data):
    """从 Odometry 消息提取位置"""
    if odom_data is None:
        return None, None
    return odom_data.pose.pose.position.x, odom_data.pose.pose.position.y

# ==================== 机械臂进程检查 ====================

def check_arm_process():
    """检查机械臂进程 arm_task_node.py 是否在运行"""
    try:
        result = subprocess.run(
            ["pgrep", "-f", "arm_task_node.py"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2
        )
        return result.returncode == 0
    except Exception:
        return False

# ==================== 参数加载 ====================

def _read_config():
    """内部：读取配置文件并返回 ConfigParser 对象"""
    config = configparser.ConfigParser()
    if os.path.exists(SDK_PARAMS_FILE):
        try:
            config.read(SDK_PARAMS_FILE)
        except Exception as e:
            log.warn(f"读取配置文件失败: {e}")
    return config


def load_network_config():
    """加载网络拓扑配置（IP地址、SSH等），返回字典"""
    config = _read_config()
    defaults = {
        "host_5": "192.168.123.5",
        "host_164": "192.168.123.164",
        "host_4090": "192.168.100.100",
        "ssh_user": "unitree",
        "robot_if": "eth0",
    }
    if config.has_section("network"):
        for k in defaults:
            try:
                val = config.get("network", k, fallback=defaults[k])
                defaults[k] = val.strip()
            except Exception:
                pass
    return defaults


def load_service_config():
    """加载服务端口配置，返回字典"""
    config = _read_config()
    defaults = {
        "port_monitor": 28087,
        "port_control_api": 28091,
        "port_offset_detector": 28089,
        "port_dashboard": 28090,
        "port_service_dashboard": 28092,
        "port_yolo": 18081,
        "port_arm": 18083,
        "port_adjust": 18084,
        "port_gripper": 18080,
        "port_nav": 8080,
    }
    if config.has_section("service"):
        for k in defaults:
            try:
                defaults[k] = config.getint("service", k, fallback=defaults[k])
            except Exception:
                pass
    return defaults


def load_db_config():
    """加载数据库配置，返回字典（可直接传给 pymysql.connect）"""
    config = _read_config()
    db_cfg = {
        "host": os.environ.get("MYSQL_HOST", "127.0.0.1"),
        "port": int(os.environ.get("MYSQL_PORT", "3306")),
        "user": os.environ.get("MYSQL_USER", "bwton"),
        "password": os.environ.get("MYSQL_PASSWORD", "bwton@888"),
        "database": os.environ.get("MYSQL_DB", "digitaltwins"),
        "charset": "utf8mb4",
        "cursorclass": None,  # 由调用方设置
    }
    if config.has_section("database"):
        for k in ("host", "user", "password", "database"):
            try:
                val = config.get("database", k, fallback=db_cfg[k])
                db_cfg[k] = val.strip()
            except Exception:
                pass
        try:
            db_cfg["port"] = config.getint("database", "port", fallback=db_cfg["port"])
        except Exception:
            pass
    # 环境变量优先级最高（覆盖 ini）
    for k in ("host", "port", "user", "password", "database"):
        env_key = f"MYSQL_{k.upper()}"
        if env_key in os.environ:
            val = os.environ[env_key]
            if k == "port":
                db_cfg[k] = int(val)
            else:
                db_cfg[k] = val
    return db_cfg


def load_ros2_config():
    """加载 ROS2 配置，返回字典"""
    config = _read_config()
    ros2_cfg = {
        "topic_hispeed": "/hispeed_state",
        "topic_odom": "/agv/odom",
        "topic_cmd_vel": "/cmd_vel",
        "domain_id": 0,
    }
    if config.has_section("ros2"):
        for k in ("topic_hispeed", "topic_odom", "topic_cmd_vel"):
            try:
                ros2_cfg[k] = config.get("ros2", k, fallback=ros2_cfg[k]).strip()
            except Exception:
                pass
        try:
            ros2_cfg["domain_id"] = config.getint("ros2", "domain_id", fallback=ros2_cfg["domain_id"])
        except Exception:
            pass
    return ros2_cfg


def load_ssh_config():
    """加载每设备独立 SSH 用户配置，返回字典 {ssh_user_5, ssh_user_164, ssh_user_4090}"""
    config = _read_config()
    net = load_network_config()
    default_user = net.get("ssh_user", "unitree")
    ssh_cfg = {
        "ssh_user_5": default_user,
        "ssh_user_164": default_user,
        "ssh_user_4090": default_user,
    }
    if config.has_section("ssh"):
        for k in ssh_cfg:
            try:
                val = config.get("ssh", k, fallback=ssh_cfg[k])
                ssh_cfg[k] = val.strip()
            except Exception:
                pass
    return ssh_cfg


def get_ssh_params_for_host(host_ip):
    """根据 IP 地址返回对应的 (ssh_user, ssh_host)"""
    net = load_network_config()
    ssh_cfg = load_ssh_config()
    host_5 = net.get("host_5", "192.168.123.5")
    host_164 = net.get("host_164", "192.168.123.164")
    host_4090 = net.get("host_4090", "192.168.100.100")
    if host_ip == host_5:
        return ssh_cfg.get("ssh_user_5", net.get("ssh_user", "robot")), host_5
    elif host_ip == host_164:
        return ssh_cfg.get("ssh_user_164", net.get("ssh_user", "unitree")), host_164
    elif host_ip == host_4090:
        return ssh_cfg.get("ssh_user_4090", net.get("ssh_user", "ubuntu")), host_4090
    else:
        return net.get("ssh_user", "unitree"), host_ip


def build_urls_from_config():
    """根据网络和服务配置构建所有 HTTP URL，返回字典"""
    net = load_network_config()
    svc = load_service_config()

    host_164 = net["host_164"]
    host_5 = net["host_5"]

    return {
        "GRIPPER_URL": f"http://{host_164}:{svc['port_gripper']}",
        "ARM_API_URL": f"http://{host_164}:{svc['port_arm']}/api/actions/execute",
        "ARM_HEALTH_URL": f"http://{host_164}:{svc['port_arm']}/api/health",
        "YOLO_XYZ_URL": f"http://{host_164}:{svc['port_yolo']}/xyz",
        "YOLO_HEALTH_URL": f"http://{host_164}:{svc['port_yolo']}/health",
        "ADJUST_URL": f"http://{host_164}:{svc['port_adjust']}/adjust",
        "ADJUST_HEALTH_URL": f"http://{host_164}:{svc['port_adjust']}/health",
        "NAV_API_URL": f"http://127.0.0.1:{svc['port_nav']}",
        "MONITOR_URL": f"http://{host_5}:{svc['port_monitor']}",
        "CONTROL_API_URL": f"http://{host_5}:{svc['port_control_api']}",
        "CONTROL_HEALTH_URL": f"http://{host_5}:{svc['port_control_api']}/health",
        "OFFSET_URL": f"http://{host_164}:{svc['port_offset_detector']}/api/basic_status",
        "DASHBOARD_URL": f"http://{net['host_4090']}:{svc['port_dashboard']}",
        "SERVICE_DASHBOARD_URL": f"http://{host_5}:{svc['port_service_dashboard']}",
    }


def load_sdk_params():
    """加载 sdk_params.ini，返回字典"""
    params = {
        "near_edge_min": DEFAULT_NEAR_EDGE_MIN,
        "near_edge_max": DEFAULT_NEAR_EDGE_MAX,
        "height_threshold_mm": DEFAULT_HEIGHT_THRESHOLD_MM,
        "search_step_height": DEFAULT_SEARCH_STEP_HEIGHT,
        "search_backup": DEFAULT_SEARCH_BACKUP,
        "max_search": DEFAULT_MAX_SEARCH,
        "max_adjust_retries": DEFAULT_MAX_ADJUST_RETRIES,
        "small_deviation_mm": DEFAULT_SMALL_DEVIATION_MM,
        "weitiao_mode": "enhanced",
        "enhanced_max_loops": DEFAULT_ENHANCED_MAX_LOOPS,
        "lift_timeout": DEFAULT_LIFT_TIMEOUT,
        "arm_max_retries": DEFAULT_ARM_MAX_RETRIES,
        "arm_retry_delay": DEFAULT_ARM_RETRY_DELAY,
        "rotate_segments": DEFAULT_ROTATE_SEGMENTS,
        "http_max_retries": DEFAULT_HTTP_MAX_RETRIES,
        "http_retry_delay": DEFAULT_HTTP_RETRY_DELAY,
        "full_travel": FULL_TRAVEL,
        "cam_to_base_t": list(DEFAULT_CAM_TO_BASE),
        "step_timeout": DEFAULT_STEP_TIMEOUT,
        "rotate_speed": 0.5,   # rad/s
        "move_speed": 0.2,     # m/s
    }
    if not os.path.exists(SDK_PARAMS_FILE):
        return params

    config = configparser.ConfigParser()
    try:
        config.read(SDK_PARAMS_FILE)
        if config.has_section("sdk_params"):
            s = "sdk_params"
            params["near_edge_min"] = config.getfloat(s, "near_edge_min_mm", fallback=DEFAULT_NEAR_EDGE_MIN)
            params["near_edge_max"] = config.getfloat(s, "near_edge_max_mm", fallback=DEFAULT_NEAR_EDGE_MAX)
            params["height_threshold_mm"] = config.getfloat(s, "height_threshold_mm", fallback=DEFAULT_HEIGHT_THRESHOLD_MM)
            params["search_step_height"] = config.getfloat(s, "search_step_height_m", fallback=DEFAULT_SEARCH_STEP_HEIGHT)
            params["search_backup"] = config.getfloat(s, "search_backup_m", fallback=DEFAULT_SEARCH_BACKUP)
            params["max_search"] = config.getint(s, "max_search_attempts", fallback=DEFAULT_MAX_SEARCH)
            params["max_adjust_retries"] = config.getint(s, "max_adjust_retries", fallback=DEFAULT_MAX_ADJUST_RETRIES)
            params["small_deviation_mm"] = config.getfloat(s, "small_deviation_mm", fallback=DEFAULT_SMALL_DEVIATION_MM)
            params["weitiao_mode"] = config.get(s, "weitiao_mode", fallback="enhanced").strip().lower()
            params["enhanced_max_loops"] = config.getint(s, "enhanced_max_loops", fallback=DEFAULT_ENHANCED_MAX_LOOPS)
            params["lift_timeout"] = config.getint(s, "lift_timeout_sec", fallback=DEFAULT_LIFT_TIMEOUT)
            params["arm_max_retries"] = config.getint(s, "arm_max_retries", fallback=DEFAULT_ARM_MAX_RETRIES)
            params["arm_retry_delay"] = config.getfloat(s, "arm_retry_delay_sec", fallback=DEFAULT_ARM_RETRY_DELAY)
            params["rotate_segments"] = config.getint(s, "rotate_segments", fallback=DEFAULT_ROTATE_SEGMENTS)
            params["http_max_retries"] = config.getint(s, "http_max_retries", fallback=DEFAULT_HTTP_MAX_RETRIES)
            params["http_retry_delay"] = config.getfloat(s, "http_retry_delay_sec", fallback=DEFAULT_HTTP_RETRY_DELAY)
            params["full_travel"] = config.getfloat(s, "full_travel_m", fallback=FULL_TRAVEL)
            params["step_timeout"] = config.getint(s, "step_timeout_sec", fallback=DEFAULT_STEP_TIMEOUT)
            params["rotate_speed"] = config.getfloat(s, "rotate_speed", fallback=0.5)
            params["move_speed"] = config.getfloat(s, "move_speed", fallback=0.2)
            params["cam_to_base_t"] = [
                config.getfloat(s, "cam_to_base_x", fallback=0.3),
                config.getfloat(s, "cam_to_base_y", fallback=0.0),
                config.getfloat(s, "cam_to_base_z", fallback=0.2),
            ]
    except Exception as e:
        log.warn(f"读取 sdk_params.ini 失败: {e}")
    return params

# ==================== HTTP 重试 ====================

def http_get(url, params=None, timeout=10, max_retries=None, retry_delay=None):
    """带重试的 HTTP GET"""
    if max_retries is None:
        max_retries = load_sdk_params()["http_max_retries"]
    if retry_delay is None:
        retry_delay = load_sdk_params()["http_retry_delay"]
    for attempt in range(max_retries + 1):
        try:
            return requests.get(url, params=params, timeout=timeout)
        except requests.RequestException as e:
            if attempt < max_retries:
                log.warning(f"HTTP GET 重试 {attempt+1}/{max_retries}: {url} - {e}")
                time.sleep(retry_delay)
            else:
                raise

def http_post(url, payload=None, timeout=10, max_retries=None, retry_delay=None):
    """带重试的 HTTP POST"""
    if max_retries is None:
        max_retries = load_sdk_params()["http_max_retries"]
    if retry_delay is None:
        retry_delay = load_sdk_params()["http_retry_delay"]
    for attempt in range(max_retries + 1):
        try:
            return requests.post(url, json=payload, timeout=timeout)
        except requests.RequestException as e:
            if attempt < max_retries:
                log.warning(f"HTTP POST 重试 {attempt+1}/{max_retries}: {url} - {e}")
                time.sleep(retry_delay)
            else:
                raise

# ==================== 远程通知 ====================

def notify(message, level="INFO"):
    """发送 webhook 通知（如果配置了 WEBHOOK_URL）"""
    if not WEBHOOK_URL:
        return
    try:
        payload = {
            "msgtype": "text",
            "text": {
                "content": f"[G1D {level}] {message}"
            }
        }
        resp = requests.post(WEBHOOK_URL, json=payload, timeout=5)
        log.debug(f"Webhook 通知已发送: {resp.status_code}",
                  extra={"webhook_status": resp.status_code})
    except Exception as e:
        log.warning(f"Webhook 通知失败: {e}")

# ==================== 原子写入 JSON ====================

def atomic_write_json(filepath, data):
    """原子写入 JSON 文件（tempfile + os.replace）"""
    try:
        tmp_fd, tmp_path = tempfile.mkstemp(
            dir=os.path.dirname(filepath), suffix='.json'
        )
        with os.fdopen(tmp_fd, 'w') as f:
            json.dump(data, f, ensure_ascii=False)
        os.replace(tmp_path, filepath)
    except Exception as e:
        # 降级：直接写入
        try:
            with open(filepath, 'w') as f:
                json.dump(data, f, ensure_ascii=False)
        except Exception:
            log.error(f"原子写入失败: {e}", extra={"error": str(e)})

# ==================== SSH 远程命令 ====================

def run_remote_ssh(command_parts, timeout=15, ssh_user=None, ssh_host=None):
    """通过 SSH 执行远程命令（支持指定目标主机用户名）"""
    if ssh_user is None:
        ssh_user = SSH_USER
    if ssh_host is None:
        ssh_host = SSH_HOST
    full_cmd = ["ssh", "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5",
                "-o", "ServerAliveCountMax=2", f"{ssh_user}@{ssh_host}"] + command_parts
    try:
        result = subprocess.run(full_cmd, capture_output=True, text=True, timeout=timeout)
        if result.returncode == 0:
            log.info(f"SSH 命令成功: {' '.join(command_parts)}")
            if result.stdout:
                print(result.stdout.strip())
        else:
            log.warning(f"SSH 命令返回非零: {result.returncode}",
                        extra={"retcode": result.returncode})
            if result.stderr:
                print(result.stderr.strip())
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        log.error(f"SSH 命令超时: {' '.join(command_parts)}")
        return False
    except Exception as e:
        log.error(f"SSH 命令异常: {e}", extra={"error": str(e)})
        return False
