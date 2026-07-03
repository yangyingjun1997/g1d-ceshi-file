#!/usr/bin/env python3
"""
G1D 服务状态面板 V1.0
- 统一展示所有自建服务、外部API、ROS2话题的健康状态
- 每10秒自动检测，故障时浏览器通知+声音报警
- 部署在 192.168.123.5，HTTP 端口 28092
- 所有IP/端口从 sdk_params.ini 统一读取
"""
import os, sys, json, time, threading, subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from datetime import datetime

# 自动安装依赖
try:
    import requests
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "requests", "-q"])
    import requests

# 导入共享配置
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "lib"))
from g1d_common import load_network_config, load_service_config, load_ros2_config

# ===== 从配置文件加载 =====
NET = load_network_config()
SVC = load_service_config()
ROS2 = load_ros2_config()

HTTP_PORT = SVC.get("port_service_dashboard", 28092)
CHECK_INTERVAL = 10  # 秒
REQUEST_TIMEOUT = 5  # 秒

# ===== 服务定义（IP/端口从配置读取）=====
# type: http / ros2_via_api / ssh
SERVICES = [
    # ---- 自建服务 ----
    {
        "name": "状态监控", "group": "自建服务",
        "host": NET["host_5"], "port": SVC["port_monitor"],
        "type": "http", "health_path": "/api/ready",
        "desc": "机器人状态监控与命令代理",
    },
    {
        "name": "控制API", "group": "自建服务",
        "host": NET["host_5"], "port": SVC["port_control_api"],
        "type": "http", "health_path": "/health",
        "desc": "统一控制接口(旋转/移动/升降/抓取)",
    },
    {
        "name": "Offset检测", "group": "自建服务",
        "host": NET["host_164"], "port": SVC["port_offset_detector"],
        "type": "http", "health_path": "/api/basic_status",
        "desc": "164端立柱偏移量自动检测",
    },
    {
        "name": "订单看板", "group": "自建服务",
        "host": NET["host_4090"], "port": SVC["port_dashboard"],
        "type": "http", "health_path": "/api/orders",
        "desc": "G1D烟草分拣订单看板(4090服务器)",
    },
    # ---- 外部API ----
    {
        "name": "YOLO识别", "group": "外部API",
        "host": NET["host_164"], "port": SVC["port_yolo"],
        "type": "http", "health_path": "/",
        "desc": "YOLO目标检测服务",
    },
    {
        "name": "机械臂API", "group": "外部API",
        "host": NET["host_164"], "port": SVC["port_arm"],
        "type": "http", "health_path": "/",
        "desc": "机械臂动作执行接口",
    },
    {
        "name": "微调服务", "group": "外部API",
        "host": NET["host_164"], "port": SVC["port_adjust"],
        "type": "http", "health_path": "/",
        "desc": "机器人位置微调接口",
    },
    {
        "name": "夹爪API", "group": "外部API",
        "host": NET["host_164"], "port": SVC["port_gripper"],
        "type": "http", "health_path": "/",
        "desc": "吸盘夹爪控制接口",
    },
    {
        "name": "导航API", "group": "外部API",
        "host": "127.0.0.1", "port": SVC["port_nav"],
        "type": "http", "health_path": "/",
        "desc": "ROS2导航服务接口",
    },
    # ---- ROS2 (通过 control API /health 间接检测) ----
    {
        "name": ROS2["topic_hispeed"], "group": "ROS2话题",
        "type": "ros2_via_api", "api_url": f"http://{NET['host_5']}:{SVC['port_control_api']}/health",
        "check_key": "hispeed_ready", "topic": ROS2["topic_hispeed"],
        "desc": "立柱编码器高度数据",
    },
    {
        "name": ROS2["topic_odom"], "group": "ROS2话题",
        "type": "ros2_via_api", "api_url": f"http://{NET['host_5']}:{SVC['port_control_api']}/health",
        "check_key": "odom_ready", "topic": ROS2["topic_odom"],
        "desc": "底盘里程计数据",
    },
    {
        "name": ROS2["topic_cmd_vel"], "group": "ROS2话题",
        "type": "ros2_via_api", "api_url": f"http://{NET['host_5']}:{SVC['port_control_api']}/health",
        "check_key": "rclpy_ok", "topic": ROS2["topic_cmd_vel"],
        "desc": "速度控制话题",
    },
    # ---- SSH连通性 ----
    {
        "name": "164 SSH", "group": "系统连通",
        "type": "ssh", "host": NET["host_164"],
        "desc": "机器人本体SSH连通性",
    },
]

# ===== 状态缓存 =====
_service_status = {}
_status_lock = threading.Lock()
_last_check_time = 0
_alert_services = set()  # 当前告警中的服务


def check_http_service(svc):
    """检测HTTP服务健康状态"""
    host = svc.get("host", "127.0.0.1")
    port = svc.get("port", 80)
    path = svc.get("health_path", "/")
    url = f"http://{host}:{port}{path}"
    start = time.time()
    try:
        resp = requests.get(url, timeout=REQUEST_TIMEOUT, allow_redirects=True)
        latency = round((time.time() - start) * 1000)
        if resp.status_code < 500:
            detail = ""
            try:
                data = resp.json()
                # 提取关键信息
                for key in ("ready", "rclpy_ok", "odom_ready", "hispeed_ready",
                            "offset_valid", "physical_height_m", "uptime_sec"):
                    if key in data:
                        detail += f"{key}={data[key]} "
                if not detail and resp.status_code == 200:
                    detail = "服务运行中"
                elif not detail:
                    detail = f"HTTP {resp.status_code} (服务可达)"
            except Exception:
                if resp.status_code == 200:
                    detail = "服务运行中"
                elif resp.status_code == 404:
                    detail = "服务运行中 (404)"
                else:
                    detail = f"HTTP {resp.status_code} (服务可达)"
            return {"status": "up", "latency_ms": latency, "detail": detail.strip()}
        else:
            return {"status": "degraded", "latency_ms": latency, "detail": f"HTTP {resp.status_code}"}
    except requests.Timeout:
        return {"status": "down", "latency_ms": None, "detail": "超时"}
    except requests.ConnectionError:
        return {"status": "down", "latency_ms": None, "detail": "连接拒绝"}
    except Exception as e:
        return {"status": "down", "latency_ms": None, "detail": str(e)[:80]}


def check_ros2_via_api(svc):
    """通过 control API /health 间接检测ROS2话题状态"""
    api_url = svc.get("api_url", "")
    check_key = svc.get("check_key", "")
    topic = svc.get("topic", "")
    start = time.time()
    try:
        resp = requests.get(api_url, timeout=REQUEST_TIMEOUT)
        latency = round((time.time() - start) * 1000)
        if resp.status_code == 200:
            data = resp.json()
            is_ready = data.get(check_key, False)
            if is_ready:
                return {"status": "up", "latency_ms": latency, "detail": f"话题数据正常 ({check_key}=True)"}
            else:
                return {"status": "down", "latency_ms": latency, "detail": f"话题无数据 ({check_key}=False)"}
        else:
            return {"status": "down", "latency_ms": latency, "detail": f"API返回 HTTP {resp.status_code}"}
    except requests.ConnectionError:
        return {"status": "down", "latency_ms": None, "detail": "control API不可达"}
    except Exception as e:
        return {"status": "down", "latency_ms": None, "detail": str(e)[:60]}


def check_ros2_topic(svc):
    """检测ROS2话题是否有发布者(备用，需ros2 CLI)"""
    topic = svc.get("topic", "")
    try:
        env = os.environ.copy()
        env.setdefault("ROS_DOMAIN_ID", "0")
        result = subprocess.run(
            ["ros2", "topic", "info", topic, "--verbose"],
            capture_output=True, text=True, timeout=8, env=env
        )
        output = result.stdout + result.stderr
        pub_count = 0
        sub_count = 0
        for line in output.split("\n"):
            if "Publisher count:" in line or "Subscription count:" in line:
                try:
                    count = int(line.split(":")[-1].strip())
                    if "Publisher" in line:
                        pub_count = count
                    else:
                        sub_count = count
                except ValueError:
                    pass
        if pub_count > 0:
            return {"status": "up", "latency_ms": None, "detail": f"发布者:{pub_count} 订阅者:{sub_count}"}
        else:
            return {"status": "down", "latency_ms": None, "detail": f"无发布者(订阅者:{sub_count})"}
    except subprocess.TimeoutExpired:
        return {"status": "down", "latency_ms": None, "detail": "检测超时"}
    except FileNotFoundError:
        return {"status": "down", "latency_ms": None, "detail": "ros2命令未找到"}
    except Exception as e:
        return {"status": "down", "latency_ms": None, "detail": str(e)[:60]}


def check_ssh(svc):
    """检测SSH连通性"""
    host = svc.get("host", "")
    try:
        result = subprocess.run(
            ["ssh", "-n", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
             f"unitree@{host}", "echo", "ok"],
            capture_output=True, text=True, timeout=8
        )
        if result.returncode == 0:
            return {"status": "up", "latency_ms": None, "detail": "SSH可达"}
        else:
            return {"status": "down", "latency_ms": None, "detail": "SSH认证失败"}
    except subprocess.TimeoutExpired:
        return {"status": "down", "latency_ms": None, "detail": "SSH超时"}
    except Exception as e:
        return {"status": "down", "latency_ms": None, "detail": str(e)[:60]}


def check_all_services():
    """检测所有服务状态"""
    global _last_check_time, _alert_services
    results = {}
    new_alerts = set()

    for svc in SERVICES:
        name = svc["name"]
        svc_type = svc.get("type", "http")

        if svc_type == "http":
            result = check_http_service(svc)
        elif svc_type == "ros2_via_api":
            result = check_ros2_via_api(svc)
        elif svc_type == "ros2":
            result = check_ros2_topic(svc)
        elif svc_type == "ssh":
            result = check_ssh(svc)
        else:
            result = {"status": "unknown", "latency_ms": None, "detail": "未知类型"}

        result["name"] = name
        result["group"] = svc.get("group", "")
        result["desc"] = svc.get("desc", "")
        result["host"] = svc.get("host", "")
        result["port"] = svc.get("port", "")
        if svc_type == "ros2_via_api":
            result["type"] = "ros2"
        else:
            result["type"] = svc_type
        result["check_time"] = datetime.now().strftime("%H:%M:%S")

        if result["status"] == "down":
            new_alerts.add(name)
        results[name] = result

    with _status_lock:
        _service_status.clear()
        _service_status.update(results)
        _last_check_time = time.time()
        _alert_services = new_alerts


def check_loop():
    """后台检测循环"""
    while True:
        try:
            check_all_services()
        except Exception as e:
            print(f"[check_loop] error: {e}")
        time.sleep(CHECK_INTERVAL)


# ===== HTTP Server =====
class ServiceDashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def _json_response(self, data, code=200):
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(data, ensure_ascii=False).encode("utf-8"))

    def _html_response(self, html, code=200):
        self.send_response(code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(html.encode("utf-8"))

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/" or path == "/index.html":
            self._html_response(render_page())
        elif path == "/api/status":
            with _status_lock:
                data = {
                    "services": dict(_service_status),
                    "last_check": _last_check_time,
                    "alert_services": list(_alert_services),
                    "check_interval": CHECK_INTERVAL,
                }
            self._json_response(data)
        else:
            self._json_response({"error": "not found"}, 404)


class ReuseThreadingHTTPServer(HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def render_page():
    return """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>G1D 服务状态面板</title>
<style>
:root{
  --bg-primary:#0d1117;--bg-card:#161b22;--bg-tertiary:#21262d;
  --text-heading:#e6edf3;--text-primary:#c9d1d9;--text-secondary:#8b949e;--text-muted:#484f58;
  --border-primary:#30363d;
  --success:#3fb950;--success-bg:#3fb95020;--success-border:#3fb95040;
  --danger:#f85149;--danger-bg:#f8514920;--danger-border:#f8514940;
  --warning:#d29922;--warning-bg:#d2992220;--warning-border:#d2992240;
  --info:#58a6ff;--info-bg:#58a6ff20;--info-border:#58a6ff40;
  --accent:#7c3aed;--accent-bg:#7c3aed15;
}
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
  background:var(--bg-primary);color:var(--text-primary);min-height:100vh}

.header{padding:16px 24px;display:flex;align-items:center;justify-content:space-between;
  border-bottom:1px solid var(--border-primary);background:var(--bg-card)}
.header h1{font-size:18px;color:var(--text-heading);display:flex;align-items:center;gap:8px}
.header h1 .icon{font-size:22px}
.header-info{display:flex;align-items:center;gap:16px;font-size:12px;color:var(--text-secondary)}
.header-info .timer{font-family:'JetBrains Mono','Fira Code',monospace;color:var(--info)}

.summary{display:flex;gap:12px;padding:12px 24px;background:var(--bg-card);
  border-bottom:1px solid var(--border-primary)}
.summary-card{display:flex;align-items:center;gap:8px;padding:6px 14px;
  border-radius:6px;background:var(--bg-tertiary);font-size:12px;font-weight:500}
.summary-dot{width:8px;height:8px;border-radius:50%;flex-shrink:0}
.summary-dot.up{background:var(--success)}
.summary-dot.down{background:var(--danger)}
.summary-dot.degraded{background:var(--warning)}
.summary-count{font-size:18px;font-weight:700;color:var(--text-heading)}

.content{padding:16px 24px}
.group{margin-bottom:20px}
.group-title{font-size:13px;color:var(--text-secondary);text-transform:uppercase;
  letter-spacing:1px;margin-bottom:8px;display:flex;align-items:center;gap:6px;
  padding-bottom:6px;border-bottom:1px solid var(--border-primary)}
.group-title .count{font-size:11px;background:var(--bg-tertiary);padding:1px 6px;
  border-radius:10px;color:var(--text-muted)}

.service-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:10px}
.svc-card{background:var(--bg-card);border:1px solid var(--border-primary);border-radius:8px;
  padding:12px 14px;transition:all .2s;position:relative;overflow:hidden}
.svc-card:hover{border-color:var(--text-muted);transform:translateY(-1px);
  box-shadow:0 4px 12px rgba(0,0,0,.3)}
.svc-card.status-up{border-left:3px solid var(--success)}
.svc-card.status-down{border-left:3px solid var(--danger);background:var(--danger-bg)}
.svc-card.status-degraded{border-left:3px solid var(--warning)}
.svc-card.status-unknown{border-left:3px solid var(--text-muted)}

.svc-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:6px}
.svc-name{font-size:14px;font-weight:600;color:var(--text-heading);display:flex;align-items:center;gap:6px}
.svc-status{display:flex;align-items:center;gap:4px;font-size:11px;font-weight:600;
  padding:2px 8px;border-radius:10px}
.svc-status.up{background:var(--success-bg);color:var(--success);border:1px solid var(--success-border)}
.svc-status.down{background:var(--danger-bg);color:var(--danger);border:1px solid var(--danger-border)}
.svc-status.degraded{background:var(--warning-bg);color:var(--warning);border:1px solid var(--warning-border)}
.svc-status-dot{width:6px;height:6px;border-radius:50%}
.svc-status-dot.up{background:var(--success);animation:blink 2s infinite}
.svc-status-dot.down{background:var(--danger);animation:blink-fast .5s infinite}
.svc-status-dot.degraded{background:var(--warning)}

.svc-addr{font-size:11px;color:var(--text-secondary);font-family:'JetBrains Mono',monospace;
  margin-bottom:4px;display:flex;align-items:center;gap:4px}
.svc-addr .tag{font-size:9px;background:var(--bg-tertiary);padding:1px 5px;border-radius:3px;
  color:var(--text-muted)}
.svc-desc{font-size:11px;color:var(--text-muted);margin-bottom:4px}
.svc-detail{font-size:10px;color:var(--text-secondary);background:var(--bg-tertiary);
  padding:3px 8px;border-radius:3px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;
  max-width:100%}
.svc-meta{display:flex;gap:8px;margin-top:4px;font-size:10px;color:var(--text-muted)}
.svc-latency{font-family:'JetBrains Mono',monospace}
.svc-latency.fast{color:var(--success)}
.svc-latency.medium{color:var(--warning)}
.svc-latency.slow{color:var(--danger)}

.footer{text-align:center;padding:12px;font-size:10px;color:var(--text-muted);
  border-top:1px solid var(--border-primary);margin-top:20px}

@keyframes blink{0%,100%{opacity:1}50%{opacity:.4}}
@keyframes blink-fast{0%,100%{opacity:1}50%{opacity:.2}}

.empty-state{text-align:center;padding:40px;color:var(--text-muted);font-size:14px}
</style>
</head>
<body>

<div class="header">
  <h1><span class="icon">📡</span> G1D 服务状态面板</h1>
  <div class="header-info">
    <span id="check_timer">等待首次检测...</span>
    <span id="refresh_btn" style="cursor:pointer;color:var(--accent)" onclick="manualRefresh()">🔄 刷新</span>
  </div>
</div>

<div class="summary" id="summary_row"></div>
<div class="content" id="content_area">
  <div class="empty-state">正在检测服务状态...</div>
</div>

<div class="footer">
  G1D Service Dashboard · 自动检测间隔 """ + str(CHECK_INTERVAL) + """秒 · """ + datetime.now().strftime("%Y-%m-%d") + """
</div>

<script>
let prevAlertSet = new Set();

function fetchStatus() {
  fetch('/api/status')
    .then(function(r) { return r.json(); })
    .then(function(data) {
      renderSummary(data);
      renderServices(data);
      checkAlerts(data);
      document.getElementById('check_timer').textContent =
        '上次检测: ' + (data.last_check ? new Date(data.last_check*1000).toLocaleTimeString() : '-') +
        ' · ' + Object.keys(data.services).length + '个服务';
    })
    .catch(function(e) {
      document.getElementById('check_timer').textContent = '获取失败: ' + e.message;
    });
}

function renderSummary(data) {
  const svcs = data.services || {};
  let up=0, down=0, degraded=0, unknown=0;
  for (var k in svcs) {
    var s = svcs[k].status;
    if (s==='up') up++; else if (s==='down') down++; else if (s==='degraded') degraded++; else unknown++;
  }
  var total = up+down+degraded+unknown;
  var html = '';
  html += '<div class="summary-card"><div class="summary-dot up"></div><span class="summary-count">'+up+'</span> 正常</div>';
  if (down > 0) html += '<div class="summary-card"><div class="summary-dot down"></div><span class="summary-count" style="color:var(--danger)">'+down+'</span> 故障</div>';
  if (degraded > 0) html += '<div class="summary-card"><div class="summary-dot degraded"></div><span class="summary-count" style="color:var(--warning)">'+degraded+'</span> 降级</div>';
  if (unknown > 0) html += '<div class="summary-card"><div class="summary-dot" style="background:var(--text-muted)"></div><span class="summary-count">'+unknown+'</span> 未知</div>';
  html += '<div class="summary-card" style="margin-left:auto;color:var(--text-secondary)">共 '+total+' 项</div>';
  document.getElementById('summary_row').innerHTML = html;
}

function renderServices(data) {
  const svcs = data.services || {};
  // 按 group 分组
  var groups = {};
  for (var k in svcs) {
    var s = svcs[k];
    var g = s.group || '其他';
    if (!groups[g]) groups[g] = [];
    groups[g].push(s);
  }
  var html = '';
  var groupOrder = ['自建服务','外部API','ROS2话题','系统连通'];
  // 先渲染已知分组，再渲染其他
  for (var i=0; i<groupOrder.length; i++) {
    var gn = groupOrder[i];
    if (!groups[gn]) continue;
    var items = groups[gn];
    var upCnt = items.filter(function(x){return x.status==='up'}).length;
    html += '<div class="group">';
    html += '<div class="group-title">'+gn+' <span class="count">'+upCnt+'/'+items.length+'</span></div>';
    html += '<div class="service-grid">';
    for (var j=0; j<items.length; j++) {
      html += renderSvcCard(items[j]);
    }
    html += '</div></div>';
    delete groups[gn];
  }
  // 渲染剩余分组
  for (var gn in groups) {
    var items = groups[gn];
    var upCnt = items.filter(function(x){return x.status==='up'}).length;
    html += '<div class="group">';
    html += '<div class="group-title">'+gn+' <span class="count">'+upCnt+'/'+items.length+'</span></div>';
    html += '<div class="service-grid">';
    for (var j=0; j<items.length; j++) {
      html += renderSvcCard(items[j]);
    }
    html += '</div></div>';
  }
  document.getElementById('content_area').innerHTML = html;
}

function renderSvcCard(s) {
  var statusText = s.status==='up' ? '正常' : s.status==='down' ? '故障' : s.status==='degraded' ? '降级' : '未知';
  var addr = '';
  if (s.type === 'http' && s.host && s.port) {
    addr = s.host + ':' + s.port;
  } else if (s.type === 'ros2') {
    addr = s.name;
  } else if (s.type === 'ssh') {
    addr = s.host || '';
  }
  var latencyHtml = '';
  if (s.latency_ms !== null && s.latency_ms !== undefined) {
    var cls = s.latency_ms < 200 ? 'fast' : s.latency_ms < 1000 ? 'medium' : 'slow';
    latencyHtml = '<span class="svc-latency '+cls+'">'+s.latency_ms+'ms</span>';
  }
  var typeLabel = s.type==='http' ? 'HTTP' : s.type==='ros2' ? 'ROS2' : s.type==='ssh' ? 'SSH' : '';
  var html = '<div class="svc-card status-'+s.status+'">';
  html += '<div class="svc-header">';
  html += '<div class="svc-name"><span class="svc-status-dot '+s.status+'"></span>'+esc(s.name)+'</div>';
  html += '<div class="svc-status '+s.status+'">'+statusText+'</div>';
  html += '</div>';
  if (addr) html += '<div class="svc-addr">'+esc(addr)+' <span class="tag">'+typeLabel+'</span></div>';
  if (s.desc) html += '<div class="svc-desc">'+esc(s.desc)+'</div>';
  if (s.detail) html += '<div class="svc-detail">'+esc(s.detail)+'</div>';
  html += '<div class="svc-meta">'+latencyHtml+'<span>'+s.check_time+'</span></div>';
  html += '</div>';
  return html;
}

function esc(s) {
  if (!s) return '';
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}

function checkAlerts(data) {
  var newAlerts = new Set(data.alert_services || []);
  // 新增故障：之前没有，现在有
  var newDown = [];
  newAlerts.forEach(function(name) {
    if (!prevAlertSet.has(name)) newDown.push(name);
  });
  // 故障恢复：之前有，现在没有
  var recovered = [];
  prevAlertSet.forEach(function(name) {
    if (!newAlerts.has(name)) recovered.push(name);
  });
  // 浏览器通知
  if (newDown.length > 0 && Notification.permission === 'granted') {
    new Notification('G1D 服务故障', {body: newDown.join(', ') + ' 故障!'});
    playAlertSound();
  }
  if (recovered.length > 0 && Notification.permission === 'granted') {
    new Notification('G1D 服务恢复', {body: recovered.join(', ') + ' 已恢复'});
  }
  prevAlertSet = newAlerts;
}

function playAlertSound() {
  try {
    var ctx = new (window.AudioContext || window.webkitAudioContext)();
    var osc = ctx.createOscillator(); var gain = ctx.createGain();
    osc.connect(gain); gain.connect(ctx.destination);
    osc.frequency.value = 880; osc.type = 'square';
    gain.gain.value = 0.15;
    osc.start(); osc.stop(ctx.currentTime + 0.2);
    setTimeout(function(){
      var osc2 = ctx.createOscillator(); var gain2 = ctx.createGain();
      osc2.connect(gain2); gain2.connect(ctx.destination);
      osc2.frequency.value = 660; osc2.type = 'square';
      gain2.gain.value = 0.15;
      osc2.start(); osc2.stop(ctx.currentTime + 0.3);
    }, 250);
  } catch(e) {}
}

function manualRefresh() {
  document.getElementById('refresh_btn').textContent = '⏳ 检测中...';
  fetch('/api/status?action=refresh').finally(function() {
    setTimeout(function() {
      document.getElementById('refresh_btn').textContent = '🔄 刷新';
      fetchStatus();
    }, 1000);
  });
}

// 请求通知权限
if ('Notification' in window && Notification.permission === 'default') {
  Notification.requestPermission();
}

// 启动轮询
setInterval(fetchStatus, 3000);
fetchStatus();
</script>
</body>
</html>"""


def main():
    # 首次检测
    print("[service_dashboard] 首次检测服务状态...")
    check_all_services()

    # 启动后台检测线程
    t = threading.Thread(target=check_loop, daemon=True)
    t.start()

    # 启动HTTP服务
    server = ReuseThreadingHTTPServer(("0.0.0.0", HTTP_PORT), ServiceDashboardHandler)
    print(f"[service_dashboard] 服务状态面板已启动: http://0.0.0.0:{HTTP_PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[service_dashboard] 正在停止...")
        server.shutdown()


if __name__ == "__main__":
    main()
