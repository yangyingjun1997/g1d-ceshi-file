#!/usr/bin/env python3
"""
G1D 订单看板 V3.3
- 工作流步骤查询：flow.task_record_id 直接 JOIN runtime_node
- 执行卡片：当前执行中订单的工作流进度展示
- 分页列表：20条/页，默认按创建时间降序
- 3秒轮询刷新
- V3.3 新增: 执行历史Tab, 失败高亮, 暂停刷新, 步骤图标区分,
  浏览器通知, 点击卡片筛选, 导出CSV, 失败声音报警,
  刷新耗时, 暗亮主题切换, 键盘快捷键
- 配置从 sdk_params.ini 统一读取（不依赖 g1d_common）
"""
import os, sys, json, time, threading, math, configparser
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from datetime import datetime, timedelta

# 自动安装依赖
try:
    import pymysql
except ImportError:
    import subprocess as _sp
    _sp.check_call([sys.executable, "-m", "pip", "install", "pymysql", "-q"])
    import pymysql

# 连接池（线程安全复用）
try:
    from queue import Queue
except ImportError:
    from Queue import Queue

# ===== 从 sdk_params.ini 读取配置（独立实现，不依赖 g1d_common）=====
# 查找配置文件：优先当前目录，再找项目目录
_INI_PATHS = [
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "conf", "params.ini"),
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "sdk_params.ini"),
]

def _find_ini():
    for p in _INI_PATHS:
        if os.path.exists(p):
            return p
    return None

_INI_FILE = _find_ini()
_config = configparser.ConfigParser()
if _INI_FILE:
    _config.read(_INI_FILE)

# 数据库配置：环境变量 > ini > 默认值
MYSQL_CONFIG = {
    "host": os.environ.get("MYSQL_HOST", _config.get("database", "host", fallback="127.0.0.1")),
    "port": int(os.environ.get("MYSQL_PORT", _config.get("database", "port", fallback="3306"))),
    "user": os.environ.get("MYSQL_USER", _config.get("database", "user", fallback="bwton")),
    "password": os.environ.get("MYSQL_PASSWORD", _config.get("database", "password", fallback="bwton@888")),
    "database": os.environ.get("MYSQL_DB", _config.get("database", "database", fallback="digitaltwins")),
    "charset": "utf8mb4",
    "cursorclass": pymysql.cursors.DictCursor,
}

# 服务端口
HTTP_PORT = _config.getint("service", "port_dashboard", fallback=28090)

REFRESH_INTERVAL = 3  # 秒
PAGE_SIZE = 20

ORDER_STATUS_MAP = {
    "WAITING": "等待中", "SORTING": "分拣中",
    "ROBOT_RUNNING": "机器人执行中", "RUNNING": "执行中",
    "COMPLETED": "已完成", "SUCCESS": "已完成",
    "FAILED": "失败", "CANCELLED": "已取消",
    "INTERRUPTED": "中断",
}
ORDER_STATUS_COLOR = {
    "WAITING": "#f0ad4e", "SORTING": "#5bc0de",
    "ROBOT_RUNNING": "#0275d8", "RUNNING": "#0275d8",
    "COMPLETED": "#5cb85c", "SUCCESS": "#5cb85c",
    "FAILED": "#d9534f", "CANCELLED": "#999",
    "INTERRUPTED": "#e3b341",
}
ACTIVE_STATUSES = {"ROBOT_RUNNING", "RUNNING", "SORTING"}
FLOW_STATUS_MAP = {
    "WAITING": "等待", "RUNNING": "执行中", "COMPLETED": "完成",
    "FAILED": "失败", "RETRY": "重试中",
}
NODE_STATUS_MAP = {
    "COMPLETED": "完成", "FAILED": "失败", "INTERRUPTED": "中断",
    "RUNNING": "执行中", "PENDING": "等待", "SCHEDULED": "待执行",
    "EXECUTING": "执行中",
}
NODE_STATUS_ICON = {
    "COMPLETED": "✓", "FAILED": "✗", "INTERRUPTED": "⊘",
    "RUNNING": "●", "PENDING": "○", "SCHEDULED": "○", "EXECUTING": "●",
}
NODE_STATUS_COLOR = {
    "COMPLETED": "#5cb85c", "FAILED": "#d9534f", "INTERRUPTED": "#f0ad4e",
    "RUNNING": "#5bc0de", "PENDING": "#666", "SCHEDULED": "#666", "EXECUTING": "#5bc0de",
}

RUNNING_STATES = {"RUNNING", "EXECUTING", "PENDING", "SCHEDULED"}

_cache = {"orders": [], "stats": {}, "total": {}, "last_update": 0, "error": None, "fetch_duration": 0}
_cache_lock = threading.Lock()


# ===== 连接池（复用连接，避免频繁建立）=====
_DB_POOL = Queue(maxsize=5)
_DB_POOL_LOCK = threading.Lock()

def get_db_connection():
    """优先从连接池获取，池空则新建"""
    try:
        conn = _DB_POOL.get_nowait()
        # 检测连接是否仍然有效
        try:
            conn.ping(reconnect=True)
            return conn
        except Exception:
            pass  # 连接已失效，新建
    except Exception:
        pass
    return pymysql.connect(**MYSQL_CONFIG)

def release_db_connection(conn):
    """归还连接到池"""
    if conn is None:
        return
    try:
        _DB_POOL.put_nowait(conn)
    except Exception:
        try:
            conn.close()
        except Exception:
            pass


def _parse_payload_json(payload_str):
    """从 request_payload_json 提取关键信息"""
    if not payload_str:
        return {}
    try:
        data = json.loads(payload_str)
        params = data.get("params", data)
        result = {
            "command_type": params.get("command_type", ""),
        }
        point = params.get("point")
        if point:
            result["point_name"] = point.get("route_point_name", "")
        check_item = params.get("check_item")
        if check_item:
            result["check_item_name"] = check_item.get("check_item_name", "")
            result["action_type"] = check_item.get("action_type", "")
            info = check_item.get("check_item_info_json", {}) or {}
            arm_ctrl = info.get("arm_control") or {}
            if arm_ctrl:
                result["target_object"] = arm_ctrl.get("target_object", "")
                result["target_height_m"] = arm_ctrl.get("target_height_m", "")
                result["arm_phase"] = arm_ctrl.get("phase", "")
        return result
    except Exception:
        return {}


def fetch_orders():
    start_time = time.time()
    conn = None
    try:
        conn = get_db_connection()
        with conn.cursor() as cur:
            # 1. 订单（最多500条）
            cur.execute("""
                SELECT order_item_id, order_date, order_no, brand, brand_code,
                       quantity, sort_level, extend, status, completed_quantity,
                       robot_start_time, robot_end_time, notify_time,
                       create_time, update_time
                FROM dp_cigar_sort_order
                WHERE deleted = 'COMMON'
                ORDER BY create_time DESC
                LIMIT 500
            """)
            rows = cur.fetchall()
            item_ids = [r["order_item_id"] for r in rows]

            # 2. flow 记录
            flow_map = {}
            all_task_record_ids = []
            if item_ids:
                placeholders = ",".join(["%s"] * len(item_ids))
                cur.execute(f"""
                    SELECT flow_id, order_item_id, task_id, task_record_id,
                           status, robot_start_time, robot_end_time,
                           error_message, create_time
                    FROM dp_cigar_sort_order_flow
                    WHERE order_item_id IN ({placeholders})
                      AND deleted = 'COMMON'
                    ORDER BY create_time DESC
                """, item_ids)
                for fr in cur.fetchall():
                    oid = fr["order_item_id"]
                    if oid not in flow_map:
                        flow_map[oid] = []
                    for k in ("robot_start_time", "robot_end_time", "create_time"):
                        v = fr.get(k)
                        if v is not None:
                            fr[k] = str(v)
                    fr["workflows"] = []
                    fr["task_error_message"] = None
                    flow_map[oid].append(fr)
                    trid = fr.get("task_record_id")
                    if trid:
                        all_task_record_ids.append(trid)

            # 3. 工作流节点
            node_map = {}
            if all_task_record_ids:
                unique_trids = list(set(all_task_record_ids))
                ph = ",".join(["%s"] * len(unique_trids))
                cur.execute(f"""
                    SELECT runtime_node_id, runtime_id, task_record_id,
                           node_name, node_type, biz_type, node_status,
                           request_payload_json, error_message,
                           send_time, callback_time, create_time
                    FROM dp_insp_workflow_runtime_node
                    WHERE task_record_id IN ({ph})
                      AND deleted = 'COMMON'
                    ORDER BY create_time ASC
                """, unique_trids)
                for nr in cur.fetchall():
                    trid = nr["task_record_id"]
                    if trid not in node_map:
                        node_map[trid] = []
                    payload_info = _parse_payload_json(nr.get("request_payload_json"))
                    nr["payload_info"] = payload_info
                    for k in ("send_time", "callback_time", "create_time"):
                        v = nr.get(k)
                        if v is not None:
                            nr[k] = str(v)
                    node_map[trid].append(nr)

            # 4. 填充 flow 的 workflows
            for oid, flows in flow_map.items():
                for fr in flows:
                    trid = fr.get("task_record_id")
                    if trid and trid in node_map:
                        fr["workflows"] = node_map[trid]

            # 5. 组装订单
            for row in rows:
                ext = row.get("extend")
                if isinstance(ext, str):
                    try:
                        ext = json.loads(ext)
                    except Exception:
                        ext = {}
                row["extend"] = ext or {}
                row["flows"] = flow_map.get(row["order_item_id"], [])
                row["latest_flow"] = row["flows"][0] if row["flows"] else None
                for k in ("robot_start_time", "robot_end_time", "notify_time",
                          "create_time", "update_time", "order_date"):
                    v = row.get(k)
                    if v is not None:
                        row[k] = str(v)

            # 6. 统计
            cur.execute("""
                SELECT status, COUNT(*) as cnt, SUM(quantity) as total_qty,
                       SUM(completed_quantity) as done_qty
                FROM dp_cigar_sort_order WHERE deleted = 'COMMON' GROUP BY status
            """)
            stats = {}
            for s in cur.fetchall():
                stats[s["status"]] = {
                    "count": s["cnt"],
                    "total_qty": int(s["total_qty"] or 0),
                    "done_qty": int(s["done_qty"] or 0),
                }
            cur.execute("""
                SELECT COUNT(*) as total_orders, SUM(quantity) as total_qty,
                       SUM(completed_quantity) as done_qty
                FROM dp_cigar_sort_order WHERE deleted = 'COMMON'
            """)
            total = cur.fetchone()
            cur.execute("""
                SELECT COUNT(*) as today_orders, SUM(quantity) as today_qty,
                       SUM(completed_quantity) as today_done
                FROM dp_cigar_sort_order
                WHERE deleted = 'COMMON' AND order_date = CURDATE()
            """)
            today = cur.fetchone()

            cur.execute("""
                SELECT COUNT(*) as today_success, SUM(quantity) as today_success_qty
                FROM dp_cigar_sort_order
                WHERE deleted = 'COMMON' AND order_date = CURDATE()
                  AND status IN ('SUCCESS', 'COMPLETED')
            """)
            today_success = cur.fetchone()

            cur.execute("""
                SELECT COUNT(*) as today_failed, SUM(quantity) as today_failed_qty
                FROM dp_cigar_sort_order
                WHERE deleted = 'COMMON' AND order_date = CURDATE()
                  AND status = 'FAILED'
            """)
            today_failed = cur.fetchone()

            cur.execute("""
                SELECT AVG(TIMESTAMPDIFF(SECOND, robot_start_time, robot_end_time)) as avg_duration_sec
                FROM dp_cigar_sort_order
                WHERE deleted = 'COMMON'
                  AND status IN ('SUCCESS', 'COMPLETED')
                  AND robot_start_time IS NOT NULL AND robot_end_time IS NOT NULL
            """)
            avg_duration = cur.fetchone()

            cur.execute("""
                SELECT COUNT(DISTINCT brand) as brand_count
                FROM dp_cigar_sort_order WHERE deleted = 'COMMON'
            """)
            brand_count = cur.fetchone()

            with _cache_lock:
                _cache["fetch_duration"] = round(time.time() - start_time, 2)
                _cache["orders"] = rows
                _cache["stats"] = stats
                _cache["total"] = {
                    "orders": total["total_orders"],
                    "total_qty": int(total["total_qty"] or 0),
                    "done_qty": int(total["done_qty"] or 0),
                    "today_orders": today["today_orders"],
                    "today_qty": int(today["today_qty"] or 0),
                    "today_done": int(today["today_done"] or 0),
                    "today_success": today_success["today_success"],
                    "today_success_qty": int(today_success["today_success_qty"] or 0),
                    "today_failed": today_failed["today_failed"],
                    "today_failed_qty": int(today_failed["today_failed_qty"] or 0),
                    "avg_duration_sec": avg_duration["avg_duration_sec"],
                    "brand_count": brand_count["brand_count"],
                }
                _cache["last_update"] = time.time()
                _cache["error"] = None

    except Exception as e:
        import traceback
        traceback.print_exc()
        with _cache_lock:
            _cache["error"] = str(e)
            _cache["fetch_duration"] = round(time.time() - start_time, 2)
    finally:
        if conn:
            release_db_connection(conn)


def fetch_history(days=7):
    """获取已完成的工作流历史记录（默认最近7天，提升查询性能）"""
    conn = None
    try:
        since_date = (datetime.now() - timedelta(days=days)).strftime("%Y-%m-%d %H:%M:%S")
        conn = get_db_connection()
        with conn.cursor() as cur:
            # Step 1: 找到最近的有终态节点的 runtime_id（限定时间范围）
            cur.execute("""
                SELECT runtime_id, MAX(create_time) as latest_time
                FROM dp_insp_workflow_runtime_node
                WHERE node_status IN ('COMPLETED', 'FAILED', 'INTERRUPTED')
                  AND deleted = 'COMMON'
                  AND create_time >= %s
                GROUP BY runtime_id
                ORDER BY latest_time DESC
                LIMIT 200
            """, [since_date])
            runtime_rows = cur.fetchall()
            if not runtime_rows:
                return []
            runtime_ids = [r["runtime_id"] for r in runtime_rows]

            # Step 2: 获取这些 runtime_id 的所有节点
            ph = ",".join(["%s"] * len(runtime_ids))
            cur.execute(f"""
                SELECT runtime_node_id, runtime_id, task_record_id,
                       node_name, node_type, biz_type, node_status,
                       request_payload_json, error_message,
                       send_time, callback_time, create_time
                FROM dp_insp_workflow_runtime_node
                WHERE runtime_id IN ({ph})
                  AND deleted = 'COMMON'
                ORDER BY create_time ASC
            """, runtime_ids)
            nodes = cur.fetchall()

            # Step 3: 获取订单信息
            task_record_ids = list(set(n["task_record_id"] for n in nodes if n.get("task_record_id")))
            order_info_map = {}
            if task_record_ids:
                ph2 = ",".join(["%s"] * len(task_record_ids))
                cur.execute(f"""
                    SELECT f.task_record_id, f.order_item_id, f.status as flow_status,
                           o.order_no, o.brand, o.brand_code, o.quantity,
                           o.status as order_status, o.extend
                    FROM dp_cigar_sort_order_flow f
                    LEFT JOIN dp_cigar_sort_order o
                        ON f.order_item_id = o.order_item_id AND o.deleted = 'COMMON'
                    WHERE f.task_record_id IN ({ph2})
                      AND f.deleted = 'COMMON'
                """, task_record_ids)
                for r in cur.fetchall():
                    ext = r.get("extend")
                    if isinstance(ext, str):
                        try:
                            ext = json.loads(ext)
                        except Exception:
                            ext = {}
                    order_info_map[r["task_record_id"]] = {
                        "order_no": r.get("order_no", ""),
                        "brand": r.get("brand", ""),
                        "brand_code": r.get("brand_code", ""),
                        "quantity": r.get("quantity", 0),
                        "order_status": r.get("order_status", ""),
                        "customer": (ext or {}).get("customer", ""),
                        "flow_status": r.get("flow_status", ""),
                    }

            # Step 4: 按 runtime_id 分组
            groups = {}
            for n in nodes:
                rid = n["runtime_id"]
                if rid not in groups:
                    groups[rid] = {
                        "runtime_id": str(rid),
                        "task_record_id": str(n.get("task_record_id", "")),
                        "nodes": [],
                        "first_send_time": None,
                        "last_callback_time": None,
                    }
                    trid = n.get("task_record_id")
                    if trid and trid in order_info_map:
                        groups[rid].update(order_info_map[trid])
                    else:
                        groups[rid].update({
                            "order_no": "", "brand": "", "brand_code": "",
                            "quantity": 0, "order_status": "", "customer": "",
                            "flow_status": "",
                        })

                payload_info = _parse_payload_json(n.get("request_payload_json"))
                node_data = {
                    "node_name": n["node_name"] or "",
                    "node_type": n.get("node_type", ""),
                    "biz_type": n.get("biz_type", ""),
                    "node_status": n["node_status"] or "PENDING",
                    "error_message": n.get("error_message", "") or "",
                    "send_time": str(n["send_time"])[:19] if n.get("send_time") else None,
                    "callback_time": str(n["callback_time"])[:19] if n.get("callback_time") else None,
                    "payload_info": payload_info,
                }
                groups[rid]["nodes"].append(node_data)

                st = n.get("send_time")
                ct = n.get("callback_time")
                if st:
                    st_str = str(st)[:19]
                    if groups[rid]["first_send_time"] is None or st_str < groups[rid]["first_send_time"]:
                        groups[rid]["first_send_time"] = st_str
                if ct:
                    ct_str = str(ct)[:19]
                    if groups[rid]["last_callback_time"] is None or ct_str > groups[rid]["last_callback_time"]:
                        groups[rid]["last_callback_time"] = ct_str

            # Step 5: 计算耗时和最终状态
            result = []
            for rid, g in groups.items():
                node_statuses = set(nd["node_status"] for nd in g["nodes"])
                if "FAILED" in node_statuses:
                    g["status"] = "FAILED"
                elif "INTERRUPTED" in node_statuses:
                    g["status"] = "INTERRUPTED"
                else:
                    g["status"] = "COMPLETED"

                if g["first_send_time"] and g["last_callback_time"]:
                    try:
                        start = datetime.strptime(g["first_send_time"][:19], "%Y-%m-%d %H:%M:%S")
                        end = datetime.strptime(g["last_callback_time"][:19], "%Y-%m-%d %H:%M:%S")
                        g["total_duration_sec"] = round((end - start).total_seconds(), 1)
                    except Exception:
                        g["total_duration_sec"] = 0
                else:
                    g["total_duration_sec"] = 0

                result.append(g)

            result.sort(key=lambda x: x.get("first_send_time") or "", reverse=True)
            return result
    except Exception as e:
        import traceback
        traceback.print_exc()
        return []
    finally:
        if conn:
            release_db_connection(conn)


def bg_refresh():
    while True:
        fetch_orders()
        time.sleep(REFRESH_INTERVAL)


def build_html():
    return """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>G1D 订单看板 V3.3</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
html,body{height:100%;overflow:hidden}

/* ===== 主题变量 ===== */
:root,.dark{
  --bg-primary:#0d1117;--bg-card:#161b22;--bg-tertiary:#21262d;--bg-input:#0d1117;
  --border-primary:#30363d;--border-secondary:#21262d;
  --text-primary:#d4dadf;--text-secondary:#a3aaae;--text-muted:#8b949e;--text-dim:#6e7681;--text-heading:#f0f6fc;
  --accent:#58a6ff;--accent-dark:#1f6feb;
  --success:#3fb950;--success-bg:#0d2818;--success-border:#238636;
  --warning:#d29922;--warning-bg:#2a1f00;--warning-border:#d29922;
  --danger:#f85149;--danger-bg:#3d0e0e;--danger-border:#da3633;
  --info:#5bc0de;--info-bg:#0c2d6b;--info-border:#1f6feb;
  --row-hover:#161b22;--row-running:#0c1a2e;
  --exec-gradient:linear-gradient(135deg,#161b22 0%,#0d1926 100%);--exec-border:#1f6feb;
  --scrollbar-track:transparent;--scrollbar-thumb:#30363d;--scrollbar-thumb-hover:#484f58;
  --badge-waiting:#f0ad4e;--badge-sorting:#5bc0de;--badge-running:#0275d8;
  --badge-success:#5cb85c;--badge-failed:#d9534f;--badge-cancelled:#999;--badge-interrupted:#e3b341;
}
.light{
  --bg-primary:#f6f8fa;--bg-card:#ffffff;--bg-tertiary:#eaeef2;--bg-input:#ffffff;
  --border-primary:#d0d7de;--border-secondary:#d8dee4;
  --text-primary:#1f2328;--text-secondary:#656d76;--text-muted:#8b949e;--text-dim:#b1bac4;--text-heading:#1f2328;
  --accent:#0969da;--accent-dark:#0550ae;
  --success:#1a7f37;--success-bg:#dafbe1;--success-border:#1a7f37;
  --warning:#9a6700;--warning-bg:#fff8c5;--warning-border:#9a6700;
  --danger:#cf222e;--danger-bg:#ffebe9;--danger-border:#cf222e;
  --info:#0969da;--info-bg:#ddf4ff;--info-border:#0969da;
  --row-hover:#f6f8fa;--row-running:#ddf4ff;
  --exec-gradient:linear-gradient(135deg,#ffffff 0%,#f0f7ff 100%);--exec-border:#0969da;
  --scrollbar-track:#f6f8fa;--scrollbar-thumb:#d0d7de;--scrollbar-thumb-hover:#b1bac4;
  --badge-waiting:#9a6700;--badge-sorting:#0969da;--badge-running:#0969da;
  --badge-success:#1a7f37;--badge-failed:#cf222e;--badge-cancelled:#656d76;--badge-interrupted:#9a6700;
}

body{font-family:-apple-system,"Segoe UI","Microsoft YaHei",sans-serif;background:var(--bg-primary);color:var(--text-primary);display:flex;flex-direction:column;height:100vh}

/* ===== 顶栏 ===== */
.header{background:var(--bg-card);padding:10px 24px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid var(--border-primary);flex-shrink:0}
.header h1{font-size:20px;font-weight:600;color:var(--accent);letter-spacing:1px}
.header-right{display:flex;align-items:center;gap:10px}
.header .time{color:var(--text-secondary);font-size:15px}
.refresh-dot{width:8px;height:8px;border-radius:50%;background:var(--success);display:inline-block}
.refresh-dot.error{background:var(--danger)}
.icon-btn{background:var(--bg-tertiary);color:var(--text-primary);border:1px solid var(--border-primary);border-radius:6px;padding:4px 8px;cursor:pointer;font-size:16px;line-height:1;transition:all .2s}
.icon-btn:hover{border-color:var(--accent);background:var(--border-primary)}
.pause-indicator{color:var(--warning);font-size:15px;font-weight:600}

/* ===== 帮助提示 ===== */
.help-wrap{position:relative;cursor:help;background:var(--bg-tertiary);color:var(--text-secondary);border:1px solid var(--border-primary);border-radius:50%;width:22px;height:22px;display:inline-flex;align-items:center;justify-content:center;font-size:14px;font-weight:700}
.help-wrap:hover .help-tip{display:block}
.help-tip{display:none;position:absolute;top:30px;right:0;background:var(--bg-card);border:1px solid var(--border-primary);border-radius:8px;padding:12px 16px;font-size:15px;color:var(--text-primary);white-space:nowrap;z-index:100;box-shadow:0 4px 12px rgba(0,0,0,.3)}
.help-tip kbd{background:var(--bg-tertiary);border:1px solid var(--border-primary);border-radius:3px;padding:2px 6px;font-size:13px;font-family:monospace}

/* ===== Tab 栏 ===== */
.tab-bar{display:flex;background:var(--bg-card);border-bottom:1px solid var(--border-primary);padding:0 24px;flex-shrink:0}
.tab{padding:9px 22px;cursor:pointer;border:none;background:transparent;color:var(--text-secondary);font-size:16px;border-bottom:2px solid transparent;transition:all .2s;font-weight:500}
.tab:hover{color:var(--text-primary)}
.tab.active{color:var(--accent);border-bottom-color:var(--accent)}

/* ===== 统计行 ===== */
.stats-row{display:flex;gap:6px;padding:8px 20px 6px;flex-wrap:nowrap;flex-shrink:0;align-items:stretch}
.stat-card{background:var(--bg-card);border-radius:6px;padding:8px 12px;min-width:90px;border:1px solid var(--border-primary);display:flex;flex-direction:column;flex:1;transition:all .2s}
.stat-card.clickable{cursor:pointer}
.stat-card.clickable:hover{border-color:var(--accent);transform:translateY(-1px);box-shadow:0 2px 8px rgba(0,0,0,.15)}
.stat-card .label{font-size:15px;color:var(--text-primary);text-transform:uppercase;letter-spacing:.5px;margin-bottom:3px;font-weight:600}
.stat-card .value{font-size:22px;font-weight:700;color:var(--text-heading)}
.stat-card .value .unit{font-size:15px;color:var(--text-secondary);font-weight:400}
.stat-card .sub{font-size:14px;color:var(--text-secondary);margin-top:3px}
.stat-card .sub-row{display:flex;gap:8px;flex-wrap:wrap;margin-top:4px}
.stat-card .sub-item{font-size:14px;color:var(--text-primary);background:var(--bg-tertiary);padding:2px 7px;border-radius:3px}

/* ===== 执行卡片 ===== */
.exec-card{flex:4;min-width:380px;max-width:750px;border:1px solid var(--exec-border);background:var(--exec-gradient);position:relative;overflow:hidden;max-height:250px;overflow-y:auto;padding:8px 12px}
.exec-card::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:linear-gradient(90deg,var(--accent-dark),var(--accent),var(--accent-dark));animation:shimmer 2s infinite}
@keyframes shimmer{0%{opacity:.6}50%{opacity:1}100%{opacity:.6}}
.exec-card .label{color:var(--accent);font-size:16px;margin-bottom:5px;font-weight:600}
.exec-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:5px}
.exec-header .exec-order{font-size:17px;color:var(--text-heading);font-weight:600;display:flex;align-items:center;gap:4px}
.exec-header .exec-timer{font-size:15px;color:var(--accent);font-family:'JetBrains Mono','Fira Code',monospace}
.exec-progress{font-size:15px;color:var(--text-primary);font-weight:400;margin-left:6px}
.exec-meta{display:flex;gap:14px;flex-wrap:wrap;margin-bottom:6px;font-size:15px;color:var(--text-primary)}
.exec-meta .meta-item{display:flex;align-items:center;gap:3px}
.exec-meta .meta-label{color:var(--text-primary);font-size:15px;font-weight:500}
.exec-meta .meta-value{color:var(--text-heading);font-weight:600}
.exec-meta .meta-code{font-family:'JetBrains Mono','Fira Code',monospace;font-size:14px;color:var(--text-primary);background:var(--bg-tertiary);padding:1px 5px;border-radius:2px}
.exec-steps{display:flex;align-items:center;gap:2px;flex-wrap:wrap}
.exec-step-col{display:flex;flex-direction:column;gap:1px}
.exec-step{display:flex;align-items:center;gap:2px;padding:3px 8px;border-radius:3px;font-size:12px;font-weight:500;background:var(--bg-tertiary);border:1px solid var(--border-primary);cursor:default;white-space:nowrap}
.exec-step.completed{background:var(--success-bg);border-color:var(--success-border);color:var(--success)}
.exec-step.running{background:var(--info-bg);border-color:var(--info-border);color:var(--accent);animation:pulse 1.5s infinite}
.exec-step.failed{background:var(--danger-bg);border-color:var(--danger-border);color:var(--danger)}
.exec-step.interrupted{background:var(--warning-bg);border-color:var(--warning-border);color:var(--warning)}
.exec-step .step-icon{font-size:12px}
.exec-step .step-time{font-size:11px;color:var(--text-secondary);margin-left:2px}
.exec-step-error{color:var(--danger);font-size:11px;max-width:120px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.exec-error-summary{color:var(--danger);font-size:11px;margin-top:2px;display:flex;align-items:center;gap:2px}
.exec-connector{color:var(--text-muted);font-size:11px;margin:0 1px;padding-top:1px}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.7}}
.exec-empty{color:var(--text-dim);font-size:15px;font-style:italic;margin-top:3px}

/* ===== 筛选行 ===== */
.filters{padding:6px 24px;display:flex;gap:8px;align-items:center;flex-shrink:0;flex-wrap:wrap}
.filters label{font-size:15px;color:var(--text-primary)}
.filters select,.filters input{background:var(--bg-input);color:var(--text-primary);border:1px solid var(--border-primary);border-radius:6px;padding:6px 12px;font-size:15px;outline:none;transition:border-color .2s}
.filters select:focus,.filters input:focus{border-color:var(--accent)}
.filters button{background:var(--bg-tertiary);color:var(--text-primary);border:1px solid var(--border-primary);border-radius:6px;padding:6px 16px;cursor:pointer;font-size:15px;transition:all .2s}
.filters button:hover{background:var(--border-primary);border-color:var(--accent)}
.filters .btn-primary{background:var(--accent-dark);color:#fff;border-color:var(--accent-dark)}
.filters .btn-primary:hover{background:var(--accent)}

/* ===== 表格区 ===== */
.table-area{flex:1;display:flex;flex-direction:column;padding:0 24px 8px;overflow:hidden}
.table-header{display:flex;justify-content:space-between;align-items:center;padding:4px 0;flex-shrink:0}
.table-header .count{font-size:15px;color:var(--text-primary)}
.table-scroll{flex:1;overflow-y:auto;border-radius:8px;border:1px solid var(--border-secondary)}
table{width:100%;border-collapse:collapse;font-size:15px}
thead th{background:var(--bg-card);padding:10px 12px;text-align:left;position:sticky;top:0;white-space:nowrap;font-size:15px;color:var(--text-primary);font-weight:600;letter-spacing:.3px;border-bottom:1px solid var(--border-primary);z-index:1;cursor:pointer;user-select:none}
thead th:hover{color:var(--accent)}
thead th .sort-arrow{margin-left:5px;font-size:14px;opacity:.3}
thead th.sorted .sort-arrow{opacity:1;color:var(--accent);font-size:15px}
tbody tr{border-bottom:1px solid var(--border-secondary);transition:background .15s}
tbody tr:hover{background:var(--row-hover)}
tbody tr.row-running{background:var(--row-running)}
tbody td{padding:8px 12px;white-space:nowrap;max-width:200px;overflow:hidden;text-overflow:ellipsis;color:var(--text-primary);font-size:15px}

/* ===== 状态徽章 ===== */
.badge{display:inline-flex;align-items:center;gap:4px;padding:3px 12px;border-radius:12px;font-size:14px;font-weight:600;color:#fff;letter-spacing:.3px}
.badge-dot{width:8px;height:8px;border-radius:50%;background:currentColor;opacity:.8}

/* ===== 进度条 ===== */
.progress-wrap{display:flex;align-items:center;gap:6px}
.progress-bar{width:65px;height:8px;background:var(--bg-tertiary);border-radius:4px;overflow:hidden}
.progress-fill{height:100%;border-radius:4px;transition:width .3s}
.progress-text{font-size:15px;color:var(--text-primary);min-width:40px}

/* ===== 工作流徽章 ===== */
.wf-badges{display:flex;gap:3px;flex-wrap:wrap}
.wf-badge{padding:3px 8px;border-radius:3px;font-size:14px;color:#fff;font-weight:500}

/* ===== 历史表格步骤 ===== */
.hist-steps{display:flex;align-items:flex-start;gap:3px;flex-wrap:wrap}
.hist-step{display:inline-flex;align-items:center;gap:3px;padding:4px 10px;border-radius:4px;font-size:14px;font-weight:500;white-space:nowrap}
.hist-step.completed{background:var(--success-bg);color:var(--success);border:1px solid var(--success-border)}
.hist-step.running{background:var(--info-bg);color:var(--accent);border:1px solid var(--info-border)}
.hist-step.failed{background:var(--danger-bg);color:var(--danger);border:1px solid var(--danger-border)}
.hist-step.interrupted{background:var(--warning-bg);color:var(--warning);border:1px solid var(--warning-border)}
.hist-step.pending{background:var(--bg-tertiary);color:var(--text-muted);border:1px solid var(--border-primary)}
.hist-step .step-icon{font-size:15px}
.hist-connector{color:var(--text-primary);font-size:14px}
.hist-step-error{color:var(--danger);font-size:13px;max-width:140px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;display:block}

/* ===== 分页 ===== */
.pagination{display:flex;justify-content:center;align-items:center;gap:6px;padding:8px 0;flex-shrink:0}
.pagination button{background:var(--bg-tertiary);color:var(--text-primary);border:1px solid var(--border-primary);border-radius:6px;padding:5px 12px;cursor:pointer;font-size:14px;min-width:36px;transition:all .15s}
.pagination button:hover:not(:disabled){background:var(--border-primary);border-color:var(--accent)}
.pagination button:disabled{opacity:.3;cursor:not-allowed}
.pagination button.active{background:var(--accent-dark);color:#fff;border-color:var(--accent-dark)}
.pagination .page-info{font-size:14px;color:var(--text-primary);margin:0 8px}

/* ===== 通用 ===== */
.mono{font-family:'JetBrains Mono','Fira Code',monospace;font-size:14px;color:var(--text-primary)}
.error-text{color:var(--danger);font-size:14px;max-width:140px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}

/* ===== 滚动条 ===== */
::-webkit-scrollbar{width:5px;height:5px}
::-webkit-scrollbar-track{background:var(--scrollbar-track)}
::-webkit-scrollbar-thumb{background:var(--scrollbar-thumb);border-radius:3px}
::-webkit-scrollbar-thumb:hover{background:var(--scrollbar-thumb-hover)}
</style>
</head>
<body class="dark">
<div class="header">
  <h1>G1D 订单看板</h1>
  <div class="header-right">
    <button id="theme_btn" class="icon-btn" onclick="toggleTheme()" title="切换主题">🌙</button>
    <button id="pause_btn" class="icon-btn" onclick="togglePause()" title="暂停/恢复刷新">⏸</button>
    <span id="pause_indicator" class="pause-indicator" style="display:none">已暂停</span>
    <span class="help-wrap">?<span class="help-tip">
      <div style="margin-bottom:4px;font-weight:600">快捷键</div>
      <div><kbd>Space</kbd> 暂停/恢复刷新</div>
      <div><kbd>R</kbd> 手动刷新</div>
      <div><kbd>Esc</kbd> 重置筛选</div>
    </span></span>
    <span class="refresh-dot" id="refresh_dot"></span>
    <span id="refresh_info" style="font-size:14px;color:var(--text-secondary)"></span>
    <span class="time" id="clock"></span>
  </div>
</div>
<div class="tab-bar">
  <button class="tab active" id="tab_orders" onclick="switchTab('orders')">订单列表</button>
  <button class="tab" id="tab_history" onclick="switchTab('history')">执行历史</button>
</div>
<div class="stats-row" id="stats_row"></div>
<div class="filters" id="filters_row">
  <label>状态</label><select id="filter_status"><option value="">全部</option></select>
  <label>品牌</label><select id="filter_brand"><option value="">全部</option></select>
  <label>日期</label><input type="date" id="filter_date">
  <label>搜索</label><input type="text" id="filter_search" placeholder="订单号/品牌/客户" style="width:140px">
  <button class="btn-primary" onclick="applyFilters()">筛选</button>
  <button onclick="resetFilters()">重置</button>
  <button onclick="exportCSV()">导出 CSV</button>
</div>
<div class="table-area" id="orders_area">
  <div class="table-header">
    <span class="count" id="table_count"></span>
    <span class="count" id="sort_info">排序: 创建时间 ↓</span>
  </div>
  <div class="table-scroll" id="table_scroll">
    <table>
      <thead><tr>
        <th onclick="sortBy('index')" data-col="index"># <span class="sort-arrow"></span></th>
        <th onclick="sortBy('order_date')" data-col="order_date">日期 <span class="sort-arrow"></span></th>
        <th onclick="sortBy('order_no')" data-col="order_no">订单号 <span class="sort-arrow"></span></th>
        <th onclick="sortBy('brand')" data-col="brand">品牌(编码) <span class="sort-arrow"></span></th>
        <th onclick="sortBy('quantity')" data-col="quantity">数量 <span class="sort-arrow"></span></th>
        <th>进度</th>
        <th onclick="sortBy('status')" data-col="status">状态 <span class="sort-arrow"></span></th>
        <th>工作流</th>
        <th>客户</th>
        <th>机器人时间</th>
        <th onclick="sortBy('create_time')" data-col="create_time" class="sorted">创建时间 <span class="sort-arrow"></span></th>
      </tr></thead>
      <tbody id="tbody"></tbody>
    </table>
  </div>
  <div class="pagination" id="pagination"></div>
</div>
<div class="table-area" id="history_area" style="display:none">
  <div class="table-header">
    <span class="count" id="history_count"></span>
    <span class="count" id="history_sort_info">排序: 时间 ↓</span>
  </div>
  <div class="table-scroll" id="history_scroll">
    <table>
      <thead><tr>
        <th onclick="sortHistoryBy('runtime_id')" data-col="runtime_id">Runtime ID <span class="sort-arrow"></span></th>
        <th>订单号</th>
        <th>品牌</th>
        <th>工作流步骤</th>
        <th onclick="sortHistoryBy('duration')" data-col="duration">总耗时 <span class="sort-arrow"></span></th>
        <th onclick="sortHistoryBy('status')" data-col="status">状态 <span class="sort-arrow"></span></th>
        <th onclick="sortHistoryBy('time')" data-col="time" class="sorted">时间 <span class="sort-arrow"></span></th>
      </tr></thead>
      <tbody id="history_tbody"></tbody>
    </table>
  </div>
  <div class="pagination" id="history_pagination"></div>
</div>

<script>
/* ===== 常量 ===== */
const STATUS_MAP = """ + json.dumps(ORDER_STATUS_MAP, ensure_ascii=False) + """;
const STATUS_COLOR = """ + json.dumps(ORDER_STATUS_COLOR, ensure_ascii=False) + """;
const FLOW_MAP = """ + json.dumps(FLOW_STATUS_MAP, ensure_ascii=False) + """;
const FLOW_COLOR = {WAITING:'#d29922',RUNNING:'#58a6ff',COMPLETED:'#3fb950',FAILED:'#f85149',RETRY:'#db6d28'};
const NODE_MAP = """ + json.dumps(NODE_STATUS_MAP, ensure_ascii=False) + """;
const NODE_ICON = """ + json.dumps(NODE_STATUS_ICON, ensure_ascii=False) + """;
const NODE_COLOR = """ + json.dumps(NODE_STATUS_COLOR, ensure_ascii=False) + """;
const RUNNING_STATES = new Set(""" + json.dumps(list(RUNNING_STATES)) + """);
const ACTIVE_STATUSES = new Set(""" + json.dumps(list(ACTIVE_STATUSES)) + """);
const PAGE_SIZE = """ + str(PAGE_SIZE) + """;

/* ===== 状态 ===== */
let allOrders = [];
let filteredOrders = [];
let currentPage = 1;
let sortCol = 'create_time';
let sortDir = 'desc';

let currentTab = 'orders';
let isPaused = false;
let isManualPause = false;
let autoPaused = false;

let historyData = [];
let historyPage = 1;
let historySortCol = 'time';
let historySortDir = 'desc';

let prevOrderStatuses = new Map();
let lastActiveOrder = null;
let refreshTimer = null;
let audioCtx = null;

/* ===== 工具函数 ===== */
function ts(v) { return v && v!=='None' ? v.slice(0,19) : '-'; }
function tsShort(v) { return v && v!=='None' ? v.slice(11,19) : '-'; }
function esc(s) { const d=document.createElement('div'); d.textContent=String(s||''); return d.innerHTML; }

function getStepIcon(node) {
  const nodeType = (node.node_type || '').toUpperCase();
  const pi = node.payload_info || {};
  const actionType = (pi.action_type || '').toLowerCase();
  if (nodeType === 'NAVIGATION') return '\\u{1F9ED}';
  if (nodeType === 'CHECK_ITEM') {
    if (actionType === 'pick') return '\\u{1F9BE}';
    if (actionType === 'place') return '\\u{1F4E6}';
    return '\\u{1F50D}';
  }
  return NODE_ICON[node.node_status] || '\\u25CB';
}

function formatDuration(sec) {
  if (!sec || sec <= 0) return '-';
  sec = Math.round(sec);
  if (sec < 60) return sec + 's';
  const m = Math.floor(sec / 60);
  const s = sec % 60;
  if (m < 60) return m + 'm' + s + 's';
  const h = Math.floor(m / 60);
  return h + 'h' + (m % 60) + 'm';
}

/* ===== 声音报警 ===== */
function playAlertSound() {
  try {
    if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
    if (audioCtx.state === 'suspended') audioCtx.resume();
    const osc = audioCtx.createOscillator();
    osc.type = 'sine';
    osc.frequency.value = 800;
    osc.connect(audioCtx.destination);
    osc.start();
    setTimeout(function() { osc.stop(); }, 200);
  } catch(e) {}
}

/* ===== 浏览器通知 ===== */
function requestNotificationPermission() {
  if ('Notification' in window && Notification.permission === 'default') {
    Notification.requestPermission();
  }
}

function sendNotification(title, body) {
  if ('Notification' in window && Notification.permission === 'granted') {
    try { new Notification(title, { body: body }); } catch(e) {}
  }
}

function checkNotifications(orders) {
  orders.forEach(function(o) {
    const prevStatus = prevOrderStatuses.get(o.order_item_id);
    const newStatus = o.status;
    if (prevStatus && prevStatus !== newStatus) {
      if (['SUCCESS','COMPLETED','FAILED','INTERRUPTED'].indexOf(newStatus) >= 0) {
        sendNotification('G1D 任务通知', '订单 ' + o.order_no + ' \\u2192 ' + (STATUS_MAP[newStatus]||newStatus));
        if (newStatus === 'FAILED') playAlertSound();
      }
    }
    prevOrderStatuses.set(o.order_item_id, newStatus);
  });
}

/* ===== 音频初始化 ===== */
document.addEventListener('click', function initAudio() {
  if (!audioCtx) audioCtx = new (window.AudioContext || window.webkitAudioContext)();
  if (audioCtx.state === 'suspended') audioCtx.resume();
  requestNotificationPermission();
  document.removeEventListener('click', initAudio);
}, { once: true });

/* ===== 数据加载 ===== */
async function loadData() {
  try {
    const resp = await fetch('/api/orders');
    const data = await resp.json();
    allOrders = data.orders || [];
    renderStats(data);
    renderFilters(data);
    applyFilters();
    const durStr = data.fetch_duration ? ' \\u00b7 ' + data.fetch_duration + 's' : '';
    document.getElementById('refresh_info').textContent =
      '\\u66f4\\u65b0 ' + new Date(data.last_update * 1000).toLocaleTimeString('zh-CN') + durStr;
    document.getElementById('refresh_dot').className = 'refresh-dot' + (data.error ? ' error' : '');
    checkNotifications(allOrders);
  } catch(e) {
    console.error(e);
    document.getElementById('refresh_dot').className = 'refresh-dot error';
  }
}

async function loadHistory() {
  try {
    const resp = await fetch('/api/history');
    const data = await resp.json();
    historyData = data.history || [];
    renderHistory();
  } catch(e) {
    console.error(e);
  }
}

/* ===== 刷新控制 ===== */
function startRefresh() {
  if (refreshTimer) clearInterval(refreshTimer);
  refreshTimer = setInterval(function() {
    if (isPaused || autoPaused) return;
    loadData();
    if (currentTab === 'history') loadHistory();
  }, """ + str(REFRESH_INTERVAL * 1000) + """);
}

function togglePause() {
  isPaused = !isPaused;
  isManualPause = isPaused;
  document.getElementById('pause_btn').textContent = isPaused ? '\\u25b6' : '\\u23f8';
  document.getElementById('pause_indicator').style.display = isPaused ? 'inline' : 'none';
  if (!isPaused) {
    loadData();
    if (currentTab === 'history') loadHistory();
  }
}

function manualRefresh() {
  loadData();
  if (currentTab === 'history') loadHistory();
}

/* ===== 自动暂停 ===== */
let autoPauseTimeout = null;
function setAutoPause(val) {
  if (isManualPause) return;
  autoPaused = val;
  if (autoPaused) {
    clearTimeout(autoPauseTimeout);
    autoPauseTimeout = setTimeout(function() {
      if (!isManualPause) autoPaused = false;
    }, 3000);
  }
}

/* ===== Tab 切换 ===== */
function switchTab(tab) {
  currentTab = tab;
  document.getElementById('tab_orders').className = 'tab' + (tab==='orders' ? ' active' : '');
  document.getElementById('tab_history').className = 'tab' + (tab==='history' ? ' active' : '');
  document.getElementById('orders_area').style.display = tab==='orders' ? '' : 'none';
  document.getElementById('history_area').style.display = tab==='history' ? '' : 'none';
  document.getElementById('filters_row').style.display = tab==='orders' ? '' : 'none';
  if (tab === 'history') loadHistory();
}

/* ===== 统计渲染 ===== */
function renderStats(data) {
  const total = data.total || {};
  const stats = data.stats || {};
  const pctVal = total.orders > 0 ? (total.orders - (stats.WAITING?.count||0)) / total.orders : 0;
  const pct = (pctVal * 100).toFixed(2);
  const successCnt = (stats.SUCCESS?.count||0) + (stats.COMPLETED?.count||0);
  const avgDur = total.avg_duration_sec ? Math.round(total.avg_duration_sec) : '-';

  let html = '';
  html += '<div class="stat-card clickable" onclick="filterByStatus(null)">'
    + '<div class="label">📊 总订单</div><div class="value">' + (total.orders||0) + '</div>'
    + '<div class="sub-row"><span class="sub-item">今日: ' + (total.today_orders||0) + '单</span>'
    + '<span class="sub-item">品牌: ' + (total.brand_count||0) + '种</span></div>'
    + '<div class="sub">香烟总量 ' + (total.total_qty||0) + ' 条</div></div>';

  html += '<div class="stat-card clickable" onclick="filterByStatus(\\'SUCCESS,COMPLETED\\')">'
    + '<div class="label">✅ 已完成</div><div class="value" style="color:var(--success)">' + successCnt + '</div>'
    + '<div class="sub-row"><span class="sub-item">今日: ' + (total.today_success||0) + '单</span>'
    + '<span class="sub-item">香烟: ' + ((stats.SUCCESS?.done_qty||0)+(stats.COMPLETED?.done_qty||0)) + '条</span></div>'
    + '<div class="sub">平均耗时 ' + avgDur + '秒</div></div>';

  if (stats.WAITING) {
    html += '<div class="stat-card clickable" onclick="filterByStatus(\\'WAITING\\')">'
      + '<div class="label">⏳ 等待中</div><div class="value" style="color:var(--warning)">' + stats.WAITING.count + '</div>'
      + '<div class="sub-row"><span class="sub-item">香烟: ' + stats.WAITING.total_qty + '条</span></div>'
      + '<div class="sub">占比 ' + (total.orders > 0 ? ((stats.WAITING.count/total.orders)*100).toFixed(1) : '0') + '%</div></div>';
  }

  if (stats.FAILED) {
    html += '<div class="stat-card clickable" onclick="filterByStatus(\\'FAILED\\')">'
      + '<div class="label">❌ 失败</div><div class="value" style="color:var(--danger)">' + stats.FAILED.count + '</div>'
      + '<div class="sub-row"><span class="sub-item">今日: ' + (total.today_failed||0) + '单</span>'
      + '<span class="sub-item">香烟: ' + stats.FAILED.total_qty + '条</span></div>'
      + '<div class="sub">失败率 ' + (total.orders > 0 ? ((stats.FAILED.count/total.orders)*100).toFixed(2) : '0') + '%</div></div>';
  }

  html += renderExecCard();
  document.getElementById('stats_row').innerHTML = html;
}

/* ===== 点击卡片筛选 ===== */
let clickFilterStatus = null;
function filterByStatus(statusStr) {
  clickFilterStatus = statusStr;
  if (currentTab !== 'orders') switchTab('orders');
  applyFilters();
}

/* ===== 执行卡片 ===== */
function renderExecCard() {
  const running = allOrders.filter(function(o) { return ACTIVE_STATUSES.has(o.status); });
  let displayOrders = running;

  if (!running.length) {
    const recentDone = allOrders
      .filter(function(o) { return ['SUCCESS','COMPLETED','FAILED','INTERRUPTED'].indexOf(o.status) >= 0; })
      .sort(function(a, b) { return (b.update_time||'').localeCompare(a.update_time||''); });
    if (recentDone.length) displayOrders = [recentDone[0]];
    else if (lastActiveOrder) displayOrders = [lastActiveOrder];
  } else {
    lastActiveOrder = running[0];
  }

  if (!displayOrders.length) {
    return '<div class="stat-card exec-card"><div class="label">任务状态</div>'
      + '<div class="exec-empty">暂无任务记录</div></div>';
  }

  let html = '';
  displayOrders.forEach(function(order) {
    const ext = order.extend || {};
    const flows = order.flows || [];
    const isActive = ACTIVE_STATUSES.has(order.status);
    const isFailed = order.status === 'FAILED';
    const isSuccess = order.status === 'SUCCESS' || order.status === 'COMPLETED';
    const isInterrupted = order.status === 'INTERRUPTED';

    let cardLabel = '任务状态';
    let badgeBg = 'var(--accent-dark)';
    let badgeText = '执行中';
    if (isFailed) { badgeBg = 'var(--danger-border)'; badgeText = '失败'; }
    else if (isInterrupted) { badgeBg = 'var(--warning-border)'; badgeText = '中断'; }
    else if (isSuccess) { badgeBg = 'var(--success-border)'; badgeText = '完成'; }

    let timerStr = '';
    if (order.robot_start_time && order.robot_start_time !== 'None') {
      const startMs = new Date(order.robot_start_time.replace(' ', 'T')).getTime();
      let endMs = Date.now();
      if (order.robot_end_time && order.robot_end_time !== 'None') {
        endMs = new Date(order.robot_end_time.replace(' ', 'T')).getTime();
      }
      const elapsed = Math.floor((endMs - startMs) / 1000);
      if (elapsed > 0) {
        const m = Math.floor(elapsed / 60);
        const s = elapsed % 60;
        timerStr = (m > 0 ? m + 'm ' : '') + s + 's';
      }
    }

    const orderDone = order.completed_quantity || 0;
    const orderTotal = order.quantity || 0;
    const orderPct = orderTotal > 0 ? (orderDone / orderTotal * 100).toFixed(2) : '0.00';
    html += '<div class="stat-card exec-card"><div class="label">' + cardLabel + ' <span class="exec-progress">' + orderDone + '/' + orderTotal + ' ' + orderPct + '%</span></div>'
      + '<div class="exec-header">'
      + '<div class="exec-order"><span class="badge" style="background:'+badgeBg+'">'+badgeText+'</span> '
      + esc(order.order_no) + '</div>'
      + (timerStr ? '<div class="exec-timer">' + timerStr + '</div>' : '')
      + '</div>'
      + '<div class="exec-meta">';
    html += '<span class="meta-item"><span class="meta-label">品牌</span><span class="meta-value">' + esc(order.brand) + '</span>'
      + '<span class="meta-code">' + esc(order.brand_code) + '</span></span>';
    if (ext.customer) html += '<span class="meta-item"><span class="meta-label">客户</span><span class="meta-value">' + esc(ext.customer) + '</span></span>';
    html += '<span class="meta-item"><span class="meta-label">数量</span><span class="meta-value">' + order.completed_quantity + '/' + order.quantity + '</span></span>';
    html += '</div>';

    if (!flows.length) {
      html += '<div class="exec-empty">无流程数据</div></div>';
      return;
    }

    let hasFailedStep = false;
    let failedMessages = [];

    flows.forEach(function(flow) {
      const nodes = flow.workflows || [];
      if (!nodes.length) {
        html += '<div style="color:var(--text-muted);font-size:11px">流程无步骤数据</div>';
        return;
      }

      html += '<div class="exec-steps">';
      nodes.forEach(function(node, idx) {
        if (idx > 0) html += '<span class="exec-connector">\\u2192</span>';
        const st = node.node_status || 'PENDING';
        const cls = st === 'COMPLETED' ? 'completed' : (st==='FAILED'?'failed':(st==='INTERRUPTED'?'interrupted':(RUNNING_STATES.has(st)?'running':'')));
        const icon = getStepIcon(node);
        let name = node.node_name || '';
        if (name.length > 8) name = name.slice(0,8);

        let stepTime = '';
        if (node.callback_time && node.send_time) {
          const cbMs = new Date(node.callback_time.replace(' ', 'T')).getTime();
          const sdMs = new Date(node.send_time.replace(' ', 'T')).getTime();
          const dur = Math.round((cbMs - sdMs) / 1000);
          stepTime = dur >= 60 ? Math.floor(dur/60)+'m'+dur%60+'s' : dur+'s';
        }

        const titleParts = [node.node_name||'', NODE_MAP[st]||st];
        if (stepTime) titleParts.push('耗时 ' + stepTime);
        const pi = node.payload_info || {};
        if (pi.target_object) titleParts.push('编码 ' + pi.target_object);

        html += '<div class="exec-step-col">';
        html += '<div class="exec-step ' + cls + '" title="' + esc(titleParts.join(' | ')) + '">'
          + '<span class="step-icon">' + icon + '</span>' + esc(name)
          + (stepTime ? '<span class="step-time">' + stepTime + '</span>' : '')
          + '</div>';
        if (st === 'FAILED' && node.error_message) {
          html += '<div class="exec-step-error">' + esc(node.error_message.slice(0, 40)) + '</div>';
          hasFailedStep = true;
          failedMessages.push(node.error_message);
        }
        html += '</div>';
      });
      html += '</div>';
    });

    if (hasFailedStep) {
      const errSummary = failedMessages.join('; ').slice(0, 100);
      html += '<div class="exec-error-summary">\\u26a0 ' + esc(errSummary) + '</div>';
    }

    html += '</div>';
  });
  return html;
}

/* ===== 筛选渲染 ===== */
function renderFilters(data) {
  const statuses = Object.keys(data.stats || {});
  const sel = document.getElementById('filter_status');
  const prevVal = sel.value;
  sel.innerHTML = '<option value="">全部</option>';
  statuses.forEach(function(s) { sel.innerHTML += '<option value="'+s+'"'+(s===prevVal?' selected':'')+'>'+(STATUS_MAP[s]||s)+'</option>'; });
  const brands = [...new Set(allOrders.map(function(o){return o.brand}))].sort();
  const bsel = document.getElementById('filter_brand');
  const prevBrand = bsel.value;
  bsel.innerHTML = '<option value="">全部</option>';
  brands.forEach(function(b) { bsel.innerHTML += '<option value="'+b+'"'+(b===prevBrand?' selected':'')+'>'+b+'</option>'; });
}

/* ===== 筛选与排序 ===== */
function applyFilters() {
  const fStatus = clickFilterStatus || document.getElementById('filter_status').value;
  const fBrand = document.getElementById('filter_brand').value;
  const fDate = document.getElementById('filter_date').value;
  const fSearch = document.getElementById('filter_search').value.toLowerCase();
  filteredOrders = allOrders.filter(function(o) {
    if (fStatus) {
      if (fStatus.indexOf(',') >= 0) {
        if (fStatus.split(',').indexOf(o.status) < 0) return false;
      } else {
        if (o.status !== fStatus) return false;
      }
    }
    if (fBrand && o.brand !== fBrand) return false;
    if (fDate && (o.order_date||'').slice(0,10) !== fDate) return false;
    if (fSearch) {
      const ext = o.extend || {};
      if (!((o.order_no+o.brand+o.brand_code+(ext.customer||'')+(ext.task||'')).toLowerCase().indexOf(fSearch) >= 0)) return false;
    }
    return true;
  });
  doSort();
  currentPage = 1;
  renderTable();
}

function doSort() {
  const col = sortCol;
  const dir = sortDir === 'asc' ? 1 : -1;
  filteredOrders.sort(function(a, b) {
    let va = a[col], vb = b[col];
    if (col === 'index') { va = allOrders.indexOf(a); vb = allOrders.indexOf(b); }
    if (col === 'quantity') { va = Number(va)||0; vb = Number(vb)||0; return (va-vb)*dir; }
    if (typeof va === 'string') return va.localeCompare(vb||'') * dir;
    return 0;
  });
}

function sortBy(col) {
  if (sortCol === col) {
    sortDir = sortDir === 'desc' ? 'asc' : 'desc';
  } else {
    sortCol = col;
    sortDir = col === 'create_time' ? 'desc' : 'asc';
  }
  document.querySelectorAll('#orders_area thead th').forEach(function(th) {
    th.classList.remove('sorted');
    const arrow = th.querySelector('.sort-arrow');
    if (arrow) arrow.textContent = '';
  });
  const th = document.querySelector('#orders_area thead th[data-col="'+col+'"]');
  if (th) {
    th.classList.add('sorted');
    const arrow = th.querySelector('.sort-arrow');
    if (arrow) arrow.textContent = sortDir === 'desc' ? ' \\u25BC' : ' \\u25B2';
  }
  const colNames = {index:'序号',order_date:'日期',order_no:'订单号',brand:'品牌',quantity:'数量',status:'状态',create_time:'创建时间'};
  document.getElementById('sort_info').textContent = '排序: ' + (colNames[col]||col) + (sortDir==='desc'?' \\u25BC':' \\u25B2');
  doSort();
  currentPage = 1;
  renderTable();
}

/* ===== 订单表格 ===== */
function renderTable() {
  const tbody = document.getElementById('tbody');
  const totalPages = Math.max(1, Math.ceil(filteredOrders.length / PAGE_SIZE));
  if (currentPage > totalPages) currentPage = totalPages;
  const start = (currentPage - 1) * PAGE_SIZE;
  const pageOrders = filteredOrders.slice(start, start + PAGE_SIZE);

  document.getElementById('table_count').textContent =
    '共 ' + filteredOrders.length + ' 条' + (filteredOrders.length !== allOrders.length ? ' (筛选自 ' + allOrders.length + ' 条)' : '');

  if (!pageOrders.length) {
    tbody.innerHTML = '<tr><td colspan="11" style="text-align:center;color:var(--text-dim);padding:40px">暂无订单数据</td></tr>';
    renderPagination('pagination', 0, 'orders');
    return;
  }

  let html = '';
  pageOrders.forEach(function(o, i) {
    const ext = o.extend || {};
    const color = STATUS_COLOR[o.status] || '#8b949e';
    const label = STATUS_MAP[o.status] || o.status;
    const pct = o.quantity > 0 ? Math.round(o.completed_quantity / o.quantity * 100) : 0;
    const isRunning = ACTIVE_STATUSES.has(o.status);

    let wfHtml = '';
    const flows = o.flows || [];
    if (flows.length) {
      const latest = flows[0];
      const nodes = latest.workflows || [];
      if (nodes.length) {
        wfHtml = '<div class="wf-badges">';
        nodes.forEach(function(n) {
          const st = n.node_status || 'PENDING';
          const nc = NODE_COLOR[st] || '#484f58';
          const icon = getStepIcon(n);
          let badgeName = n.node_name || '';
          if (badgeName.length > 6) badgeName = badgeName.slice(0,6);
          wfHtml += '<span class="wf-badge" style="background:'+nc+'33;color:'+nc+';border:1px solid '+nc+'55">' + icon + ' ' + esc(badgeName) + '</span>';
        });
        wfHtml += '</div>';
      } else {
        const fc = FLOW_COLOR[latest.status] || '#8b949e';
        wfHtml = '<span class="wf-badge" style="background:'+fc+'33;color:'+fc+';border:1px solid '+fc+'55">'+(FLOW_MAP[latest.status]||latest.status)+'</span>';
      }
    } else {
      wfHtml = '<span style="color:var(--text-dim)">-</span>';
    }

    let robotTime = '-';
    if (o.robot_start_time && o.robot_start_time !== 'None') {
      robotTime = tsShort(o.robot_start_time);
      if (o.robot_end_time && o.robot_end_time !== 'None') {
        robotTime += ' \\u2192 ' + tsShort(o.robot_end_time);
      }
    }

    let pctColor = 'var(--success)';
    if (pct < 30) pctColor = 'var(--danger)';
    else if (pct < 70) pctColor = 'var(--warning)';

    html += '<tr class="'+(isRunning?'row-running':'')+'">'
      + '<td>'+(start+i+1)+'</td>'
      + '<td>'+ (o.order_date||'').slice(0,10) +'</td>'
      + '<td class="mono">'+esc(o.order_no)+'</td>'
      + '<td>'+esc(o.brand)+' <span class="mono" style="font-size:9px">('+esc(o.brand_code)+')</span></td>'
      + '<td>'+o.quantity+'</td>'
      + '<td><div class="progress-wrap"><div class="progress-bar"><div class="progress-fill" style="width:'+pct+'%;background:'+pctColor+'"></div></div><span class="progress-text">'+o.completed_quantity+'/'+o.quantity+'</span></div></td>'
      + '<td><span class="badge" style="background:'+color+'22;color:'+color+';border:1px solid '+color+'44"><span class="badge-dot" style="background:'+color+'"></span>'+label+'</span></td>'
      + '<td>'+wfHtml+'</td>'
      + '<td>'+(ext.customer?esc(ext.customer):'-')+'</td>'
      + '<td class="mono">'+robotTime+'</td>'
      + '<td class="mono">'+ts(o.create_time).slice(5,16)+'</td>'
      + '</tr>';
  });
  tbody.innerHTML = html;
  renderPagination('pagination', totalPages, 'orders');
}

/* ===== 历史表格 ===== */
function renderHistory() {
  const tbody = document.getElementById('history_tbody');
  const sorted = historyData.slice();
  const col = historySortCol;
  const dir = historySortDir === 'asc' ? 1 : -1;
  sorted.sort(function(a, b) {
    if (col === 'duration') return ((a.total_duration_sec||0) - (b.total_duration_sec||0)) * dir;
    if (col === 'status') return (a.status||'').localeCompare(b.status||'') * dir;
    if (col === 'time') return ((a.first_send_time||'').localeCompare(b.first_send_time||'')) * dir;
    if (col === 'runtime_id') return String(a.runtime_id||'').localeCompare(String(b.runtime_id||'')) * dir;
    return 0;
  });

  const totalPages = Math.max(1, Math.ceil(sorted.length / PAGE_SIZE));
  if (historyPage > totalPages) historyPage = totalPages;
  const start = (historyPage - 1) * PAGE_SIZE;
  const pageData = sorted.slice(start, start + PAGE_SIZE);

  document.getElementById('history_count').textContent = '共 ' + sorted.length + ' 条历史记录';

  if (!pageData.length) {
    tbody.innerHTML = '<tr><td colspan="7" style="text-align:center;color:var(--text-dim);padding:40px">暂无历史数据</td></tr>';
    renderPagination('history_pagination', 0, 'history');
    return;
  }

  let html = '';
  pageData.forEach(function(g, i) {
    const rid = String(g.runtime_id || '');
    const ridShort = rid.length > 8 ? rid.slice(0,8) + '..' : rid;

    let stepsHtml = '<div class="hist-steps">';
    const nodes = g.nodes || [];
    nodes.forEach(function(node, idx) {
      if (idx > 0) stepsHtml += '<span class="hist-connector">\\u2192</span>';
      const st = node.node_status || 'PENDING';
      const cls = st === 'COMPLETED' ? 'completed' : (st==='FAILED'?'failed':(st==='INTERRUPTED'?'interrupted':(RUNNING_STATES.has(st)?'running':'pending')));
      const icon = getStepIcon(node);
      let name = node.node_name || '';
      if (name.length > 8) name = name.slice(0,8);

      const titleParts = [node.node_name||'', NODE_MAP[st]||st];
      const pi = node.payload_info || {};
      if (pi.action_type) titleParts.push(pi.action_type);

      stepsHtml += '<span class="hist-step ' + cls + '" title="' + esc(titleParts.join(' | ')) + '">'
        + '<span class="step-icon">' + icon + '</span> ' + esc(name) + '</span>';
      if (st === 'FAILED' && node.error_message) {
        stepsHtml += '<span class="hist-step-error">' + esc(node.error_message.slice(0, 30)) + '</span>';
      }
    });
    stepsHtml += '</div>';

    const statusColor = g.status === 'COMPLETED' ? 'var(--success)' : (g.status === 'FAILED' ? 'var(--danger)' : 'var(--warning)');
    const statusLabel = NODE_MAP[g.status] || g.status;

    html += '<tr>'
      + '<td class="mono" title="'+esc(rid)+'">' + esc(ridShort) + '</td>'
      + '<td class="mono">' + esc(g.order_no || '-') + '</td>'
      + '<td>' + esc(g.brand || '-') + '</td>'
      + '<td>' + stepsHtml + '</td>'
      + '<td class="mono">' + formatDuration(g.total_duration_sec) + '</td>'
      + '<td><span class="badge" style="background:'+statusColor+'22;color:'+statusColor+';border:1px solid '+statusColor+'44"><span class="badge-dot" style="background:'+statusColor+'"></span>'+statusLabel+'</span></td>'
      + '<td class="mono">' + ts(g.first_send_time).slice(5,16) + '</td>'
      + '</tr>';
  });
  tbody.innerHTML = html;
  renderPagination('history_pagination', totalPages, 'history');
}

function sortHistoryBy(col) {
  if (historySortCol === col) {
    historySortDir = historySortDir === 'desc' ? 'asc' : 'desc';
  } else {
    historySortCol = col;
    historySortDir = col === 'time' ? 'desc' : 'asc';
  }
  document.querySelectorAll('#history_area thead th').forEach(function(th) {
    th.classList.remove('sorted');
    const arrow = th.querySelector('.sort-arrow');
    if (arrow) arrow.textContent = '';
  });
  const th = document.querySelector('#history_area thead th[data-col="'+col+'"]');
  if (th) {
    th.classList.add('sorted');
    const arrow = th.querySelector('.sort-arrow');
    if (arrow) arrow.textContent = historySortDir === 'desc' ? ' \\u25BC' : ' \\u25B2';
  }
  const colNames = {runtime_id:'Runtime ID',duration:'总耗时',status:'状态',time:'时间'};
  document.getElementById('history_sort_info').textContent = '排序: ' + (colNames[col]||col) + (historySortDir==='desc'?' \\u25BC':' \\u25B2');
  renderHistory();
}

/* ===== 分页 ===== */
function renderPagination(containerId, totalPages, type) {
  const div = document.getElementById(containerId);
  if (totalPages <= 1) { div.innerHTML = ''; return; }
  const cp = type === 'history' ? historyPage : currentPage;

  let html = '<button onclick="goPage(1,\\''+type+'\\')"'+(cp===1?' disabled':'')+'>\\u00ab</button>';
  html += '<button onclick="goPage('+(cp-1)+',\\''+type+'\\')"'+(cp===1?' disabled':'')+'>\\u2039</button>';

  let pages = [];
  const range = 2;
  for (let p = Math.max(1, cp-range); p <= Math.min(totalPages, cp+range); p++) {
    pages.push(p);
  }
  if (pages[0] > 1) { html += '<button onclick="goPage(1,\\''+type+'\\')">1</button>'; if (pages[0] > 2) html += '<span class="page-info">...</span>'; }
  pages.forEach(function(p) {
    html += '<button onclick="goPage('+p+',\\''+type+'\\')" class="'+(p===cp?'active':'')+'">'+p+'</button>';
  });
  if (pages[pages.length-1] < totalPages) { if (pages[pages.length-1] < totalPages-1) html += '<span class="page-info">...</span>'; html += '<button onclick="goPage('+totalPages+',\\''+type+'\\')">'+totalPages+'</button>'; }

  html += '<button onclick="goPage('+(cp+1)+',\\''+type+'\\')"'+(cp===totalPages?' disabled':'')+'>\\u203a</button>';
  html += '<button onclick="goPage('+totalPages+',\\''+type+'\\')"'+(cp===totalPages?' disabled':'')+'>\\u00bb</button>';
  html += '<span class="page-info">'+cp+' / '+totalPages+'</span>';
  div.innerHTML = html;
}

function goPage(p, type) {
  if (type === 'history') {
    const totalPages = Math.max(1, Math.ceil(historyData.length / PAGE_SIZE));
    if (p < 1 || p > totalPages) return;
    historyPage = p;
    renderHistory();
    document.getElementById('history_scroll').scrollTop = 0;
  } else {
    const totalPages = Math.max(1, Math.ceil(filteredOrders.length / PAGE_SIZE));
    if (p < 1 || p > totalPages) return;
    currentPage = p;
    renderTable();
    document.getElementById('table_scroll').scrollTop = 0;
  }
}

/* ===== 重置筛选 ===== */
function resetFilters() {
  clickFilterStatus = null;
  document.getElementById('filter_status').value = '';
  document.getElementById('filter_brand').value = '';
  document.getElementById('filter_date').value = '';
  document.getElementById('filter_search').value = '';
  sortCol = 'create_time';
  sortDir = 'desc';
  applyFilters();
}

/* ===== 导出 CSV ===== */
function exportCSV() {
  const rows = [['订单号','日期','品牌','编码','数量','已完成','状态','客户','创建时间']];
  filteredOrders.forEach(function(o) {
    const ext = o.extend || {};
    rows.push([
      o.order_no || '',
      (o.order_date || '').slice(0, 10),
      o.brand || '',
      o.brand_code || '',
      o.quantity || 0,
      o.completed_quantity || 0,
      STATUS_MAP[o.status] || o.status,
      ext.customer || '',
      o.create_time ? o.create_time.slice(0, 19) : '',
    ]);
  });
  const csv = rows.map(function(r) {
    return r.map(function(c) { return '"' + String(c).replace(/"/g, '""') + '"'; }).join(',');
  }).join('\\n');
  const bom = '\\ufeff';
  const blob = new Blob([bom + csv], { type: 'text/csv;charset=utf-8' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = 'orders_' + new Date().toISOString().slice(0, 10) + '.csv';
  a.click();
  URL.revokeObjectURL(url);
}

/* ===== 主题切换 ===== */
function toggleTheme() {
  const body = document.body;
  const isDark = body.classList.contains('dark');
  if (isDark) {
    body.classList.remove('dark');
    body.classList.add('light');
    document.getElementById('theme_btn').textContent = '\\u2600';
  } else {
    body.classList.remove('light');
    body.classList.add('dark');
    document.getElementById('theme_btn').textContent = '\\u{1F319}';
  }
  try { localStorage.setItem('dashboard_theme', body.classList.contains('dark') ? 'dark' : 'light'); } catch(e) {}
}

function loadTheme() {
  try {
    const saved = localStorage.getItem('dashboard_theme');
    if (saved === 'light') {
      document.body.classList.remove('dark');
      document.body.classList.add('light');
      document.getElementById('theme_btn').textContent = '\\u2600';
    }
  } catch(e) {}
}

/* ===== 键盘快捷键 ===== */
document.addEventListener('keydown', function(e) {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT' || e.target.tagName === 'TEXTAREA') {
    if (e.key === 'Escape') {
      e.target.blur();
      resetFilters();
    }
    return;
  }
  if (e.key === ' ' || e.code === 'Space') {
    e.preventDefault();
    togglePause();
  } else if (e.key === 'r' || e.key === 'R') {
    manualRefresh();
  } else if (e.key === 'Escape') {
    resetFilters();
  }
});

/* ===== 自动暂停事件 ===== */
document.getElementById('filter_status').addEventListener('focus', function() { setAutoPause(true); });
document.getElementById('filter_brand').addEventListener('focus', function() { setAutoPause(true); });
document.getElementById('filter_date').addEventListener('focus', function() { setAutoPause(true); });
document.getElementById('filter_search').addEventListener('focus', function() { setAutoPause(true); });
document.getElementById('filter_status').addEventListener('blur', function() { setAutoPause(false); });
document.getElementById('filter_brand').addEventListener('blur', function() { setAutoPause(false); });
document.getElementById('filter_date').addEventListener('blur', function() { setAutoPause(false); });
document.getElementById('filter_search').addEventListener('blur', function() { setAutoPause(false); });

let historyScrollTimeout = null;
document.getElementById('history_scroll').addEventListener('scroll', function() {
  setAutoPause(true);
  clearTimeout(historyScrollTimeout);
  historyScrollTimeout = setTimeout(function() { setAutoPause(false); }, 2000);
});

/* ===== 时钟 ===== */
setInterval(function() { document.getElementById('clock').textContent = new Date().toLocaleString('zh-CN'); }, 1000);

/* ===== 初始化 ===== */
loadTheme();
requestNotificationPermission();
loadData();
startRefresh();
</script>
</body>
</html>"""


class OrderHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _send(self, code, ct, body):
        self.send_response(code)
        self.send_header("Content-Type", ct)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, data):
        self._send(200, "application/json; charset=utf-8",
                   json.dumps(data, ensure_ascii=False, default=str).encode())

    def do_GET(self):
        p = urlparse(self.path).path
        if p == "/" or p == "/index.html":
            self._send(200, "text/html; charset=utf-8", build_html().encode())
        elif p == "/api/orders":
            with _cache_lock:
                self._json({
                    "orders": _cache["orders"],
                    "stats": _cache["stats"],
                    "total": _cache.get("total", {}),
                    "last_update": _cache["last_update"],
                    "error": _cache["error"],
                    "fetch_duration": _cache.get("fetch_duration", 0),
                })
        elif p == "/api/history":
            self._json({"history": fetch_history()})
        else:
            self._send(404, "text/plain", b"not found")


class ReuseHTTPServer(HTTPServer):
    allow_reuse_address = True


def main():
    t = threading.Thread(target=bg_refresh, daemon=True)
    t.start()
    fetch_orders()  # 首次同步加载
    server = ReuseHTTPServer(("0.0.0.0", HTTP_PORT), OrderHandler)
    print(f"G1D 订单看板 V3.3 已启动: http://0.0.0.0:{HTTP_PORT}/")
    server.serve_forever()


if __name__ == "__main__":
    main()
