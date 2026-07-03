#!/usr/bin/env python3
"""G1D 仪表盘 HTML 模板 V4.1
- 状态页：参数卡片内嵌进度条 + YOLO 四画面 + 任务内嵌健康行
- 控制页：状态概览 + 远程控制
- Tab 分页：状态 / 控制 / 日志 / 设置
"""

DASHBOARD_HTML = r"""<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1D 仪表盘</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,sans-serif;background:#1e1e2f;color:#eee;font-size:15px}
.tab-nav{display:flex;background:#151528;border-bottom:2px solid #2a2a3e;position:sticky;top:0;z-index:100}
.tab-btn{flex:1;padding:12px 6px;border:none;background:transparent;color:#888;font-size:1em;font-weight:bold;cursor:pointer;border-bottom:3px solid transparent;transition:all .2s}
.tab-btn:hover{color:#ccc;background:#1a1a2e}
.tab-btn.active{color:#6cf;border-bottom-color:#6cf;background:#1e1e2f}
.tab-page{display:none;padding:10px}
.tab-page.active{display:block}
/* 参数卡片（带内嵌进度条） */
.card{background:#2a2a3e;border-radius:8px;padding:8px 12px;margin:3px 0;box-shadow:0 1px 3px rgba(0,0,0,.2)}
.card-bar{height:4px;background:#1a1a2e;border-radius:2px;margin-top:4px;overflow:hidden}
.card-bar-fill{height:100%;border-radius:2px;transition:width .5s}
.val{font-size:1.1em;font-weight:bold;color:#6cf}
.good{color:#6f6}.warn{color:#fa0}.bad{color:#f44}
.status-dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:4px}
.grid4{display:grid;grid-template-columns:repeat(4,1fr);gap:5px}
.offset-card{background:#1a2a1e;border:1px solid #3a5a3e}
/* 健康+任务 一行 */
.info-strip{background:#22223a;border-radius:8px;padding:8px 12px;margin:5px 0;display:flex;flex-wrap:wrap;align-items:center;gap:8px 14px;font-size:.88em}
.info-strip .sep{color:#3a3a4e;font-size:.8em}
.task-inline{margin-left:auto;display:flex;align-items:center;gap:6px;font-size:.85em}
.task-inline .step-chip{padding:2px 8px;border-radius:10px;font-weight:bold;font-size:.8em}
.step-pending{background:#333;color:#888}.step-running{background:#2a4a6e;color:#6cf;animation:chipPulse 1.5s infinite}
.step-success{background:#1e4a1e;color:#6f6}.step-failed{background:#4a1e1e;color:#f44}
.step-arrow{color:#555;font-size:.75em}
@keyframes chipPulse{0%,100%{opacity:1}50%{opacity:.6}}
/* YOLO 目标信息 */
.yolo-target-card{background:#1a2a3e;border:1px solid #2a4a6e;border-radius:8px;padding:10px 14px;margin:5px 0}
.yolo-target-name{font-size:1.3em;font-weight:bold;color:#6cf;margin-right:10px}
.yolo-target-conf{font-size:1.1em;font-weight:bold}
.yolo-target-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:5px 10px;margin-top:6px;font-size:.92em}
.yolo-target-grid .val{font-size:1em}
.alignment-badge{padding:3px 10px;border-radius:12px;font-size:.85em;font-weight:bold}
/* YOLO 四画面 */
.yolo-4img{display:grid;grid-template-columns:repeat(4,1fr);gap:5px;margin:6px 0}
.yolo-4img-box{background:#0a0a1e;border-radius:6px;overflow:hidden;text-align:center;position:relative}
.yolo-4img-box img{width:100%;height:auto;display:block;min-height:30px}
.yolo-4img-label{padding:2px 5px;font-size:.72em;color:#aaa;background:#1a1a2e}
.yolo-4img-placeholder{position:absolute;top:0;left:0;right:0;bottom:0;display:flex;align-items:center;justify-content:center;color:#666;font-size:.75em;background:#0a0a1e}
.yolo-4img-box img[src=""] + .yolo-4img-placeholder{display:flex}
.yolo-4img-box img:not([src=""]) + .yolo-4img-placeholder{display:none}
/* 控制页 */
.ctrl-status-bar{display:grid;grid-template-columns:repeat(4,1fr);gap:5px;margin-bottom:10px}
.ctrl-status-item{background:#1a1a2e;border-radius:6px;padding:5px 8px;text-align:center}
.ctrl-status-label{color:#888;font-size:.75em}
.ctrl-status-val{font-size:.95em;font-weight:bold;color:#6cf}
.ctrl-section{margin-bottom:12px}
.ctrl-section h3{margin:0 0 6px;color:#6cf;font-size:.95em}
.ctrl-row{display:flex;gap:5px;align-items:center;flex-wrap:wrap;margin:5px 0}
.cbtn{padding:7px 14px;border:none;border-radius:8px;cursor:pointer;font-size:.9em;font-weight:bold;transition:all .15s;min-width:46px;text-align:center}
.cbtn:hover{transform:translateY(-2px);box-shadow:0 3px 8px rgba(0,0,0,.3)}
.cbtn:active{transform:translateY(0)}
.cbtn-blue{background:linear-gradient(135deg,#2a5a8e,#1a3a5e);color:#8cf}
.cbtn-green{background:linear-gradient(135deg,#1e5a1e,#1a3a1a);color:#7f7}
.cbtn-red{background:linear-gradient(135deg,#6a1a1a,#4a1010);color:#f88}
.cbtn-orange{background:linear-gradient(135deg,#5a3a1a,#3a2a0a);color:#fc8}
.cbtn-stop{background:linear-gradient(135deg,#8a1a1a,#5a0a0a);color:#f44;font-size:1.05em;padding:10px 24px;min-width:100px;animation:pulse 2s infinite}
@keyframes pulse{0%,100%{box-shadow:0 0 0 0 rgba(255,68,68,.4)}50%{box-shadow:0 0 0 8px rgba(255,68,68,0)}}
.cinput{width:65px;padding:5px 8px;border:2px solid #3a5a6e;border-radius:6px;background:#1a1a2e;color:#eee;font-size:.9em;text-align:center}
.cinput:focus{outline:none;border-color:#6cf}
.cmd-feedback{background:#0a0a1e;border:1px solid #2a4a6e;border-radius:8px;padding:8px;margin:6px 0;max-height:140px;overflow-y:auto;font-family:monospace;font-size:.82em;line-height:1.4;white-space:pre-wrap;word-break:break-all}
.cmd-feedback-title{color:#6cf;font-weight:bold;margin-bottom:3px}
/* 日志 */
.log-box{background:#0a0a1e;border-radius:8px;padding:10px;max-height:460px;overflow-y:auto;font-family:monospace;font-size:.9em;line-height:1.7}
.log-entry{padding:3px 0;border-bottom:1px solid #1a1a2e}
.log-ts{color:#666;margin-right:6px}
.log-info{color:#6cf}.log-warn{color:#fa0}.log-error{color:#f44}.log-success{color:#6f6}
.log-detail{background:#1a1a2e;border-radius:5px;padding:5px;margin:2px 0;font-size:.78em;white-space:pre-wrap;max-height:100px;overflow-y:auto}
/* 折叠 */
.collapsible-header{padding:7px 10px;cursor:pointer;display:flex;justify-content:space-between;align-items:center;font-size:.9em;border-radius:6px}
.collapsible-header:hover{background:#22223a}
.collapsed-content{display:none;padding:0 10px 8px}
.collapsed-content.open{display:block}
/* Toast */
.toast{position:fixed;top:14px;right:14px;padding:10px 18px;border-radius:8px;color:#fff;font-weight:bold;font-size:.9em;z-index:9999;animation:slideIn .3s;max-width:320px;box-shadow:0 4px 14px rgba(0,0,0,.4)}
.toast-info{background:#2a4a6e}.toast-success{background:#1e4a1e}.toast-error{background:#4a1e1e}.toast-warn{background:#4a3a1e}
@keyframes slideIn{from{transform:translateX(100%);opacity:0}to{transform:translateX(0);opacity:1}}
@keyframes slideOut{from{opacity:1}to{transform:translateX(100%);opacity:0}}
table{width:100%;border-collapse:collapse;font-size:.88em}
th,td{padding:5px 7px;text-align:left;border-bottom:1px solid #2a2a3e}th{color:#888}
@media(max-width:768px){.grid4{grid-template-columns:1fr 1fr}.ctrl-row{gap:3px}.ctrl-status-bar{grid-template-columns:1fr 1fr}.yolo-target-grid{grid-template-columns:1fr 1fr}.yolo-4img{grid-template-columns:1fr 1fr}.info-strip{font-size:.8em}}
@media(max-width:480px){.grid4{grid-template-columns:1fr 1fr}.cbtn{padding:5px 10px;font-size:.82em}.yolo-4img{grid-template-columns:1fr 1fr}}
</style></head><body>

<div class="tab-nav">
  <button class="tab-btn active" onclick="switchTab('tab_status')">状态</button>
  <button class="tab-btn" onclick="switchTab('tab_control')">控制</button>
  <button class="tab-btn" onclick="switchTab('tab_logs')">日志</button>
  <button class="tab-btn" onclick="switchTab('tab_settings')">设置</button>
</div>

<!-- ==================== 状态页 ==================== -->
<div class="tab-page active" id="tab_status">
  <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:5px">
    <h2 style="font-size:1.1em;color:#6cf">G1D 状态</h2>
    <div><span id="time" style="color:#666;font-size:.85em"></span>
    <button class="cbtn cbtn-blue" onclick="requestNotifications()" style="padding:3px 8px;font-size:.75em">通知</button></div>
  </div>

  <!-- 参数卡片（内嵌进度条） -->
  <div class="grid4">
    <div class="card"><b>位置 X</b><br><span class="val" id="x">-</span> m</div>
    <div class="card"><b>位置 Y</b><br><span class="val" id="y">-</span> m</div>
    <div class="card"><b>朝向</b><br><span class="val" id="yaw">-</span>°</div>
    <div class="card">
      <b>线速度</b><br><span class="val" id="lx">-</span> m/s
      <div class="card-bar"><div class="card-bar-fill" id="bar_speed" style="width:0%;background:#6cf"></div></div>
    </div>
    <div class="card"><b>角速度</b><br><span class="val" id="az">-</span> rad/s</div>
    <div class="card"><b>立柱(SDK)</b><br><span class="val" id="col">-</span> m</div>
    <div class="card offset-card"><b>Offset</b><br><span class="val" id="offset_val">-</span> m</div>
    <div class="card offset-card">
      <b>物理高度</b><br><span class="val" id="phys">-</span> m
      <div class="card-bar"><div class="card-bar-fill" id="bar_height" style="width:0%;background:#6f6"></div></div>
    </div>
  </div>

  <!-- 健康 + 任务 一行 -->
  <div class="info-strip">
    <div><span class="status-dot" id="dot_yolo_health"></span>YOLO: <span id="yolo_health">-</span></div>
    <div><span class="status-dot" id="dot_adjust_health"></span>微调: <span id="adjust_health">-</span></div>
    <div><span class="status-dot" id="dot_arm_health"></span>机械臂: <span id="arm_combined">-</span></div>
    <div>吸盘: <span id="gripper_state">-</span></div>
    <span class="sep">|</span>
    <div class="task-inline">
      <span id="task_name" style="color:#6cf;font-weight:bold">暂无任务</span>
      <span id="task_meta" style="color:#888"></span>
      <span id="steps_container"></span>
    </div>
  </div>

  <!-- YOLO 目标信息 -->
  <div class="yolo-target-card">
    <div style="display:flex;justify-content:space-between;align-items:center">
      <div>
        <b>YOLO</b>
        <span class="yolo-target-name" id="yolo_label">--</span>
        <span class="yolo-target-conf" id="yolo_conf_display">--</span>
      </div>
      <div style="display:flex;align-items:center;gap:6px">
        <span id="alignment_badge" class="alignment-badge">--</span>
        <button class="cbtn cbtn-blue" onclick="refreshYoloImages(true)" style="padding:3px 8px;font-size:.75em">拍照</button>
        <a id="camera_link" href="" target="_blank" style="color:#6cf;font-size:.75em">debug</a>
      </div>
    </div>
    <div class="yolo-target-grid">
      <div>距离: <span class="val" id="yolo_range">-</span>mm</div>
      <div>近端前向: <span class="val" id="yolo_near_fwd">-</span>mm</div>
      <div>中心前向: <span class="val" id="yolo_fwd">-</span>mm</div>
      <div>中心垂直: <span class="val" id="yolo_vert">-</span>mm</div>
      <div>左右偏差: <span class="val" id="yolo_lat">-</span>mm</div>
      <div>朝目标转角: <span class="val" id="yolo_turn">-</span>°</div>
      <div>长轴角: <span class="val" id="yolo_box_yaw">-</span>°</div>
      <div>重投影: <span class="val" id="yolo_reproj">-</span>px
        <div class="card-bar" style="margin-top:3px"><div class="card-bar-fill" id="bar_yolo" style="width:0%;background:#fa0"></div></div>
      </div>
    </div>
  </div>

  <!-- YOLO 四画面横排 -->
  <div class="card" style="margin-top:3px;padding:5px">
    <div class="yolo-4img">
      <div class="yolo-4img-box">
        <div class="yolo-4img-label">左目原图</div>
        <img id="yolo_left_input" src="/api/yolo_img/left_input.jpg" alt="" onerror="this.src=''">
        <div class="yolo-4img-placeholder">暂无</div>
      </div>
      <div class="yolo-4img-box">
        <div class="yolo-4img-label">左目候选</div>
        <img id="yolo_left_candidates" src="/api/yolo_img/left_candidates.jpg" alt="" onerror="this.src=''">
        <div class="yolo-4img-placeholder">暂无</div>
      </div>
      <div class="yolo-4img-box">
        <div class="yolo-4img-label">左目四点</div>
        <img id="yolo_left_points" src="/api/yolo_img/left_points.jpg" alt="" onerror="this.src=''">
        <div class="yolo-4img-placeholder">暂无</div>
      </div>
      <div class="yolo-4img-box">
        <div class="yolo-4img-label">左目上方点</div>
        <img id="yolo_left_projected" src="/api/yolo_img/left_projected.jpg" alt="" onerror="this.src=''">
        <div class="yolo-4img-placeholder">暂无</div>
      </div>
    </div>
  </div>
</div>

<!-- ==================== 控制页 ==================== -->
<div class="tab-page" id="tab_control">
  <h2 style="font-size:1.1em;color:#6cf;margin-bottom:6px">远程控制</h2>
  <div class="ctrl-status-bar">
    <div class="ctrl-status-item"><div class="ctrl-status-label">位置</div><div class="ctrl-status-val" id="cs_pos">-</div></div>
    <div class="ctrl-status-item"><div class="ctrl-status-label">朝向</div><div class="ctrl-status-val" id="cs_yaw">-</div></div>
    <div class="ctrl-status-item"><div class="ctrl-status-label">物理高度</div><div class="ctrl-status-val" id="cs_height">-</div></div>
    <div class="ctrl-status-item"><div class="ctrl-status-label">速度</div><div class="ctrl-status-val" id="cs_speed">-</div></div>
  </div>
  <div class="cmd-feedback" id="cmd_feedback">
    <div class="cmd-feedback-title">命令反馈</div>
    <div id="cmd_feedback_content" style="color:#888">等待命令...</div>
  </div>
  <div class="ctrl-section">
    <h3>旋转</h3>
    <div class="ctrl-row">
      <button class="cbtn cbtn-blue" onclick="cmdRotate(-30)">←30°</button>
      <button class="cbtn cbtn-blue" onclick="cmdRotate(-10)">←10°</button>
      <button class="cbtn cbtn-blue" onclick="cmdRotate(-5)">←5°</button>
      <input class="cinput" id="rotate_angle" type="number" value="90" placeholder="角度">
      <button class="cbtn cbtn-blue" onclick="cmdRotate(parseFloat(document.getElementById('rotate_angle').value))">旋转</button>
      <button class="cbtn cbtn-blue" onclick="cmdRotate(5)">5°→</button>
      <button class="cbtn cbtn-blue" onclick="cmdRotate(10)">10°→</button>
      <button class="cbtn cbtn-blue" onclick="cmdRotate(30)">30°→</button>
    </div>
  </div>
  <div class="ctrl-section">
    <h3>移动</h3>
    <div class="ctrl-row">
      <button class="cbtn cbtn-green" onclick="cmdMove('forward',0.2)">前进0.2</button>
      <button class="cbtn cbtn-green" onclick="cmdMove('forward',0.5)">前进0.5</button>
      <input class="cinput" id="move_dist" type="number" value="0.5" step="0.1" placeholder="距离">
      <button class="cbtn cbtn-green" onclick="cmdMove('forward',parseFloat(document.getElementById('move_dist').value))">前进</button>
      <button class="cbtn cbtn-orange" onclick="cmdMove('backward',parseFloat(document.getElementById('move_dist').value))">后退</button>
      <button class="cbtn cbtn-orange" onclick="cmdMove('backward',0.5)">后退0.5</button>
      <button class="cbtn cbtn-orange" onclick="cmdMove('backward',0.2)">后退0.2</button>
    </div>
  </div>
  <div class="ctrl-section">
    <h3>升降</h3>
    <div class="ctrl-row">
      <button class="cbtn cbtn-green" onclick="cmdLiftTo(0.0)">最低</button>
      <button class="cbtn cbtn-green" onclick="cmdLiftTo(0.1)">0.1</button>
      <button class="cbtn cbtn-green" onclick="cmdLiftTo(0.2)">0.2</button>
      <button class="cbtn cbtn-green" onclick="cmdLiftTo(0.3)">0.3</button>
      <button class="cbtn cbtn-green" onclick="cmdLiftTo(0.4)">0.4</button>
      <input class="cinput" id="lift_height" type="number" value="0.2" step="0.05" placeholder="高度">
      <button class="cbtn cbtn-green" onclick="cmdLiftTo(parseFloat(document.getElementById('lift_height').value))">升至</button>
    </div>
    <div class="ctrl-row">
      <button class="cbtn cbtn-green" onclick="cmdLiftRel('up',0.05)">↑5cm</button>
      <button class="cbtn cbtn-green" onclick="cmdLiftRel('up',0.1)">↑10cm</button>
      <button class="cbtn cbtn-orange" onclick="cmdLiftRel('down',0.05)">↓5cm</button>
      <button class="cbtn cbtn-orange" onclick="cmdLiftRel('down',0.1)">↓10cm</button>
    </div>
  </div>
  <div class="ctrl-section">
    <h3>夹爪 & 机械臂</h3>
    <div class="ctrl-row">
      <button class="cbtn cbtn-green" onclick="cmdGripper('suck')">吸合</button>
      <button class="cbtn cbtn-orange" onclick="cmdGripper('release')">释放</button>
      <button class="cbtn cbtn-blue" onclick="cmdArm('RESET')">机械臂复位</button>
      <button class="cbtn cbtn-blue" onclick="cmdArm('PICK')">PICK</button>
      <button class="cbtn cbtn-blue" onclick="cmdArm('PLACE')">PLACE</button>
    </div>
  </div>
  <div style="text-align:center;margin:14px 0">
    <button class="cbtn cbtn-stop" onclick="cmdStop()">紧急停止</button>
  </div>
</div>

<!-- ==================== 日志页 ==================== -->
<div class="tab-page" id="tab_logs">
  <h2 style="font-size:1.1em;color:#6cf;margin-bottom:6px">操作日志</h2>
  <div style="display:flex;gap:6px;align-items:center;margin-bottom:6px;flex-wrap:wrap">
    <input class="cinput" id="log_search" placeholder="搜索..." style="width:160px;text-align:left" oninput="filterLogs()">
    <button class="cbtn cbtn-blue" onclick="loadLogs()" style="padding:5px 10px;font-size:.8em">刷新</button>
    <span id="log_count" style="color:#666;font-size:.85em">0 条</span>
  </div>
  <div class="log-box" id="log_container"></div>
</div>

<!-- ==================== 设置页 ==================== -->
<div class="tab-page" id="tab_settings">
  <h2 style="font-size:1.1em;color:#6cf;margin-bottom:6px">设置</h2>
  <div class="card">
    <div class="collapsible-header" onclick="toggleContent('points_content')"><b>点位库</b><span id="point_count">0 ▼</span></div>
    <div class="collapsed-content" id="points_content"></div>
  </div>
  <div class="card">
    <div class="collapsible-header" onclick="toggleContent('sdk_content')"><b>SDK 参数</b><span id="sdk_summary">▼</span></div>
    <div class="collapsed-content" id="sdk_content"></div>
  </div>
  <div class="card">
    <div class="collapsible-header" onclick="toggleContent('lastcmd_content')"><b>上次命令详情</b><span>▼</span></div>
    <div class="collapsed-content" id="lastcmd_content">
      <div id="lastcmd_detail" style="color:#888;font-family:monospace;font-size:.85em">暂无</div>
    </div>
  </div>
</div>

<script>
function switchTab(tabId) {
    document.querySelectorAll('.tab-page').forEach(p => p.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.getElementById(tabId).classList.add('active');
    event.target.classList.add('active');
}
function toggleContent(id) { document.getElementById(id).classList.toggle('open'); }

let lastEventTs = 0, logData = [], cmdRunning = false;

function requestNotifications() {
    if (!('Notification' in window)) { alert('浏览器不支持通知'); return; }
    Notification.requestPermission().then(p => showToast(p === 'granted' ? '通知已开启' : '通知被拒绝', p === 'granted' ? 'success' : 'warn'));
}
function sendBrowserNotification(title, body) {
    if (!('Notification' in window) || Notification.permission !== 'granted') return;
    try { new Notification(title, {body}); } catch(e) {}
}
function showToast(msg, type='info') {
    const t = document.createElement('div');
    t.className = 'toast toast-' + type; t.innerText = msg;
    document.body.appendChild(t);
    setTimeout(() => { t.style.animation = 'slideOut .3s'; setTimeout(() => t.remove(), 300); }, 3500);
}

async function sendCommand(cmdData) {
    if (cmdRunning) { showToast('命令执行中，请等待', 'warn'); return; }
    const fb = document.getElementById('cmd_feedback_content');
    fb.innerHTML = '<span style="color:#fa0">⏳ 发送: ' + cmdData.cmd + '...</span>';
    try {
        const resp = await fetch('/api/command', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(cmdData)});
        const result = await resp.json();
        if (result.ok) { showToast(result.msg, 'success'); fb.innerHTML = '<span style="color:#6f6">✅ ' + result.msg + '</span>'; if (['rotate','move','lift_to','lift_rel','arm','nav','weitiao','yolo_pick','task','nav_start_lift','rotate_lift','move_lift','arm_lift'].includes(cmdData.cmd)) { cmdRunning = true; fb.innerHTML += '<br><span style="color:#fa0">⏳ 执行中...</span>'; } }
        else { showToast(result.msg, 'error'); fb.innerHTML = '<span style="color:#f44">❌ ' + result.msg + '</span>'; }
    } catch(e) { showToast('发送失败: '+e, 'error'); fb.innerHTML = '<span style="color:#f44">❌ 失败: '+e+'</span>'; }
}
function cmdRotate(a) { sendCommand({cmd:'rotate',angle:a}); }
function cmdMove(d,l) { sendCommand({cmd:'move',direction:d,distance:l}); }
function cmdLiftTo(h) { sendCommand({cmd:'lift_to',height:h}); }
function cmdLiftRel(d,l) { sendCommand({cmd:'lift_rel',direction:d,distance:l}); }
function cmdGripper(a) { sendCommand({cmd:'gripper',action:a}); }
function cmdArm(p) { sendCommand({cmd:'arm',phase:p}); }
function cmdStop() { sendCommand({cmd:'stop'}); }

let yoloImgRefreshing = false;
async function refreshYoloImages(triggerDetect = false) {
    if (yoloImgRefreshing && triggerDetect) return;
    const t = Date.now();
    if (triggerDetect) {
        yoloImgRefreshing = true;
        try { await fetch('/api/yolo_detect'); } catch(e) {}
        await new Promise(r => setTimeout(r, 1000));
        yoloImgRefreshing = false;
    }
    document.getElementById('yolo_left_input').src = '/api/yolo_img/left_input.jpg?t=' + t;
    document.getElementById('yolo_left_candidates').src = '/api/yolo_img/left_candidates.jpg?t=' + t;
    document.getElementById('yolo_left_points').src = '/api/yolo_img/left_points.jpg?t=' + t;
    document.getElementById('yolo_left_projected').src = '/api/yolo_img/left_projected.jpg?t=' + t;
    const host = location.hostname || '192.168.123.164';
    document.getElementById('camera_link').href = 'http://' + host + ':18081/debug';
}
setInterval(() => refreshYoloImages(false), 3000);

async function loadLogs() {
    try { const resp = await fetch('/api/logs?count=200'); logData = await resp.json(); document.getElementById('log_count').innerText = logData.length + ' 条'; renderLogs(); } catch(e) {}
}
function renderLogs() {
    const search = document.getElementById('log_search').value.toLowerCase();
    const c = document.getElementById('log_container');
    let html = '';
    logData.forEach(e => { let t = JSON.stringify(e).toLowerCase(); if (search && !t.includes(search)) return; let ts = e.ts ? e.ts.split('T')[1]?.split('.')[0]||'' : ''; let cls = e.arm_status==='FAILED'?'log-error':(e.arm_phase?'log-warn':'log-info'); html += `<div class="log-entry ${cls}"><span class="log-ts">${ts}</span>高=${(e.height||0).toFixed(3)} 物理=${e.physical_height!=null?e.physical_height.toFixed(3):'-'} 机械臂=${e.arm_phase||'-'}(${e.arm_status||'-'}) 爪=${e.gripper||'-'} YOLO=${e.yolo_range||'-'}mm 位=(${(e.x||0).toFixed(2)},${(e.y||0).toFixed(2)},${((e.yaw||0)*180/Math.PI).toFixed(1)}°)</div>`; });
    c.innerHTML = html || '<div style="color:#666;padding:14px">暂无日志</div>'; c.scrollTop = c.scrollHeight;
}
function filterLogs() { renderLogs(); }

function updateBars(d) {
    // 高度进度条（内嵌在物理高度卡片中）
    let ph = d.physical_height;
    let bh = document.getElementById('bar_height');
    if (ph != null && ph !== -1) {
        let pct = Math.max(0, Math.min(100, (ph / 0.427) * 100));
        bh.style.width = pct + '%';
        bh.style.background = (ph < 0 || ph > 0.427) ? '#f44' : '#6f6';
    } else { bh.style.width = '0%'; }
    // 速度进度条（内嵌在线速度卡片中）
    let spd = Math.abs(d.velocity.linear_x);
    document.getElementById('bar_speed').style.width = Math.min(100, (spd / 0.5) * 100) + '%';
    // YOLO 距离进度条（内嵌在重投影格子中）
    let yr = d.yolo.range_mm;
    let by = document.getElementById('bar_yolo');
    if (yr > 0) { by.style.width = Math.max(0, Math.min(100, (1 - yr / 1000) * 100)) + '%'; by.style.background = (yr < 600) ? '#6f6' : '#fa0'; }
    else { by.style.width = '0%'; }
}

async function update() {
    try {
        const r = await fetch('/api/status'); const d = await r.json();
        document.getElementById('x').innerText = d.odom.x.toFixed(3);
        document.getElementById('y').innerText = d.odom.y.toFixed(3);
        document.getElementById('yaw').innerText = (d.odom.yaw*180/Math.PI).toFixed(1);
        document.getElementById('lx').innerText = d.velocity.linear_x.toFixed(3);
        document.getElementById('az').innerText = d.velocity.angular_z.toFixed(3);
        document.getElementById('col').innerText = d.column_height.toFixed(4);
        let pe = document.getElementById('phys');
        if (d.physical_height!=null&&d.physical_height!==-1) { pe.innerText=d.physical_height.toFixed(4); pe.className='val '+(d.physical_height<0||d.physical_height>0.427?'bad':'good'); }
        else { pe.innerText='未检测'; pe.className='val warn'; }
        let oe = document.getElementById('offset_val');
        if (d.lift_offset!=null) { oe.innerText=d.lift_offset.toFixed(4); oe.className='val good'; }
        else { oe.innerText='检测中'; oe.className='val warn'; }
        // 机械臂：合并 API健康 + 进程状态 + 阶段/状态 为一行
        const phaseMap = {idle:'空闲',NAVIGATING:'导航中',ADJUSTING:'微调中',PICKING:'抓取中',PLACING:'放置中',RESET:'复位中',DONE:'完成',FAILED:'失败',EXECUTING:'执行中'};
        const statusMap = {idle:'空闲',DONE:'完成',FAILED:'失败',RUNNING:'运行中',PENDING:'等待',SUCCESS:'成功'};
        let armPhase = phaseMap[d.arm_status.phase] || d.arm_status.phase || '';
        let armStatusText = statusMap[d.arm_status.status_text||'idle'] || d.arm_status.status_text || '';
        // 避免重复：如果阶段和状态相同（如都是"空闲"），只显示一个
        let armLabel = (armPhase && armStatusText && armPhase !== armStatusText) ? armPhase + '(' + armStatusText + ')' : (armPhase || armStatusText || '空闲');
        if (!d.arm_process_running) armLabel = '进程停止';
        let armApiOk = d.yolo.arm_api_health === 'OK';
        let armEl = document.getElementById('arm_combined');
        armEl.innerText = armLabel;
        armEl.className = 'val ' + ((armStatusText==='完成'||armStatusText==='成功')?'good':(armStatusText==='失败'?'bad':''));
        let armDot = document.getElementById('dot_arm_health');
        if(armDot) armDot.style.backgroundColor = (armApiOk && d.arm_process_running) ? '#6f6' : '#f44';
        let gs = d.gripper.state.toLowerCase();
        const gripperMap = {suck:'吸合',sucked:'吸合',release:'已释放',released:'已释放',unknown:'未知',down:'离线'};
        document.getElementById('gripper_state').innerText = gripperMap[gs] || d.gripper.state;

        // 任务状态（内嵌在 info-strip 中）
        let tp=d.task_progress;
        if(tp&&tp.item){
            document.getElementById('task_name').innerText = tp.item + (tp.point ? '→' + tp.point : '');
            document.getElementById('task_meta').innerText = `${tp.current}/${tp.total}`;
            let sc=document.getElementById('steps_container');
            sc.innerHTML='';
            (tp.actions||[]).forEach((a,i)=>{
                if(i>0){let ar=document.createElement('span');ar.className='step-arrow';ar.innerText='→';sc.appendChild(ar);}
                let ch=document.createElement('span');ch.className='step-chip step-'+a.status;
                let icon = a.status==='success'?'✓':a.status==='failed'?'✗':a.status==='running'?'⟳':'·';
                ch.innerText=icon+a.name;sc.appendChild(ch);
            });
        } else {
            document.getElementById('task_name').innerText='暂无任务';
            document.getElementById('task_meta').innerText='';
            document.getElementById('steps_container').innerHTML='';
        }

        // YOLO 目标信息
        let y = d.yolo;
        document.getElementById('yolo_label').innerText = y.selected_label || '--';
        let confEl = document.getElementById('yolo_conf_display');
        let conf = y.confidence || 0;
        if (y.selected_label && y.selected_label !== '--') {
            confEl.innerText = 'conf ' + conf.toFixed(3);
            confEl.className = 'yolo-target-conf ' + (conf >= 0.7 ? 'good' : (conf >= 0.4 ? 'warn' : 'bad'));
        } else {
            confEl.innerText = '未检测到';
            confEl.className = 'yolo-target-conf warn';
        }
        document.getElementById('yolo_range').innerText = y.range_mm>0?y.range_mm.toFixed(1):'-';
        document.getElementById('yolo_near_fwd').innerText = (y.near_edge_forward_mm||0)>0?y.near_edge_forward_mm.toFixed(1):'-';
        document.getElementById('yolo_fwd').innerText = (y.center_forward_mm||0)>0?y.center_forward_mm.toFixed(1):'-';
        document.getElementById('yolo_vert').innerText = (y.center_vertical_mm||0)>0?y.center_vertical_mm.toFixed(1):'-';
        document.getElementById('yolo_lat').innerText = (y.lateral_mm||0)!==0?y.lateral_mm.toFixed(1):'-';
        document.getElementById('yolo_turn').innerText = y.turn_first_yaw_deg!==-1?y.turn_first_yaw_deg.toFixed(1):'-';
        document.getElementById('yolo_box_yaw').innerText = y.box_parallel_yaw_deg!==-1?y.box_parallel_yaw_deg.toFixed(1):'-';
        document.getElementById('yolo_reproj').innerText = (y.reproj_error_px||0)>0?y.reproj_error_px.toFixed(2):'-';

        let badge = document.getElementById('alignment_badge'), st = y.alignment_status;
        const bm = {aligned:['✅ 对齐','#1e4a1e','#6f6'],needs_adjust:['⚠ 微调','#4a3a1e','#fa0'],large_deviation:['❌ 偏差','#4a1e1e','#f44']};
        if(bm[st]){badge.innerText=bm[st][0];badge.style.cssText=`background:${bm[st][1]};color:${bm[st][2]};padding:3px 10px;border-radius:12px;font-size:.85em;font-weight:bold`;}
        else{badge.innerText='--';badge.style.cssText='background:#333;color:#888;padding:3px 10px;border-radius:12px;font-size:.85em;font-weight:bold';}

        function sh(id,v){let dot=document.getElementById('dot_'+id),t=document.getElementById(id);if(!t)return;let c=v==='OK'?'good':(v.startsWith('ERR')?'warn':'bad');if(dot)dot.style.backgroundColor=c==='good'?'#6f6':(c==='warn'?'#fa0':'#f44');t.className=c;t.innerText=v;}
        sh('yolo_health',y.yolo_health);sh('adjust_health',y.adjust_health);

        // 控制页状态
        document.getElementById('cs_pos').innerText = `(${d.odom.x.toFixed(2)}, ${d.odom.y.toFixed(2)})`;
        document.getElementById('cs_yaw').innerText = (d.odom.yaw*180/Math.PI).toFixed(1) + '°';
        document.getElementById('cs_height').innerText = (d.physical_height!=null&&d.physical_height!==-1)?d.physical_height.toFixed(3)+'m':'-';
        document.getElementById('cs_speed').innerText = Math.abs(d.velocity.linear_x).toFixed(3) + 'm/s';

        // 设置页
        let pts=d.point_inf||[];document.getElementById('point_count').innerText=pts.length+' ▼';
        let pc=document.getElementById('points_content');
        if(pts.length){let h='<table><tr><th>名称</th><th>X</th><th>Y</th><th>香烟</th></tr>';pts.forEach(p=>{h+=`<tr><td>${p.name}</td><td>${p.x.toFixed(3)}</td><td>${p.y.toFixed(3)}</td><td>${p.cigarette}</td></tr>`;});pc.innerHTML=h+'</table>';}else pc.innerHTML='无数据';
        let params=d.sdk_params||{},pkeys=Object.keys(params);
        document.getElementById('sdk_summary').innerText=pkeys.length?pkeys.length+' ▼':'▼';
        let sk=document.getElementById('sdk_content');
        if(pkeys.length){let h='<table><tr><th>参数</th><th>值</th></tr>';pkeys.forEach(k=>{h+=`<tr><td>${k}</td><td>${params[k]}</td></tr>`;});sk.innerHTML=h+'</table>';}else sk.innerHTML='无数据';
        let lc=d.last_command_result;
        let lcd=document.getElementById('lastcmd_detail');
        if(lc){lcd.innerHTML=`<b>步骤:</b> ${lc.step}<br><b>参数:</b> ${JSON.stringify(lc.kwargs)}<br><b>返回码:</b> ${lc.returncode}<br><b>stdout:</b><br><pre style="color:#6f6;max-height:80px;overflow-y:auto">${lc.stdout||'(空)'}</pre><b>stderr:</b><br><pre style="color:#f44;max-height:80px;overflow-y:auto">${lc.stderr||'(空)'}</pre>`;}

        updateBars(d);
        document.getElementById('time').innerText = new Date().toLocaleTimeString().replace(/:\d{2}$/, '');
    } catch(e) {}
}

async function pollEvents() {
    try {
        const resp = await fetch('/api/events?since='+lastEventTs);
        const events = await resp.json();
        const fb = document.getElementById('cmd_feedback_content');
        events.forEach(e => {
            lastEventTs = Math.max(lastEventTs, e.ts);
            if (e.type==='command_start') { fb.innerHTML='<span style="color:#6cf">⏳ '+e.message+'</span>';
            } else if (e.type==='command_done') { cmdRunning=false; fb.innerHTML='<span style="color:#6f6">✅ '+e.message+'</span>'; if(e.detail) fb.innerHTML+='<div class="log-detail" style="margin-top:3px">'+e.detail+'</div>'; showToast(e.message,'success'); sendBrowserNotification('G1D 完成',e.message);
            } else if (e.type==='command_error') { cmdRunning=false; fb.innerHTML='<span style="color:#f44">❌ '+e.message+'</span>'; if(e.detail) fb.innerHTML+='<div class="log-detail" style="margin-top:3px;color:#f88">'+e.detail+'</div>'; showToast(e.message,'error'); sendBrowserNotification('G1D 错误',e.message);
            } else if (e.type==='emergency') { fb.innerHTML='<span style="color:#f44;font-size:1.05em">🛑 紧急停止！</span>'; showToast(e.message,'error'); sendBrowserNotification('G1D 紧急',e.message);
            } else { showToast(e.message, e.level==='error'?'error':(e.level==='success'?'success':'info')); }
        });
    } catch(e) {}
}

update(); loadLogs(); refreshYoloImages();
setInterval(update, 2000);
setInterval(pollEvents, 1000);
setInterval(loadLogs, 30000);
</script>
</body></html>"""
