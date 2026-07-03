# G1D 机器人控制系统

G1D 机器人烟草分拣系统 —— 统一部署、统一配置、一键启动。基于 ROS2 + Python 的多设备分布式控制系统，支持导航定位、机械臂抓取、立柱升降、视觉微调、订单看板等完整业务闭环。

## 系统架构

```
┌──────────────────────────────────────────────────────────────────┐
│                        G1D 分布式控制系统                         │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐   ┌──────────────────┐   ┌──────────────┐ │
│  │  NUC (5端)       │   │  PC4 (164端)     │   │  4090 服务器  │ │
│  │  192.168.123.5   │   │  192.168.123.164  │   │  192.168.100.100│
│  │  robot           │   │  unitree          │   │  ubuntu       │ │
│  ├──────────────────┤   ├──────────────────┤   ├──────────────┤ │
│  │ :28091 控制API   │   │ :28089 立柱偏移   │   │ :28090 订单看板│ │
│  │ :28087 状态监控   │   │ :18081 YOLO识别   │   │ MySQL 数据库  │ │
│  │ :28092 服务面板   │   │ :18083 机械臂API   │   │              │ │
│  │ :8080  导航服务   │   │ :18084 微调服务   │   │              │ │
│  │ systemd 服务管理  │   │ :18080 夹爪API    │   │              │ │
│  └──────────────────┘   └──────────────────┘   └──────────────┘ │
│        │  ▲                      │  ▲                             │
│        │  └──────────────────────┘  │                             │
│        │     HTTP API 调用          └── 订单数据查询              │
│        │                            MySQL                        │
│        ▼                                                        │
│  ┌──────────────────┐                                           │
│  │ 浏览器仪表盘      │                                           │
│  │ 状态/控制/日志/设置│                                           │
│  └──────────────────┘                                           │
│                                                                  │
│  共享层: lib/g1d_common.py (配置加载/URL构建/SSH/HTTP/日志)       │
│  统一配置: conf/params.ini (所有参数集中管理)                      │
│  一键部署: ./g1d.sh install (systemd 自动安装)                    │
└──────────────────────────────────────────────────────────────────┘
```

## 目录结构

```
xiangmu_ceshi_file/
├── g1d.sh                        # ★ 主入口脚本（统一入口）
├── README.md                     # 本文件
│
├── conf/                         # 所有配置（唯一需要按设备微调的地方）
│   ├── params.ini                # 统一参数（SDK/网络/服务/数据库/ROS2/SSH）
│   ├── task_list.ini             # 任务步骤定义（配置抓取任务流程）
│   └── point_inf                 # 导航点位缓存（由 mainbash.sh 建立）
│
├── lib/                          # 共享库（所有 Python 脚本 import）
│   ├── g1d_common.py             # 核心共享模块（常量/配置/HTTP/SSH/日志）
│   └── robot_nav_arm_flow.py     # 导航客户端（ROS2 导航API封装）
│
├── bin/                          # 可执行脚本
│   ├── g1d_control_api.py        # [5端] 控制API服务 (HTTP REST, port 28091)
│   ├── status_monitor.py         # [5端] 状态监控服务 (port 28087)
│   ├── service_dashboard.py      # [5端] 服务仪表盘 (port 28092)
│   ├── g1d_offset_detector.py    # [164端] 立柱偏移检测 (port 28089)
│   ├── order_dashboard.py        # [4090端] 订单看板 (port 28090)
│   ├── step_executor.py          # [通用] 步骤执行器（旋转/移动/升降/抓取）
│   ├── mainbash.sh               # [5端] 主控交互脚本（点位库/任务执行/SDK控制）
│   └── g1d_utils.sh              # [通用] Shell 工具函数库
│
├── web/
│   └── dashboard_template.py     # 前端仪表盘 HTML/CSS/JS 模板
│
└── services/                     # systemd 服务定义文件
    ├── 164/
    │   └── g1d-offset-detector.service
    ├── 5/
    │   ├── g1d-control-api.service
    │   ├── g1d-monitor.service
    │   └── service-dashboard.service
    └── 4090/
        └── order-dashboard.service
```

## 设备 & 服务映射

| 设备 | IP | SSH 用户 | 部署服务 | 端口 |
|------|-----|----------|---------|------|
| **NUC (5端)** | 192.168.123.5 | robot | 控制API、状态监控、服务面板、导航服务 | 28091, 28087, 28092, 8080 |
| **PC4 (164端)** | 192.168.123.164 | unitree | 立柱Offset检测、YOLO视觉、机械臂API、微调、夹爪 | 28089, 18081, 18083, 18084, 18080 |
| **4090 服务器** | 192.168.100.100 | ubuntu | 订单看板、MySQL数据库 | 28090, 3306 |

> **SSH 用户名**在 `conf/params.ini → [ssh]` 段集中配置，一键修改即可同步到所有脚本。

## 快速开始

### 1. 首次部署

```bash
# 将整个项目文件夹复制到目标设备
# 5端（NUC）
scp -r xiangmu_ceshi_file/ robot@192.168.123.5:~/g1d/

# 164端（PC4）
scp -r xiangmu_ceshi_file/ unitree@192.168.123.164:~/g1d/

# 4090端
scp -r xiangmu_ceshi_file/ ubuntu@192.168.100.100:~/g1d/

# SSH 到目标设备后，一键安装本机 systemd 服务
ssh robot@192.168.123.5
cd ~/g1d
chmod +x g1d.sh
./g1d.sh install
```

### 2. 日常使用

```bash
cd ~/g1d

# 交互菜单（自动检测设备）
./g1d.sh

# 直接启动某个服务（开发调试）
./g1d.sh control       # 控制API
./g1d.sh monitor       # 状态监控
./g1d.sh dashboard     # 服务面板
./g1d.sh offset        # Offset检测
./g1d.sh order         # 订单看板

# 查看本机服务状态
./g1d.sh status

# 编辑配置文件
./g1d.sh config

# 从任意设备 SSH 触发控制命令（示例）
curl -X POST http://192.168.123.5:28091/api/control/rotate \
  -H "Content-Type: application/json" \
  -d '{"angle": 90}'
```

## 配置说明 (`conf/params.ini`)

所有配置集中在一个文件中，修改后重启对应服务即可生效，无需逐个脚本修改。

### `[sdk_params]` — 运行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `near_edge_min_mm` | 150 | 近端边前向最小值（mm） |
| `near_edge_max_mm` | 250 | 近端边前向最大值（mm） |
| `height_threshold_mm` | 50 | 高度偏差阈值（mm） |
| `search_step_height_m` | 0.05 | 搜索时升降步长（m） |
| `search_backup_m` | 0.2 | 搜索后退距离（m） |
| `weitiao_mode` | enhanced | 微调模式：enhanced（新版）/ classic（旧版） |
| `full_travel_m` | 0.427 | 立柱全行程（m） |
| `step_timeout_sec` | 180 | 全局步骤超时（秒） |
| `rotate_segments` | 1 | 旋转分段数 |
| `arm_max_retries` | 2 | 机械臂最大重试次数 |

### `[network]` — 网络拓扑

| 参数 | 说明 |
|------|------|
| `host_5` | 5端（NUC）IP 地址 |
| `host_164` | 164端（PC4）IP 地址 |
| `host_4090` | 4090 服务器 IP 地址 |
| `ssh_user` | SSH 默认用户名 |
| `robot_if` | 机器人网络接口名（如 eth0） |

### `[ssh]` — 每设备独立 SSH 用户

| 参数 | 对应设备 | 默认值 |
|------|---------|--------|
| `ssh_user_5` | NUC (192.168.123.5) | robot |
| `ssh_user_164` | PC4 (192.168.123.164) | unitree |
| `ssh_user_4090` | 4090 服务器 (192.168.100.100) | ubuntu |

> 修改这里即可同时更新所有 Python 脚本中的 SSH 用户名，无需逐个文件修改。

### `[service]` — 服务端口

| 参数 | 端口 | 说明 |
|------|------|------|
| `port_control_api` | 28091 | 控制API |
| `port_monitor` | 28087 | 状态监控 |
| `port_service_dashboard` | 28092 | 服务面板 |
| `port_offset_detector` | 28089 | 立柱偏移检测 |
| `port_dashboard` | 28090 | 订单看板 |
| `port_yolo` | 18081 | YOLO 视觉识别 |
| `port_arm` | 18083 | 机械臂API |
| `port_adjust` | 18084 | 微调服务 |
| `port_gripper` | 18080 | 夹爪控制 |
| `port_nav` | 8080 | 导航服务 |

### `[database]` — MySQL 数据库

订单看板使用的数据库连接信息。环境变量 `MYSQL_HOST`、`MYSQL_PORT`、`MYSQL_USER`、`MYSQL_PASSWORD`、`MYSQL_DB` 优先级高于 ini 配置。

### `[ros2]` — ROS2 配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `topic_hispeed` | /hispeed_state | 立柱高度话题 |
| `topic_odom` | /agv/odom | 里程计话题 |
| `topic_cmd_vel` | /cmd_vel | 底盘速度控制话题 |
| `domain_id` | 0 | ROS_DOMAIN_ID |

## API 文档

### 控制API (`:28091`)

| 端点 | 方法 | 参数 | 说明 |
|------|------|------|------|
| `/api/status` | GET | — | 获取当前状态 |
| `/api/control/rotate` | POST | `angle: float` | 旋转指定角度（度） |
| `/api/control/move` | POST | `direction: str, distance: float` | 前后移动（forward/backward） |
| `/api/control/lift_to` | POST | `height: float` | 升降至目标物理高度 |
| `/api/control/lift_rel` | POST | `direction: str, distance: float` | 相对升降（up/down） |
| `/api/control/stop` | POST | — | 紧急停止 |
| `/api/control/arm` | POST | `phase: str, target: str` | 机械臂操作（PICK/PLACE/RESET） |
| `/api/control/rotate_lift` | POST | `angle: float, height: float` | 并行：旋转+升降 |
| `/api/command` | POST | `cmd: str, ...` | 通用命令（仪表盘使用） |

### 状态监控 (`:28087`)

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/status` | GET | 完整状态 JSON（里程计/高度/YOLO/任务） |
| `/api/events` | GET | 服务端事件流（命令完成/错误通知） |
| `/api/logs` | GET | 操作日志列表 |

### 立柱偏移检测 (`:28089`)

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/basic_status` | GET | 立柱基本状态（offset 值） |

### 订单看板 (`:28090`)

| 端点 | 方法 | 说明 |
|------|------|------|
| `GET /` | GET | 仪表盘 HTML 页面 |
| `/api/orders` | GET | 当前订单列表（含连接池） |
| `/api/history?days=7` | GET | 历史订单（支持 days 参数） |

## 关键技术设计

### 线程安全模型（控制代际）

控制API 使用 `_control_gen` 计数器 + `_active_gens` 集合实现线程安全的任务取消：

- 每个控制命令提交时生成唯一 generation，注册到 `_active_gens`
- 执行循环通过 `_check_stop()` 双重校验（比较 gen + 检查是否在 active 集合中）
- `/api/control/stop` 递增 `_control_gen` 并清空 `_active_gens`，所有运行中任务感知到后退出
- `TaskManager.submit()` 包装器在 `finally` 中清理

### 数据库连接池（订单看板）

`order_dashboard.py` 使用 `Queue(maxsize=5)` 实现简易连接池：

- `get_db_connection()` 获取连接时执行 `conn.ping(reconnect=True)` 检测并重建失效连接
- `release_db_connection()` 归还连接，池满则直接关闭
- 避免每次请求创建/销毁连接的开销

### ROS2 Fallback 模式

`status_monitor.py` 使用本地 `Twist Publisher` 实现 fallback 控制：

- 创建独立的 `/cmd_vel` Publisher（不与主 control 节点冲突）
- 旋转/移动 fallback 通过本地 publish 实现，无需 spawn 子进程
- 升降 fallback 仍需 SSH 子进程（需要远程 SDK 支持）

### 原子日志轮转

`status_monitor.py` 的日志归档使用 `tempfile + os.replace()` 原子操作：

- 先写入临时文件，再 `os.replace()` 到目标路径
- 在 POSIX 系统上 `os.replace` 是原子操作，避免多进程读写到一半的日志

### 每设备 SSH 独立用户

`g1d_common.py` 提供 `get_ssh_params_for_host(host_ip)` 函数：

- 根据目标 IP 自动匹配对应 SSH 用户名
- `run_remote_ssh()` 支持可选的 `ssh_user` 和 `ssh_host` 参数
- 所有升降控制命令使用统一入口，配置变更一键生效

### 前端仪表盘优化

- **状态轮询**：2秒间隔（降低 CPU/网络负载）
- **事件轮询**：1秒间隔（命令反馈保持及时）
- **日志刷新**：30秒间隔（低频数据无需高频刷新）
- **YOLO 图像**：3秒间隔，支持手动触发拍照

## V7.1 改动记录

基于原始代码库的全面优化，共 18 项改进：

| # | 类别 | 文件 | 改动说明 |
|---|------|------|---------|
| 1 | Bug 修复 | `bin/g1d_control_api.py` | 移除重复的 `arm()` 方法定义 |
| 2 | 线程安全 | `bin/g1d_control_api.py` | 新增 `_active_gens` 集合 + 双重停止检查，防止任务取消竞态 |
| 3 | 代码清理 | `bin/status_monitor.py` | 移除未使用的 `notify` 导入 |
| 4 | ROS2 优化 | `bin/status_monitor.py` | Fallback 模式使用本地 Publisher 替代子进程，避免 ROS2 节点冲突 |
| 5 | 配置驱动 | `lib/g1d_common.py` | URL 从 `build_urls_from_config()` 动态构建，不再硬编码 |
| 6 | 安全 | `conf/params.ini` | 数据库密码保留（未改动） |
| 7 | 性能 | `bin/order_dashboard.py` | 新增数据库连接池（Queue, maxsize=5），支持连接健康检查 |
| 8 | 性能 | `bin/order_dashboard.py` | `fetch_history()` 增加 `since_date` 参数化查询过滤 |
| 9 | 线程 | `bin/g1d_offset_detector.py` | ROS2 订阅移到独立线程，主线程专用于 HTTP 服务 |
| 10 | 兼容性 | `g1d.sh` | IP 检测增加 `ip addr show` 和 `ifconfig` 回退策略，支持更多系统 |
| 11 | 可靠性 | `bin/step_executor.py` | `_cmd_vel_twist()` 使用本地 Python Publisher 替代 `ros2 topic pub` shell 命令 |
| 12 | 稳定性 | `bin/mainbash.sh` | `set -e` 替换为 `set -u`，避免 curl/read 等非关键命令导致整个脚本退出 |
| 13 | 可维护性 | `conf/params.ini` + `lib/g1d_common.py` | 新增 `[ssh]` 段，每设备独立 SSH 用户名统一管理 |
| 14 | 前端优化 | `web/dashboard_template.py` | 轮询间隔优化：状态 2s、事件 1s、日志 30s |
| 15 | 文件安全 | `bin/status_monitor.py` | 日志归档改为原子写入（tempfile + os.replace） |
| 16 | 设计规范 | — | 统一配置管理：所有 URL/IP/端口从 params.ini 加载，方便一键切换环境 |
| 17 | 设计规范 | — | HTTP 请求全部使用 `g1d_common.http_get/post` 带重试的统一入口 |
| 18 | 设计规范 | — | 原子写入模式：所有 JSON 文件统一使用 `atomic_write_json()` |

## 环境变量

| 变量名 | 说明 | 默认值 |
|--------|------|--------|
| `G1D_DRY_RUN` | Dry-run 模式（1/true/yes），只打印不执行 | 未设置 |
| `G1D_WEBHOOK_URL` | Webhook 通知地址（企业微信等） | 未设置 |
| `MYSQL_HOST` | 数据库主机（覆盖 ini 配置） | 127.0.0.1 |
| `MYSQL_PORT` | 数据库端口 | 3306 |
| `MYSQL_USER` | 数据库用户 | bwton |
| `MYSQL_PASSWORD` | 数据库密码 | — |
| `MYSQL_DB` | 数据库名 | digitaltwins |
| `EDITOR` | 配置文件编辑器（g1d.sh config） | nano |

## 设计建议与扩展

### 1. 错误处理策略

当前系统已实现多层错误处理：
- **HTTP 层**：`http_get/post` 内置重试机制（次数和间隔可通过 params.ini 配置）
- **步骤层**：`execute_step()` 带超时控制，超时/失败自动标记状态
- **任务层**：`CONTINUE_ON_TASK_ERROR` 开关支持断点续跑
- **进程层**：`cleanup_all()` 在脚本退出时确保 SDK 进程清理

建议增强：
- 增加故障恢复状态机（如升降失败→尝试复位→重试）
- 关键资源泄漏监控（ROS2 话题订阅数、SSH 连接数）

### 2. 配置管理

- 所有可变参数已集中到 `conf/params.ini`
- 环境变量 `MYSQL_*` 和 `G1D_*` 提供运行时覆盖
- SSH 用户名按设备独立配置
- 建议部署时使用 `git` 追踪 `params.ini` 模板，实际密码通过环境变量注入

### 3. 测试建议

建议添加以下测试覆盖：
- **单元测试**：`lib/g1d_common.py` 中的 URL 构建、offset 计算、配置加载函数
- **集成测试**：Dry-run 模式下验证步骤执行器各子命令的参数解析
- **仿真测试**：在 Gazebo/Docker 环境中验证导航+抓取流程

### 4. 扩展方向

- **多机器人支持**：params.ini 中的网络配置可扩展为 `[robot_1]`、`[robot_2]` 等多段
- **监控告警**：Webhook 通知可对接企业微信/钉钉/飞书
- **日志集中**：考虑接入 ELK/Loki 实现跨设备日志聚合

## 故障排查

### 服务启动失败

```bash
# 查看服务日志
sudo journalctl -u g1d-control-api -n 50 --no-pager

# 检查端口是否被占用
sudo ss -tlnp | grep 28091

# 手动启动调试
cd ~/g1d && python3 bin/g1d_control_api.py
```

### IP 检测失败（unknown 设备）

`g1d.sh` 使用三层回退检测 IP：`hostname -I` → `ip addr show` → `ifconfig`。

如仍显示 unknown，请确认：
1. 设备 IP 是否在 `192.168.123.5`、`192.168.123.164`、`192.168.100.100` 范围内
2. 如果是其他 IP，在 `g1d.sh` 的 `detect_device()` 函数中添加对应 case

### ROS2 节点冲突

如果出现 "Node already exists" 错误：
- `step_executor.py` 和 `status_monitor.py` 都使用了本地 Publisher 模式，不再 spawn 子进程，已避免冲突
- 确保没有在多个终端同时启动同一个脚本

### SSH 连接失败

```bash
# 测试 SSH 连通性
ssh -o ConnectTimeout=5 robot@192.168.123.5 "echo ok"

# 确认 SSH 用户名配置正确
grep ssh_user conf/params.ini

# 检查密钥认证
ssh-copy-id robot@192.168.123.5
```

## 常见问题

**Q: 改了 IP 地址需要重启吗？**
A: 编辑 `conf/params.ini` 后重启对应 systemd 服务即可：`sudo systemctl restart g1d-*`

**Q: 如何添加新的抓取任务？**
A: 编辑 `conf/task_list.ini`，按现有格式添加 `[section]`，指定 name/count/point/actions。

**Q: 如何添加新服务？**
A: 在 `bin/` 下放脚本，在 `services/<设备>/` 下放 `.service` 文件，运行 `./g1d.sh install`。

**Q: 多台设备如何同步代码？**
A: 整个项目文件夹复制即可，只有 `conf/params.ini` 中本机相关的参数（IP/SSH 用户）需要按设备微调。建议使用 git 管理代码，每台设备 `git pull` 更新。

**Q: Dry-run 模式怎么用？**
A: `G1D_DRY_RUN=1 python3 bin/step_executor.py rotate --angle=90` 仅打印不执行，用于验证参数。
