# 运控11 · 底盘（Chassis）· 限时移动（思岚 REST）

[English](motion11_chassis_guide.md) | **简体中文**

**一句话**：用思岚底盘的 REST 动作接口（`MoveByAction`，**限时自停**）让底盘前进/后退/转向，读 `/localization/pose` 看位姿（状态）。

| 配套代码 | 演示视频 |
|---|---|
| `atom/motion/motion11_chassis_slamware.py`（真机可运行版）、`atom/motion/motion11_chassis_slamware_robust.py`（生产版） | `atom/motion/assets/videos/` 下与配套代码同名 |

## 1. 速览（点进来先看这块）

### 1.1 跑起来

前提：在**能连底盘 `192.168.11.x` 子网的机器人板子上**（x86/Orin），底盘上电、急停松开、四周留 >1m。本原子走 REST（纯 `requests`），**不需要 body_control、也不用 source ros2ws**。

```bash
ssh ubuntu@<机器人IP>          # ssh 到板子（网线直连口 x86 192.168.41.1 / Orin 192.168.41.2）
python3 atom/motion/motion11_chassis_slamware.py
# 输入 GO 确认 → 打印运动前位姿 → 前进/后退/转向（均限时自停）→ 打印运动后位姿
```

⚠ 中途 `Ctrl-C` 会自动发**软急停**（取消当前动作）；物理急停始终在手。

### 1.2 接口

| 项 | 值 |
|---|---|
| 底盘 API 基址 | `http://192.168.11.1:1448`（厂商默认；部署不同就改 `BASE_URL`） |
| 下发动作 | `POST /api/core/motion/v1/actions` → `MoveByAction`，`options={direction, duration}` |
| direction | `0`前进 `1`后退 `2`右转 `3`左转 |
| duration | 毫秒(ms)，**到时底盘自停** |
| 读位姿（状态） | `GET /api/core/slam/v1/localization/pose` → `x` / `y` / `yaw` |
| 软急停 | `DELETE /api/core/motion/v1/actions/:current` |
| 依赖 | Python `requests`（**无需 ROS2**） |

### 1.3 安全约束

底盘没有关节限位，它的安全靠**限时自停 + 子网隔离 + 物理留空**：

| 约束 | 说明 |
|---|---|
| 限时自停 | 每个 MoveBy 带 `duration`，到时底盘自己停——脚本崩了也不飞车 |
| 必须在底盘子网 | 只能在能连 `192.168.11.x` 的板子上跑，**不能在自己笔记本上跑** |
| 四周留空 | 运行前架空或四周留 >1m，物理急停在手 |
| 软急停 | `Ctrl-C` / `DELETE :current` 立刻取消当前动作 |

⚠ **为什么用 REST 不用 `/cmd_vel`**：REST 限时自停、走思岚运动控制、可软急停；裸 `/cmd_vel` 是开环速度、需持续下发、脚本一停可能继续跑（飞车）。真机首选 REST。

## 2. 三个核心操作

### 2.1 让它动 —— POST 一个限时动作

```python
body = {"action_name": "slamtec.agent.actions.MoveByAction",
        "options": {"direction": DIRECTION[direction], "duration": int(duration_ms)}}
r = requests.post(ACTIONS, json=body,
                  headers={"accept": "application/json", "Content-Type": "application/json"},
                  timeout=TIMEOUT)
action_id = r.json().get("action_id")
```

- `direction`：`0/1/2/3`（前/后/右/左）。
- `duration`：毫秒，到时底盘自停——所以每段都短、安全。

### 2.2 读状态 —— GET 位姿

底盘的「状态」是位姿 `(x, y, yaw)`：

```python
r = requests.get(POSE, headers={"accept": "application/json"}, timeout=TIMEOUT)
d = r.json()
pose = (d["x"], d["y"], d["yaw"])
```

⚠ **关键**：action 报 `result:0` **≠ 轮子真的转了**。思岚固件把动作规划执行完就回成功，但**离合(手动推行)按下 / 电机未使能 / 急停未松**时会「报成功却纹丝不动」。**读运动前后位姿、比对 `x/y/yaw` 变化**才是硬证据——位姿骗不了人。

### 2.3 软急停 —— DELETE :current

```python
requests.delete(f"{ACTIONS}/:current", timeout=TIMEOUT)   # 取消当前动作，底盘立停
```

脚本用信号处理：`Ctrl-C` / 被杀时自动 `soft_stop`，避免失控。

## 3. 代码解读（核心）

`motion11_chassis_slamware.py` 全文 = **5 个模块**。注意底盘走 **REST（`requests`）**，和其它原子的 ROS2 话题是不同的接口家族。

### 3.1 模块地图

| # | 模块 | 代码锚点 | 职责 | 换部署要改？ |
|---|---|---|---|---|
| 1 | 配置常量 | `BASE_URL` / `ACTIONS` / `POSE` / `DIRECTION` / `PLAN` | API 地址、方向映射、动作计划 | ✅ `BASE_URL`（底盘 IP） |
| 2 | 下发运动 | `move_by` | POST `MoveByAction` → `sleep` 等限时跑完 | ⭕ 通用不改 |
| 3 | 读状态 | `get_pose` | GET `/localization/pose` 读 x/y/yaw | ⭕ 通用不改 |
| 4 | 软急停 | `soft_stop` + `_signal_handler` | DELETE `:current`；Ctrl-C 自动急停 | ⭕ 通用不改 |
| 5 | 主流程 | `confirm` + `main` | GO 确认 → 读位姿 → 依次 `move_by` → 读位姿 | ⭕ 通用不改 |

### 3.2 逐模块看

- **模块 1 配置常量**：API 基址、动作/位姿/急停三个 endpoint、方向映射 `DIRECTION`、动作计划 `PLAN`。换底盘/部署就改 `BASE_URL`。
- **模块 2 `move_by`**：组装 `MoveByAction` 的 body → `POST` → 拿 `action_id` → `sleep(duration+0.4s)` 等它限时跑完。
- **模块 3 `get_pose`**：`GET /localization/pose` 读回 `(x, y, yaw)`——底盘的状态读取。
- **模块 4 `soft_stop` + 信号**：`DELETE :current` 取消当前动作；`Ctrl-C`/被杀时自动软急停，脚本失控也能停住。
- **模块 5 `main`**：`confirm` 交互确认（输 GO + 倒计时）→ 读运动前位姿 → 依次 `move_by` → 读运动后位姿（对比看动没动）。

### 3.3 举一反三

```python
BASE_URL = "http://<你的底盘IP>:1448"   # 部署不同就改这里
PLAN = [("forward", 1500), ("left", 600)]   # 改动作计划：方向 + 时长(ms)
```

> 注意：底盘是 **REST 接口家族**（HTTP + `requests`），和头/臂/手的 ROS2 话题不同——「举一反三」在换底盘 IP、改动作计划；跨到运动关节是另一套接口。

## 4. 改一改，看变化

| 改什么 | 会怎样 |
|---|---|
| `PLAN` 里某段 `duration` | 那段走更久/更短 |
| 加一段 `("right", 800)` | 多一个右转 |
| `BASE_URL` | 底盘 IP 不同（部署差异）时改这里 |

先预测再跑，看是否和预期一致。

## 5. 排错

| 现象 | 原因 / 处理 |
|---|---|
| ★ **报成功但纹丝不动**（`result:0` 却不动） | 先查底盘上的**手动推行/离合(free-wheel)按钮**是否被按下——按下时电机与驱动脱开(松抱闸)以便人力推整机，此时 MoveBy 驱动不了轮子，但固件仍回 `result:0`，造成"API 一路成功、机器人却不动"的假象。**释放该按钮**即恢复。再查：物理急停是否按下、是否在充电桩、电量。读运动前后位姿确认到底动没动 |
| 连不上 API | 多半是**你不在底盘子网**（在笔记本上跑了）：先 `ssh` 到板子（x86 `192.168.41.1` / Orin `192.168.41.2`）再跑；确认底盘上电、急停松开、开机自检完成 |
| IP 不对 | 把 `BASE_URL` 改成你底盘的实际地址 |

## 6. 进阶

- **生产级加固**（见 `atom/motion/motion11_chassis_slamware_robust.py`）：HTTP 失败重试、信号用标志位而非在处理器里执行副作用、**运动前后位姿自检并自动报警「报成功却没动」**、软急停。
- **裸 `/cmd_vel` 变体**：仅供理解最底层开环速度控制，有飞车风险，**不作首选**，另见对应变体。
