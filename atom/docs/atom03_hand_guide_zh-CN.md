# 原3 · 灵巧手（Hand）· 五指张合控制

[English](atom03_hand_guide.md) | **简体中文**

**一句话**：向话题 `/inspire_hand/ctrl/left_hand` 发一条 `JointState`（6 个手指的张合百分比 0~1）让灵巧手张开/握拳，订阅 `/inspire_hand/state/left_hand` 读回当前张合。

| 配套代码 | 演示视频 |
|---|---|
| `atom/demos/atom03_hand_ros2.py`（ROS2 原生版）、`atom/demos/atom03_hand_ros2_robust.py`（生产版） | `atom/assets/videos/` 下与配套代码同名，建议先看 30 秒 |

## 1. 速览（点进来先看这块）

### 1.1 跑起来

前提：已按《前置 · 环境配置》（`atom/docs/environment_setup_zh-CN.md`）起好 body_control（灵巧手随本体驱动一起）。

```bash
ssh ubuntu@<机器人IP>
source /home/ubuntu/ros2ws/install/setup.bash   # 每个新终端都要执行
python3 atom/demos/atom03_hand_ros2.py
```

预期现象：打印当前手指张合 → 完全张开 → 握拳(0.1) → 再张开 → 只弯食指 → 收尾张开。

⚠ 手指别夹到东西/人；握拳建议到 **0.1** 而不是 0.0，给机械留余量、避免顶死。

### 1.2 接口

| 项 | 值 |
|---|---|
| 下发话题 | `/inspire_hand/ctrl/left_hand`（右手 `right_hand`）→ `sensor_msgs/JointState` |
| 状态话题 | `/inspire_hand/state/left_hand` → `sensor_msgs/JointState` |
| 下发字段 | `name[]`（手指ID字符串）· `position[]`（张合百分比 0~1） |
| 手指 ID | `"1"`小指 `"2"`无名指 `"3"`中指 `"4"`食指 `"5"`拇指弯 `"6"`拇指旋 |
| 单位 | **张合百分比**：`0.0`=握紧 / `1.0`=完全张开（⚠ 不是弧度！） |

### 1.3 取值范围与安全

灵巧手没有弧度限位，它的「范围」是**张合百分比 [0, 1]**：

| 项 | 值 |
|---|---|
| `position` 取值 | 每个手指 `0.0`（握紧）~ `1.0`（完全张开） |
| 安全下限 | 握拳建议 **0.1**，别到 `0.0`（会顶死，给机械留余量） |
| 越界处理 | 生产版 robust 自动把值夹到 `[0,1]`；简洁版不夹，别发范围外的值 |

⚠ 灵巧手用**张合百分比**，和头/臂/腰/腿的**弧度**完全不同，别混。

## 2. 三个核心操作

### 2.1 让它动 —— 发一条张合指令

6 个手指的目标张合装进一条 `JointState`：

```python
msg = JointState()
msg.header = Header(stamp=self.get_clock().now().to_msg())
msg.name = FINGER_NAMES                     # ["1","2","3","4","5","6"]
msg.position = [r1, r2, r3, r4, r5, r6]     # 每个 0~1
self.pub.publish(msg)
```

- `position[i]`：第 i 个手指的目标张合（`0.0` 握 ~ `1.0` 张）。
- **单指控制** = 只改那个手指、其余给 `1.0`——如只弯食指：`[1, 1, 1, 0.1, 1, 1]`。

### 2.2 读状态 —— 订阅状态话题

和头/臂同一套三步：

```python
# 1) 订阅（__init__）
self.state_sub_ = self.create_subscription(
    JointState, HAND_STATE_TOPIC, self._on_state, 10)

# 2) 回调把最新值存起来
def _on_state(self, msg):
    self.cur_pos = list(msg.position)   # 6 个手指当前张合(0~1)

# 3) 用之前先 spin 收帧（main 开头收一次再读）
```

也可命令行看：`ros2 topic echo /inspire_hand/state/left_hand`。

### 2.3 守范围 —— 值收在 [0,1]

见 1.3。简洁版不校验（发多少是多少）；生产版 robust 把每个值夹到 `[0,1]`，并对长度/范围告警。

## 3. 代码解读（核心）

`atom03_hand_ros2.py` 全文 = **5 个模块**。和头/臂是同一套「发指令 + 读状态」骨架，区别在**消息类型是 `JointState`、值是张合百分比**。

### 3.1 模块地图

| # | 模块 | 代码锚点 | 职责 | 换手/部位要改？ |
|---|---|---|---|---|
| 1 | 配置常量 | `HAND_CMD_TOPIC` / `HAND_STATE_TOPIC` / `FINGER_NAMES` | 话题名、手指ID列表 | ✅ 话题名（左/右手） |
| 2 | 建节点与收发 | `HandDemo.__init__` | 建 publisher + 订阅 state | ✅ 话题名 |
| 3 | 读状态回调 | `_on_state` | state 帧写入 `cur_pos` | ⭕ 通用不改 |
| 4 | 下发张合 | `set_open_ratio` | 建 `JointState`（6 指百分比）→ `publish` → `sleep` | ⭕ 通用不改 |
| 5 | 主流程 | `main` | 读一次状态 → 一串张合动作 → 关停 | ⭕ 通用不改 |

### 3.2 逐模块看

- **模块 1 配置常量**：下发/状态话题名、6 个手指 ID 字符串。换左右手就改话题名。
- **模块 2 `__init__`**：`create_publisher` 建下发口、`create_subscription` 建状态订阅、`cur_pos=None` 存当前张合。订阅存 `self.state_sub_`（尾下划线约定）。
- **模块 3 `_on_state`**：把每帧 state 的 `position` 存进 `cur_pos`（6 个手指当前张合）。
- **模块 4 `set_open_ratio`**：一条指令的生命周期——建 `JointState` → 填 `name` + `position`(6 值) → `publish` → `sleep` 等到位。
- **模块 5 `main`**：先 spin 收一帧读当前张合并打印，再依次 `set_open_ratio` 走张开/握拳/单指 → 关停。

### 3.3 举一反三

```python
HAND_CMD_TOPIC = "/inspire_hand/ctrl/right_hand"   # 换右手
# 单指手型：改 position 里对应手指的值，其余给 1.0
# 半弯：给中间值，如 0.5
```

> 注意：灵巧手是**独立的接口家族**（`JointState` + 张合百分比），和头/臂/腰/腿的 `CmdSetMotorPosition` + 弧度不同——「举一反三」主要在换左右手、换手型；跨到运动关节要换消息类型。

## 4. 改一改，看变化

| 改什么 | 会怎样 |
|---|---|
| `[0.1]*6` 的 `0.1` 调大/调小 | 握得松/紧 |
| `[1,1,1,1,0.1,0.3]` | 做一个「捏」的手型 |
| 话题改成 `right_hand` | 控制右手 |
| 某个手指给 `0.5` | 半弯 |

先预测再跑，看是否和预期一致。

## 5. 排错

| 现象 | 原因 / 处理 |
|---|---|
| 手不动、也不报错 | `/inspire_hand/ctrl/...` 没订阅者（灵巧手节点没起）：`ros2 topic list \| grep inspire` 确认话题在 |
| 读不到状态 / `cur_pos` 空 | 确认 `/inspire_hand/state/...` 有数据（`ros2 topic hz`）；生产版会提示 `position` 长度是否为 6 |
| 值发了没反应 | 检查发的是**百分比 0~1** 而不是弧度；确认左/右手话题对 |
| 手指顶死/异响 | 值到了 `0.0`，改成 `0.1` 留余量 |

## 6. 进阶

- **生产级加固**：等订阅就绪、状态超时拒动、显式校验（替代 `assert`，`-O` 下不失效）、值域夹到 `[0,1]`、状态长度不符告警、`spin_once` 非负 timeout——见 `atom/demos/atom03_hand_ros2_robust.py`。
- **更细的控制**：因时手另有 **service 接口**可设**力矩 / 速度**（本原子只用最简单的 topic 张合控制）。走 xRocs 等封装时话题/服务名可能不同，另见对应变体。
