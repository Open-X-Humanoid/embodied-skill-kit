# 原12 · 腰部（Waist）· 转腰（yaw）

[English](atom12_waist_guide.md) | **简体中文**

**一句话**：向话题 `/waist/cmd_pos` 发一条 `CmdSetMotorPosition` 转腰（yaw 绕竖直轴拧上身），订阅 `/waist/status` 读回当前角度。腰有 yaw/pitch 两个自由度，**本原子只动 yaw、保持 pitch**（pitch 与腿耦合，单独动有倾覆风险）。

| 配套代码 | 演示视频 |
|---|---|
| `atom/demos/atom12_waist_ros2.py`（ROS2 原生版）、`atom/demos/atom12_waist_ros2_robust.py`（生产版） | `atom/assets/videos/` 下与配套代码同名 |

## 1. 速览（点进来先看这块）

### 1.1 跑起来

前提：已按《前置 · 环境配置》（`atom/docs/environment_setup_zh-CN.md`）起好 body_control。

```bash
ssh ubuntu@<机器人IP>
source /home/ubuntu/ros2ws/install/setup.bash   # 每个新终端都要执行
python3 atom/demos/atom12_waist_ros2.py
```

预期现象：打印当前 yaw/pitch → 向一侧转腰 → 向另一侧 → 回起点，pitch 全程保持不变。

⚠ 腰部承重、力矩大；急停在手。只演示 yaw（水平拧身，不影响身高/平衡）。

### 1.2 接口

| 项 | 值 |
|---|---|
| 下发话题 | `/waist/cmd_pos` → `bodyctrl_msgs/CmdSetMotorPosition` |
| 状态话题 | `/waist/status` → `bodyctrl_msgs/MotorStatusMsg`（含 `pos` 当前角、`error` 错误码） |
| 下发字段 | `name`（电机ID）· `pos`（目标角/rad）· `spd`（速度）· `cur`（最大电流/A，承重需较大） |
| 电机 ID | `31` = yaw 转腰（左右拧身）· `32` = pitch 前倾俯仰（与下肢升降耦合） |
| 单位 | 弧度（`0.2 rad ≈ 11°`），`pos` 是**绝对目标角、不是增量** |

### 1.3 关节限位与安全

⚠ 以 URDF 为准，超限撞机械限位或触发过流保护。

| 关节 | ID | 建议软限位 | URDF 硬限位 |
|---|---|---|---|
| yaw 转腰 | 31 | ±0.5 rad（±29°） | (−2.967, 3.142) rad |
| pitch 前倾 | 32 | ±0.3 rad（±17°） | (−0.785, 2.094) rad |

⚠ **pitch(32) 与下肢升降耦合**：单独乱动会改变整机重心、有**倾覆风险**——本 demo 只把 pitch **保持**在当前测量值，不主动动它。要动 pitch 请走腿部 atom23（与升降协调）。

⚠⚠ **电压不足会报错码 `12832`（欠压）**：腰是承重关节，需要足够供电/力矩。**供电不足时电机报错误码 `12832`**，表现为「发了指令却动不了 / 无力 / 报错」。**生产版 robust 会在运动前自动读 `/waist/status` 的 `error` 字段，发现 `12832` 等错误码就拒绝运动并打印**（见第 6 节）；简洁版不查错误码，可能欠压带病运动。

## 2. 三个核心操作

### 2.1 让它动 —— 发一条腰部指令

腰的一条指令里同时给 yaw 和 pitch——**yaw 给目标角，pitch 给"当前测量值"来保持不动**：

```python
msg = CmdSetMotorPosition()
msg.header = Header(stamp=self.get_clock().now().to_msg())
msg.cmds = [
    SetMotorPosition(name=31, pos=yaw,        spd=SPEED, cur=CURRENT_LIMIT),  # 转腰
    SetMotorPosition(name=32, pos=pitch_hold, spd=SPEED, cur=CURRENT_LIMIT),  # 保持 pitch
]
self.pub.publish(msg)
```

- `cur`（最大电流）比头/臂大（demo 用 `20.0`）——腰承重需要足够力矩。
- **pitch 必须给当前测量值**：这条消息是"腰部整体目标"，不发 pitch 可能被当成 0、导致 pitch 乱动（倾覆风险）。始终把 pitch 保持在当前值才安全。

### 2.2 读角度 —— 订阅状态话题

和头/臂同一套；腰尤其**必须先读到 pitch 当前值**才能"保持"它：

```python
# 1) 订阅（__init__）
self.status_sub_ = self.create_subscription(
    MotorStatusMsg, WAIST_STATUS_TOPIC, self._on_status, 1)

# 2) 回调存最新值（生产版同时记录 error 错误码，用于欠压等故障自检）
def _on_status(self, msg):
    for s in msg.status:
        self.cur_pos[s.name] = s.pos

# 3) wait_status([31, 32]) 先读到 yaw+pitch 才动（读不到就拒动）
```

### 2.3 守限位 & 保持 pitch

yaw 收在软限位内（见 1.3）；pitch 全程 = 当前测量值（不主动动）。简洁版不校验；生产版 robust 做**软限位 + 电机错误码（含欠压 `12832`）双检查**。

## 3. 代码解读（核心）

`atom12_waist_ros2.py` 全文 = **6 个模块**。和头/臂是同一套「发指令 + 读状态」骨架，特殊点在**一次发两个关节：yaw 动、pitch 保持**。

### 3.1 模块地图

| # | 模块 | 代码锚点 | 职责 | 换部位要改？ |
|---|---|---|---|---|
| 1 | 配置常量 | `WAIST_CMD_TOPIC` / `WAIST_STATUS_TOPIC` / `WAIST_YAW_ID` / `WAIST_PITCH_ID` / `CURRENT_LIMIT` / `SPEED` | 话题名、电机ID、限流限速 | ✅ 话题名 |
| 2 | 建节点与收发 | `WaistDemo.__init__` | 建 publisher + 订阅 status | ✅ 话题名 |
| 3 | 读状态回调 | `_on_status` | status 帧写入 `cur_pos` | ✅ 电机ID |
| 4 | 等到状态 | `wait_status` | spin 到读到 yaw+pitch 才动 | ⭕ 通用不改 |
| 5 | 下发指令 | `command` | 建消息（yaw 动 + pitch 保持）→ `publish` → 等到位 | ✅ 电机ID |
| 6 | 主流程 | `main` | 等状态 → 读 yaw0/pitch0 → 转腰来回 → 回起点 | ⭕ 通用不改 |

### 3.2 逐模块看

- **模块 1 配置常量**：话题名、`31`=yaw / `32`=pitch、限流 `CURRENT_LIMIT=20.0`（承重大）、限速、幅度。
- **模块 2 `__init__`**：`create_publisher` + `create_subscription` + `cur_pos={}`。订阅存 `self.status_sub_`（尾下划线约定）。
- **模块 3 `_on_status`**：把每帧 status 的 `name→pos` 刷进 `cur_pos`（生产版还记 `error` 错误码，用于欠压自检）。
- **模块 4 `wait_status`**：spin 到读齐 yaw+pitch 才返回——读不到就**拒动**（腰不能盲发）。
- **模块 5 `command`**：建消息，`31` 给目标 yaw、`32` 给 `pitch_hold`（保持）→ `publish` → 等到位。
- **模块 6 `main`**：`wait_status` → 读 `yaw0/pitch0` → `command` 转腰 ±0.2 来回、pitch 始终传 `pitch0` → 回起点。

### 3.3 举一反三

```python
YAW_AMP = 0.3                       # 转更大角度（勿超 yaw 软限位 0.5）
# 关键：pitch 永远传"当前测量值"来保持，别主动改它（倾覆风险）
```

> 注意：腰和头/臂共用 `CmdSetMotorPosition` + 弧度，套路一致；特殊在**必须读到 pitch 当前值来保持它**——这是承重耦合关节的通用做法。

## 4. 改一改，看变化

| 改什么 | 会怎样 |
|---|---|
| `YAW_AMP` 0.2 → 0.1 | 转腰幅度更小 |
| `SPEED` 调小 | 转得更慢 |
| ~~主动改 pitch~~ | ⚠ 别动 pitch，倾覆风险；要升降走 atom23 腿 |

先预测再跑，看是否和预期一致。

## 5. 排错

| 现象 | 原因 / 处理 |
|---|---|
| ★ **电机报错 `12832`** | **电压不够（欠压）**——腰承重需足够供电。查电池电量/供电，电量低先充电再跑。生产版会在运动前读到该码、拒动并打印，提前发现 |
| 腰不动、也不报错 | `/waist/cmd_pos` 没订阅者（body_control 没起）：`ros2 topic list \| grep waist` 确认话题在 |
| 读不到 `/waist/status` | 确认 body_control 已起、话题有数据（`ros2 topic hz /waist/status`）；读不到时脚本拒动 |
| `import bodyctrl_msgs` 报错 | 没 source：`source /home/ubuntu/ros2ws/install/setup.bash` |

## 6. 进阶

- **生产级加固（含电压/故障自检）**：`atom/demos/atom12_waist_ros2_robust.py` 的 `_check_motor_errors` 会在**每次运动前**读 `/waist/status` 的 `error` 字段，发现非 0 错误码（如 **`12832` 欠压**）就**拒绝运动并打印**——提前发现电压/电机故障，不带病运动。此外还有软限位校验、等订阅就绪、`spin_once` 非负 timeout。
- **要动 pitch（前倾 / 升降）**：走腿部 **atom23**，与下肢协调升降，别单独动腰 pitch（倾覆风险）。
