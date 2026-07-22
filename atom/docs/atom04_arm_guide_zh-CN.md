# 原4 · 手臂（Arm）· 单关节位置控制

[English](atom04_arm_guide.md) | **简体中文**

**一句话**：向话题 `/arm/cmd_pos` 发一条 `CmdSetMotorPosition` 移动手臂的**单个关节**，订阅 `/arm/status` 读回当前角度。这是理解「手臂怎么被控制」的最底层入口。

| 配套代码 | 演示视频 |
|---|---|
| `atom/demos/atom04_arm_ros2.py`（ROS2 原生版）、`atom/demos/atom04_arm_ros2_robust.py`（生产版） | `atom/assets/videos/` 下与配套代码同名 |

## 1. 速览（点进来先看这块）

### 1.1 跑起来

前提：已按《前置 · 环境配置》（`atom/docs/environment_setup_zh-CN.md`）起好 body_control。

```bash
ssh ubuntu@<机器人IP>
source /home/ubuntu/ros2ws/install/setup.bash   # 每个新终端都要执行
python3 atom/demos/atom04_arm_ros2.py
```

预期现象：读取左臂 2 号关节（ID=12）当前角度 → +0.1 → −0.1 → 回起点，全程小幅慢速。

⚠ 手臂力矩大、范围大：先确保臂周围无人无物、急停在手；第一次只动 1~2 个关节、幅度 < 0.3 rad、速度 0.2 rad/s。

### 1.2 接口

| 项 | 值 |
|---|---|
| 下发话题 | `/arm/cmd_pos` → `bodyctrl_msgs/CmdSetMotorPosition` |
| 状态话题 | `/arm/status` → `bodyctrl_msgs/MotorStatusMsg` |
| 下发消息体 | `cmds: SetMotorPosition[]`，一个关节一项（手臂 demo 每次只发一个） |
| 下发字段 | `name`（电机ID）· `pos`（目标角/rad）· `spd`（速度/rad·s⁻¹）· `cur`（最大电流/A） |
| 状态字段 | `status: MotorStatus[]`，每项 `name`（电机ID）· `pos`（当前角/rad） |
| 电机 ID | 左臂 `11~17`、右臂 `21~27`（1=肩俯仰 … 7=腕旋） |
| 单位 | 弧度（`0.1 rad ≈ 5.7°`），`pos` 是**绝对目标角、不是增量** |

### 1.3 关节限位

⚠ 以 URDF 为准，超限会撞机械限位或触发过流保护。下表为左臂（`11~17`）；右臂（`21~27`）中 J2 肩侧展 / J4 肘弯 / J7 腕旋方向**左右镜像**，其余相同。

| 关节 | ID | 建议软限位 | URDF 硬限位 |
|---|---|---|---|
| J1 肩俯仰 | 11 | ±2.96 rad（±170°） | ±2.967 rad |
| J2 肩侧展 | 12 | (−0.26, 2.61) rad（−15°~150°） | (−0.262, 2.618) rad |
| J3 上臂旋转 | 13 | ±2.96 rad（±170°） | ±2.967 rad |
| J4 肘弯 | 14 | (−2.61, 0.26) rad（−150°~15°） | (−2.618, 0.262) rad |
| J5 前臂旋转 | 15 | ±2.96 rad（±170°） | ±2.967 rad |
| J6 腕弯 | 16 | (−0.78, 1.04) rad（−45°~60°） | (−0.785, 1.047) rad |
| J7 腕旋 | 17 | (−1.65, 1.30) rad（−95°~75°） | (−1.658, 1.309) rad |

本简洁版**不做限位校验**；生产版 robust 内置软限位 + 单步位移（≤0.5 rad）双重校验并拒绝超限。目标角要你自己收在软限位内。

## 2. 三个核心操作

### 2.1 让它动 —— 发一条单关节位置指令

手臂一次控一个关节：`cmds` 里只放一个 `SetMotorPosition`。

```python
msg = CmdSetMotorPosition()
msg.header = Header(stamp=self.get_clock().now().to_msg())
msg.cmds = [SetMotorPosition(name=motor_id, pos=target_pos, spd=SPEED, cur=MAX_CUR)]
self.pub.publish(msg)
```

- `pos`：绝对目标角（rad），不是增量。
- `spd`：限速，demo 用 `0.2`（手臂慢一点更安全）。
- `cur`：最大电流（A），保护用。

> 对比头部 atom01：头一次发 3 个关节，手臂一次发 1 个——`cmds` 列表长度不同，套路一样。

### 2.2 读角度 —— 订阅状态话题

和头部同一套三步：

```python
# 1) 订阅（__init__）
self.status_sub_ = self.create_subscription(
    MotorStatusMsg, ARM_STATUS_TOPIC, self._on_status, 1)

# 2) 回调把最新值存起来
def _on_status(self, msg):
    for s in msg.status:
        self.cur_pos[s.name] = s.pos   # 电机ID -> 当前角(rad)

# 3) 用之前先 spin 收帧——wait_for_status(12) 就是 spin 到读到 12 号关节角度为止
```

⚠ 手臂是位置控制，**必须先读当前角、再从当前角小步增量**——不知道当前在哪就直接发目标，可能一次大位移把手臂甩过去。也可命令行看：`ros2 topic echo /arm/status`。

### 2.3 守限位 —— 目标角别超界

限位表见 1.3。本简洁版不自动拦截，超限后果是撞机械限位或过流保护。要代码自动校验（软限位 + 单步幅度）见第 6 节进阶。

## 3. 代码解读（核心）

`atom04_arm_ros2.py` 全文 = **6 个模块**。和头部 atom01 是同一套「位置控制」骨架，区别只在手臂一次控**单个关节**。

### 3.1 模块地图

| # | 模块 | 代码锚点 | 职责 | 换部位要改？ |
|---|---|---|---|---|
| 1 | 配置常量 | `ARM_CMD_TOPIC` / `ARM_STATUS_TOPIC` / `DEMO_JOINT_ID` / `SPEED` / `MAX_CUR` | 话题名、演示关节、限速限流 | ✅ 话题名 |
| 2 | 建节点与收发 | `ArmDemo.__init__` | 建 publisher + 订阅 status | ✅ 话题名 |
| 3 | 读状态回调 | `_on_status` | status 帧写入 `cur_pos` | ✅ 电机ID |
| 4 | 等到当前角度 | `wait_for_status` | spin 到读到该关节角，作运动起点 | ⭕ 通用不改 |
| 5 | 下发单关节 | `move_joint` | 建消息（1 个 `SetMotorPosition`）→ `publish` → `sleep` | ✅ 电机ID |
| 6 | 主流程 | `main` | 读起点 → ±0.1 来回 → 回起点 | ⭕ 通用不改 |

### 3.2 逐模块看

- **模块 1 配置常量**：话题名、演示关节 ID、速度、电流。换部位第一件事就是改话题名。
- **模块 2 `__init__`**：`create_publisher` 建下发口、`create_subscription` 建状态订阅、`cur_pos={}` 存当前角。订阅存成 `self.status_sub_`（尾下划线，风格约定）。
- **模块 3 `_on_status`**：把每帧 status 的 `name→pos` 刷进 `cur_pos`，只存不算。
- **模块 4 `wait_for_status`**：spin 到读到目标关节角度并返回，作为**运动起点**——这是「先读后动」的关键。⚠ 简洁版超时会假设 `0.0`（有跳变风险）；生产版返回 `None` 并拒绝运动。
- **模块 5 `move_joint`**：一条运动指令的生命周期——新建消息 → 填**一个**电机的 `name/pos/spd/cur` → `publish` → `time.sleep` 等到位。
- **模块 6 `main`**：`wait_for_status` 读起点 → `move_joint` 做 ±0.1 来回 → 回起点 → 关停。

### 3.3 举一反三：换关节 / 换部位

```python
DEMO_JOINT_ID = 22               # 换关节：改动右臂 2 号（左 11~17 / 右 21~27）
CMD_TOPIC     = "/xxx/cmd_pos"    # 换部位：换下发/状态话题名
# 下发方法按部位调整：手臂一次发 1 个关节(move_joint)；头部一次发 3 个(move_to，见 atom01)
# 目标角自己收在该部位 URDF 限位内（本简洁版不校验）
```

## 4. 改一改，看变化

| 改什么 | 会怎样 |
|---|---|
| `DEMO_JOINT_ID = 22` | 改成动右臂 2 号关节 |
| 增量 `0.1` → `0.05` | 动得更小 |
| `SPEED` 调小 | 动得更慢 |

先预测再跑，看是否和预期一致。

## 5. 排错

| 现象 | 原因 / 处理 |
|---|---|
| 关节不动、也不报错 | `/arm/cmd_pos` 没订阅者（body_control 没起）：`ros2 topic list \| grep arm` 确认话题在 |
| 读不到 `/arm/status` | 简洁版会警告并假设当前角=0.0（**有大位移风险**）——**先别继续**，确认 `ros2 topic hz /arm/status` 有数据 |
| `import bodyctrl_msgs` 报错 | 没 source：`source /home/ubuntu/ros2ws/install/setup.bash`（每个新终端都要） |
| 关节报错/不使能 | 关节可能处于错误态：确认 body_control 日志无异常、急停已松开 |

## 6. 进阶

- **生产级加固**：等订阅就绪、状态超时返回 `None` 拒动、软限位 + 单步幅度双校验、`spin_once`（非负 timeout）替代 `time.sleep`——见 `atom/demos/atom04_arm_ros2_robust.py`。
- **手臂的高级控制**：本原子是最底层的裸关节位置控制。工程里手臂还有更上层的 **xArm 封装**（带逆解、避障、力控），后续提供。
