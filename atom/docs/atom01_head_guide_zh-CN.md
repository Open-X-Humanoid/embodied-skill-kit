# 原1 · 头部（Head）· 位置控制与读角度

[English](atom01_head_guide.md) | **简体中文**

**一句话**：向话题 `/head/cmd_pos` 发一条 `CmdSetMotorPosition` 让头部 3 个电机转到指定角度，订阅 `/head/status` 读回当前角度。

| 配套代码 | 演示视频 |
|---|---|
| `atom/demos/atom01_head_ros2.py`（ROS2 原生版） | `atom/assets/videos/` 下与配套代码同名 |

## 1. 速览（点进来先看这块）

### 1.1 跑起来

前提：已按《前置 · 环境配置》（`atom/docs/environment_setup_zh-CN.md`）起好 body_control。

```bash
ssh ubuntu@<机器人IP>
source /home/ubuntu/ros2ws/install/setup.bash   # 每个新终端都要执行
python3 atom/demos/atom01_head_ros2.py
```

预期现象：打印一次当前角度 → 回正 → 低头 → 抬头 → 左转 → 右转 → 回正，每步停约 1.5 秒。

### 1.2 接口

| 项 | 值 |
|---|---|
| 下发话题 | `/head/cmd_pos` → `bodyctrl_msgs/CmdSetMotorPosition` |
| 状态话题 | `/head/status` → `bodyctrl_msgs/MotorStatusMsg` |
| 下发消息体 | `cmds: SetMotorPosition[]`，每个电机一项 |
| 下发字段 | `name`（电机ID）· `pos`（目标角/rad）· `spd`（速度/rad·s⁻¹）· `cur`（最大电流/A） |
| 状态字段 | `status: MotorStatus[]`，每项 `name`（电机ID）· `pos`（当前角/rad） |
| 电机 ID | `1`=roll 歪头 · `2`=pitch 低头/抬头 · `3`=yaw 左右转头 |
| 单位 | 弧度（`0.25 rad ≈ 14°`），`pos` 是**绝对目标角、不是增量** |

### 1.3 关节限位

⚠ 以 URDF 为准，超限会撞机械限位或触发过流保护。

| 关节 | ID | 建议软限位 | URDF 硬限位 |
|---|---|---|---|
| roll 歪头 | 1 | ±0.30 rad（±17°） | ±0.4538 rad（±26°） |
| pitch 低头/抬头 | 2 | ±0.40 rad（±23°） | ±0.4363 rad（±25°） |
| yaw 左右转头 | 3 | ±0.60 rad（±34°） | ±1.5708 rad（±90°） |

本 demo **不做限位校验**，发多少转多少——目标角要你自己收敛在软限位内。

## 2. 三个核心操作

### 2.1 让它动 —— 发一条位置指令

一次运动 = 组装一条 `CmdSetMotorPosition`、`publish` 出去。三个电机在同一条消息里各给一个目标角：

```python
msg = CmdSetMotorPosition()
msg.header = Header(stamp=self.get_clock().now().to_msg())
msg.cmds = [
    SetMotorPosition(name=1, pos=roll,  spd=MAX_SPEED, cur=MAX_CUR[0]),   # roll
    SetMotorPosition(name=2, pos=pitch, spd=MAX_SPEED, cur=MAX_CUR[1]),   # pitch
    SetMotorPosition(name=3, pos=yaw,   spd=MAX_SPEED, cur=MAX_CUR[2]),   # yaw
]
self.pub.publish(msg)
```

- `pos`：绝对目标角（rad），不是增量。
- `spd`：限速，越小越稳，demo 用 `0.5`。
- `cur`：最大电流（A），保护用。

### 2.2 读角度 —— 订阅状态话题

`/head/status` 会持续推送每个电机的当前角。读取套路三步：

```python
# 1) 订阅（__init__）
self.status_sub_ = self.create_subscription(
    MotorStatusMsg, HEAD_STATUS_TOPIC, self._on_status, 10)

# 2) 回调把最新值存起来
def _on_status(self, msg):
    for s in msg.status:              # 每个电机一条
        self.cur_pos[s.name] = s.pos  # 电机ID -> 当前角(rad)

# 3) 用的时候：status 是被动推送的，先 spin 收帧再读
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.1)
    if node.cur_pos:
        break
print(node.cur_pos)
```

关键点：**必须 `rclpy.spin_once/spin` 跑起来，回调才会触发、`cur_pos` 才有值**。生产写法据此「先读当前角、再从当前角小步增量」，避免开机盲发大角度导致猛跳。

也可以不写代码、直接命令行看：`ros2 topic echo /head/status`。

### 2.3 守限位 —— 目标角别超界

限位表见 1.3。本 demo 不自动拦截，超限的后果是撞机械限位或过流保护。要代码自动校验超限拒发，见第 6 节进阶。

## 3. 代码解读（核心）

`atom01_head_ros2.py` 全文 = **5 个模块**。任何一个「电机位置控制」部位（臂/腰/腿）都是同样 5 块，**换部位只改配置、逻辑不动**。

### 3.1 模块地图

| # | 模块 | 代码锚点 | 职责 | 换部位要改？ |
|---|---|---|---|---|
| 1 | 配置常量 | `HEAD_CMD_TOPIC` / `HEAD_STATUS_TOPIC` / `MAX_SPEED` / `MAX_CUR` | 话题名、限速、限流 | ✅ 话题名 |
| 2 | 建节点与收发 | `HeadDemo.__init__` | 建 publisher + 订阅 status | ✅ 话题名 |
| 3 | 读状态回调 | `_on_status` | status 帧写入 `cur_pos` | ✅ 电机ID |
| 4 | 下发一条指令 | `move_to` | 建消息 → `publish` → `sleep` 等到位 | ✅ 电机ID |
| 5 | 主流程 | `main` | 读一次当前角 → 依次 `move_to` → 关停 | ⭕ 通用不改 |

### 3.2 逐模块看

- **模块 1 配置常量**：话题名、速度、电流都抽成常量。换部位第一件事就是改这里的话题名。
- **模块 2 `__init__`**：`create_publisher` 建下发口、`create_subscription` 建状态订阅、`cur_pos={}` 存当前角。订阅对象存成 `self.status_sub_`（尾下划线，风格约定）。
- **模块 3 `_on_status`**：唯一职责是把每帧 status 的 `name→pos` 刷进 `cur_pos`。回调只存不算，逻辑放外面。
- **模块 4 `move_to`**：一条运动指令的完整生命周期——① 新建消息 → ② 打时间戳 → ③ 每个电机填 `name/pos/spd/cur` → ④ `publish` → ⑤ `time.sleep` 粗略等到位。
- **模块 5 `main`**：先 spin 收帧读一次当前角并打印（演示读取），再依次 `move_to` 走演示动作，最后 `destroy_node` + `shutdown` 收尾。

### 3.3 举一反三：写一个新部位控制器

照模块地图把配置换掉、套路照搬（**下发方法按该部位关节数微调**）：

```python
CMD_TOPIC    = "/arm/cmd_pos"       # 1) 换下发话题
STATUS_TOPIC = "/arm/status"        # 2) 换状态话题
# 3) 电机 ID 换成该部位的（如手臂左 11~17 / 右 21~27）
# 4) 下发方法按部位调整：头一次发 3 关节(move_to)；手臂关节多，常一次发 1 个(move_joint)——
#    套路不变：填 SetMotorPosition(name/pos/spd/cur) → publish
# 5) 目标角自己收敛在该部位 URDF 限位内（本简洁版不校验）
```

## 4. 改一改，看变化

| 改什么 | 会怎样 |
|---|---|
| `move_to(0, 0.25, 0)` 的 `0.25` 调大/调小 | 低头更多/更少（勿超 pitch 软限位 0.40） |
| `MAX_SPEED` 调小 | 转头变慢 |
| 加一句 `move_to(0.1, 0, 0)` | 头歪一点（roll） |

先预测再跑，看是否和预期一致。

## 5. 排错

| 现象 | 原因 / 处理 |
|---|---|
| 头不动、也不报错 | `/head/cmd_pos` 没订阅者（body_control 没起）：`ros2 topic list \| grep head` 确认话题在 |
| `cur_pos` 打印为空 | 没收到 status：确认 body_control 已起、`/head/status` 有数据（`ros2 topic hz /head/status`） |
| `import bodyctrl_msgs` 报错 | 没 source：`source /home/ubuntu/ros2ws/install/setup.bash`（每个新终端都要） |

## 6. 进阶

- **生产级加固**：状态等待就绪、限位校验拒发、用 `spin_once`（非负 timeout）替代 `time.sleep`——见 `atom/demos/atom01_head_ros2_robust.py`。
- **其它运动模式**：头部除位置模式（`cmd_pos`）外还支持其它控制模式（如速度、电流等），不同部位可用的模式也不同。本原子只演示位置模式，其余模式查官方《天轶 2.0 ROS2 SDK 二次开发文档》。
