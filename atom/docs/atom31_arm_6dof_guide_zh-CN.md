# 原31 · 六维力传感器（6-DoF F/T Sensor）· 腕部力/力矩读取

[English](atom31_arm_6dof_guide.md) | **简体中文**

**一句话**：订阅 `/arm_6dof_left` 和 `/arm_6dof_right`，每帧读取腕部三轴力（Fx/Fy/Fz，N）和三轴力矩（Tx/Ty/Tz，Nm）。本原子不发任何控制指令，是纯感知入口。

| 配套代码 |
|---|
| `atom/demos/atom31_arm_6dof_ros2.py`（ROS2 原生版） |

---

## 1. 速览（点进来先看这块）

### 1.1 跑起来

前提：已按《前置 · 环境配置》（`atom/docs/environment_setup_zh-CN.md`）起好 body_control。

```bash
ssh ubuntu@<机器人IP>
source /home/ubuntu/ros2ws/install/setup.bash   # 每个新终端都要执行
python3 atom/demos/atom31_arm_6dof_ros2.py
```

预期现象：终端以 2 Hz 持续打印左右腕部的力和力矩，例如：

```
[INFO] [atom_ft_sensor_demo]: [左腕] F=(  +0.012,  -0.034,  +9.812) N  T=(  +0.001,  +0.002,  -0.003) Nm
[INFO] [atom_ft_sensor_demo]: [右腕] F=(  +0.008,  -0.021,  +9.793) N  T=(  +0.000,  +0.001,  -0.001) Nm
```

静止时 Fz ≈ 重力分量（手臂自重），其余分量接近 0。用手轻推腕部后可见各轴数值明显变化。

按 `Ctrl-C` 退出。

### 1.2 接口

| 项 | 值 |
|---|---|
| 左腕话题 | `/arm_6dof_left` → `geometry_msgs/WrenchStamped` |
| 右腕话题 | `/arm_6dof_right` → `geometry_msgs/WrenchStamped` |
| 消息字段 | `wrench.force.x/y/z`（N）· `wrench.torque.x/y/z`（Nm） |
| 坐标系 | 传感器本体坐标系（以 URDF frame_id 为准） |
| 上报频率 | 1000 Hz |
| 控制话题 | **无**（六维力传感器只读） |

### 1.3 物理含义

| 字段 | 含义 | 单位 |
|---|---|---|
| `force.x` | 腕部 X 轴受力（沿传感器 X 方向） | N |
| `force.y` | 腕部 Y 轴受力 | N |
| `force.z` | 腕部 Z 轴受力（静止时含重力分量） | N |
| `torque.x` | 腕部绕 X 轴力矩 | Nm |
| `torque.y` | 腕部绕 Y 轴力矩 | Nm |
| `torque.z` | 腕部绕 Z 轴力矩 | Nm |

⚠ 坐标轴方向以 URDF 中传感器 frame 定义为准；正负号与参考坐标系有关，实际标定前以实测为准。

---

## 2. 两个核心操作

### 2.1 读数据 —— 订阅话题

`geometry_msgs/WrenchStamped` 的订阅套路与其他原子的 status 完全一致：

```python
from geometry_msgs.msg import WrenchStamped

# 1) 订阅（__init__）
self.sub_left_ = self.create_subscription(
    WrenchStamped, "/arm_6dof_left", self._cb_left, 10)

# 2) 回调把最新值存起来
def _cb_left(self, msg: WrenchStamped) -> None:
    self._latest["left"] = msg   # 直接存整个 msg

# 3) 用时读取字段
f = msg.wrench.force    # .x  .y  .z  (N)
t = msg.wrench.torque   # .x  .y  .z  (Nm)
```

与关节状态话题不同：这里存整个 `WrenchStamped`（而非只存一个 float），因为力/力矩是 6 个独立分量，直接存 msg 更方便。

### 2.2 降采样打印 —— 用定时器控制频率

原始话题 1000 Hz，直接在回调里打印会让终端刷屏且难以阅读。用 `create_timer` 以固定频率读取最新值并打印：

```python
# 1) 建 2 Hz 定时器（__init__）
self.timer_ = self.create_timer(1.0 / PRINT_HZ, self._on_print_timer)

# 2) 定时器回调里读最新值
def _on_print_timer(self) -> None:
    msg = self._latest["left"]
    if msg is None:
        return
    f, t = msg.wrench.force, msg.wrench.torque
    self.get_logger().info(
        f"[左腕] F=({f.x:+7.3f}, {f.y:+7.3f}, {f.z:+7.3f}) N  "
        f"T=({t.x:+7.3f}, {t.y:+7.3f}, {t.z:+7.3f}) Nm")
```

调大 `PRINT_HZ` 可看到更密集的采样；要获取原始 1000 Hz 数据（用于算法），直接在 `_cb_left` 回调里处理即可，不需要定时器。

---

## 3. 代码解读（核心）

`atom31_arm_6dof_ros2.py` 全文 = **4 个模块**。这是所有原子中最短的一个——六维力传感器只读，无控制流程。

### 3.1 模块地图

| # | 模块 | 代码锚点 | 职责 | 换部位要改？ |
|---|---|---|---|---|
| 1 | 配置常量 | `LEFT_TOPIC` / `RIGHT_TOPIC` / `PRINT_HZ` | 话题名、打印频率 | ✅ 话题名 |
| 2 | 建节点与订阅 | `FTSensorDemo.__init__` | 建两个 subscriber + 打印定时器 | ✅ 话题名 |
| 3 | 数据回调 | `_cb_left` / `_cb_right` | 把最新 WrenchStamped 存入 `_latest` | ⭕ 通用不改 |
| 4 | 定时打印 | `_on_print_timer` / `_log_wrench` | 从 `_latest` 读最新值并 log | ⭕ 通用不改 |

### 3.2 为什么用 `rclpy.spin` 而不是 `spin_once` 循环？

其他原子用 `spin_once` 是因为需要在指令之间穿插等待。本原子没有控制流，定时器和订阅回调完全由 spin 事件循环驱动，`rclpy.spin(node)` 更简洁，直到 Ctrl-C 才退出。

---

## 4. 改一改，看变化

| 改什么 | 会怎样 |
|---|---|
| `PRINT_HZ = 10.0` | 打印更密，能看到 100ms 级别的力变化 |
| 只订阅一侧（注释掉另一个 sub）| 只打印单侧，适合专注测试一只手臂 |
| 用手施加不同方向的力 | 看 `force.x/y/z` 哪个分量响应，验证坐标轴方向 |
| 手臂保持静止，读 Fz | 可估算手臂末端的有效重力分量（用于重力补偿标定） |

先预测哪个轴会响应，推完再看，看是否和预期一致。

---

## 5. 排错

| 现象 | 原因 / 处理 |
|---|---|
| 两侧一直打印「尚未收到数据」 | body_control 未启动，或六维力驱动未加载：`ros2 topic list \| grep 6dof` 确认话题在 |
| 话题存在但始终无数据 | `ros2 topic hz /arm_6dof_left` 看频率；若为 0，检查 body_control 日志有无传感器报错 |
| `import geometry_msgs` 报错 | 没 source：`source /home/ubuntu/ros2ws/install/setup.bash`（每个新终端都要） |
| 静止时数值大幅漂移 | 传感器需要标零（零偏补偿） |

---

## 6. 进阶

- **零偏补偿**：静止时读取若干帧取均值作为 offset，后续每帧减去 offset，得到净外力。这是力控应用的第一步。
- **接触检测**：当 `|force|` 超过阈值时触发停止或顺应运动，可结合力位混合等控制模式使用。
- **重力补偿**：根据手臂姿态计算末端自重在传感器坐标系的投影，从读数中减去，得到纯外力分量。
