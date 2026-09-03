# 感知3 · 电源状态（Power）· 电池电量与急停按键读取

[English](perception03_power_guide.md) | **简体中文**

**一句话**：订阅 `/power/battery/status` 读取电池电量/电压/充放电状态，订阅 `/power/board/key_status` 读取急停是否被按下。本原子不发任何控制指令，是电源状态的纯感知入口。

| 配套代码 |
|---|
| `atom/perception/perception03_power_ros2.py`（ROS2 原生版） |

---

## 1. 速览（点进来先看这块）

### 1.1 跑起来

前提：已按《前置 · 环境配置》（`docs/environment_setup_zh-CN.md`）起好 body_control。

```bash
ssh ubuntu@<机器人IP>
source /home/ubuntu/ros2ws/install/setup.bash   # 每个新终端都要执行
python3 atom/perception/perception03_power_ros2.py
```

预期现象：终端以 1 Hz 持续打印电池和急停状态，例如：

```
[INFO] [atom_power_demo]: [电池] 大电池: 87.3%  48.12V  -3.45A  (放电中)  |  小电池: 0.0%  0.00V  0.00A
[INFO] [atom_power_demo]: [急停] 硬急停: 未按下  软急停: 未触发  供电: 正常
```

按 `Ctrl-C` 退出。

### 1.2 接口

| 项 | 值 |
|---|---|
| 电池话题 | `/power/battery/status` → `bodyctrl_msgs/PowerBatteryStatus` |
| 按键话题 | `/power/board/key_status` → `bodyctrl_msgs/PowerBoardKeyStatus` |
| 上报频率 | 1 Hz（两个话题均为 1 Hz） |
| 控制话题 | **无**（只读） |

### 1.3 关键字段说明

**电池（`PowerBatteryStatus`）**

| 字段 | 含义 | 单位 / 备注 |
|---|---|---|
| `master_battery_power` | 大电池电量 | %（0~100） |
| `master_battery_voltage` | 大电池电压 | V |
| `master_battery_current` | 大电池电流 | A；**负值=放电，正值=充电** |
| `little_battery_*` | 小电池同上 | 同上 |
| `battery_installed` | 已安装的电池（当前固件未填充，始终为 0） | `0x01`=仅小电池，`0x02`=仅大电池，`0x03`=两者均在 |
| `battery_working` | 当前工作中的电池 | `0x01`=小电池，`0x10`=大电池 |

**急停按键（`PowerBoardKeyStatus`）**

| 字段 | 含义 |
|---|---|
| `is_estop.data` | 物理急停按钮是否被按下（`True`=按下，需旋开才能继续） |
| `is_remote_estop.data` | 软件急停是否已触发 |
| `is_power_on.data` | 整机电源是否正常供电 |
| `work_time` | 上电后工作时长（当前固件未填充，始终为 0，不显示） |

---

## 2. 两个核心操作

### 2.1 读电池数据

```python
from bodyctrl_msgs.msg import PowerBatteryStatus

self.sub_bat_ = self.create_subscription(
    PowerBatteryStatus, "/power/battery/status", self._cb_battery, 10)

def _cb_battery(self, msg: PowerBatteryStatus) -> None:
    self._battery = msg   # 存最新一帧

# 使用时读取字段
b = self._battery
print(f"电量: {b.master_battery_power:.1f}%")
print(f"电压: {b.master_battery_voltage:.2f} V")
print(f"电流: {b.master_battery_current:+.2f} A")  # 正=充电, 负=放电
```

### 2.2 读急停状态

```python
from bodyctrl_msgs.msg import PowerBoardKeyStatus

self.sub_key_ = self.create_subscription(
    PowerBoardKeyStatus, "/power/board/key_status", self._cb_key, 10)

def _cb_key(self, msg: PowerBoardKeyStatus) -> None:
    self._key = msg

# 判断急停
if self._key.is_estop.data:
    print("⚠ 急停已按下！请先旋开急停按钮再继续。")
```

⚠ **`is_estop` 是 `std_msgs/Bool` 类型，读布尔值需要 `.data`，不能直接当 bool 用。**

---

## 3. 代码解读（核心）

`perception03_power_ros2.py` 全文 = **4 个模块**，结构与 perception02（六维力）相同——纯读取，定时器打印。

### 3.1 模块地图

| # | 模块 | 代码锚点 | 职责 |
|---|---|---|---|
| 1 | 配置常量 | `BATTERY_TOPIC` / `KEY_STATUS_TOPIC` | 话题名 |
| 2 | 建节点与订阅 | `PowerDemo.__init__` | 建两个 subscriber + 1 Hz 定时器 |
| 3 | 数据回调 | `_cb_battery` / `_cb_key` | 存最新帧到 `_battery` / `_key` |
| 4 | 定时打印 | `_on_print_timer` / `_log_battery` / `_log_key` | 格式化并 log 摘要 |

### 3.2 充放电状态判断

`master_battery_current` 的正负直接反映充放电方向：

```python
def _charge_str(current: float) -> str:
    if current > 0.05:   return "充电中"
    if current < -0.05:  return "放电中"
    return "待机"
```

阈值 `±0.05A` 用于过滤接近零时的浮动噪声，避免在充放电边界反复切换标签。


---

## 4. 改一改，看变化

| 改什么 | 会怎样 |
|---|---|
| 接上充电器后观察 `master_battery_current` | 应变为正值（充电中） |
| 按下急停按钮后观察 `is_estop.data` | 应变为 `True`，旋开后恢复 `False` |
| 增加低电量告警：`if power < 20: get_logger().warn(...)` | 电量不足时弹出警告，适合长时间运行的实验 |
| 订阅 `/power/board/status` 看各模块温度 | 见进阶章节 |

---

## 5. 排错

| 现象 | 原因 / 处理 |
|---|---|
| 一直打印「尚未收到数据」 | body_control 未启动：`ros2 topic list \| grep power` 确认话题在 |
| `master_battery_power` 一直为 0 | 小电池在位但大电池不在，或 BMS 通信异常；确认 `battery_installed` 值 |
| `is_estop.data` 一直为 `True` | 急停按钮未旋开：顺时针旋转急停按钮直至弹出 |
| `import bodyctrl_msgs` 报错 | 没 source：`source /home/ubuntu/ros2ws/install/setup.bash` |

---

## 6. 进阶

- **低电量保护**：在回调或定时器里判断 `master_battery_power < 阈值`，自动停止运动类 demo，是长时间实验的安全措施。
- **电源板详细状态（`/power/board/status`）**：包含各模块（臂/腰/腿）的 MOS 温度、电流、电压最大/最小值，适合调试硬件异常；消息类型为 `bodyctrl_msgs/PowerStatus`，字段极多，按需取用。
- **急停联动**：将 `is_estop` 检查加入控制循环，急停被按下时立即停发运动指令——这是生产级代码的标配安全逻辑。
