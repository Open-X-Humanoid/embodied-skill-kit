# Atom 32 · Power · Battery Level & E-Stop Key Readout

**English** | [简体中文](atom32_power_guide_zh-CN.md)

**In one line**: subscribe to `/power/battery/status` for battery level / voltage / charge–discharge state, and to `/power/board/key_status` for whether the e-stop is pressed. This atom publishes no control commands — it is a pure power-status perception entry point.

| Companion code |
|---|
| `atom/demos/atom32_power_ros2.py` (native ROS2) |

---

## 1. At a Glance (read this first)

### 1.1 Run it

Prerequisite: body_control is up — see *Prerequisite · Environment Setup* (`atom/docs/environment_setup.md`).

```bash
ssh ubuntu@<robot-IP>
source /home/ubuntu/ros2ws/install/setup.bash   # run in every new terminal
python3 atom/demos/atom32_power_ros2.py
```

Expected: the terminal prints battery and e-stop status at 1 Hz, for example:

```
[INFO] [atom_power_demo]: [电池] 大电池: 87.3%  48.12V  -3.45A  (放电中)  |  小电池: 0.0%  0.00V  0.00A
[INFO] [atom_power_demo]: [急停] 硬急停: 未按下  软急停: 未触发  供电: 正常
```

Press `Ctrl-C` to exit.

### 1.2 Interface

| Item | Value |
|---|---|
| Battery topic | `/power/battery/status` → `bodyctrl_msgs/PowerBatteryStatus` |
| Key topic | `/power/board/key_status` → `bodyctrl_msgs/PowerBoardKeyStatus` |
| Publish rate | 1 Hz (both topics) |
| Command topic | **none** (read-only) |

### 1.3 Key fields

**Battery (`PowerBatteryStatus`)**

| Field | Meaning | Unit / notes |
|---|---|---|
| `master_battery_power` | main battery SOC | % (0–100) |
| `master_battery_voltage` | main battery voltage | V |
| `master_battery_current` | main battery current | A; **negative = discharging, positive = charging** |
| `little_battery_*` | small battery, same fields | same |
| `battery_installed` | which batteries are installed (firmware currently leaves this unset — always 0) | `0x01` = small only, `0x02` = main only, `0x03` = both |
| `battery_working` | which battery is active | `0x01` = small, `0x10` = main |

**E-stop key (`PowerBoardKeyStatus`)**

| Field | Meaning |
|---|---|
| `is_estop.data` | physical e-stop pressed (`True` = pressed; twist to release before continuing) |
| `is_remote_estop.data` | software e-stop triggered |
| `is_power_on.data` | main power supply OK |
| `work_time` | uptime since power-on (firmware currently leaves this unset — always 0, not printed) |

---

## 2. The Two Core Operations

### 2.1 Read battery data

```python
from bodyctrl_msgs.msg import PowerBatteryStatus

self.sub_bat_ = self.create_subscription(
    PowerBatteryStatus, "/power/battery/status", self._cb_battery, 10)

def _cb_battery(self, msg: PowerBatteryStatus) -> None:
    self._battery = msg   # keep latest frame

# read fields when needed
b = self._battery
print(f"SOC: {b.master_battery_power:.1f}%")
print(f"Voltage: {b.master_battery_voltage:.2f} V")
print(f"Current: {b.master_battery_current:+.2f} A")  # + = charging, - = discharging
```

### 2.2 Read e-stop status

```python
from bodyctrl_msgs.msg import PowerBoardKeyStatus

self.sub_key_ = self.create_subscription(
    PowerBoardKeyStatus, "/power/board/key_status", self._cb_key, 10)

def _cb_key(self, msg: PowerBoardKeyStatus) -> None:
    self._key = msg

# check e-stop
if self._key.is_estop.data:
    print("⚠ E-stop pressed! Twist to release before continuing.")
```

⚠ **`is_estop` is `std_msgs/Bool` — read the boolean via `.data`; do not treat the field itself as a Python `bool`.**

---

## 3. Code Walkthrough (core)

`atom32_power_ros2.py` is **four modules**, same structure as atom31 (F/T) — pure read, timer-based print.

### 3.1 Module map

| # | Module | Code anchor | Role |
|---|---|---|---|
| 1 | Config constants | `BATTERY_TOPIC` / `KEY_STATUS_TOPIC` | topic names |
| 2 | Node & subscriptions | `PowerDemo.__init__` | two subscribers + 1 Hz timer |
| 3 | Data callbacks | `_cb_battery` / `_cb_key` | store latest frame in `_battery` / `_key` |
| 4 | Timed print | `_on_print_timer` / `_log_battery` / `_log_key` | format and log a summary |

### 3.2 Charge / discharge label

The sign of `master_battery_current` indicates charge direction:

```python
def _charge_str(current: float) -> str:
    if current > 0.05:   return "充电中"
    if current < -0.05:  return "放电中"
    return "待机"
```

The `±0.05 A` deadband filters near-zero noise so the label does not flicker at the charge/discharge boundary.

---

## 4. Tweak & Observe

| Change | What happens |
|---|---|
| Plug in the charger and watch `master_battery_current` | should become positive (charging) |
| Press the e-stop and watch `is_estop.data` | should become `True`; twist to release → `False` |
| Add a low-SOC warning: `if power < 20: get_logger().warn(...)` | warn when battery is low — useful for long experiments |
| Subscribe to `/power/board/status` for module temperatures | see Going Further |

---

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Keeps printing “no data yet” | body_control not up: `ros2 topic list \| grep power` to confirm topics exist |
| `master_battery_power` stuck at 0 | small battery present but main missing, or BMS link fault; check `battery_installed` |
| `is_estop.data` stuck at `True` | e-stop not released: twist clockwise until it pops out |
| `import bodyctrl_msgs` fails | workspace not sourced: `source /home/ubuntu/ros2ws/install/setup.bash` |

---

## 6. Going Further

- **Low-SOC protection**: in a callback or timer, if `master_battery_power < threshold`, stop motion demos — a basic safety measure for long runs.
- **Board detail status (`/power/board/status`)**: MOS temperatures, currents, and voltage min/max for arm / waist / leg modules — useful for hardware debugging; type is `bodyctrl_msgs/PowerStatus` (many fields — pick what you need).
- **E-stop interlock**: check `is_estop` in control loops and stop publishing motion commands when pressed — standard production safety logic.
