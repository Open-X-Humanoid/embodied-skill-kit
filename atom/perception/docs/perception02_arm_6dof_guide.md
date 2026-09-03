# Perception 2 · 6-DoF F/T Sensor · Wrist Force / Torque Readout

**English** | [简体中文](perception02_arm_6dof_guide_zh-CN.md)

**In one line**: subscribe to `/arm_6dof_left` and `/arm_6dof_right`, and read wrist triaxial force (Fx/Fy/Fz, N) and torque (Tx/Ty/Tz, Nm) each frame. This atom publishes no control commands — it is a pure perception entry point.

| Companion code |
|---|
| `atom/perception/perception02_arm_6dof_ros2.py` (native ROS2) |

---

## 1. At a Glance (read this first)

### 1.1 Run it

Prerequisite: body_control is up — see *Prerequisite · Environment Setup* (`docs/environment_setup.md`).

```bash
ssh ubuntu@<robot-IP>
source /home/ubuntu/ros2ws/install/setup.bash   # run in every new terminal
python3 atom/perception/perception02_arm_6dof_ros2.py
```

Expected: the terminal prints left/right wrist force and torque at 2 Hz, for example:

```
[INFO] [atom_ft_sensor_demo]: [左腕] F=(  +0.012,  -0.034,  +9.812) N  T=(  +0.001,  +0.002,  -0.003) Nm
[INFO] [atom_ft_sensor_demo]: [右腕] F=(  +0.008,  -0.021,  +9.793) N  T=(  +0.000,  +0.001,  -0.001) Nm
```

At rest, Fz ≈ the gravity component (arm self-weight); other components stay near 0. Lightly push a wrist and you should see clear changes on the corresponding axes.

Press `Ctrl-C` to exit.

### 1.2 Interface

| Item | Value |
|---|---|
| Left wrist topic | `/arm_6dof_left` → `geometry_msgs/WrenchStamped` |
| Right wrist topic | `/arm_6dof_right` → `geometry_msgs/WrenchStamped` |
| Message fields | `wrench.force.x/y/z` (N) · `wrench.torque.x/y/z` (Nm) |
| Frame | sensor body frame (see URDF `frame_id`) |
| Publish rate | 1000 Hz |
| Command topic | **none** (F/T sensor is read-only) |

### 1.3 Physical meaning

| Field | Meaning | Unit |
|---|---|---|
| `force.x` | force along sensor X | N |
| `force.y` | force along sensor Y | N |
| `force.z` | force along sensor Z (includes gravity at rest) | N |
| `torque.x` | torque about sensor X | Nm |
| `torque.y` | torque about sensor Y | Nm |
| `torque.z` | torque about sensor Z | Nm |

⚠ Axis directions follow the sensor frame in the URDF; signs depend on the reference frame — trust measured values until you have calibrated.

---

## 2. The Two Core Operations

### 2.1 Read data — subscribe to the topics

Subscribing to `geometry_msgs/WrenchStamped` follows the same pattern as other atoms' status topics:

```python
from geometry_msgs.msg import WrenchStamped

# 1) subscribe (__init__)
self.sub_left_ = self.create_subscription(
    WrenchStamped, "/arm_6dof_left", self._cb_left, 10)

# 2) callback stores the latest message
def _cb_left(self, msg: WrenchStamped) -> None:
    self._latest["left"] = msg   # store the whole msg

# 3) read fields when needed
f = msg.wrench.force    # .x  .y  .z  (N)
t = msg.wrench.torque   # .x  .y  .z  (Nm)
```

Unlike joint-status topics, here we store the full `WrenchStamped` (not a single float): force/torque are six independent components, so keeping the whole msg is more convenient.

### 2.2 Downsample printing — control rate with a timer

The raw topic is 1000 Hz; printing inside the callback floods the terminal. Use `create_timer` to read the latest value at a fixed rate:

```python
# 1) create a 2 Hz timer (__init__)
self.timer_ = self.create_timer(1.0 / PRINT_HZ, self._on_print_timer)

# 2) read the latest value in the timer callback
def _on_print_timer(self) -> None:
    msg = self._latest["left"]
    if msg is None:
        return
    f, t = msg.wrench.force, msg.wrench.torque
    self.get_logger().info(
        f"[左腕] F=({f.x:+7.3f}, {f.y:+7.3f}, {f.z:+7.3f}) N  "
        f"T=({t.x:+7.3f}, {t.y:+7.3f}, {t.z:+7.3f}) Nm")
```

Raise `PRINT_HZ` for denser samples. For raw 1000 Hz processing (algorithms), handle data directly in `_cb_left` — no timer needed.

---

## 3. Code Walkthrough (core)

`perception02_arm_6dof_ros2.py` is **four modules**. It is one of the shortest atoms — F/T is read-only, with no control flow.

### 3.1 Module map

| # | Module | Code anchor | Role | Change when porting? |
|---|---|---|---|---|
| 1 | Config constants | `LEFT_TOPIC` / `RIGHT_TOPIC` / `PRINT_HZ` | topic names, print rate | ✅ topic names |
| 2 | Node & subscriptions | `FTSensorDemo.__init__` | two subscribers + print timer | ✅ topic names |
| 3 | Data callbacks | `_cb_left` / `_cb_right` | store latest `WrenchStamped` in `_latest` | ⭕ reusable as-is |
| 4 | Timed print | `_on_print_timer` / `_log_wrench` | read `_latest` and log | ⭕ reusable as-is |

### 3.2 Why `rclpy.spin` instead of a `spin_once` loop?

Other atoms use `spin_once` because they interleave waits between commands. This atom has no control flow; the timer and subscription callbacks are driven entirely by the spin event loop, so `rclpy.spin(node)` is simpler and runs until Ctrl-C.

---

## 4. Tweak & Observe

| Change | What happens |
|---|---|
| `PRINT_HZ = 10.0` | denser prints; see force changes at ~100 ms scale |
| Subscribe to one side only (comment out the other sub) | print one wrist only — useful when focusing on one arm |
| Push the wrist in different directions by hand | see which `force.x/y/z` component responds; check axis directions |
| Hold the arm still and read Fz | estimate the effective gravity component at the tip (for gravity-compensation calibration) |

Predict which axis should respond, then push and check whether it matches.

---

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Both sides keep printing “no data yet” | body_control not up, or F/T driver not loaded: `ros2 topic list \| grep 6dof` to confirm topics exist |
| Topics exist but never publish | `ros2 topic hz /arm_6dof_left` — if rate is 0 / no output, check body_control logs for sensor errors |
| `import geometry_msgs` fails | workspace not sourced: `source /home/ubuntu/ros2ws/install/setup.bash` (every new terminal) |
| Large drift while still | sensor needs zero-offset calibration |

---

## 6. Going Further

- **Zero-offset compensation**: average several frames at rest as an offset, subtract it each frame to get net external force — the first step toward force control.
- **Contact detection**: when `|force|` exceeds a threshold, stop or switch to compliant motion; can combine with force–position hybrid control.
- **Gravity compensation**: project tip self-weight into the sensor frame from the arm pose, subtract it from the reading to isolate external force.
