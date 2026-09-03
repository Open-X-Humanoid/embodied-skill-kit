# Motion 2 · Waist · Twist (yaw)

**English** | [简体中文](motion02_waist_guide_zh-CN.md)

**In one line**: publish a `CmdSetMotorPosition` to `/waist/cmd_pos` to twist the waist (yaw, rotating the upper body about the vertical axis); subscribe to `/waist/status` to read the current angle. The waist has two DOF (yaw/pitch); **this atom only moves yaw and holds pitch** (pitch is coupled with the legs — moving it alone risks tipping).

| Companion code | Demo video |
|---|---|
| `atom/motion/motion02_waist_ros2.py` (native ROS2), `atom/motion/motion02_waist_ros2_robust.py` (production) | in `atom/motion/assets/videos/`, same name as the code |

## 1. At a Glance (read this first)

### 1.1 Run it

Prerequisite: body_control is up — see *Prerequisite · Environment Setup* (`docs/environment_setup.md`).

```bash
ssh ubuntu@<robot-IP>
source /home/ubuntu/ros2ws/install/setup.bash   # run in every new terminal
python3 atom/motion/motion02_waist_ros2.py
```

Expected: prints the current yaw/pitch → twist to one side → to the other side → back to start, with pitch held constant throughout.

⚠ The waist bears load and has high torque; keep the e-stop in hand. Only yaw is demonstrated (a horizontal twist that doesn't change height/balance).

### 1.2 Interface

| Item | Value |
|---|---|
| Command topic | `/waist/cmd_pos` → `bodyctrl_msgs/CmdSetMotorPosition` |
| Status topic | `/waist/status` → `bodyctrl_msgs/MotorStatusMsg` (has `pos` current angle, `error` code) |
| Command fields | `name` (motor ID) · `pos` (target angle / rad) · `spd` (speed) · `cur` (max current / A; load needs it high) |
| Motor IDs | `31` = yaw (twist, left/right) · `32` = pitch (forward lean, coupled with leg lift) |
| Unit | radians (`0.2 rad ≈ 11°`); `pos` is an **absolute target, not an increment** |

### 1.3 Joint Limits & Safety

⚠ URDF is authoritative; exceeding a limit hits the mechanical stop or trips over-current protection.

| Joint | ID | Suggested soft limit | URDF hard limit |
|---|---|---|---|
| yaw (twist) | 31 | ±0.5 rad (±29°) | (−2.967, 3.142) rad |
| pitch (lean) | 32 | ±0.3 rad (±17°) | (−0.785, 2.094) rad |

⚠ **pitch(32) is coupled with leg lift**: moving it alone shifts the whole-robot center of mass and **risks tipping** — this demo only **holds** pitch at its currently measured value, never drives it. To move pitch, use the leg motion10 (coordinated lift).

⚠⚠ **Insufficient voltage raises error code `12832` (under-voltage)**: the waist is a load-bearing joint that needs adequate power/torque. **When power is insufficient the motor reports error code `12832`**, showing up as "command sent but it won't move / no force / error." **The robust version automatically reads the `error` field of `/waist/status` before each move and refuses to move (and prints it) when it sees codes like `12832`** (see §6); the plain version does not check error codes and may move while under-volted.

## 2. The Three Core Operations

### 2.1 Make it move — send one waist command

One waist command gives both yaw and pitch — **yaw gets the target, pitch gets the "currently measured value" to hold it still**:

```python
msg = CmdSetMotorPosition()
msg.header = Header(stamp=self.get_clock().now().to_msg())
msg.cmds = [
    SetMotorPosition(name=31, pos=yaw,        spd=SPEED, cur=CURRENT_LIMIT),  # twist
    SetMotorPosition(name=32, pos=pitch_hold, spd=SPEED, cur=CURRENT_LIMIT),  # hold pitch
]
self.pub.publish(msg)
```

- `cur` (max current) is higher than head/arm (the demo uses `20.0`) — the load-bearing waist needs enough torque.
- **pitch must be the current measured value**: this message is the "whole waist target"; omitting pitch may be read as 0 and make pitch move (tipping risk). Always holding pitch at the current value is the safe practice.

### 2.2 Read angles — subscribe to the status topic

Same pattern as head/arm; the waist especially **must read pitch's current value first** in order to "hold" it:

```python
# 1) subscribe (in __init__)
self.status_sub_ = self.create_subscription(
    MotorStatusMsg, WAIST_STATUS_TOPIC, self._on_status, 1)

# 2) the callback stores the latest values (the robust version also records the error code, for under-voltage self-checks)
def _on_status(self, msg):
    for s in msg.status:
        self.cur_pos[s.name] = s.pos

# 3) wait_status([31, 32]) reads yaw+pitch before moving (refuses to move if it can't)
```

### 2.3 Respect limits & hold pitch

Keep yaw within the soft limits (see 1.3); pitch stays at the current measured value (never actively driven). The plain version does no checking; the robust version does **soft-limit + motor-error (incl. under-voltage `12832`) double checks**.

## 3. Code Walkthrough (core)

`motion02_waist_ros2.py` is **6 modules**. It's the same "send command + read state" skeleton as head/arm; the twist is that it **sends two joints at once: yaw moves, pitch holds**.

### 3.1 Module map

| # | Module | Code anchor | Role | Change per part? |
|---|---|---|---|---|
| 1 | Config constants | `WAIST_CMD_TOPIC` / `WAIST_STATUS_TOPIC` / `WAIST_YAW_ID` / `WAIST_PITCH_ID` / `CURRENT_LIMIT` / `SPEED` | topic names, motor IDs, current/speed caps | ✅ topic names |
| 2 | Node & I/O | `WaistDemo.__init__` | build publisher + subscribe to status | ✅ topic names |
| 3 | Status callback | `_on_status` | write status frames into `cur_pos` | ✅ motor IDs |
| 4 | Wait for status | `wait_status` | spin until yaw+pitch are read before moving | ⭕ generic |
| 5 | Send command | `command` | build msg (yaw moves + pitch holds) → `publish` → wait | ✅ motor IDs |
| 6 | Main flow | `main` | wait for status → read yaw0/pitch0 → twist back and forth → return | ⭕ generic |

### 3.2 Module by module

- **Module 1 — config constants**: topic names, `31`=yaw / `32`=pitch, current cap `CURRENT_LIMIT=20.0` (heavy load), speed, amplitude.
- **Module 2 — `__init__`**: `create_publisher` + `create_subscription` + `cur_pos={}`. The subscription is stored as `self.status_sub_` (trailing underscore convention).
- **Module 3 — `_on_status`**: flushes each status frame's `name→pos` into `cur_pos` (the robust version also records the `error` code for under-voltage self-checks).
- **Module 4 — `wait_status`**: spins until yaw+pitch are both read, then returns — **refuses to move** if it can't (the waist must never fire blind).
- **Module 5 — `command`**: builds the message, `31` gets the target yaw and `32` gets `pitch_hold` (held) → `publish` → wait to settle.
- **Module 6 — `main`**: `wait_status` → read `yaw0/pitch0` → `command` twists ±0.2 back and forth while always passing `pitch0` → return to start.

### 3.3 Reuse it

```python
YAW_AMP = 0.3                       # twist a larger angle (stay under yaw soft limit 0.5)
# Key: always pass the "current measured value" for pitch to hold it; never drive it (tipping risk)
```

> Note: the waist shares `CmdSetMotorPosition` + radians with head/arm — same pattern; what's special is that you **must read pitch's current value to hold it** — the standard practice for a load-bearing coupled joint.

## 4. Tweak & Observe

| Change | Effect |
|---|---|
| `YAW_AMP` 0.2 → 0.1 | smaller twist |
| lower `SPEED` | slower twist |
| ~~drive pitch~~ | ⚠ don't move pitch — tipping risk; for lift use the leg motion10 |

Predict first, then run, and check against your prediction.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| ★ **Motor reports `12832`** | **Under-voltage** — the load-bearing waist needs adequate power. Check the battery level / power supply; if low, charge before running. The robust version reads this code before moving, refuses, and prints it — catching it early |
| Waist doesn't move, no error | no subscriber on `/waist/cmd_pos` (body_control not up): `ros2 topic list \| grep waist` to confirm |
| No `/waist/status` | confirm body_control is up and the topic has data (`ros2 topic hz /waist/status`); the script refuses to move when it can't read it |
| `import bodyctrl_msgs` fails | not sourced: `source /home/ubuntu/ros2ws/install/setup.bash` |

## 6. Going Further

- **Production hardening (incl. voltage/fault self-check)**: `_check_motor_errors` in `atom/motion/motion02_waist_ros2_robust.py` reads the `error` field of `/waist/status` **before every move** and, on a non-zero code (e.g. **`12832` under-voltage**), **refuses to move and prints it** — catching voltage/motor faults early so it never moves while faulted. It also does soft-limit validation, wait-for-subscriber-ready, and `spin_once` with a non-negative timeout.
- **To move pitch (lean / lift)**: use the leg motion10, coordinated with the lower body — never drive the waist pitch alone (tipping risk).
