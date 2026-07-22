# Atom 1 · Head · Position Control & Reading Angles

**English** | [简体中文](atom01_head_guide_zh-CN.md)

**In one line**: publish a `CmdSetMotorPosition` to `/head/cmd_pos` to drive the head's 3 motors to target angles; subscribe to `/head/status` to read the current angles back. **Learn this one atom and position control for the arm / waist / leg follows the same pattern.**

| Companion code | Demo video |
|---|---|
| `atom/demos/atom01_head_ros2.py` (native ROS2) | in `atom/assets/videos/`, same name as the code |

## 1. At a Glance (read this first)

### 1.1 Run it

Prerequisite: body_control is up — see *Prerequisite · Environment Setup* (`atom/docs/environment_setup.md`).

```bash
ssh ubuntu@<robot-IP>
source /home/ubuntu/ros2ws/install/setup.bash   # run in every new terminal
python3 atom/demos/atom01_head_ros2.py
```

Expected: prints the current angles once → recenter → nod down → tilt up → turn left → turn right → recenter, ~1.5s per step.

### 1.2 Interface

| Item | Value |
|---|---|
| Command topic | `/head/cmd_pos` → `bodyctrl_msgs/CmdSetMotorPosition` |
| Status topic | `/head/status` → `bodyctrl_msgs/MotorStatusMsg` |
| Command body | `cmds: SetMotorPosition[]`, one entry per motor |
| Command fields | `name` (motor ID) · `pos` (target angle / rad) · `spd` (speed / rad·s⁻¹) · `cur` (max current / A) |
| Status fields | `status: MotorStatus[]`, each `name` (motor ID) · `pos` (current angle / rad) |
| Motor IDs | `1` = roll (tilt) · `2` = pitch (nod) · `3` = yaw (turn) |
| Unit | radians (`0.25 rad ≈ 14°`); `pos` is an **absolute target, not an increment** |

### 1.3 Joint Limits

⚠ URDF is authoritative. Exceeding a limit hits the mechanical stop or triggers over-current protection.

| Joint | ID | Suggested soft limit | URDF hard limit |
|---|---|---|---|
| roll (tilt) | 1 | ±0.30 rad (±17°) | ±0.4538 rad (±26°) |
| pitch (nod) | 2 | ±0.40 rad (±23°) | ±0.4363 rad (±25°) |
| yaw (turn) | 3 | ±0.60 rad (±34°) | ±1.5708 rad (±90°) |

This demo does **no limit checking** — it sends whatever you give it, so keep targets within the soft limits yourself.

## 2. The Three Core Operations

### 2.1 Make it move — send one position command

One motion = assemble a `CmdSetMotorPosition` and `publish` it. All three motors get a target in the same message:

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

- `pos`: absolute target angle (rad), not an increment.
- `spd`: speed cap — smaller is steadier; the demo uses `0.5`.
- `cur`: max current (A), a protection limit.

### 2.2 Read angles — subscribe to the status topic

`/head/status` continuously pushes each motor's current angle. Reading it is three steps:

```python
# 1) subscribe (in __init__)
self.status_sub_ = self.create_subscription(
    MotorStatusMsg, HEAD_STATUS_TOPIC, self._on_status, 10)

# 2) the callback stores the latest values
def _on_status(self, msg):
    for s in msg.status:              # one entry per motor
        self.cur_pos[s.name] = s.pos  # motor ID -> current angle (rad)

# 3) when you need it: status is pushed passively, so spin to receive first
for _ in range(30):
    rclpy.spin_once(node, timeout_sec=0.1)
    if node.cur_pos:
        break
print(node.cur_pos)
```

Key point: **you must run `rclpy.spin_once/spin` for the callback to fire and `cur_pos` to fill.** Production code uses this to "read the current angle first, then step in small increments," avoiding a violent jump from sending a large angle blindly at startup.

You can also read it without code: `ros2 topic echo /head/status`.

### 2.3 Respect the limits — keep targets in range

Limit table in 1.3. This demo does not intercept out-of-range targets; the consequence is a mechanical stop or over-current. For automatic limit checking that rejects out-of-range commands, see §6.

## 3. Code Walkthrough (core)

`atom01_head_ros2.py` is **5 modules**. Any "motor position control" body part (arm / waist / leg) is the same 5 modules — **swap the config, keep the logic**.

### 3.1 Module map

| # | Module | Code anchor | Role | Change per part? |
|---|---|---|---|---|
| 1 | Config constants | `HEAD_CMD_TOPIC` / `HEAD_STATUS_TOPIC` / `MAX_SPEED` / `MAX_CUR` | topic names, speed & current caps | ✅ topic names |
| 2 | Node & I/O | `HeadDemo.__init__` | build publisher + subscribe to status | ✅ topic names |
| 3 | Status callback | `_on_status` | write status frames into `cur_pos` | ✅ motor IDs |
| 4 | Send one command | `move_to` | build msg → `publish` → `sleep` to settle | ✅ motor IDs |
| 5 | Main flow | `main` | read current angle once → `move_to` in sequence → shut down | ⭕ generic |

### 3.2 Module by module

- **Module 1 — config constants**: topic names, speed, current pulled out as constants. Changing part = change the topic names here first.
- **Module 2 — `__init__`**: `create_publisher` for the command port, `create_subscription` for status, `cur_pos={}` to hold current angles. The subscription is stored as `self.status_sub_` (trailing underscore, a style convention).
- **Module 3 — `_on_status`**: its only job is to flush each status frame's `name→pos` into `cur_pos`. The callback only stores; logic lives outside.
- **Module 4 — `move_to`**: the full life of one motion command — ① new message → ② timestamp → ③ fill `name/pos/spd/cur` per motor → ④ `publish` → ⑤ `time.sleep` as a rough settle wait.
- **Module 5 — `main`**: spin to receive and print the current angle once (demonstrating the read), then run the demo motions via `move_to`, then `destroy_node` + `shutdown`.

### 3.3 Reuse it: write a new part controller

Swap the config per the module map and reuse the same pattern (**shape the send method to the part's joint count**):

```python
CMD_TOPIC    = "/arm/cmd_pos"       # 1) change the command topic
STATUS_TOPIC = "/arm/status"        # 2) change the status topic
# 3) change the motor IDs to that part's (e.g. arm: left 11-17 / right 21-27)
# 4) shape the send method to the part: the head sends 3 joints at once (move_to); an arm has
#    more joints and often sends one at a time (move_joint) — same core: fill SetMotorPosition
#    (name/pos/spd/cur) -> publish
# 5) keep targets within that part's URDF limits yourself (this plain version does not check)
```

## 4. Tweak & Observe

| Change | Effect |
|---|---|
| the `0.25` in `move_to(0, 0.25, 0)` up/down | nod more / less (stay under pitch soft limit 0.40) |
| lower `MAX_SPEED` | slower turns |
| add `move_to(0.1, 0, 0)` | head tilts a little (roll) |

Predict first, then run, and check against your prediction.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Head doesn't move, no error | no subscriber on `/head/cmd_pos` (body_control not up): `ros2 topic list \| grep head` to confirm the topic exists |
| `cur_pos` prints empty | no status received: confirm body_control is up and `/head/status` has data (`ros2 topic hz /head/status`) |
| `import bodyctrl_msgs` fails | not sourced: `source /home/ubuntu/ros2ws/install/setup.bash` (every new terminal) |


## 6. Going Further

- **Production hardening**: wait-for-status-ready, limit checking that rejects out-of-range commands, `spin_once` (non-negative timeout) instead of `time.sleep` — see `atom/demos/atom01_head_ros2_robust.py`.
- **Other motion modes**: besides position mode (`cmd_pos`), the head supports other control modes (e.g. velocity, current), and the available modes differ per part. This atom only demonstrates position mode; for the rest see the official *TianYi 2.0 ROS2 SDK Secondary-Development Guide*.
