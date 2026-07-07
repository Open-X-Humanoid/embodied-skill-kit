# Atom 4 · Arm · Single-Joint Position Control

**English** | [简体中文](atom04_arm_guide_zh-CN.md)

**In one line**: publish a `CmdSetMotorPosition` to `/arm/cmd_pos` to move a single arm joint; subscribe to `/arm/status` to read the current angle back. This is the lowest-level entry point for understanding "how the arm is controlled."

| Companion code | Demo video |
|---|---|
| `atom/demos/atom04_arm_ros2.py` (native ROS2), `atom/demos/atom04_arm_ros2_robust.py` (production) | in `atom/assets/videos/`, same name as the code — watch 30s first |

## 1. At a Glance (read this first)

### 1.1 Run it

Prerequisite: body_control is up — see *Prerequisite · Environment Setup* (`atom/docs/environment_setup.md`).

```bash
ssh ubuntu@<robot-IP>
source /home/ubuntu/ros2ws/install/setup.bash   # run in every new terminal
python3 atom/demos/atom04_arm_ros2.py
```

Expected: reads the current angle of the left arm's joint 2 (ID=12) → +0.1 → −0.1 → back to start, all small and slow.

⚠ The arm has high torque and a large range: clear people/objects around it, keep the e-stop in hand; on the first run move only 1–2 joints, amplitude < 0.3 rad, speed 0.2 rad/s.

### 1.2 Interface

| Item | Value |
|---|---|
| Command topic | `/arm/cmd_pos` → `bodyctrl_msgs/CmdSetMotorPosition` |
| Status topic | `/arm/status` → `bodyctrl_msgs/MotorStatusMsg` |
| Command body | `cmds: SetMotorPosition[]`, one entry per joint (this arm demo sends one at a time) |
| Command fields | `name` (motor ID) · `pos` (target angle / rad) · `spd` (speed / rad·s⁻¹) · `cur` (max current / A) |
| Status fields | `status: MotorStatus[]`, each `name` (motor ID) · `pos` (current angle / rad) |
| Motor IDs | left arm `11–17`, right arm `21–27` (1 = shoulder pitch … 7 = wrist roll) |
| Unit | radians (`0.1 rad ≈ 5.7°`); `pos` is an **absolute target, not an increment** |

### 1.3 Joint Limits

⚠ URDF is authoritative; exceeding a limit hits the mechanical stop or triggers over-current protection. The table below is the left arm (`11–17`); on the right arm (`21–27`), J2 shoulder-roll / J4 elbow-pitch / J7 wrist-roll are **mirrored L/R**, the rest are the same.

| Joint | ID | Suggested soft limit | URDF hard limit |
|---|---|---|---|
| J1 shoulder pitch | 11 | ±2.96 rad (±170°) | ±2.967 rad |
| J2 shoulder roll | 12 | (−0.26, 2.61) rad (−15°~150°) | (−0.262, 2.618) rad |
| J3 upper-arm yaw | 13 | ±2.96 rad (±170°) | ±2.967 rad |
| J4 elbow pitch | 14 | (−2.61, 0.26) rad (−150°~15°) | (−2.618, 0.262) rad |
| J5 forearm yaw | 15 | ±2.96 rad (±170°) | ±2.967 rad |
| J6 wrist pitch | 16 | (−0.78, 1.04) rad (−45°~60°) | (−0.785, 1.047) rad |
| J7 wrist roll | 17 | (−1.65, 1.30) rad (−95°~75°) | (−1.658, 1.309) rad |

This plain version does **no limit checking**; the robust version has soft-limit + single-step (≤0.5 rad) dual checks that reject out-of-range commands. Keep targets within the soft limits yourself.

## 2. The Three Core Operations

### 2.1 Make it move — send one single-joint position command

The arm moves one joint at a time: put just one `SetMotorPosition` in `cmds`.

```python
msg = CmdSetMotorPosition()
msg.header = Header(stamp=self.get_clock().now().to_msg())
msg.cmds = [SetMotorPosition(name=motor_id, pos=target_pos, spd=SPEED, cur=MAX_CUR)]
self.pub.publish(msg)
```

- `pos`: absolute target angle (rad), not an increment.
- `spd`: speed cap — the demo uses `0.2` (slower is safer for the arm).
- `cur`: max current (A), a protection limit.

> Compared with the head (atom01): the head sends 3 joints at once, the arm sends 1 — the `cmds` list length differs, the pattern is the same.

### 2.2 Read angles — subscribe to the status topic

Same three steps as the head:

```python
# 1) subscribe (in __init__)
self.status_sub_ = self.create_subscription(
    MotorStatusMsg, ARM_STATUS_TOPIC, self._on_status, 1)

# 2) the callback stores the latest values
def _on_status(self, msg):
    for s in msg.status:
        self.cur_pos[s.name] = s.pos   # motor ID -> current angle (rad)

# 3) before using it, spin to receive — wait_for_status(12) spins until joint 12's angle arrives
```

⚠ The arm is position-controlled, so **read the current angle first, then step in small increments** — sending a target without knowing the current position can fling the arm across a large displacement. You can also read it from the command line: `ros2 topic echo /arm/status`.

### 2.3 Respect the limits — keep targets in range

Limit table in 1.3. This plain version does not intercept out-of-range targets; the consequence is a mechanical stop or over-current. For automatic checking (soft limit + step amplitude), see §6.

## 3. Code Walkthrough (core)

`atom04_arm_ros2.py` is **6 modules**. It's the same "position control" skeleton as the head (atom01); the only difference is the arm controls **one joint at a time**.

### 3.1 Module map

| # | Module | Code anchor | Role | Change per part? |
|---|---|---|---|---|
| 1 | Config constants | `ARM_CMD_TOPIC` / `ARM_STATUS_TOPIC` / `DEMO_JOINT_ID` / `SPEED` / `MAX_CUR` | topic names, demo joint, speed & current caps | ✅ topic names |
| 2 | Node & I/O | `ArmDemo.__init__` | build publisher + subscribe to status | ✅ topic names |
| 3 | Status callback | `_on_status` | write status frames into `cur_pos` | ✅ motor IDs |
| 4 | Wait for current angle | `wait_for_status` | spin until the joint's angle arrives, as the motion start | ⭕ generic |
| 5 | Send one joint | `move_joint` | build msg (one `SetMotorPosition`) → `publish` → `sleep` | ✅ motor IDs |
| 6 | Main flow | `main` | read start → ±0.1 round trip → back to start | ⭕ generic |

### 3.2 Module by module

- **Module 1 — config constants**: topic names, demo joint ID, speed, current. Changing part = change the topic names first.
- **Module 2 — `__init__`**: `create_publisher` for the command port, `create_subscription` for status, `cur_pos={}` to hold current angles. The subscription is stored as `self.status_sub_` (trailing underscore, a style convention).
- **Module 3 — `_on_status`**: flushes each status frame's `name→pos` into `cur_pos`; stores only.
- **Module 4 — `wait_for_status`**: spins until the target joint's angle arrives and returns it as the **motion start** — the key to "read before you move." ⚠ The plain version assumes `0.0` on timeout (jump risk); the robust version returns `None` and refuses to move.
- **Module 5 — `move_joint`**: the life of one motion command — new message → fill **one** motor's `name/pos/spd/cur` → `publish` → `time.sleep` to settle.
- **Module 6 — `main`**: `wait_for_status` reads the start → `move_joint` does a ±0.1 round trip → back to start → shut down.

### 3.3 Reuse it: change the joint / change the part

```python
DEMO_JOINT_ID = 22               # change joint: move the right arm's joint 2 (left 11–17 / right 21–27)
CMD_TOPIC     = "/xxx/cmd_pos"    # change part: change the command/status topic names
# shape the send method to the part: the arm sends one joint at a time (move_joint); the head sends 3 (move_to, see atom01)
# keep targets within that part's URDF limits yourself (this plain version does not check)
```

## 4. Tweak & Observe

| Change | Effect |
|---|---|
| `DEMO_JOINT_ID = 22` | move the right arm's joint 2 |
| increment `0.1` → `0.05` | moves less |
| lower `SPEED` | moves slower |

Predict first, then run, and check against your prediction.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Joint doesn't move, no error | no subscriber on `/arm/cmd_pos` (body_control not up): `ros2 topic list \| grep arm` to confirm the topic exists |
| No `/arm/status` received | the plain version warns and assumes current angle = 0.0 (**large-displacement risk**) — **don't continue**; confirm `ros2 topic hz /arm/status` has data |
| `import bodyctrl_msgs` fails | not sourced: `source /home/ubuntu/ros2ws/install/setup.bash` (every new terminal) |
| Joint errors / not enabled | the joint may be in a fault state: check body_control logs for errors and that the e-stop is released |

## 6. Going Further

- **Production hardening**: wait-for-subscriber-ready, return `None` and refuse on status timeout, soft-limit + step-amplitude dual checks, `spin_once` (non-negative timeout) instead of `time.sleep` — see `atom/demos/atom04_arm_ros2_robust.py`.
- **Higher-level arm control**: this atom is the lowest-level raw joint position control. The arm also has a higher-level **xArm wrapper** (with IK, obstacle avoidance, force control), coming later.
