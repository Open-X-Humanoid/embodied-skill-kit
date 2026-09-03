# Motion 9 · Hand · Five-Finger Open/Close Control

**English** | [简体中文](motion09_hand_guide_zh-CN.md)

**In one line**: publish a `JointState` (the six fingers' open ratios, 0–1) to `/inspire_hand/ctrl/left_hand` to open/close the dexterous hand; subscribe to `/inspire_hand/state/left_hand` to read the current opening back.

| Companion code | Demo video |
|---|---|
| `atom/motion/motion09_hand_ros2.py` (native ROS2), `atom/motion/motion09_hand_ros2_robust.py` (production) | in `atom/motion/assets/videos/`, same name as the code  |

## 1. At a Glance (read this first)

### 1.1 Run it

Prerequisite: body_control is up (the dexterous hand comes with the body driver) — see *Prerequisite · Environment Setup* (`docs/environment_setup.md`).

```bash
ssh ubuntu@<robot-IP>
source /home/ubuntu/ros2ws/install/setup.bash   # run in every new terminal
python3 atom/motion/motion09_hand_ros2.py
```

Expected: prints the current opening → fully open → fist (0.1) → open again → bend only the index finger → open to finish.

⚠ Don't let the fingers pinch objects/people; for a fist use **0.1** rather than 0.0, to leave mechanical margin and avoid jamming.

### 1.2 Interface

| Item | Value |
|---|---|
| Command topic | `/inspire_hand/ctrl/left_hand` (right hand `right_hand`) → `sensor_msgs/JointState` |
| Status topic | `/inspire_hand/state/left_hand` → `sensor_msgs/JointState` |
| Command fields | `name[]` (finger ID strings) · `position[]` (open ratio 0–1) |
| Finger IDs | `"1"` little · `"2"` ring · `"3"` middle · `"4"` index · `"5"` thumb-bend · `"6"` thumb-rotate |
| Unit | **open ratio**: `0.0` = closed / `1.0` = fully open (⚠ not radians!) |

### 1.3 Value Range & Safety

The hand has no radian limits; its "range" is the **open ratio [0, 1]**:

| Item | Value |
|---|---|
| `position` value | each finger `0.0` (closed) ~ `1.0` (fully open) |
| Safety floor | for a fist use **0.1**, not `0.0` (avoids jamming, leaves mechanical margin) |
| Out of range | the robust version clamps to `[0,1]`; the plain version does not — don't send out-of-range values |

⚠ The hand uses an **open ratio**, completely different from the **radians** of head/arm/waist/leg — don't mix them up.

## 2. The Three Core Operations

### 2.1 Make it move — send one open/close command

The six fingers' targets go into one `JointState`:

```python
msg = JointState()
msg.header = Header(stamp=self.get_clock().now().to_msg())
msg.name = FINGER_NAMES                     # ["1","2","3","4","5","6"]
msg.position = [r1, r2, r3, r4, r5, r6]     # each 0~1
self.pub.publish(msg)
```

- `position[i]`: finger i's target opening (`0.0` closed ~ `1.0` open).
- **Single-finger control** = change only that finger, set the rest to `1.0` — e.g. bend only the index finger: `[1, 1, 1, 0.1, 1, 1]`.

### 2.2 Read state — subscribe to the status topic

Same three steps as head/arm:

```python
# 1) subscribe (in __init__)
self.state_sub_ = self.create_subscription(
    JointState, HAND_STATE_TOPIC, self._on_state, 10)

# 2) the callback stores the latest values
def _on_state(self, msg):
    self.cur_pos = list(msg.position)   # six fingers' current opening (0~1)

# 3) before using it, spin to receive (main reads once at the start)
```

You can also read it from the command line: `ros2 topic echo /inspire_hand/state/left_hand`.

### 2.3 Stay in range — keep values within [0,1]

See 1.3. The plain version does no checking (sends whatever you give); the robust version clamps each value to `[0,1]` and warns on wrong length/range.

## 3. Code Walkthrough (core)

`motion09_hand_ros2.py` is **5 modules**. It's the same "send command + read state" skeleton as head/arm; the difference is the **message type is `JointState` and the value is an open ratio**.

### 3.1 Module map

| # | Module | Code anchor | Role | Change per hand/part? |
|---|---|---|---|---|
| 1 | Config constants | `HAND_CMD_TOPIC` / `HAND_STATE_TOPIC` / `FINGER_NAMES` | topic names, finger ID list | ✅ topic names (left/right) |
| 2 | Node & I/O | `HandDemo.__init__` | build publisher + subscribe to state | ✅ topic names |
| 3 | State callback | `_on_state` | write state frames into `cur_pos` | ⭕ generic |
| 4 | Send open/close | `set_open_ratio` | build `JointState` (6 ratios) → `publish` → `sleep` | ⭕ generic |
| 5 | Main flow | `main` | read state once → a sequence of open/close → shut down | ⭕ generic |

### 3.2 Module by module

- **Module 1 — config constants**: command/status topic names, the six finger ID strings. Switching left/right hand = change the topic names.
- **Module 2 — `__init__`**: `create_publisher` for the command port, `create_subscription` for state, `cur_pos=None` to hold the current opening. The subscription is stored as `self.state_sub_` (trailing underscore convention).
- **Module 3 — `_on_state`**: stores each state frame's `position` into `cur_pos` (six fingers' current opening).
- **Module 4 — `set_open_ratio`**: the life of one command — build `JointState` → fill `name` + `position` (6 values) → `publish` → `sleep` to settle.
- **Module 5 — `main`**: spins to receive one frame and prints the current opening, then runs open / fist / single-finger via `set_open_ratio` → shut down.

### 3.3 Reuse it

```python
HAND_CMD_TOPIC = "/inspire_hand/ctrl/right_hand"   # switch to the right hand
# hand shapes: change the target finger's value in position, set the rest to 1.0
# half-bend: use a middle value like 0.5
```

> Note: the hand is a **separate interface family** (`JointState` + open ratio), unlike the `CmdSetMotorPosition` + radians of head/arm/waist/leg — "reuse" here mostly means switching left/right hand and hand shapes; crossing to motion joints needs a different message type.

## 4. Tweak & Observe

| Change | Effect |
|---|---|
| the `0.1` in `[0.1]*6` up/down | looser / tighter fist |
| `[1,1,1,1,0.1,0.3]` | make a "pinch" shape |
| topic → `right_hand` | control the right hand |
| set one finger to `0.5` | half-bend |

Predict first, then run, and check against your prediction.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Hand doesn't move, no error | no subscriber on `/inspire_hand/ctrl/...` (hand node not up): `ros2 topic list \| grep inspire` to confirm |
| No state / `cur_pos` empty | confirm `/inspire_hand/state/...` has data (`ros2 topic hz`); the robust version reports whether `position` length is 6 |
| Values sent, no response | check you're sending an **open ratio 0~1**, not radians; confirm the left/right hand topic |
| Finger jams / odd noise | the value reached `0.0`; use `0.1` to leave margin |

## 6. Going Further

- **Production hardening**: wait-for-subscriber-ready, refuse on status timeout, explicit validation (instead of `assert`, which is skipped under `-O`), clamp to `[0,1]`, warn on wrong state length, `spin_once` (non-negative timeout) — see `atom/motion/motion09_hand_ros2_robust.py`.
- **Finer control**: the Inspire hand also has a **service interface** for **torque / speed** (this atom only uses the simplest topic open/close control). Under an xRocs/xArm wrapper the topic/service names may differ.
