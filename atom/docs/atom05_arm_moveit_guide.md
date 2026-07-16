# Atom 5 · Arm · XARM MoveIt Joint Motion

**English** | [简体中文](atom05_arm_moveit_guide_zh-CN.md)

**In one line**: let **MoveIt** plan and smoothly move a 7-DOF arm to a target joint configuration — instead of publishing raw angles yourself (that's atom04), you "tell it where to go, and it computes a collision-free trajectory and executes it." This atom uses the **standard MoveIt2 `/move_action`** (no project wrapper); execution goes through XARM's `moveit_*_arm_controller`.

| Companion | Path |
|---|---|
| Code (simple) | `atom/demos/atom05_arm_moveit.py` |
| Code (robust) | `atom/demos/atom05_arm_moveit_robust.py` (auto-avoids controllers claiming this arm + stricter checks) |
| Demo video | same name as the code, under `atom/assets/videos/` |

> The simple version teaches the principle (assumes XARM just started, no controller conflict); use the robust version when you hit a controller conflict / CONTROL_FAILED(-4) on the real robot.

> Relationship to atom04 is in §4: atom04 = "you publish angles by hand", atom05 = "MoveIt plans, XARM publishes" — same bottom layer.

## 1. Overview

### 1.1 Run it (heavier prerequisites than atom04 — follow the order)

- **Board**: **x86, user ubuntu** (same board as body_control; XARM lives at `/home/ubuntu/XARM`).
- ★**Source XARM, not ros2ws**: `source /home/ubuntu/XARM/install/setup.bash` (`moveit_msgs`, `tianyi2_bringup` are here).
- **Real-robot SOP has 5 steps** (skip any → arm won't move): start body_control → start XARM body → start MoveIt component → **enable arm** → **switch to MoveIt controller** → (run the demo). The demo does the last two automatically.

**One-click prerequisites** (recommended, see `scripts/start_xarm.sh`):

```bash
# Verify in sim first (zero risk; no real robot / no body_control; use RViz or /joint_states)
bash scripts/start_xarm.sh sim

# Real robot (start body_control on the x86 first)
bash scripts/start_body_control.sh          # another terminal, see Prerequisite · Environment Setup
bash scripts/start_xarm.sh real

# In the terminal that runs the demo, set the env then run
source scripts/start_xarm.sh                # = source /home/ubuntu/XARM/install/setup.bash
python3 atom/demos/atom05_arm_moveit.py
```

> **Strongly prefer sim first**: the arm is powerful and wide-ranging; verify planning/execution in simulation before switching to `real`. For the base ROS environment see *Prerequisite · Environment Setup* (`atom/docs/environment_setup.md`).

### 1.2 Interfaces (one action + two services)

| Interface | Type | Purpose |
|---|---|---|
| `/move_action` | `moveit_msgs/action/MoveGroup` (**Action**) | send target joint angles; MoveIt plans and executes |
| `/EAIHardware/set_arm_enable` | `std_srvs/SetBool` (Service) | enable the arm in real mode (no such service in sim) |
| `/controller_manager/switch_controller` | ros2_control (Service) | activate `moveit_left_arm_controller` |
| `/joint_states` | `sensor_msgs/JointState` (Topic) | read current joint angles as the motion start |

> One key concept: an **Action = a long task** (send a goal → get progress feedback → get a result, cancelable). Motion takes seconds and can fail, so MoveIt uses an action, not a one-shot service.

### 1.3 Joint limits (left arm, 7 joints; URDF is authoritative)

MoveIt **rejects out-of-limit goals** (returns invalid-goal / planning failure). Check before changing targets:

| Joint | Name | Hard limit (rad) | ≈ deg |
|---|---|---|---|
| J1 shoulder pitch | `shoulder_pitch_l_joint` | −2.967 ~ +2.967 | ≈±170° |
| J2 shoulder roll | `shoulder_roll_l_joint` | −0.262 ~ +2.618 | ≈−15°~+150° |
| J3 shoulder yaw | `shoulder_yaw_l_joint` | −2.967 ~ +2.967 | ≈±170° |
| J4 elbow pitch | `elbow_pitch_l_joint` | −2.618 ~ +0.262 | ≈−150°~+15° |
| J5 elbow yaw | `elbow_yaw_l_joint` | −2.967 ~ +2.967 | ≈±170° |
| J6 wrist pitch | `wrist_pitch_l_joint` | −0.785 ~ +1.047 | ≈−45°~+60° |
| J7 wrist roll | `wrist_roll_l_joint` | −1.658 ~ +1.309 | ≈−95°~+75° |

> For the right arm swap `_l_joint` → `_r_joint`; some joints (e.g. shoulder roll) mirror in sign — defer to the URDF.

## 2. Three must-dos in real mode (skip any → no motion; the demo does them)

### 2.1 ★Enable the arm (the #1 real-mode pitfall)

**XARM's "enable" is not about powering motors — it's about whether XARM sends commands to body_control.** Without enable, MoveIt planning/execution both report success, the controller is OK, but XARM never publishes `/arm/cmd_pos` → the physical arm doesn't move. The demo's `enable_arm()` calls `/EAIHardware/set_arm_enable`; manual equivalent:

```bash
ros2 service call /EAIHardware/set_arm_enable std_srvs/srv/SetBool "{data: true}"
```

> Enabling **checks for conflicts**: if another app publishes `/arm/cmd_pos` (e.g. a leftover atom04), enable fails or gets force-disabled — clear first with `bash scripts/stop_all.sh`.

### 2.2 Switch to the MoveIt controller

XARM controllers are mutually-exclusive and pluggable; MoveIt must have `moveit_left_arm_controller` active before executing, else execution fails with `error_code=-4 CONTROL_FAILED`. The demo's `activate_moveit_controller()` switches automatically.

### 2.3 Send the planning goal

`move_to_joints()` builds one `JointConstraint` per joint into a `MoveGroup` goal sent to `/move_action`; `error_code == 1` means success.

## 3. Code walkthrough (core)

| Module | Code anchor | Role | Change for right/dual arm? |
|---|---|---|---|
| enable | `enable_arm()` → `/EAIHardware/set_arm_enable` | open the "send-commands gate" in real mode | unchanged (`set_all_enable` enables all at once) |
| switch controller | `activate_moveit_controller()` → `switch_controller` | activate the moveit controller | change `MOVEIT_CONTROLLER` |
| read start | `read_current()` on `/joint_states` | current angles as motion start | change `JOINT_NAMES` |
| plan+execute | `move_to_joints()` → `MoveGroup` action | build JointConstraints, send /move_action, check error_code | change `GROUP` + `JOINT_NAMES` |

**Generalize**: right arm = `GROUP="right_arm"` + joint names `_l_`→`_r_` + controller `moveit_right_arm_controller`; dual arm = a dual-arm planning group (name per SRDF) with 14 joint constraints. The MoveGroup pattern is identical.

## 4. Difference from atom04

| | atom04 (raw) | atom05 (MoveIt) |
|---|---|---|
| Who computes the path | you (publish target angles; no planning, no avoidance) | MoveIt (collision-free smooth trajectory) |
| Path taken | publish `/arm/cmd_pos` directly | `/move_action` → XARM controller → still ends at `/arm/cmd_pos` or `/arm/cmd_ctrl` |
| Enable needed? | no (you publish the topic, no XARM) | **yes** (goes through XARM, which has the "enable gate") |
| Good for | learning the principle, single-joint jog | production: multi-joint coordination, avoidance, smoothness |

> Same bottom layer: both end at `/arm/cmd_*` → body_control → motors. The only difference is "publish by hand" vs "MoveIt plans, XARM publishes". Because they contend for the same `/arm/cmd_pos`, **atom04 and XARM enable are mutually exclusive** (enable fails while atom04 is running).

## 5. Troubleshooting

| Symptom | error_code / cause | Fix |
|---|---|---|
| Plan/execute succeed but arm static | not enabled (real #1 pitfall); or in sim (physical arm never moves) | `set_arm_enable true`; in sim watch RViz/joint_states |
| `error_code=-4` CONTROL_FAILED | moveit controller not active | switch controller (demo does it; manual `switch_controllers --activate`) |
| `error_code=-15` INVALID_GROUP_NAME | wrong planning group | check SRDF: `ros2 param get /move_group robot_description_semantic` |
| goal rejected / planning failed | target out of limits or in collision | check §1.3 limits; move the arm near the target first |
| `/move_action` won't connect | MoveIt component not up | `ros2 action list \| grep move_action`; start `tianyi2_moveit.launch.py` |
| enable fails / auto-disables | another app publishes `/arm/cmd_pos` (e.g. leftover atom04) | `bash scripts/stop_all.sh`, then retry |
| real-mode "watchdog timeout" | body_control not up / no `/arm/status` | start body_control first, then XARM |

Two self-checks: `ros2 service call /EAIHardware/debug ...` (see `arm_enable=1`), `ros2 control list_controllers | grep -i moveit` (controller active).

## 6. Going further

- **arm_mode (hardware mode)**: `0` compliant/force-position (this demo's default) / `3` stiff position loop (tighter tracking) / `1` stiff velocity loop (needs realtime kernel) / `2` gravity compensation. Switch: `ros2 service call /EAIHardware/set_arm_mode eai_manipulator_msgs/srv/Mode "{mode: 3}"`. Mode also decides whether XARM publishes `/arm/cmd_pos` or `/arm/cmd_ctrl`.
- **Cartesian (end-effector) control**: this atom is joint-space MoveIt; to command an end-effector pose (instead of joint angles), see **atom06 (MoveIt Cartesian control)**.
- **Library vs raw action**: this demo sends `/move_action` directly (lightest, most transparent); you could also use MoveIt's Python library `moveit_py` (`.plan()/.execute()`), but that requires loading the MoveIt config params.
