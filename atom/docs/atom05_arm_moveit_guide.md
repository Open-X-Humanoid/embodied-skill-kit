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
- ★**Running the demo needs only base ROS 2** (`/opt/ros/humble`, auto-sourced by `~/.bashrc`): the demo imports only standard message packages (`moveit_msgs`, `controller_manager_msgs`, `std_srvs`, `sensor_msgs` all live in base ROS), so **you don't need to source XARM**. Sourcing XARM (`/home/ubuntu/XARM/install`, with `tianyi2_bringup`) is only for **starting** the XARM body + MoveIt, which `start_xarm.sh` does; source it manually only if `import moveit_msgs` fails on your machine: `source /home/ubuntu/XARM/install/setup.bash`.
- **Real-robot SOP has 5 steps** (skip any → arm won't move): start body_control → start XARM body → start MoveIt component → **enable arm** → **switch to MoveIt controller** → (run the demo). The demo does the last two automatically.

**One-click prerequisites** (recommended, see `scripts/start_xarm.sh`):

```bash
# ⓪ Stop the teleop service (auto-starts on boot, occupies /arm/cmd_pos; enable fails if it's running.
#    Comes back automatically after reboot; harmless to run when already stopped — just run it once per boot.
#    Optional pre-check:
#    systemctl is-active teleop_robot   # active = running, stop it; inactive = already stopped;
#                                       # "could not find unit" = this machine doesn't have it (older system), skip this step
sudo systemctl stop teleop_robot

# Real robot (verified): start body_control on the x86 first, then XARM + MoveIt
bash scripts/start_body_control.sh          # another terminal, see Prerequisite · Environment Setup
bash scripts/start_xarm.sh real

# Run the demo (base ROS is already sourced; no need to source XARM)
python3 atom/demos/atom05_arm_moveit.py
```

> Why ⓪: the remote-controller dispatch service `teleop_robot` auto-starts on boot and registers as a publisher on `/arm/cmd_pos` — mutually exclusive with programmatic arm control (XARM's enable gate rejects when the topic is occupied). Affects arm atoms only; head/voice/camera/chassis are unaffected. To use the remote controller again: `sudo systemctl start teleop_robot` or just reboot.

> **Sim mode** (`bash scripts/start_xarm.sh sim`, with RViz, no real robot / no body_control): use it to preview planning/execution when no robot is available. ⚠ Not tested in sim in this project — commands should work in theory, adapt to your machine; `real` is what's verified. For the base ROS environment see *Prerequisite · Environment Setup* (`atom/docs/environment_setup.md`).

### 1.2 Interfaces (one action + two services)

| Interface | Type | Purpose |
|---|---|---|
| `/move_action` | `moveit_msgs/action/MoveGroup` (**Action**) | send target joint angles; MoveIt plans and executes |
| `/moveit_controller_enable` (older builds: `/EAIHardware/set_arm_enable`) | `std_srvs/SetBool` (Service) | enable the arm in real mode; name varies by XARM version, demo auto-detects at runtime (no such service in sim) |
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

**XARM's "enable" is not about powering motors — it's about whether XARM sends commands to body_control.** Without enable, MoveIt planning/execution both report success, the controller is OK, but XARM never publishes `/arm/cmd_pos` → the physical arm doesn't move. The demo's `enable_arm()` probes the enable services in order (new `/moveit_controller_enable`, older `/EAIHardware/set_arm_enable`), **falling through to the next candidate on failure**; if all fail it checks `/EAIHardware/debug` — `arm_enable: 1` means already enabled and it proceeds (enable is a persistent hardware state across processes; re-enabling after a previous demo is often rejected — harmless noise). Manual equivalent (use whichever name exists on your machine):

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
| enable | `enable_arm()` → `ENABLE_SRV_CANDIDATES` (detects `/moveit_controller_enable` or older `/EAIHardware/set_arm_enable`) | open the "send-commands gate" in real mode | unchanged (`set_all_enable` enables all at once) |
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

> Same bottom layer: both end at `/arm/cmd_*` → body_control → motors. The only difference is "publish by hand" vs "MoveIt plans, XARM publishes". Because they contend for the same `/arm/cmd_pos`, **atom04 and XARM enable are mutually exclusive in both directions**: enable fails while atom04 is running; conversely, **while XARM is enabled (which it is after running this atom) it streams commands at 250Hz, so atom04's direct commands are instantly overridden → the arm "silently doesn't move"**. One arm, one command source at a time — the enable switch is the hand-over of the steering wheel. Both directions are handled automatically: this atom auto-enables; atom04 auto-disables XARM enable when it detects it — switching back and forth needs no manual commands.

## 5. Troubleshooting

| Symptom | error_code / cause | Fix |
|---|---|---|
| Plan/execute succeed but arm static | not enabled (real #1 pitfall); or in sim (physical arm never moves) | `set_arm_enable true`; in sim watch RViz/joint_states |
| `error_code=-4` CONTROL_FAILED | moveit controller not active | switch controller (demo does it; manual `switch_controllers --activate`) |
| `error_code=-15` INVALID_GROUP_NAME | wrong planning group | check SRDF: `ros2 param get /move_group robot_description_semantic` |
| goal rejected / planning failed | target out of limits or in collision | check §1.3 limits; move the arm near the target first |
| `/move_action` won't connect | MoveIt component not up | `ros2 action list \| grep move_action`; start `tianyi2_moveit.launch.py` |
| enable fails / auto-disables | another app publishes `/arm/cmd_pos` (e.g. leftover atom04, or the teleop `teleop_dispatcher`) | `bash scripts/stop_all.sh`, then retry; `ros2 topic info /arm/cmd_pos -v` to list publishers (should be just 1) |
| `skip enable` (enable service not found) | **XARM upgrade renamed the enable service**: old `/EAIHardware/set_arm_enable` → new `/moveit_controller_enable` (both `SetBool`) | demo auto-detects both names and falls through on failure; verify manually with `ros2 service list \| grep -iE 'enable'`, `ros2 service type <name>` |
| log shows `enable failed... trying next candidate` then `arm already enabled (arm_enable=1)... continuing` | **harmless noise**: enable is a persistent hardware state; a previous demo already enabled the arm, so re-enabling gets rejected | nothing to do; the truth is `ros2 service call /EAIHardware/debug eai_manipulator_msgs/srv/Info` → `arm_enable` |
| after starting XARM, all spawners stuck at `waiting for /controller_manager/list_controllers` | **controller_manager failed to load the hardware plugin** (typical after a system upgrade removed a library, e.g. pinocchio 3.9 replaced by 4.0 → `dlopen libpinocchio_parsers.so.3.9.0` fails) | ① `ldd /home/ubuntu/XARM/install/tianyi_hardware/lib/libtianyi_hardware.so \| grep 'not found'` — if something's missing → ② add the vendor-provided compat lib dirs to `~/.bashrc`: `export LD_LIBRARY_PATH=<libdir1>:<libdir2>:$LD_LIBRARY_PATH` (must point at the **subdirectories** containing the .so files) → clean restart. A healthy factory machine with clean `ldd` does **not** need this; remove the line once XARM is rebuilt against the new system |
| real-mode "watchdog timeout" | body_control not up / no `/arm/status` | start body_control first, then XARM |

Two self-checks: `ros2 service list | grep -iE 'enable'` (confirm the enable service name; `ros2 service type <name>` should be `std_srvs/srv/SetBool`), `ros2 control list_controllers | grep -i moveit` (controller active).

## 6. Going further

- **arm_mode (hardware mode)**: `0` compliant/force-position (this demo's default) / `3` stiff position loop (tighter tracking) / `1` stiff velocity loop (needs realtime kernel) / `2` gravity compensation. Switch: `ros2 service call /EAIHardware/set_arm_mode eai_manipulator_msgs/srv/Mode "{mode: 3}"`. Mode also decides whether XARM publishes `/arm/cmd_pos` or `/arm/cmd_ctrl`.
- **Cartesian (end-effector) control**: this atom is joint-space MoveIt; to command an end-effector pose (instead of joint angles), see **atom06 (MoveIt Cartesian control)**.
- **Library vs raw action**: this demo sends `/move_action` directly (lightest, most transparent); you could also use MoveIt's Python library `moveit_py` (`.plan()/.execute()`), but that requires loading the MoveIt config params.
