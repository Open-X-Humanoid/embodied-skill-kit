# Atom 6 · Arm · XARM MoveIt Cartesian (End-Effector) Control

**English** | [简体中文](atom06_arm_moveit_endpose_guide_zh-CN.md)

**In one line**: give **MoveIt** an end-effector *pose* goal — where the hand should go in space (x/y/z) and how it should be oriented (quaternion) — and it solves IK, plans a collision-free trajectory, and executes it. Exactly one thing differs from atom05: **atom05 gives joint angles, atom06 gives an end-effector pose**. Everything else (enable, controller switch, `/move_action`) is identical.

| Companion | Path |
|---|---|
| Code (simple) | `atom/demos/atom06_arm_moveit_endpose.py` |
| Demo video | same name as the code, under `atom/assets/videos/` |

> No `_robust` version yet. For controller conflicts / `CONTROL_FAILED(-4)`, borrow the approach in `atom/demos/atom05_arm_moveit_robust.py` (strict switching + stepping aside from controllers claiming this arm) — it ports over directly.

> Read atom05 first: enable, controller switch, and `/move_action` are shared by both atoms and aren't re-explained here.

## 1. Overview

### 1.1 Run it (same prerequisites as atom05)

- **Board**: **x86, user ubuntu** (same board as body_control; XARM lives at `/home/ubuntu/XARM`).
- ★**Source XARM in every terminal that runs the demo**: `source /home/ubuntu/XARM/install/setup.bash`. The startup script sources XARM inside its tmux panes only; it does not modify a separately opened demo terminal.
- **Real-robot SOP has 5 steps** (skip any → arm won't move): start body_control → start XARM body → start MoveIt component → **enable arm** → **switch to MoveIt controller** → (run the demo). The demo does the last two automatically.

**One-click prerequisites** (recommended, see `scripts/start_xarm.sh`):

```bash
# Real robot (verified): start body_control on the x86 first, then XARM + MoveIt
# ⓪ Stop the teleop service (auto-starts on boot, occupies /arm/cmd_pos; enable fails if it's running.
#    Comes back automatically after reboot; harmless to run when already stopped — just run it once per boot.
#    Optional pre-check:
#    systemctl is-active teleop_robot   # active = running, stop it; inactive = already stopped;
#                                       # "could not find unit" = this machine doesn't have it (older system), skip this step
sudo systemctl stop teleop_robot

bash scripts/start_body_control.sh          # another terminal, see Prerequisite · Environment Setup
bash scripts/start_xarm.sh real

# Run the demo in a new terminal
source /home/ubuntu/XARM/install/setup.bash
python3 atom/demos/atom06_arm_moveit_endpose.py
```

> ⚠ **High-frequency pitfall: instant `error_code=99999`** — MoveIt's joint limits are **tighter** than the URDF (e.g. `shoulder_yaw_l_joint`: MoveIt=±1.5 vs URDF=±2.96), while QP/teleop follow the wider URDF limits. **After a QP atom (atom07/08) or teleop, the arm often parks outside MoveIt's bounds → this atom gets instantly rejected with 99999** (signature: millisecond failure, QP atoms fine). Check: `tmux capture-pane -t xarm.1 -p -J -S -400 | grep -i 'outside bounds'` names the joint; fix: move it back via QP (see the 99999 row in Atom 5's troubleshooting table).

What the demo does: read the current TCP pose → **keep the orientation, translate the end-effector up 5 cm** → return to the starting pose. Reversible and slow.

> **Sim mode** (`bash scripts/start_xarm.sh sim`, with RViz, no real robot / no body_control): Cartesian control collides with the robot itself more easily than joint control (the same end-effector point can map to several joint solutions), so previewing in sim can help when no robot is available. ⚠ Not tested in sim in this project — commands should work in theory, adapt to your machine; `real` is what's verified. For the base ROS environment see *Prerequisite · Environment Setup* (`atom/docs/environment_setup.md`).

### 1.2 Interfaces (one more than atom05: TF)

| Interface | Type | Purpose |
|---|---|---|
| `/move_action` | `moveit_msgs/action/MoveGroup` (**Action**) | send an **end-effector pose constraint**; MoveIt solves IK, plans, executes |
| `/moveit_controller_enable` (older builds: `/EAIHardware/set_arm_enable`) | `std_srvs/SetBool` (Service) | enable the arm in real mode; name varies by XARM version, demo auto-detects and falls through (no such service in sim) |
| `/controller_manager/switch_controller` | ros2_control (Service) | activate `moveit_left_arm_controller` |
| **`/tf`, `/tf_static`** | `tf2_msgs/TFMessage` (Topic) | **read the current TCP pose as the motion start** (atom05 uses `/joint_states`; here it's TF) |

> Why the start point switches to TF: atom05's start is "7 joint angles", which `/joint_states` hands you directly. atom06's start is "where the end-effector is in space" — turning joint angles into an end-effector pose requires **forward kinematics**, and the TF tree has already been computed and published live by `robot_state_publisher`. Just query it instead of doing the math.

### 1.3 ★The two frames (atom06's #1 pitfall)

Everything in Cartesian control rests on two frames, and **a wrong name makes the demo time out and exit**:

| Constant | Value | What it is |
|---|---|---|
| `BASE_FRAME` | **`base`** | the reference frame. What the end-effector pose is measured *relative to* |
| `EE_LINK` | **`left_tcp_link`** | the left arm's TCP (tool center point) link — literally "the point the hand moves to" |

⚠ **`base` is not `base_link`**. Many ROS tutorials name the root link `base_link`, but the TianYi 2.0 URDF **has no `base_link` frame at all**. The actual structure is:

```
base_footprint          ← the URDF's true root link
    │  world_to_base_link (fixed, zero offset: xyz="0 0 0" rpy="0 0 0")
    ↓
base                    ← the BASE_FRAME this demo uses
```

`base_footprint` and `base` are joined by a **zero-offset fixed joint**, so the two frames coincide exactly in space — either works, and the demo picks `base`. Writing `base_link` produces:

```
Invalid frameID "base_link" passed to canTransform argument target_frame - frame does not exist
```

Verify before running (it should print translation/rotation values, not an error):

```bash
ros2 run tf2_ros tf2_echo base left_tcp_link
```

To see the whole TF tree (root frame and end-effector frame at a glance):

```bash
ros2 run tf2_tools view_frames        # writes frames.pdf
```

> For the right arm swap `left_tcp_link` → `right_tcp_link`; `BASE_FRAME` is unchanged (same root).

### 1.4 Limits and reachability (what "limits" look like in Cartesian control)

Joint limits still apply — they just **stop you differently**. In atom05 you give joint angles directly and an out-of-range one is rejected. In atom06 you give a pose, **MoveIt solves IK first**, and if the resulting joint angles violate limits or the point simply can't be reached, it returns `NO_IK_SOLUTION(-31)` — **not a "limit error" but "no solution"**.

- The per-joint limit table lives in *Atom 5 · Arm · XARM MoveIt Joint Motion* (`atom/docs/atom05_arm_moveit_guide.md`) §1.3 and isn't duplicated here.
- **The reachable region is called the workspace** — roughly a shell centered on the shoulder with the arm's reach as radius, further trimmed by joint limits. **It is not a tidy sphere.**
- Practical rule: **step incrementally from the current pose** (the demo defaults to 5 cm). That is far more reliable than naming an absolute coordinate out of thin air. When a point is out of reach, move the arm near the target first, then plan.

## 2. Four must-dos (one more than atom05: reading TF)

### 2.1 Enable the arm (the #1 real-mode pitfall; same as atom05)

**XARM's "enable" is not about powering motors — it's about whether XARM sends commands to body_control.** Without it, MoveIt reports success while the physical arm doesn't budge. The demo's `enable_arm()` does it; manual equivalent:

```bash
ros2 service call /EAIHardware/set_arm_enable std_srvs/srv/SetBool "{data: true}"
```

### 2.2 Switch to the MoveIt controller (same as atom05)

`moveit_left_arm_controller` must be active before execution, else `error_code=-4 CONTROL_FAILED`. The demo's `activate_moveit_controller()` handles it.

### 2.3 ★Read the current end-effector pose (new in atom06)

`read_ee_pose()` queries TF for `base → left_tcp_link` to get the current TCP position and orientation as the start. If it can't read it, it **returns None and refuses to move** — a deliberate safety design: never move when you don't know where the hand is.

### 2.4 Send the pose goal (the real difference from atom05)

atom05 sends 7 `JointConstraint`s; atom06 sends **one position constraint + one orientation constraint**, both on `EE_LINK`:

| Constraint | Type | Demo tolerance | Meaning |
|---|---|---|---|
| Position | `PositionConstraint` | sphere of radius **0.01 m** | the TCP landing inside this small sphere counts as arrived |
| Orientation | `OrientationConstraint` | **0.05 rad ≈ 2.9°** | allowed angular deviation per axis |

> The position constraint expresses its tolerance region as a `SolidPrimitive.SPHERE` — this is MoveIt's general idiom: a goal isn't a mathematical point but **a small acceptable region**. Too tight and planning fails; too loose and you lose accuracy.

## 3. Code walkthrough (core)

| Module | Code anchor | Role | Change for right arm? |
|---|---|---|---|
| enable | `enable_arm()` → `/EAIHardware/set_arm_enable` | open the "send-commands gate" in real mode | unchanged |
| switch controller | `activate_moveit_controller()` → `switch_controller` | activate the moveit controller | change `MOVEIT_CONTROLLER` |
| **read start** | `read_ee_pose()` → TF `base → left_tcp_link` | current end-effector pose as start; refuse to move if unreadable | change `EE_LINK` |
| **build constraints** | `_pose_goal()` → Position + Orientation Constraint | translate a Pose into MoveIt constraints | change `EE_LINK` |
| plan+execute | `move_to_pose()` → `MoveGroup` action | send /move_action, check error_code | change `GROUP` |

**One line per module**:

- `read_ee_pose()`: loops `spin_once` + `lookup_transform`, returns None after a 5 s timeout. It passes `Time()` (time zero) to mean "the latest available transform".
- `_pose_goal()`: the position constraint holds a spherical tolerance region (sphere center = target point); the orientation constraint takes the target quaternion plus a per-axis tolerance. Note the sphere's own orientation is the identity quaternion `Quaternion(w=1.0)` — a sphere is isotropic so its orientation is meaningless, but the field can't be left empty.
- `move_to_pose()`: builds a `MotionPlanRequest` (group, attempts, scaling) → attaches constraints → `plan_only=False` (plan then execute) → blocks for the result → `error_code == 1` means success.
- `main()`: read the start → `copy.deepcopy` the target, **add translation only, never touch orientation** → go → come back.

**Generalize**:

- **Right arm**: `GROUP="right_arm"` + `EE_LINK="right_tcp_link"` + `MOVEIT_CONTROLLER="moveit_right_arm_controller"`. `BASE_FRAME` is unchanged.
- **Change orientation**: the demo only translates. To reorient, replace `target.orientation` with your target quaternion. Hand-writing quaternions is error-prone; in practice use `tf_transformations.quaternion_from_euler(r, p, y)` to convert from RPY.
- **Follow a series of waypoints**: just call `move_to_pose()` repeatedly — but each leg is planned independently (the arm decelerates to zero between legs). For a genuinely continuous straight line or arc, see the Cartesian Path note in §6.

## 4. Difference from atom05

| | atom05 (joint space) | atom06 (Cartesian) |
|---|---|---|
| What you give | 7 joint angles | end-effector pose (x/y/z + quaternion) |
| Start read from | `/joint_states` | **TF** (`base → left_tcp_link`) |
| Goal expressed as | 7 `JointConstraint`s | 1 `PositionConstraint` + 1 `OrientationConstraint` |
| What MoveIt adds | plans directly | **solves IK first**, then plans |
| New failure mode | out of limits | **`NO_IK_SOLUTION(-31)`** (unreachable / no solution) |
| Good for | "I know what each joint should read" | **"I know where the hand must go"** (closer to real grasp/place tasks) |

> The bottom layer is identical: both send `/move_action` → `moveit_left_arm_controller` → XARM → `/arm/cmd_*` → body_control → motors. **The difference is purely in how the goal is described**, not in the execution path.

**Which one when**: aligning the hand to an object (grabbing a box, placing a part) → atom06, because you know where the object is, not how far each joint should turn. Fixed pose changes (return home, raise a hand) → atom05, because the joint angles are known and you skip the IK step entirely.

## 5. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Invalid frameID "xxx" ... frame does not exist` | wrong frame name (**the root link is `base`, not `base_link`**) | verify with `ros2 run tf2_ros tf2_echo base left_tcp_link`; fix `BASE_FRAME`/`EE_LINK` |
| enable errors / spawners stuck at `waiting for list_controllers` / plan succeeds but arm static | same root causes as atom05: renamed enable service, redundant-enable noise, teleop occupying the topic, missing library after system upgrade (pinocchio) | see the troubleshooting table in *Atom 5 · Arm MoveIt joint motion* (`atom/docs/atom05_arm_moveit_guide.md`) |
| `timed out reading ... pose (TF)` | wrong frame name, or nobody publishes TF (MoveIt/robot_state_publisher not up) | `ros2 topic list \| grep tf` to confirm TF exists, then check the frame names |
| `error_code=-31` NO_IK_SOLUTION | target unreachable / IK solution violates joint limits | shrink the translation; move the arm near the target first; check atom05's limit table |
| `error_code=-18` INVALID_LINK_NAME | `EE_LINK` isn't a link in the URDF | check the `left_tcp_link` spelling |
| `error_code=-21` FRAME_TRANSFORM_FAILURE | `BASE_FRAME` isn't a valid frame | same as row 1 |
| `error_code=-1` PLANNING_FAILED | no collision-free path found | loosen tolerances; try a different start pose; raise `allowed_planning_time` |
| `error_code=-4` CONTROL_FAILED | moveit controller not active | the demo switches automatically; manually `ros2 control switch_controllers --activate moveit_left_arm_controller` |
| Plan/execute succeed but arm static | not enabled (real #1 pitfall); or in sim | `set_arm_enable true`; in sim watch RViz |
| `error_code=-15` INVALID_GROUP_NAME | wrong planning group | check the SRDF: `ros2 param get /move_group robot_description_semantic` |

Three self-checks:

```bash
ros2 run tf2_ros tf2_echo base left_tcp_link          # do the frames connect?
ros2 control list_controllers | grep -i moveit        # controller active?
ros2 action list | grep move_action                   # is the MoveIt component up?
```

## 6. Going further

- **Cartesian Path (a true straight line)**: this demo says "here's an end-effector goal, plan your way there" — the intermediate path is **not guaranteed straight**. For a strict line or arc (inserting a pin, dragging, following a surface), use MoveIt's `/compute_cartesian_path` service (give it a list of waypoints, get an interpolated Cartesian trajectory back) and send that to the controller yourself.
- **Relax an orientation axis**: when grasping a cylindrical object, rotation about its own axis is usually irrelevant — setting that axis's `OrientationConstraint` tolerance to `3.14` (effectively unconstrained) sharply raises the IK success rate. This is the single most common tuning knob in real projects.
- **Add collision objects**: push `CollisionObject`s (a box, a tabletop) into MoveIt's planning scene and planning routes around them automatically. This is MoveIt's biggest advantage over raw control.
- **arm_mode (hardware mode)**: `0` compliant/force-position (default) / `3` stiff position loop / `1` stiff velocity loop (needs realtime kernel) / `2` gravity compensation. Switch: `ros2 service call /EAIHardware/set_arm_mode eai_manipulator_msgs/srv/Mode "{mode: 3}"`.
