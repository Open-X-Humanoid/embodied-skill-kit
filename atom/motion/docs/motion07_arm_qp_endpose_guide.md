# Motion 7 · Arm · XARM QP Cartesian (End-Effector) Control

**English** | [简体中文](motion07_arm_qp_endpose_guide_zh-CN.md)

**In one line**: give XARM's **QP end-effector controller** (`endpose_single_arm_qp_L_controller`) an end-effector *pose* goal and it performs **local responsive tracking** to bring the TCP there, with built-in collision avoidance. Difference from motion05: **motion05 is MoveIt "plan a whole trajectory once"; motion07 is QP "online small-step tracking"** — a goal too far from the current pose is rejected outright (error bounds), so you can only feed small increments.

| Companion | Path |
|---|---|
| Code (simple) | `atom/motion/motion07_arm_qp_endpose.py` |
| Demo video | same name as the code, under `atom/motion/assets/videos/` |

> Read motion05 first (frames / reading the start pose via TF) and motion06 (QP enable / controller switching / mutual exclusion); this guide only expands what is unique to motion07.

## 1. Overview

### 1.1 Run it (same as motion06 — no MoveIt component needed)

- **Board**: **x86, user ubuntu**; ★running the demo requires `source /home/ubuntu/XARM/install/setup.bash` (`eai_manipulator_msgs` lives in XARM).

```bash
# ⓪ Stop the teleop service (occupies /arm/cmd_pos; enable fails if running)
sudo systemctl stop teleop_robot

# Real robot: body_control → XARM body (no MoveIt component)
bash scripts/start_body_control.sh
bash scripts/start_xarm.sh real

# Run the demo
source /home/ubuntu/XARM/install/setup.bash
python3 atom/motion/motion07_arm_qp_endpose.py
```

What the demo does: switch to the QP end-effector controller → **cap the speed** (`vel_limits` to 0.5 rad/s) → read the current TCP pose via TF → **keep orientation, lift the end-effector 5 cm** → return to the start pose.

> This controller has **built-in collision avoidance** (2 body spheres + 7 arm spheres) — one notch safer than motion06's joint controller; still keep people and objects away, E-stop in hand.

### 1.2 Interfaces

| Interface | Type | Purpose |
|---|---|---|
| `/endpose_single_arm_qp_L_controller/endPosSingleTarget` | `eai_manipulator_msgs/action/EndPosSingleTarget` (**Action**) | send an end-effector pose goal (`ArmTargetPose`); returns `success` on arrival |
| `/endposetarget_L` | `eai_manipulator_msgs/msg/ArmTargetPose` (Topic, streaming) | continuous pose stream — **mutually exclusive with the action** |
| `/endpose_single_arm_qp_L_controller/set_parameters` | `rcl_interfaces/SetParameters` (Service) | **code-level speed tuning** (set `vel_limits`, see 2.3) |
| `/EAIHardware/set_arm_enable` etc. | `std_srvs/SetBool` (Service) | real-mode enable (the demo probes candidate names) |
| `/controller_manager/switch_controller` | ros2_control (Service) | activate this controller (auto-stops whoever holds the arm) |
| TF `base → left_tcp_link` | Topic | read the current TCP pose as the start |

### 1.3 ★How the goal is expressed: ArmTargetPose (QP-specific)

No MoveIt constraints — one message states "the desired pose of B relative to A":

| Field | Demo value | Meaning |
|---|---|---|
| `from_frame` | `base` | reference frame (XARM manual's recommendation for TianYi; `base` coincides with the root `base_footprint` at zero offset, see motion05 guide §1.3) |
| `to_frame` | `left_tcp_link` | end-effector TCP link |
| `target` | goal Pose | **the desired pose of to_frame relative to from_frame** |
| `offset_x/y/z` | 0 | reference-point offset in the end-effector frame (e.g. a tool tip); unused in the demo |

### 1.4 ★Error bounds: a far goal is rejected outright (the biggest difference from motion05)

This controller is a **local** tracker: the desired pose must be close to the current end-effector pose (distance ≤ `dis_err_bound`, angle ≤ `ori_err_bound`), otherwise it **rejects the command, raises an alarm, and stays put**. So feed it small steps — the demo lifts 5 cm per step. For long moves, split into multiple steps, each starting from the new current pose.

Check the current bounds before running:

```bash
ros2 param get /endpose_single_arm_qp_L_controller dis_err_bound
ros2 param get /endpose_single_arm_qp_L_controller ori_err_bound
```

> Contrast with motion05: MoveIt accepts any distance (it plans the whole path); QP rejects far goals. Not a defect — a responsive tracker assumes an upstream source (e.g. vision) feeding nearby goals at high frequency.

## 2. Core operations

### 2.1 Enable + switch controller (same as motion06, not repeated)

`enable_arm()` probes candidates + `_scan_arm_controllers()` auto-stops whoever claims the arm + STRICT switch.

### 2.2 Send the pose goal

`move_to_pose()` fills `ArmTargetPose` (from/to/target/offset) → sends the action → blocks for `result.success`. **This action returns only on arrival** (it stops per `dis_threshold`/`ori_threshold`/`step_threshold`) — unlike motion06's "return ≠ arrival", so no extra wait_reached is needed here.

### 2.3 ★Code-level speed tuning (a key QP-vs-MoveIt difference)

MoveIt takes speed **per goal** (`max_velocity_scaling_factor`); **QP speed is a resident controller parameter** `vel_limits`. The demo's `set_vel_limits()` uses the parameter service, right after the controller switch, to cap it at `[0.5]×7 rad/s` (smaller = slower) — equivalent to:

```bash
ros2 param set /endpose_single_arm_qp_L_controller vel_limits '[0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]'
```

Tune via the `VEL_LIMITS` constant; `None` leaves the controller's current value untouched. ⚠ Tuning down is the safe direction.

## 3. Code walkthrough (core)

| Module | Code anchor | Role | Change for right arm? |
|---|---|---|---|
| enable | `enable_arm()` + `_already_enabled()` | candidate probing + `arm_enable` fallback | unchanged |
| conflicts / switch | `_scan_arm_controllers()` + `activate_qp_controller()` | stop the holder + STRICT activate | change `QP_CONTROLLER` (L→R) |
| **speed** | `set_vel_limits()` → `set_parameters` service | cap `vel_limits` to a safe speed | unchanged |
| read start | `read_ee_pose()` → TF `base → left_tcp_link` | current TCP pose as the start; refuse to move if unreadable | change `TO_FRAME` |
| send goal | `move_to_pose()` → `EndPosSingleTarget` action | fill ArmTargetPose, wait for arrival | change `TO_FRAME` |

**Generalize**: right arm = `L`→`R` in `QP_CONTROLLER` + `TO_FRAME="right_tcp_link"`; `FROM_FRAME` unchanged. To align a tool tip instead of the TCP, fill `offset_x/y/z` (offset in the end-effector frame) — the goal then means "bring the tool tip there".

## 4. Difference from motion05 (MoveIt Cartesian)

| | motion05 (MoveIt) | motion07 (QP) |
|---|---|---|
| How it works | IK + plan a whole trajectory offline | online small-step tracking (local optimization) |
| Prerequisites | XARM body + **MoveIt component** | XARM body only |
| Goal expression | Position + Orientation Constraint | one `ArmTargetPose` message |
| Goal distance | any (it plans the way there) | **must be nearby** (beyond the bounds → rejected) |
| Failure mode | `NO_IK_SOLUTION(-31)` etc. | rejected + alarm, arm stays put |
| Collision avoidance | planning-scene based | built-in sphere model |
| Speed | per-goal `max_velocity_scaling_factor` | controller parameter `vel_limits` |
| Good for | one-shot "hand goes there" (grasp/place) | continuous tracking (visual servoing, teleop following) |

## 5. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Goal rejected / alarm, no motion | **beyond error bounds** (one step too far) | shrink `DEMO_DELTA_XYZ`; check `dis_err_bound`; split into steps |
| `timed out reading ... pose (TF)` | wrong frame name (**`base`, not `base_link`**) or no TF publisher | verify with `ros2 run tf2_ros tf2_echo base left_tcp_link` |
| QP action unreachable (10 s) | controller not active / arm held by another family | `ros2 control list_controllers`; the demo auto-steps-aside |
| Motion too fast | `vel_limits` is a controller parameter | the demo caps it already; lower `VEL_LIMITS` further or `ros2 param set` |
| `set vel_limits failed` | the controller may forbid runtime changes | fall back to `ros2 param set`; if that also fails, accept the default speed and shrink steps |
| Enable-related errors | same roots as motion04/07 | see *Motion 4*'s troubleshooting table; `sudo systemctl stop teleop_robot` |
| Topic stream ignored | action and topic are mutually exclusive | stop the action client, then stream |

## 6. Going further

- **Relax orientation axes**: parameter `OriWeight: [x, y, z]` (0.0–1.0) can zero a given axis's orientation weight — 5-DOF (line pose) / 4-DOF (plane pose) control, often combined with a tool frame. Freeing rotation about a cylinder's axis markedly improves reachability when grasping.
- **Redundant arm angle**: a 7-DOF arm reaches the same pose with infinitely many elbow orientations. `redundant_degrees` (shoulder-yaw reference) + `redundant_degrees_weight` (0–500; higher tracks the reference tighter at slight cost to precision) control the null-space arm angle — for dodging obstacles or a tidy elbow pose.
- **Streaming control**: switch to the `/endposetarget_L` topic and feed poses at high frequency (visual servoing / teleop following in full). Stop the action client first.
- **Extra avoidance sphere**: `collision_freeBall_pos` (x, y, z, r) adds one free collision sphere in space — a lightweight "no-go zone" compared to MoveIt's planning scene.
- **Dual arm**: coordinated dual-arm Cartesian control lives in `endpose_dual_arm_qp_controller` (not covered by this kit yet; same interface family).
