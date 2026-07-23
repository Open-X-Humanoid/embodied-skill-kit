# Atom 7 · Arm · XARM QP Joint-Space Control

**English** | [简体中文](atom07_arm_qp_guide_zh-CN.md)

**In one line**: hand 7 target joint angles to XARM's **QP joint controller** (`jointspace_arm_L_controller`), which smooths velocity/acceleration/jerk online, checks limits, and executes. Exactly one thing differs from atom05: **atom05 is MoveIt "plan a whole trajectory offline, then execute"; atom07 is QP "responsive online tracking"** — each new goal is smoothly tracked the moment it arrives, suited to continuously changing targets (e.g. visual servoing).

| Companion | Path |
|---|---|
| Code (simple) | `atom/demos/atom07_arm_qp.py` |
| Demo video | same name as the code, under `atom/assets/videos/` |

> Read atom05 first: enable and controller switching are shared by all arm atoms; this guide only expands what is QP-specific.

## 1. Overview

### 1.1 Run it (one step fewer than atom05 — QP needs no MoveIt component)

- **Board**: **x86, user ubuntu** (same board as body_control; XARM lives at `/home/ubuntu/XARM`).
- ★**Running the demo requires sourcing XARM** (the QP message package `eai_manipulator_msgs` lives in XARM, not in base ROS): `source /home/ubuntu/XARM/install/setup.bash`, in every new terminal.

```bash
# ⓪ Stop the teleop service (auto-starts on boot, occupies /arm/cmd_pos; enable fails if running)
sudo systemctl stop teleop_robot

# Real robot: body_control first, then the XARM body (no tianyi2_moveit.launch.py needed for QP)
bash scripts/start_body_control.sh          # another terminal, see Prerequisite · Environment Setup
bash scripts/start_xarm.sh real

# Run the demo (source XARM first)
source /home/ubuntu/XARM/install/setup.bash
python3 atom/demos/atom07_arm_qp.py
```

What the demo does: read current joint angles → **move only the elbow pitch by +0.3 rad (≈17°)** → wait for actual arrival → return to the start. Reversible, small-amplitude.

> ⚠ **This controller has no collision avoidance** (stated in the XARM manual; the self-collision-aware variant is `jointspace_arm_qpik_L_controller`, but it does not support the action interface). Keep people and objects away from the arm, E-stop in hand. For the base ROS environment see *Prerequisite · Environment Setup* (`atom/docs/environment_setup.md`).

### 1.2 Interfaces (XARM-native, not /move_action)

| Interface | Type | Purpose |
|---|---|---|
| `/jointspace_arm_L_controller/jointspace` | `eai_manipulator_msgs/action/JointSpace` (**Action**) | send 7 target joint angles; `result.success` reports the outcome |
| `/jointspace_commands_L` | `std_msgs/Float64MultiArray` (Topic, streaming) | continuous goal stream — **mutually exclusive with the action; never use both** |
| `/EAIHardware/set_arm_enable` etc. | `std_srvs/SetBool` (Service) | real-mode enable (name varies by XARM version; the demo probes candidates) |
| `/controller_manager/switch_controller` | ros2_control (Service) | activate `jointspace_arm_L_controller` |
| `/joint_states` | `sensor_msgs/JointState` (Topic) | current angles as the start + **arrival check** |

> Division of labor: the **action** suits "send one goal and wait for arrival" (this demo); the **topic** suits "feed goals at high frequency" (visual servoing). Only one at a time — when using the action, make sure nobody streams to `/jointspace_commands_L`.

### 1.3 Joint limits (same as atom05, not duplicated)

The 7-joint limit table is in *Atom 5 · Arm · XARM MoveIt Joint Motion* (`atom/docs/atom05_arm_moveit_guide.md`) §1.3. The QP controller **checks limits itself**: above the upper limit → `600101`, below the lower → `600102`; the command is rejected and the arm stays put.

## 2. Core operations (including one QP-specific pitfall found on the real robot)

### 2.1 Enable + switch controller (same pattern as atom05/06)

Enable opens the "XARM → body_control command gate"; when switching, the **moveit / jointspace / endpose controller families are mutually exclusive** (they all claim the same arm's joint interfaces). The demo's `_scan_arm_controllers()` finds and stops whoever holds the arm, then switches STRICT.

### 2.2 Send the goal (far simpler than MoveIt)

No constraints to build — the action goal is just a 7-element array:

```python
goal = JointSpace.Goal(target_positions=[0.0, 1.18, 0.0, -1.3, 0.0, -0.13, 0.18])
```

Order = J1 shoulder pitch → J7 wrist roll (same as `JOINT_NAMES`).

### 2.3 ★Wait for actual arrival (this atom's #1 real-robot pitfall)

**The QP action's `success` means "goal accepted", not "goal reached"** (typical of online-tracking controllers). If you send the next goal right after it returns, the previous goal gets overwritten → the arm barely moves. So after the action returns, the demo runs `wait_reached()`: poll `/joint_states` until every joint is within `target ±0.05 rad` (15 s timeout with a warning).

> Contrast with atom05: MoveIt's action result **returns only after execution finishes** (the trajectory controller confirms arrival), so atom05 needs no such step. This is the most important hands-on difference between "offline planning" and "online tracking".

## 3. Code walkthrough (core)

| Module | Code anchor | Role | Change for right arm? |
|---|---|---|---|
| enable | `enable_arm()` + `_already_enabled()` | probe enable-service candidates (name varies by version); if a repeat enable is rejected, check `arm_enable` as fallback | unchanged |
| find conflicts | `_scan_arm_controllers()` → `list_controllers` | find active controllers claiming this arm via `claimed_interfaces` | change `JOINT_NAMES` |
| switch controller | `activate_qp_controller()` → STRICT switch | stop conflicting controllers + activate QP; fail loudly | change `QP_CONTROLLER` (L→R) |
| read start | `read_current()` on `/joint_states` | current angles as the start | change `JOINT_NAMES` |
| send goal | `move_to_joints()` → `JointSpace` action | send the 7-element array, check `result.success` | unchanged |
| **wait arrival** | `wait_reached()` polling `/joint_states` | **action return ≠ arrival**; wait for convergence within ±0.05 rad | change `JOINT_NAMES` |

**Generalize**: right arm = `L`→`R` in `QP_CONTROLLER` + joint names `_l_`→`_r_`. The goal-sending pattern is identical.

## 4. Difference from atom05 (MoveIt joint)

| | atom05 (MoveIt) | atom07 (QP) |
|---|---|---|
| How it works | plan a whole trajectory offline → execute | responsive online tracking; each goal tracked on arrival |
| Prerequisites | XARM body + **MoveIt component** | XARM body only |
| Goal expression | 7 `JointConstraint`s | one 7-element array (much simpler) |
| Action return means | **execution finished** | **goal accepted** (wait for arrival yourself) |
| Collision avoidance | yes (MoveIt planning) | **none** (qpik variant has it but no action) |
| Good for | one-shot point-to-point with avoidance | continuously changing goals (visual servoing), low-latency tracking |

> Same bottom layer: both go XARM → `/arm/cmd_*` → body_control → motors. The difference is who generates the joint trajectory.

## 5. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `600101` / `600102` | target above/below joint limit | check atom05's limit table; step incrementally from the current angle |
| `600100` | QP optimization infeasible | shrink the increment; check whether the start pose is near singular/limit |
| Action succeeds but arm static / moves briefly | ① not enabled ② action return ≠ arrival, goal overwritten by the next one | ① check `arm_enable=1` ② rely on `wait_reached()` before sending the next goal |
| QP action unreachable (10 s timeout) | controller not active / arm held by another family | `ros2 control list_controllers`; the demo auto-steps-aside — manually, deactivate the holder first |
| Enable-related errors | same roots as atom05: renamed service, teleop occupying the topic, repeat-enable noise | see *Atom 5*'s troubleshooting table; `sudo systemctl stop teleop_robot` |
| Arm moves too fast | QP speed is a **controller parameter**, not part of the goal | `ros2 param set /jointspace_arm_L_controller vel_limits '[0.5,0.5,0.5,0.5,0.5,0.5,0.5]'` (rad/s, smaller = slower, takes effect immediately) |
| Topic stream ignored | someone is using the action (mutually exclusive) | stop the action client, then stream |

Controller log codes: `100101` new goal received (info), `100102` internal planner finished (info, usable as an arrival cue), `6001xx` see above.

## 6. Going further

- **Speed tuning**: `vel_limits` (per-joint velocity caps, rad/s) is the master valve — `ros2 param set` takes effect immediately; `acc_limits`/`jerk_limits` are read-only references. ⚠ The manual states vel_limits must not exceed the mechanism's physical capability — **tuning down is the safe direction**. For the code-level version see atom08's `set_vel_limits()` (same parameter service; ports directly).
- **Streaming control (the right way to do visual servoing)**: switch to the `/jointspace_commands_L` topic and feed goals at high frequency; the controller blends them smoothly — this is QP's "responsive" nature in full. Stop the action client first.
- **Need self-collision avoidance**: switch to `jointspace_arm_qpik_L_controller` (self- and dual-arm collision aware), topic-only, no action; the manual recommends this lighter controller when self-collision is not a concern.
- **End-effector version**: to command an end-effector pose instead of joint angles, see *Atom 8 · Arm · XARM QP Cartesian Control* (`atom/docs/atom08_arm_qp_endpose_guide.md`).
