# Skill 1 · Finger Tap — Coordinate Perception, Arm, and Dexterous Hand

**English** | [简体中文](skill01_finger_tap_guide_zh-CN.md)

**In one sentence:** the Orin camera detects an AprilTag card and publishes its position and surface normal; the x86 controller shapes the hand, moves the arm to an 8 cm standoff, corrects the fingertip error, presses the card with the index finger, and returns to the ready pose.

> This is the first-stage Skill migration. It intentionally keeps the real-robot-tested execution flow and human confirmation gates. A standard Goal, `SkillResult`, explicit success/failure contract, unified timeout, recovery, and data-recording interface will be added later.

| Component | Path |
|---|---|
| Perception, run on Orin | `skill/skill01_finger_tap/tag_locator.py` |
| Motion control, run on x86 | `skill/skill01_finger_tap/finger_tap.py` |
| Shared tunable configuration | `skill/skill01_finger_tap/config.py` |
| Demo assets | `skill/skill01_finger_tap/assets/` |

The Skill uses transparent ROS 2 interfaces directly. Atom 05–08 explain the individual MoveIt/QP arm controllers; this guide focuses on coordinating perception, arm motion, and the dexterous hand. Read [Atom 6 · MoveIt End Pose](../../../atom/docs/atom06_arm_moveit_endpose_guide.md) and [Atom 8 · QP End Pose](../../../atom/docs/atom08_arm_qp_endpose_guide.md) first if those controllers are unfamiliar.

## 1. Quick start

### 1.1 Run it

The Skill spans two TianYi 2.0 computers. Orin runs perception, x86 runs arm control, and both must use the same `ROS_DOMAIN_ID`.

**Orin, user `nvidia`:**

```bash
bash scripts/start_camera.sh
python3 skill/skill01_finger_tap/tag_locator.py
```

> ⚠ **The camera topic namespace is detected automatically; normally you need do nothing** (details in section 4 of the Prerequisite · Environment Setup guide, `atom/docs/environment_setup.md`). It varies by robot, is detected from the ROS graph at startup, and is printed in the log; use `export CAMERA_NS=<namespace>` only for multi-camera robots or to force a specific one. However, **if the driver already starts automatically at boot, skip `start_camera.sh` above** or a second driver will fight for the USB device.

`tag_locator.py` is perception-only. It detects the card, estimates its center and normal, and publishes `/skill01/target_point`. Inspect the topic before enabling arm motion.

**x86, user `ubuntu`:**

```bash
bash scripts/start_body_control.sh
bash scripts/start_xarm.sh real

source /home/ubuntu/XARM/install/setup.bash
python3 skill/skill01_finger_tap/finger_tap.py
```

Expected sequence: print target coordinates → human confirmation → shape the hand → move to `READY_JOINTS` → approach the 8 cm standoff → closed-loop correction → human confirmation → press with the configured compliant overtravel → wait for human pose inspection → retract → return to READY and open the hand.

> **Safety checklist**
>
> 1. Fix the card to a stationary object. Never hold the target by hand.
> 2. Keep everyone outside the arm workspace.
> 3. `SAFE_BOX` is disabled by default. The printed-coordinate check and Enter prompt are the active human gate; verify the coordinates before continuing.
> 4. Keep the emergency stop in hand. The checked-in `PRESS_DEPTH` is 1.5 cm; reduce it to 5 mm for the first contact test and increase it only after compliant contact is confirmed.
> 5. If execution stops with the arm in space, use `python3 skill/skill01_finger_tap/finger_tap.py --recover` only after checking that the return path is clear.

An instant MoveIt `error_code=99999` commonly means the current joint state is outside MoveIt's tighter limits, or that the target/constraint is unreachable. Put the card in the left arm's comfortable front-left workspace and run `reach_check.py` before contact tests.

### 1.2 Interfaces

| Direction | Name | Type | Purpose |
|---|---|---|---|
| Cross-board topic | `/skill01/target_point` | `geometry_msgs/PoseStamped` | Card center in `position`; surface orientation in `orientation`; frame is `head_roll_link` |
| Hand command | `/inspire_hand/ctrl/left_hand` | `sensor_msgs/JointState` | Extend the index finger and curl the other fingers |
| TF | `base ← head_roll_link` | tf2 | Transform the perceived target into the arm planning frame |
| TF | `base ← left_tcp_link` | tf2 | Read the current wrist TCP pose |
| TF | `left_tcp_link ← left_index_2` | tf2 | Read the fixed wrist-to-index transform |
| MoveIt approach | `/move_action` | `moveit_msgs/action/MoveGroup` | Plan the main approach when `ARM_BACKEND="moveit"` |
| QP end pose | `/endpose_single_arm_qp_L_controller/endPosSingleTarget` | `eai_manipulator_msgs/action/EndPosSingleTarget` | Short correction, press, and retract motion |
| QP joint space | `/jointspace_arm_L_controller/jointspace` | `eai_manipulator_msgs/action/JointSpace` | Move to and return from READY |

The TianYi 2.0 arm frame is `base`, not `base_link`. Verify it with:

```bash
ros2 run tf2_ros tf2_echo base left_tcp_link
```

### 1.3 Motion model

```text
startup
  └─ joint space ─▶ READY
                       └─ end pose ─▶ 8 cm standoff ─▶ press
                                         ▲                │
                                         └──── retract ───┘
  ◀─ joint space ── return to READY and open the hand
```

The four logged stages are:

1. Observe: collect target samples and transform them into `base`.
2. Position: enable control, shape the hand, go to READY, approach, and correct.
3. Press: advance through the standoff plus `PRESS_DEPTH`, then wait for human inspection.
4. Recover posture: retract, return to READY, and open the hand.

`ARM_BACKEND` switches only the main approach. Correction, pressing, and retraction always use end-pose QP; READY motion always uses joint-space QP.

## 2. Core concepts

### 2.1 Two boards and one snapshot

Orin publishes perception and x86 consumes it through `/skill01/target_point`. `finger_tap.py` collects `N_SAMPLES` observations, takes coordinate-wise medians, then freezes the target. It deliberately does not chase an object that moves after execution starts.

### 2.2 TCP versus physical finger pad

The controllers command `left_tcp_link`, not the physical finger pad. The code therefore:

1. Reads the full TCP-to-`TAP_LINK` transform.
2. Applies `PAD_LOCAL_OFFSET=[0.01365, 0.04307, 0.00499]` in the link's local coordinate frame.
3. Rotates and combines both offsets to obtain the complete TCP-to-pad displacement, then solves the TCP goal from the desired physical pad position.

The index finger must remain extended because `PAD_LOCAL_OFFSET` was derived from the extended-finger STL mesh. `POINT_POSE` curls the other fingers to keep them away from the target surface.

> `PAD_LOCAL_OFFSET` is an offline mesh-derived value and has not yet been repeatedly verified on the real robot. Before first contact, use `PRESS_ENABLE=False` to inspect the standoff and confirm that the repository mesh matches the installed hand.

### 2.3 Three error layers

| Layer | Typical source | Fixed/random | Main correction |
|---|---|---|---|
| Perception | Hand–eye extrinsics | Mostly fixed | `extrinsics.json`, `AIM_BIAS_BASE` |
| Perception | Intrinsics or incorrect `TAG_SIZE` | Fixed scale error | `camera_intrinsics.json`, measured tag size |
| Perception | Tag detection jitter | Random | Median of `N_SAMPLES` |
| Geometry | `TAP_LINK` origin differs from the physical pad | Fixed | 3D `PAD_LOCAL_OFFSET`; overall target bias uses `AIM_BIAS_BASE` |
| Execution | Wrist orientation tolerance amplified by finger length | Random | `correct_tip` closed-loop QP correction |

Tune a fixed, repeatable bias; do not tune against direction-changing random scatter. In the base frame, +y is robot-left and +z is up.

## 3. Code map

| File | Board | Role | Moves the robot? |
|---|---|---|---|
| `tag_locator.py` | Orin | AprilTag center and normal estimation | No |
| `finger_tap.py` | x86 | Full four-stage execution | Yes |
| `config.py` | Both | All tunable constants | — |
| `pose_math.py` | Either | Quaternion, rotation, and RPY helpers | No |
| `reach_check.py` | x86 | MoveIt plan-only reachability test | No execution |

Important `finger_tap.py` anchors:

| Stage | Functions | Responsibility |
|---|---|---|
| Target input | `_on_target`, `wait_target` | Collect and freeze median target samples |
| Frame transform | `to_base` | Transform point and normal to `base` |
| Target construction | `build_approach` | Build the physical pad target and solve the TCP pose |
| Pad geometry | `pad_offset`, `read_pad_pos` | Combine link TF with local `PAD_LOCAL_OFFSET` |
| Control setup | `enable_arm`, `activate_arm_controller`, `set_vel_limits` | Enable, switch, and slow the selected controller |
| Main approach | `moveit_move_to_pose` or `move_segmented` | Reach the standoff |
| Error reporting | `report_tip_error` | Report finger-pad error in useful axes |
| Closed-loop correction | `correct_tip` | Re-measure and translate by the negative residual |
| Contact | `press_only` | Advance to contact and wait for human inspection |
| Retraction | `retract_to_standoff` | Return along the approach direction |
| Recovery | `save_start_pose`, `recover` | Persist the start pose and provide explicit recovery |

The `level` orientation mode projects the tag normal onto the horizontal plane so the finger points horizontally at the card. `LEVEL_WRIST=True` restores the nominal wrist orientation before pressing, while pad position is computed from the actual rigid transform.

## 4. Tune and observe

Edit only `config.py`, predict the effect, then compare it with the logged left/right, up/down, and depth errors.

| Setting | Effect |
|---|---|
| `AIM_BIAS_BASE[1]` | Consistent left/right fingertip shift |
| `AIM_BIAS_BASE[2]` | Consistent vertical fingertip shift |
| `PAD_LOCAL_OFFSET` | 3D pad position in the `TAP_LINK` local frame; recalibrate all three components together if the mesh or installed geometry differs |
| `APPROACH_OFFSET` | Standoff distance, default 8 cm |
| `PRESS_DEPTH` | Compliant press overtravel; checked-in value is 1.5 cm, but first contact tests should use 5 mm |
| `SPIN_TOL` | MoveIt tolerance around the pointing axis |
| `ORIENT_MODE` | `level` for repeatable horizontal pressing; `tag` to follow the full surface normal |
| `PRESS_ENABLE=False` | Approach and correct without contact |
| `HAND_POSE_ENABLE=False` | Leave the current hand shape unchanged |
| `ARM_BACKEND="qp"` | Use segmented QP for the main approach; avoid long sweeps because the QP collision model excludes the hand |

## 5. Troubleshooting

| Symptom | Cause and action |
|---|---|
| No target received | Check `tag_locator.py`, tag visibility, matching `ROS_DOMAIN_ID`, and `ros2 topic echo /skill01/target_point` |
| Instant MoveIt `99999` | Inspect `tmux capture-pane -t xarm.1 -p -J -S -400 \| grep -i 'outside bounds'`; move any named joint back inside MoveIt limits with QP, or move the card into a reachable area and run `reach_check.py` |
| Palm faces the wrong way | Check `HAND_SPIN`; a very loose `SPIN_TOL` may allow a twisted IK solution |
| Repeatable vertical/side bias | Adjust the corresponding `AIM_BIAS_BASE` component only after 2–3 runs show the same direction |
| Direction-changing scatter | Treat it as execution noise; use closed-loop correction rather than a fixed bias |
| Hand does not change shape | Confirm a subscriber exists on `/inspire_hand/ctrl/left_hand` |
| TF timeout | Confirm the frame is `base`, XARM/body are running, and `left_tcp_link` exists |
| Missing TCP-to-index TF | Verify `TAP_LINK` with `ros2 run tf2_ros tf2_echo left_tcp_link left_index_2` |
| Arm stopped mid-run | Clear the return path, then run `python3 skill/skill01_finger_tap/finger_tap.py --recover` |

## 6. Advanced use

- **Recovery:** before motion, the Skill stores the starting TCP pose in `_last_start_pose.json`. `--recover` asks for confirmation, retracts slowly with QP, and returns to `READY_JOINTS` when configured. If neither recovery target exists, it stops without moving; follow the on-site safety procedure.
- **Reachability check:** `python3 skill/skill01_finger_tap/reach_check.py` sends plan-only MoveIt requests. It does not execute arm motion.
- **Debug logs:** append `--ros-args --log-level debug` to show controller switches, segmented waypoints, and internal geometry.
- **Future Skill contract:** Goal, `SkillResult`, explicit success/failure, standard timeout/recovery, and data logging remain a separate follow-up so this migration does not alter the verified execution behavior.
