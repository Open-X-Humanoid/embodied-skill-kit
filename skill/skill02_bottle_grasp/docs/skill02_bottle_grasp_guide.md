# Skill 2 · Bottle Grasp

**English** | [简体中文](skill02_bottle_grasp_guide_zh-CN.md)

Bottle Grasp combines two perception nodes on the Orin with one motion node on the x86. It locates a transparent bottle from its segmentation boundary and supporting surface, estimates the supporting box footprint, approaches through a computed clearance waypoint, closes the dexterous hand, lifts, places, and returns to READY.

## 1. Files

| File | Board | Purpose | Moves the robot? |
|---|---|---|---|
| `bottle_locator.py` | Orin | YOLO segmentation and bottle-center estimation | No |
| `box_locator.py` | Orin | Box footprint and yaw estimation | No |
| `grasp_bottle.py` | x86 | Approach, grasp, lift, place, retract, and recover | Yes |
| `config.py` | Both | Shared topics, calibration, motion, and safety margins | — |
| `pose_math.py` | Both | Quaternion and transform helpers | No |
| `debug_box_overlay.py` | Orin | Saves a visual overlay of box candidate pixels | No |

## 2. Run

Both boards must use the same `ROS_DOMAIN_ID`.

**Start the x86 side first.** The two perception nodes publish coordinates in the `base` frame, which is computed from the TF provided by the robot body services.

On the x86, start the robot services:

```bash
sudo systemctl stop teleop_robot
bash scripts/start_body_control.sh
bash scripts/start_xarm.sh real
```

Then, on the Orin, run the camera and the two perception nodes in separate terminals:

```bash
bash scripts/start_camera.sh
python3 skill/skill02_bottle_grasp/bottle_locator.py
python3 skill/skill02_bottle_grasp/box_locator.py
```

> ⚠ **The camera topic namespace is detected automatically; normally you need do nothing** (details in section 4 of the Prerequisite · Environment Setup guide, `atom/docs/environment_setup.md`). It varies by robot, is detected from the ROS graph at startup, and is printed in the log; use `export CAMERA_NS=<namespace>` only for multi-camera robots or to force a specific one. However, **if the driver already starts automatically at boot, skip `start_camera.sh` above** or a second driver will fight for the USB device.

Inspect the perception output before enabling arm motion:

```bash
ros2 topic echo /skill02/target_point
ros2 topic echo /skill02/box_pose
ros2 topic echo /skill02/box_size
```

Back on the x86, run the motion node:

```bash
source /home/ubuntu/XARM/install/setup.bash
python3 skill/skill02_bottle_grasp/grasp_bottle.py
```

The program pauses before safety-critical transitions. Verify the printed target and waypoint coordinates before every confirmation.

## 3. Interfaces

| Interface | Type | Meaning |
|---|---|---|
| `/skill02/target_point` | `geometry_msgs/PoseStamped` | Bottle center on the supporting plane, frame `base` |
| `/skill02/diameter_m` | `std_msgs/Float32` | Estimated bottle diameter; logged but not used for control |
| `/skill02/box_pose` | `geometry_msgs/PoseStamped` | Box center and yaw, frame `base` |
| `/skill02/box_size` | `geometry_msgs/Vector3` | Box length, width, and height |
| `/inspire_hand/ctrl/left_hand` | `sensor_msgs/JointState` | Dexterous-hand command |
| `/move_action` | `moveit_msgs/action/MoveGroup` | Large arm motion to the clearance waypoint and READY |
| `/endpose_single_arm_qp_L_controller/endPosSingleTarget` | `eai_manipulator_msgs/action/EndPosSingleTarget` | Short, segmented Cartesian motion |

Node names use the Skill namespace convention:

- `skill02_bottle_locator`
- `skill02_box_locator`
- `skill02_grasp_bottle`
- `skill02_box_overlay_debug`

## 4. Motion model

The approach has three stages:

```text
MoveIt  -> clearance waypoint outside the box
QP      -> calibrated standoff pose
QP      -> final grasp pose along GRASP_DIR
```

The box is not registered as a MoveIt collision object. `build_intermediate_point` computes one clearance waypoint from the frozen box snapshot and configured margins. This is not real-time obstacle avoidance.

Bottle and box estimates are frozen before arm motion. This prevents the moving arm from contaminating the camera result, but it also means the bottle and box must not move after confirmation.

## 5. Calibration and limitations

`READY_JOINTS`, `GRASP_ORIENT`, `TCP_OFFSET`, `GRASP_DIR`, `STANDOFF_MARGIN`, and `HAND_GRASP_POSE` are real-robot calibration values for one bottle, hand assembly, and workspace. A qualified operator must recalibrate and validate them after any hardware, bottle, or workstation change.

Important limitations:

- The hand has no force-feedback grasp confirmation.
- The bottle diameter estimate is not used to adapt the grasp.
- Grasp alignment is open-loop; `check_position` verifies TCP execution, not bottle-in-hand alignment.
- Placement is manually jogged and does not verify a supporting surface.
- The clearance waypoint depends on one box estimate and does not cover other obstacles.
- Multiple bottles are not supported; the highest-confidence detection is selected.

Keep the emergency stop available, clear the arm workspace, and never hold the target bottle by hand during execution.

## 6. Diagnostics and recovery

To inspect the box segmentation before moving the arm:

```bash
python3 skill/skill02_bottle_grasp/debug_box_overlay.py
```

The image is saved under `skill/skill02_bottle_grasp/captures/box_overlay.jpg`.

If execution is interrupted after motion starts, clear the return path and run:

```bash
python3 skill/skill02_bottle_grasp/grasp_bottle.py --recover
```

The recovery path uses the saved `_last_start_pose.json`, then returns to `READY_JOINTS` when configured. If automatic recovery fails, stop and follow the on-site safety procedure.

| Symptom | Check |
|---|---|
| No bottle target | Camera, YOLO model, bottle visibility, and matching `ROS_DOMAIN_ID` |
| No box result | Start `bottle_locator.py` first; inspect `BOX_HEIGHT_TOL`, `BOX_SEARCH_RADIUS`, and the overlay |
| MoveIt `error_code=99999` | Current joint state may be outside MoveIt limits, or the target may be unreachable |
| Hand passes too close to the box | Stop, recover, verify the overlay, and review `INTERMEDIATE_Y_MARGIN` / `BOX_XY_MARGIN` |
| Bottle slips or deforms | Stop testing; the calibrated hand pose does not match the current bottle |
