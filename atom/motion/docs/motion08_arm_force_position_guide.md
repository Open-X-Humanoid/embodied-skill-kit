# Motion 8 · Arm · Force–Position Hybrid Control

**English** | [简体中文](motion08_arm_force_position_guide_zh-CN.md)

**In one line**: publish a `CmdMotorCtrl` to `/arm/cmd_ctrl` to overlay velocity/torque feedforward on a position target and tune joint stiffness with `kp`/`kd`; return to the start with `/arm/cmd_pos` when done. Read motion03 (position mode) first.

| Companion code |
|---|
| `atom/motion/motion08_arm_force_position_ros2.py` (native ROS2), `atom/motion/motion08_arm_force_position_ros2_robust.py` (production) |

---

## 1. At a Glance (read this first)

### 1.1 Run it

Prerequisite: body_control is up — see *Prerequisite · Environment Setup* (`docs/environment_setup.md`).

```bash
ssh ubuntu@<robot-IP>
source /home/ubuntu/ros2ws/install/setup.bash   # run in every new terminal
python3 atom/motion/motion08_arm_force_position_ros2.py
```

Expected: joint 12 (left shoulder abduction) moves about 0.5 rad (≈28.6°) in one direction, holds for 5 s (you can gently push the joint to feel compliance), then returns to the start angle in position mode and exits.

⚠ **Read first**: the arm has high torque and a large range; clear people/objects around it and keep the e-stop in hand.

⚠ **Do not switch to the right arm by changing only the joint ID**: left/right J2 (shoulder abduction) joint angles are mirrored — left 12 limits ≈ `-0.26~2.61` (abduction positive), right 22 ≈ `-2.61~0.26` (abduction negative). The demo default `CTRL_DELTA=+0.5` is for left-arm abduction; setting only `DEMO_JOINT_ID=22` without negating the delta moves **inward** toward the torso. For the right arm set `CTRL_DELTA=-0.5` as well, and confirm the target stays within that joint’s limits.

### 1.2 Interface

| Item | Value |
|---|---|
| Control topic | `/arm/cmd_ctrl` → `bodyctrl_msgs/CmdMotorCtrl` |
| Return topic | `/arm/cmd_pos` → `bodyctrl_msgs/CmdSetMotorPosition` (cleanup only) |
| Status topic | `/arm/status` → `bodyctrl_msgs/MotorStatusMsg` |
| Motor IDs | left arm 11–17, right arm 21–27 (1 = shoulder … 7 = wrist); demo default 12 (left shoulder abduction) |

The simple version **does no validation** (target out of range, `kp`/`kd` out of legal range, publisher readiness — all published as-is). The production file `motion08_arm_force_position_ros2_robust.py` adds those checks — see §6.

---

## 2. Force–Position Hybrid Mode

**What it is**: on top of a position target, overlay velocity and torque feedforward. Output torque = `kp` × (target − current angle) + `kd` × (feedforward speed − current speed) + feedforward torque.

`kp` and `kd` set joint “stiffness”: small → compliant (can be pushed aside); large → stiff (resists external force).

**Message layout**:

```python
# bodyctrl_msgs/CmdMotorCtrl
std_msgs/Header header
MotorCtrl[] cmds

# bodyctrl_msgs/MotorCtrl
uint16  name   # motor ID
float32 kp     # position gain (range 0~2000)
float32 kd     # velocity gain (range 0~300)
float32 pos    # target position (rad)
float32 spd    # feedforward speed (rad/s); use 0.0 for pure position tracking
float32 tor    # feedforward torque (Nm); use 0.0 for pure position tracking
```

**Demo parameters**: `kp=15`, `kd=1.5` (compliant), `pos = start + 0.5 rad`, hold 5 s then return (gently push during the hold to feel compliance).

**Typical uses**: impedance control, contact-force shaping, passive arm following.

⚠ **“Compliant” is relative, not “zero resistance”**: if it still feels stiff when you push by hand, distinguish two cases —

- **Slow push** feels hard: mainly `kp` (torque ∝ position error) — try `CTRL_KP` at 10 or lower and compare.
- **Fast push / spring-back** feels hard: mainly `kd` (velocity damping) — try lowering `CTRL_KD` (e.g. 1–2).
- Tianyi arm joints are high-torque direct / quasi-direct drives (shoulder torque constants ~2.4–3.4 Nm/A, rated torque ~10–35 Nm), not small servos; the same `kp`/`kd` produces larger absolute resisting torque. For a clearer “soft” feel, try a wrist joint with smaller torque constant and rating (`DEMO_JOINT_ID = 16` or `17`).

⚠ **Settling accuracy and compliance trade off**: softer `kp`/`kd` and larger target deltas are more easily “eaten” by gravity/friction — actual arrival may undershoot `CTRL_DELTA`. Logs print both actual Δ and target Δ for comparison. For “soft and accurate,” you usually need fuller impedance control (e.g. gravity-compensation feedforward `tor`), not only retuning `kp`/`kd`.

---

## 3. Code Walkthrough (core)

`motion08_arm_force_position_ros2.py` follows the same lean skeleton as motion03 and peers, plus a small helper that keeps spinning during the hold.

### 3.1 Module map

| # | Module | Code anchor | Role |
|---|---|---|---|
| 1 | Config constants | topic names / `CTRL_KP` / `CTRL_KD` / `CTRL_DELTA` / `CTRL_HOLD` | tune here |
| 2 | Node & pub/sub | `ArmForcePositionDemo.__init__` | `pub_ctrl` + `pub_pos` + status sub |
| 3 | Status callback | `_on_status` | status frame → `cur_pos` (current angle) |
| 4 | Wait for angle | `wait_for_status` | same as motion03 — spin until joint angle arrives |
| 5 | Hold-period spin | `_spin_hold` | keep `spin_once` so `_on_status` stays fresh |
| 6 | Return home | `_return_to` | send joint back in position mode |
| 7 | Core logic | `move_ctrl` | send hybrid cmd → hold → log displacement → return |
| 8 | Main | `main` | read start → run → shut down |

### 3.2 Design note

**Why `_spin_hold` instead of `time.sleep`?**
`time.sleep` blocks the thread, so `_on_status` never runs and `cur_pos` stays frozen at the start of the hold — logged displacement would be wrong. Polling with `rclpy.spin_once` keeps `/arm/status` flowing; the angle at the end of the hold is the true latest value.

---

## 4. Tweak & Observe

| Change | What happens |
|---|---|
| `CTRL_KP = 30` | stiffer; joint resists being pushed aside |
| `CTRL_KP = 10`, `CTRL_KD = 1` | more compliant; easier to move by hand, but settling accuracy worse |
| `CTRL_DELTA = 1.0` | larger target offset — confirm target stays in the joint’s mechanical range |
| `DEMO_JOINT_ID = 22` **and** `CTRL_DELTA = -0.5` | right-arm J2 abduction; **must** negate the delta (left/right J2 mirrored) — ID alone moves inward |

⚠ **Legal range ≠ always safe**: `kp` may go up to 2000 and `kd` to 300, but large in-range values still make the joint much stiffer and tracking more aggressive — motion can be fast and still risk pinching/collision. Change parameters in small steps (e.g. raise `CTRL_KP` from 15 to 20–30, watch feel and settling speed), then increase further only if OK. When raising `CTRL_DELTA`, confirm the target stays in limits and keep the e-stop ready.

Predict the outcome, then run and check.

---

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Joint does not move at all | body_control not up, or no subscriber on the topic: `ros2 topic list \| grep arm` to confirm topics; `ros2 topic info /arm/cmd_ctrl --verbose` to confirm Subscription count > 0 |
| Joint stops mid-way (never returns) | `_return_to` uses position mode — confirm body_control accepts `/arm/cmd_pos` |
| Feels too “stiff” | see compliance tips at the end of §2: lower `kp` (slow push) and `kd` (fast push) separately; high-torque joints have larger absolute resisting torque than small servos |
| Actual Δ clearly less than target Δ | natural compliance vs accuracy trade-off (end of §2), not a bug; raise `CTRL_KP` slightly if you need tighter settling |
| `import bodyctrl_msgs` fails | workspace not sourced: `source /home/ubuntu/ros2ws/install/setup.bash` (every new terminal) |
| **Production** refuses after “soft limit” / “delta > max” | target or enlarged `CTRL_DELTA` outside that joint’s safe range — by design; lower `CTRL_DELTA` or check limits for `DEMO_JOINT_ID` |
| **Production** refuses after “kp/kd out of legal range” | `CTRL_KP`/`CTRL_KD` outside `0~2000` / `0~300` — set values inside the range |
| **Production** ends the hold early and logs `error` | fault code during hold (overcurrent / stall / …); already returned home — see `MOTOR_ERROR_DESC` |

---

## 6. Going Further

**Production hardening** (`motion08_arm_force_position_ros2_robust.py`): same pattern as `motion03_arm_ros2_robust.py` / `motion01_head_ros2_robust.py`, plus two hybrid-mode-specific items:

| Hardening | Simple version | Production version |
|---|---|---|
| Publisher ready | `publish` immediately — first packet can silently drop if DDS unmatched | `wait_publisher_ready` until ≥1 subscriber |
| Status timeout | missing `/arm/status` → assume angle = 0.0 (jump risk) | timeout → `None`, caller refuses to move |
| Target pose | no check — may hit hard stops or overcurrent | soft limits (`JOINT_SOFT_LIMITS`) + step size (`MAX_DELTA_RAD`); reject if out of range |
| `kp` / `kd` | no check — typos go out as-is | reject if outside legal range (`kp:0~2000`, `kd:0~300`) |
| Hold faults | `_spin_hold` only refreshes `cur_pos`, ignores `error` | `_spin_hold_watch` watches `error`; non-zero (overcurrent / stall / …) ends hold early and returns home |
| Interrupt fallback | none — Ctrl-C / exception leaves joint under force control with no owner | `move_ctrl` wraps hold in `try/finally` so return-home runs on normal end, early abort, or Ctrl-C |

⚠ Force–position hybrid **stays active** after a command — the joint keeps being driven toward the target until a new command arrives. Same continuous nature as position mode, but compliant mode is easier to push unintentionally, so production always tries to return home. Even so, **keep the e-stop in hand** — code fallbacks do not replace a physical e-stop.
