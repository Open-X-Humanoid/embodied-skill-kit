# Atom 23 · Lower Body (Leg, waist+leg coordinated) · Coordinated Lift with Force-Hold

**English** | [简体中文](atom23_leg_guide_zh-CN.md)

**In one line**: make the robot squat / rise — the lift is a **coordinated hip + knee + waist-pitch** motion (not a single leg joint), approached in **small interpolated steps** from the current pose, and stopped by a **force-hold** so it never goes limp.

> ⚠ Load-bearing atom: the lift legs hold up the whole upper body. TianYi is a **wheeled-base** robot — the base keeps it laterally stable; what you actually guard against is the **upper body dropping if force is lost** (via the force-hold). Read this whole page before you touch it.

| Companion code | Demo video |
|---|---|
| `atom/demos/atom23_leg_ros2.py` (native ROS2), `atom/demos/atom23_leg_ros2_robust.py` (production) | in `atom/assets/videos/`, same name as the code — **especially worth watching** to see a correct run first |

## 1. At a Glance (read this first)

### 1.1 Run it

Prerequisite: body_control is up — see *Prerequisite · Environment Setup* (`atom/docs/environment_setup.md`). **Physical prep**: the base is parked stably, no people/objects in the lift's path, physical e-stop in hand. (The wheeled base already provides lateral stability, so a sling is usually unnecessary; only add extra protection when trying a **large** lift or a big arm reach.)

```bash
ssh ubuntu@<robot-IP>
source /home/ubuntu/ros2ws/install/setup.bash   # run in every new terminal
python3 atom/demos/atom23_leg_ros2.py
# type GO to confirm + 5s countdown → squat a few cm → pause 1s → rise back → force-hold
```

⚠ Pressing `Ctrl-C` mid-run automatically **force-holds the current pose** (it won't go limp and let the upper body drop).

### 1.2 Interface

| Item | Value |
|---|---|
| Leg command/status | `/leg/cmd_pos` / `/leg/status` → `bodyctrl_msgs/CmdSetMotorPosition` · `MotorStatusMsg` |
| Waist command/status | `/waist/cmd_pos` / `/waist/status` (the lift needs the waist too) |
| Motor IDs | leg `51`=hip `52`=knee; waist `32`=pitch (part of the lift) `31`=yaw (held still) |
| Lift direction | squat (lower): hip↓ knee↓ waist↑; rise (higher): hip↑ knee↑ waist↓ |
| `cur` | motor **max-current limit** (A) = **torque cap** (torque ∝ current; a ceiling, not the actual draw); load-bearing joints use the motor's rated **20A** — enough torque to hold the upper body without exceeding the motor |

> **Why there's no "leg-only" atom**: moving the leg alone tilts the upper body and shifts the center of mass. The lift keeps the upper body upright and stable through waist-leg coordination as height changes — so we give the coordinated lift directly, not a "leg only" example.

### 1.3 Joint Limits & Safety

⚠ URDF is authoritative. But note: **this demo does not send absolute target angles — it makes a tiny relative move from the current pose**, so the real safety constraint is "small step + coordinated + force-hold," not whether it hits an absolute limit.

| Joint | ID | URDF hard limit | How this demo moves it |
|---|---|---|---|
| hip | 51 | (−0.419, 0.908) rad (−24°~52°) | ±0.08 rad from the current value |
| knee | 52 | (−1.745, 0.506) rad (−100°~29°) | ±0.08 rad from the current value |
| waist pitch | 32 | (−0.785, 2.094) rad (−45°~120°) | ±0.08 rad, opposite to the leg |
| waist yaw | 31 | (−2.967, 3.142) rad | **held still** |

**Three safety essentials (none optional; all built into the code)**:

1. **Read status before moving**: if the four current angles (leg + waist) aren't read, it **does not move**.
2. **Small, slow interpolation**: it only squats ~`0.08 rad` (a few cm) from the current pose and rises back, one step every 50ms over ~4s — **never jumping to some calibrated pose**.
3. ⚠ **Stop = force-hold (`spd=0, cur=20A`)**: on e-stop / finish / exception it holds the current pose. **★ Never set a load-bearing joint's `cur` to 0** — that means the legs lose force and the upper body drops straight down.

⚠ **Voltage / motor faults**: the leg bears load; under insufficient power the motor reports an error (e.g. under-voltage `12832`). The robust version self-checks motor error codes and, on any, immediately force-holds and exits (see §6).

## 2. The Three Core Operations

### 2.1 Make it move — coordinated 3-joint + small-step interpolation

The lift sends leg (hip/knee) **and** waist (pitch) together, and **approaches from the current pose in steps**, never jumping:

```python
# one set of targets, split across the leg and waist topics (cur=20A holds the upper body up)
leg.cmds  = [SetMotorPosition(name=51, pos=hip,  spd=spd, cur=20.0),
             SetMotorPosition(name=52, pos=knee, spd=spd, cur=20.0)]
wst.cmds  = [SetMotorPosition(name=32, pos=waist, spd=spd, cur=20.0),
             SetMotorPosition(name=31, pos=yaw,   spd=0.3, cur=20.0)]   # yaw held

# move_to: interpolate from current (hip0,knee0,waist0) to target in n steps, 50ms each
for k in range(1, n + 1):
    r = k / n
    _publish(hip0 + (hip_t-hip0)*r, knee0 + (knee_t-knee0)*r, waist0 + (waist_t-waist0)*r, yaw, spd)
    time.sleep(0.05)
```

### 2.2 Read state — read all four angles before moving

Leg and waist share **one callback** into a single `self.pos` (motor IDs 51/52/31/32 are distinct, no collision):

```python
def _on_status(self, msg):
    for s in msg.status:
        self.pos[s.name] = s.pos      # leg and waist both go into this one dict

# wait_status: spins until hip/knee/waist_pitch/waist_yaw are all read, else refuses to move
```

### 2.3 ★ Force-hold — how a load-bearing joint "stops"

An ordinary joint can "stop" by simply not sending commands; but **a load-bearing joint drops the upper body the moment it goes slack**. The correct "stop" is to **re-send the last pose with `spd=0, cur=20A`** — position locked, force still applied:

```python
def hold(self):
    if self._last is not None:
        self._publish(*self._last, spd=0.0)   # ★ spd=0 but cur stays 20A — never 0
```

`Ctrl-C`, finish, exception — **every "stop" calls `hold()`**, never letting the legs go limp.

## 3. Code Walkthrough (core)

`atom23_leg_ros2.py` (the slimmed plain version) is **7 modules**. It's more complex than other atoms because it **controls 4 motors across two topics + interpolation + force-hold**.

### 3.1 Module map

| # | Module | Code anchor | Role |
|---|---|---|---|
| 1 | Config constants | `LEG_CMD` / `WAIST_CMD` / `HIP..YAW` / `SQUAT_DELTA` | topics, motor IDs, amplitude/speed/current |
| 2 | Node & I/O | `LegDemo.__init__` | 2 publishers + 2 subscriptions (**sharing one callback**) |
| 3 | Read state | `_on_status` / `wait_status` | leg+waist state into one `pos`; move only after all 4 are read |
| 4 | Send one pose | `_publish` | split one set of targets across the leg + waist topics, record `_last` |
| 5 | ★ Force-hold | `hold` | re-send the last pose with `spd=0, cur=20A` |
| 6 | Coordinated interpolation | `move_to` | interpolate from the current pose toward the target in steps |
| 7 | Main flow | `confirm` / `main` / `_sig` | GO confirm → read state → squat → rise → hold; Ctrl-C also `hold` |

### 3.2 Module by module

- **Module 2 — `__init__`**: builds the leg/waist publishers; both subscriptions call `_on_status` — since motor IDs are globally unique, they share one `self.pos` without collision (the plain version's biggest readability simplification). `_last` records the last pose sent, for `hold`.
- **Module 4 — `_publish`**: splits (hip,knee,waist,yaw) into a leg message (51/52) + a waist message (32/31) and sends both; updates `_last` each time.
- **Module 5 — `hold`**: the linchpin of a load-bearing atom — see 2.3.
- **Module 6 — `move_to`**: reads the current (hip0,knee0,waist0), then interpolates in `n` steps, `_publish` + `sleep` each. **Open-loop interpolation from the current value, no jumping.**

### 3.3 Reuse it (be careful)

```python
SQUAT_DELTA = 0.05      # only make it SMALLER, never larger (load-bearing atom)
# to move to a "calibrated pose" (absolute target), don't use the plain version — use robust with more guards, approaching gradually
```

> Note: the leg is a **load-bearing coupled joint**, totally unlike the "fire and forget" of head/arm — every safety step (read state, small step, force-hold) prevents the upper body from dropping; don't strip them for simplicity.

## 4. Tweak & Observe (be careful)

| Change | Effect | Advice |
|---|---|---|
| `SQUAT_DELTA` | squat amplitude | only go **smaller**, never larger |
| `MOVE_TIME` | one-way duration | larger = slower & steadier |
| ~~drive `yaw`~~ | twist | ⚠ this atom holds yaw — don't move it during the lift |

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Can't read status, script won't move | **that's correct** (safety): confirm body_control is up and both `/leg/status` and `/waist/status` have data |
| ★ Motor error (e.g. under-voltage `12832`) | load-bearing power insufficient or a motor fault. The robust version immediately **force-holds and exits**; check battery/power, clear it, then retry |
| Want to stop mid-run | `Ctrl-C` — the script force-holds the current pose, won't go limp |
| `import bodyctrl_msgs` fails | not sourced: `source /home/ubuntu/ros2ws/install/setup.bash` |

## 6. Going Further

- **Production hardening** (see `atom/demos/atom23_leg_ros2_robust.py`): **per-step motor-error self-check** (force-hold and exit on any), a per-joint **displacement hard cap** `SAFETY_CAP`, a lock protecting shared state, **flag-based signal handling** (no `shutdown` inside the handler, to avoid races), and `destroy_node` cleanup. Use this version for real work with a load-bearing atom.
- **Named poses / Cartesian lift**: the next step is "rise to a calibrated height" or "lift by Cartesian IK" — material for the later skill/scene stages.
