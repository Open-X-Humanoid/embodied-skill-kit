# Motion 11 · Chassis · Timed Motion (Slamtec REST)

**English** | [简体中文](motion11_chassis_guide_zh-CN.md)

**In one line**: use the Slamtec chassis REST action API (`MoveByAction`, **self-stopping after a duration**) to drive the base forward/back/turn, and read `/localization/pose` for the pose (state).

| Companion code | Demo video |
|---|---|
| `atom/motion/motion11_chassis_slamware.py` (runs on the real robot), `atom/motion/motion11_chassis_slamware_robust.py` (production) | in `atom/motion/assets/videos/`, same name as the code |

## 1. At a Glance (read this first)

### 1.1 Run it

Prerequisite: run on a robot board that can reach the chassis `192.168.11.x` subnet (x86/Orin), with the chassis powered, e-stop released, and >1m of clearance around it. This atom uses REST (plain `requests`) — it needs **no body_control and no `source ros2ws`**.

```bash
ssh ubuntu@<robot-IP>          # ssh to a board (direct cable: x86 192.168.41.1 / Orin 192.168.41.2)
python3 atom/motion/motion11_chassis_slamware.py
# type GO to confirm → prints pose before → forward/back/turn (each self-stops) → prints pose after
```

⚠ Pressing `Ctrl-C` mid-run automatically issues a **soft stop** (cancels the current action); keep the physical e-stop in hand.

### 1.2 Interface

| Item | Value |
|---|---|
| Chassis API base | `http://192.168.11.1:1448` (vendor default; change `BASE_URL` if your deployment differs) |
| Send action | `POST /api/core/motion/v1/actions` → `MoveByAction`, `options={direction, duration}` |
| direction | `0` forward `1` backward `2` turn-right `3` turn-left |
| duration | milliseconds (ms), **the chassis stops itself when it elapses** |
| Read pose (state) | `GET /api/core/slam/v1/localization/pose` → `x` / `y` / `yaw` |
| Soft stop | `DELETE /api/core/motion/v1/actions/:current` |
| Dependency | Python `requests` (**no ROS2 needed**) |

### 1.3 Safety Constraints

The chassis has no joint limits; its safety comes from **timed self-stop + subnet isolation + physical clearance**:

| Constraint | Note |
|---|---|
| Timed self-stop | each MoveBy carries a `duration`; the chassis stops itself when it elapses — no runaway even if the script crashes |
| Must be on the chassis subnet | only run on a board that can reach `192.168.11.x`, **not on your laptop** |
| Clearance | prop it up or leave >1m around it before running; e-stop in hand |
| Soft stop | `Ctrl-C` / `DELETE :current` cancels the current action instantly |

⚠ **Why REST instead of `/cmd_vel`**: REST self-stops on a timer, goes through Slamtec motion control, and supports a soft stop; raw `/cmd_vel` is open-loop velocity that must be published continuously and can run away when the script stops. On the real robot, REST is the first choice.

## 2. The Three Core Operations

### 2.1 Make it move — POST one timed action

```python
body = {"action_name": "slamtec.agent.actions.MoveByAction",
        "options": {"direction": DIRECTION[direction], "duration": int(duration_ms)}}
r = requests.post(ACTIONS, json=body,
                  headers={"accept": "application/json", "Content-Type": "application/json"},
                  timeout=TIMEOUT)
action_id = r.json().get("action_id")
```

- `direction`: `0/1/2/3` (fwd/back/right/left).
- `duration`: milliseconds; the chassis stops itself when it elapses — so every segment is short and safe.

### 2.2 Read state — GET the pose

The chassis's "state" is its pose `(x, y, yaw)`:

```python
r = requests.get(POSE, headers={"accept": "application/json"}, timeout=TIMEOUT)
d = r.json()
pose = (d["x"], d["y"], d["yaw"])
```

⚠ **Key point**: an action reporting `result:0` **does not mean the wheels actually turned**. Slamtec firmware returns success once it finishes planning/executing, but if the **free-wheel clutch is pressed / motors aren't enabled / the e-stop isn't released**, it "reports success but doesn't budge." **Reading the pose before and after and comparing `x/y/yaw`** is the hard evidence — the pose doesn't lie.

### 2.3 Soft stop — DELETE :current

```python
requests.delete(f"{ACTIONS}/:current", timeout=TIMEOUT)   # cancel the current action, chassis stops
```

The script wires this to signals: on `Ctrl-C` / kill it auto-calls `soft_stop`, avoiding a runaway.

## 3. Code Walkthrough (core)

`motion11_chassis_slamware.py` is **5 modules**. Note the chassis uses **REST (`requests`)**, a different interface family from the ROS2 topics of the other atoms.

### 3.1 Module map

| # | Module | Code anchor | Role | Change per deployment? |
|---|---|---|---|---|
| 1 | Config constants | `BASE_URL` / `ACTIONS` / `POSE` / `DIRECTION` / `PLAN` | API addresses, direction map, action plan | ✅ `BASE_URL` (chassis IP) |
| 2 | Send motion | `move_by` | POST `MoveByAction` → `sleep` until the timer elapses | ⭕ generic |
| 3 | Read state | `get_pose` | GET `/localization/pose` for x/y/yaw | ⭕ generic |
| 4 | Soft stop | `soft_stop` + `_signal_handler` | DELETE `:current`; auto stop on Ctrl-C | ⭕ generic |
| 5 | Main flow | `confirm` + `main` | GO prompt → read pose → `move_by` in sequence → read pose | ⭕ generic |

### 3.2 Module by module

- **Module 1 — config constants**: API base, the action/pose/stop endpoints, the `DIRECTION` map, the `PLAN`. Change `BASE_URL` for a different chassis/deployment.
- **Module 2 — `move_by`**: assemble the `MoveByAction` body → `POST` → get `action_id` → `sleep(duration+0.4s)` while the timed action runs.
- **Module 3 — `get_pose`**: `GET /localization/pose` returns `(x, y, yaw)` — the chassis's state read.
- **Module 4 — `soft_stop` + signals**: `DELETE :current` cancels the current action; `Ctrl-C`/kill triggers an automatic soft stop so a runaway script still stops.
- **Module 5 — `main`**: `confirm` (type GO + countdown) → read pose before → `move_by` in sequence → read pose after (compare to see whether it actually moved).

### 3.3 Reuse it

```python
BASE_URL = "http://<your-chassis-IP>:1448"   # change for a different deployment
PLAN = [("forward", 1500), ("left", 600)]     # change the plan: direction + duration(ms)
```

> Note: the chassis is a **REST interface family** (HTTP + `requests`), unlike the ROS2 topics of head/arm/hand — "reuse" here means changing the chassis IP and the action plan; motion joints use a different interface.

## 4. Tweak & Observe

| Change | Effect |
|---|---|
| a `duration` in `PLAN` | that segment runs longer/shorter |
| add `("right", 800)` | one more right turn |
| `BASE_URL` | change it when the chassis IP differs (deployment) |

Predict first, then run, and check against your prediction.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| ★ **Reports success but doesn't budge** (`result:0` yet motionless) | First check the **free-wheel clutch button** on the chassis — when pressed, the motors disengage (brake released) so a human can push the whole robot; MoveBy then can't drive the wheels, but the firmware still returns `result:0`, creating the illusion of "API all-success, robot motionless." **Release the button** to recover. Then check: physical e-stop pressed, still on the charging dock, battery level. Read the pose before/after to confirm whether it moved |
| Can't reach the API | usually you're **not on the chassis subnet** (running on a laptop): `ssh` to a board (x86 `192.168.41.1` / Orin `192.168.41.2`) and run there; confirm the chassis is powered, e-stop released, and self-test done |
| Wrong IP | set `BASE_URL` to your chassis's actual address |

## 6. Going Further

- **Production hardening** (see `atom/motion/motion11_chassis_slamware_robust.py`): HTTP retry, signal handling via a flag rather than side effects inside the handler, **before/after pose self-check that auto-warns on "reported success but didn't move,"** and soft stop.
- **Raw `/cmd_vel` variant**: only for understanding the lowest-level open-loop velocity control; it has a runaway risk and is **not the first choice** — see the corresponding variant.
