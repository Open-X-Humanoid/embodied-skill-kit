# scripts — Repository Tooling

**English** | [简体中文](README.zh-CN.md)

Cross-stage helper scripts (shell, etc. — not tutorial code), kept separate from the teaching examples in `atom/demos/`: demos are "to learn from," scripts are "to use."

When adding a script, register a row below with its purpose and which machine / user it runs on.

| Script | Purpose | Where to run |
|---|---|---|
| `start_body_control.sh` | One-click start of body_control (wraps the manual steps in *Prerequisite · Environment Setup* §2) | robot x86, user ubuntu |
| `start_camera.sh` | One-click start of the Orbbec camera driver (prerequisite for perception atoms, e.g. atom25) | robot Orin, user nvidia |
| `start_voice.sh` | One-click start of lyre voice (chat mode; prerequisite for voice atoms atom26~29) | robot Orin, user nvidia |
| `start_xarm.sh` | One-click start of the XARM framework + MoveIt component (prerequisite for the arm-MoveIt atom, atom05) | robot x86, user ubuntu |
| `stop_all.sh` | One-click cleanup: stop the sessions/processes started by these scripts (reset when things get messy) | the matching board (body/xarm on x86, camera/voice on Orin) |

## start_body_control.sh — one-click startup (quick path)

A one-command wrapper around the manual steps in *Prerequisite · Environment Setup* §2 (`atom/docs/environment_setup.md`): tmux → sudo → source → `ros2 launch`. Use this to start fast; follow the manual steps when you want to understand each one — the two are equivalent and don't conflict.

```bash
chmod +x scripts/start_body_control.sh   # make executable, first time only
./scripts/start_body_control.sh          # on the robot's x86 board, as user ubuntu
```

- Creates the `body` tmux session, elevates to root, launches body_control, then drops you into the session to watch the logs.
- Success = `All devices ready.`; keep it running and detach with `Ctrl+B` then `D`.
- If body is already running it just attaches (no double-start); if tmux is missing it tells you how to install it.

⚠ Manual start and the remote's A-button auto-start are mutually exclusive — don't use both.

## start_camera.sh — one-click camera startup

Starts the Orbbec camera driver (Gemini 330 series). Unlike body_control, the camera runs on the **Orin** (not x86) and needs **no root and no body_control**.

```bash
chmod +x scripts/start_camera.sh    # make executable, first time only
./scripts/start_camera.sh           # on the robot's Orin, as user nvidia
```

- Creates the `cam` tmux session, sources the Orbbec workspace, runs `ros2 launch orbbec_camera gemini_330_series.launch.py`, then drops you into the session.
- Success = camera topics start publishing; verify in another terminal with `ros2 topic list | grep camera`.
- Detach with `Ctrl+B` then `D`; if it's already running it just attaches; if it can't find the driver workspace, set `ORBBEC_WS` at the top of the script.
- The atom25 camera demo can then run on the Orin locally, or on the x86 (same ROS graph, matching `ROS_DOMAIN_ID`).

## start_voice.sh — one-click voice startup (chat mode)

Starts the lyre voice service (chat mode; TTS/playback, etc.). Prerequisite in *Prerequisite · Environment Setup* (`atom/docs/environment_setup.md`); usage details in the output-side voice guide (`atom/docs/atom26-27_voice_output_guide.md`). Runs on the **Orin** (user `nvidia`).

```bash
chmod +x scripts/start_voice.sh   # make executable, first time only
./scripts/start_voice.sh          # on the robot's Orin, as user nvidia
```

- Creates the `voice` tmux session, sources `~/ros2ws`, runs `ros2 launch lyre chat.launch.py`.
- Success = voice services/topics appear: `ros2 service list | grep audio_play`, `ros2 topic list | grep audio`.
- **Usually already running by default** — the script first checks whether lyre is up to avoid a double start; if so it just exits with a note.
- If the workspace path differs, edit `ROS2WS` at the top of the script.

## start_xarm.sh — one-click XARM + MoveIt startup

Starts the XARM framework body + MoveIt component (a two-pane tmux session); prerequisite for the arm-MoveIt atom (atom05). The real mode is verified on the TianYi 2.0 robot.

```bash
chmod +x scripts/start_xarm.sh
bash scripts/start_xarm.sh real    # real mode (verified; prerequisite: start body_control on the x86 first)
bash scripts/start_xarm.sh sim     # sim mode (with RViz; no real robot / body_control) — not tested in sim in this project, for reference when no robot is available
source scripts/start_xarm.sh       # only if 'import moveit_msgs' fails — adds XARM to the current terminal (normally unnecessary)
```

- `bash` mode creates the `xarm` tmux session: pane 0 starts the XARM body, pane 1 (after a delay) starts the MoveIt component, then drops you in.
- ★ Running the demo needs **only base ROS 2** (`/opt/ros/humble`, auto-sourced by `~/.bashrc`) — the demo imports only standard message packages (`moveit_msgs` etc. live in base ROS), so **no need to source XARM**. XARM's install (`/home/ubuntu/XARM/install`, with the `tianyi2_bringup` launch) is only for **starting** the XARM body + MoveIt (the `bash` mode sources it inside the panes); source XARM manually only if `import moveit_msgs` fails on your machine.
- Verify: `ros2 control list_controllers` (should include `moveit_*_arm_controller`), `ros2 action list | grep move_action`.
- If the XARM path differs, edit `XARM_WS` at the top of the script.

## stop_all.sh — one-click cleanup (reset when messed up)

Stops all sessions/processes started by these scripts (`xarm`/`body`/`cam`/`voice` tmux + their launches + body_control), for when repeated/out-of-order starts leave the system stuck and body_control won't come up.

```bash
bash scripts/stop_all.sh    # on the matching board; killing body_control (root) uses sudo (may prompt)
```

- Then restart in the right order: **body_control first, then XARM/MoveIt** (see the full flow or the atom05 guide).
- ⚠ Commands are written for the common case; process names/paths depend on your robot.
