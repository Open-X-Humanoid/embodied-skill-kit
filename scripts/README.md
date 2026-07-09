# scripts — Repository Tooling

**English** | [简体中文](README.zh-CN.md)

Cross-stage helper scripts (shell, etc. — not tutorial code), kept separate from the teaching examples in `atom/demos/`: demos are "to learn from," scripts are "to use."

When adding a script, register a row below with its purpose and which machine / user it runs on.

| Script | Purpose | Where to run |
|---|---|---|
| `start_body_control.sh` | One-click start of body_control (wraps the manual steps in *Prerequisite · Environment Setup* §2) | robot x86, user ubuntu |
| `start_camera.sh` | One-click start of the Orbbec camera driver (prerequisite for perception atoms, e.g. atom25) | robot Orin, user nvidia |
| `start_voice.sh` | One-click start of lyre voice (chat mode; prerequisite for voice atoms atom26~29) | robot Orin, user nvidia |

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
