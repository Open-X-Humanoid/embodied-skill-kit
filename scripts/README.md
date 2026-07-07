# scripts — Repository Tooling

**English** | [简体中文](README.zh-CN.md)

Cross-stage helper scripts (shell, etc. — not tutorial code), kept separate from the teaching examples in `atom/demos/`: demos are "to learn from," scripts are "to use."

When adding a script, register a row below with its purpose and which machine / user it runs on.

| Script | Purpose | Where to run |
|---|---|---|
| `start_body_control.sh` | One-click start of body_control (wraps the manual steps in *Prerequisite · Environment Setup* §2) | robot x86, user ubuntu |

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
