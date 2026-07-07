# Prerequisite · Environment Setup (concise)

**English** | [简体中文](environment_setup_zh-CN.md)

> Common prerequisite for every atom example: network connectivity → **start the node(s) the atom needs** → prepare a dev terminal. Most atoms start body_control; perception atoms (e.g. the camera) start their own driver — see "Other nodes" at the end of §2.
> The full version (two startup modes, the proc_manager relationship, the 8080 dashboard) is in the internal doc *手动启动 BODY 简明指南.md*.

## 1. Network Connectivity

**Recommended: put the robot on the same Wi-Fi as your dev machine** for wireless SSH — no cable needed.

| Connection | Address |
|---|---|
| Same Wi-Fi (recommended) | robot's dynamic IP (changes with the network; check it live with `ifconfig`) |
| Direct cable to debug port | x86: `192.168.41.1` · Orin: `192.168.41.2` |

Check the robot's current IP (once you can log in, run it on the robot):

```bash
ifconfig    # or ip addr
```

If Windows can't connect, troubleshoot in order:

1. Does `ping <robot-IP>` succeed;
2. If not, first confirm both devices are on the **same subnet** — for a direct cable, set the Windows NIC IPv4 manually to `192.168.41.x` (x ≠ 1/2); for Wi-Fi, confirm it's the same network;
3. Still failing — check the Windows firewall / corporate network device-isolation policy.

Log in:

```bash
ssh ubuntu@<robot-IP>
```

## 2. Start body_control (needed by most atoms)

**Quick path**: just run the one-click script `./scripts/start_body_control.sh` (it wraps all the manual steps in this section). Follow the manual steps below when you want to understand each one — they're equivalent.

⚠ Before starting: no one near the robot, e-stop in hand, **the remote turned off**. Manual start and the remote's A-button auto-start are **mutually exclusive** (using both double-starts body and causes control conflicts).

```bash
tmux new -s body        # use a tmux session so body survives an SSH disconnect
sudo su                 # starting body needs root
cd /home/ubuntu/ros2ws
source install/setup.bash
ros2 launch body_control body.launch.py
```

You've succeeded when you see logs like this (the command keeps occupying the terminal — that's normal):

```
All devices ready.
Loaded node '/bodyctrl_component' in container '/body_container'
```

**Leave tmux but keep body running**: `Ctrl + B` then `D`.

| Action | Command |
|---|---|
| Re-enter the session | `tmux attach -t body` |
| List sessions | `tmux ls` |
| Stop body | attach, then `Ctrl + C` |

**Other nodes**: not every atom uses body_control. Perception atoms start their own driver, possibly on a **different board** — e.g. the **camera (atom25)** runs `ros2 launch orbbec_camera gemini_330_series.launch.py` on the **Orin**, and does **not** need body_control. Which nodes an atom needs, and on which board, is specified in that atom's guide under "Run it."

## 3. Prepare a Dev Terminal

Open another SSH terminal (**use the ubuntu user; don't run demos as root**):

```bash
ssh ubuntu@<robot-IP>
source /home/ubuntu/ros2ws/install/setup.bash
```

Verify the environment is ready:

```bash
ros2 topic list | grep -E "/head|/arm|/waist|/leg"   # you should see each part's topics
python3 -c "import bodyctrl_msgs"                     # no error = the message package is available
```

## 4. FAQ

- **Empty topic list**: body isn't up, or the current terminal didn't source the workspace.
- **`import bodyctrl_msgs` fails**: didn't source `/home/ubuntu/ros2ws/install/setup.bash`.
- **Want the whole-robot status**: open `http://<robot-IP>:8080/` in a browser for the diagnostics dashboard (starting the dashboard is covered in the full guide).
- **User roles**: root is only for starting body; day-to-day dev, running demos, and checking topics all use the ubuntu user.
