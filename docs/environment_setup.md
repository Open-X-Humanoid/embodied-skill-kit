# Prerequisite · Environment Setup (concise)

**English** | [简体中文](environment_setup_zh-CN.md)

> Common prerequisite for every atom example: network connectivity → **start the node(s) the atom needs** → prepare a dev terminal. Most atoms start body_control; perception/voice atoms (e.g. camera, voice) start their own node — see "Other nodes" at the end of §2.


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

## 2. Start body_control (needed by most atoms; runs on the x86)

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

**Other nodes**: not every atom uses body_control. Perception/voice atoms start their own node, possibly on a **different board**, and none need body_control:

- **Camera (perception01)**: on the **Orin**, `bash scripts/start_camera.sh` (one-click; equivalent to `ros2 launch orbbec_camera gemini_330_series.launch.py`).
- **Voice (interaction01/27)**: on the **Orin**, `bash scripts/start_voice.sh` (lyre chat mode — TTS/playback; usually auto-started at boot).

Which nodes an atom needs, on which board, and the voice mode/prerequisite details are specified in that atom's guide under "Run it."

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

## 4. Camera Topic Namespace (prerequisite for perception demos / Skills)

The camera driver's topic namespace **varies by factory configuration**: the orbbec driver defaults to `camera`, while some robots ship configured as `ob_camera_head`. The code hard-codes neither. It **scans the ROS graph at startup and detects the namespace automatically**, so normally you need to configure nothing.

The startup log prints what it found:

```text
[INFO] [skill02_box_locator]: 相机命名空间 = ob_camera_head（自动探测）
```

To list the camera topics yourself:

```bash
ros2 topic list | grep -i color
```

**Two cases need a manual override** — the robot has more than one camera (detection picks the first in sort order and warns), or you want to skip detection entirely:

```bash
export CAMERA_NS=ob_camera_head
```

When this variable is set it wins and no detection runs. The variable applies to `perception01`, `skill01`, and `skill02` alike.

⚠ **Detection requires the camera driver to be running already.** With no driver up, no color topic is found; the node warns, falls back to `camera`, and then times out waiting for images.

⚠ **Check whether the camera driver is already running first**: some robots start it automatically at boot, in which case running `scripts/start_camera.sh` launches a second driver that fights for the USB device. If the `ros2 topic list` command above shows color topics, the driver is already up — skip the start script.

## 5. FAQ

- **Empty topic list**: body isn't up, or the current terminal didn't source the workspace.
- **`import bodyctrl_msgs` fails**: didn't source `/home/ubuntu/ros2ws/install/setup.bash`.
- **Want the whole-robot status**: open `http://<robot-IP>:8080/` in a browser for the diagnostics dashboard.
- **Joints report `DisableMotor` failures after starting body (arm/head/waist/leg, etc.)**: most likely the **E-stop is pressed** — the body motor drivers can't enable. Release the E-stop and restart; if it still fails, check in order: remote-control mutual exclusion, full power-cycle to clear faults, motor power supply, EtherCAT/CAN bus.
- **tmux doesn't auto-exit after `All devices ready.`**: this is **normal success**, not a hang — body_control is a long-running service and the script deliberately attaches you into the session to show logs. Press `Ctrl+B` then `D` to detach and leave it running in the background, then run your demo from another terminal.
