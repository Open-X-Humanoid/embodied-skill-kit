# 前置 · 环境配置（简明版）

[English](environment_setup.md) | **简体中文**

> 所有原子示例的共同前置：网络连通 → 启动**该原子需要的节点** → 准备开发终端。多数原子起 body_control；感知/语音类（如相机、语音）起对应节点，见第 2 节末尾「其它节点」。


## 1. 网络连通

**推荐方式：让机器人连上和开发电脑相同的 Wi-Fi**，即可无线 SSH 访问，不用插网线。

| 连接方式 | 地址 |
|---|---|
| 同一 Wi-Fi（推荐） | 机器人动态 IP（会随网络变化，用 `ifconfig` 实查） |
| 网线直连调试口 | x86：`192.168.41.1` · Orin：`192.168.41.2` |

确认机器人当前 IP（已能登录时，在机器人上执行）：

```bash
ifconfig    # 或 ip addr
```

Windows 端连不通的排查顺序：

1. `ping <机器人IP>` 是否通；
2. 不通先确认两台设备在**同一网段**——网线直连时需把 Windows 网口 IPv4 手动改成 `192.168.41.x`（x ≠ 1/2）；Wi-Fi 方式下确认连的是同一个 Wi-Fi；
3. 仍不通检查 Windows 防火墙 / 公司网络的设备隔离策略。

登录：

```bash
ssh ubuntu@<机器人IP>
```

## 2. 启动 body_control（多数原子的前置，跑在 x86）

**快速版**：直接跑一键脚本 `./scripts/start_body_control.sh`（封装了本节全部手动步骤）。想理解每步在做什么就照下面手动来——两者等价。

⚠ 启动前确认：机器人周围无人、急停在手边、**遥控器保持关闭**。手动启动与遥控器 A 键自启动**二选一，不可混用**（会重复启动 body、控制冲突）。

```bash
tmux new -s body        # 用 tmux 会话，防止 SSH 断开导致 body 退出
sudo su                 # 启动 body 需要 root
cd /home/ubuntu/ros2ws
source install/setup.bash
ros2 launch body_control body.launch.py
```

看到如下日志即启动成功（该命令会持续占用终端，属正常现象）：

```
All devices ready.
Loaded node '/bodyctrl_component' in container '/body_container'
```

**退出 tmux 但保持 body 运行**：`Ctrl + B` 然后按 `D`。

| 操作 | 命令 |
|---|---|
| 重新进入会话 | `tmux attach -t body` |
| 查看会话列表 | `tmux ls` |
| 停止 body | attach 进去后 `Ctrl + C` |

**其它节点**：不是所有原子都用 body_control。感知/语音类原子要另起对应节点，且可能在**另一块板子**上，都**不需要 body_control**：

- **相机（perception01）**：在 **Orin** 上 `bash scripts/start_camera.sh`（一键；等价 `ros2 launch orbbec_camera gemini_330_series.launch.py`）。
- **语音（interaction01/27）**：在 **Orin** 上 `bash scripts/start_voice.sh`（lyre chat 模式，含朗读/播放；出厂通常已自启）。

每个原子该起哪些节点、在哪块板子、语音模式与前置细节，以该原子 guide 的「怎么跑」为准。

## 3. 开发终端准备

另开一个 SSH 终端（**ubuntu 用户即可，不要用 root 跑 demo**）：

```bash
ssh ubuntu@<机器人IP>
source /home/ubuntu/ros2ws/install/setup.bash
```

验证环境就绪：

```bash
ros2 topic list | grep -E "/head|/arm|/waist|/leg"   # 应能看到各部位话题
python3 -c "import bodyctrl_msgs"                     # 不报错 = 消息包可用
```

## 4. 相机话题名（感知类 demo / Skill 的前置）

相机驱动发布的话题命名空间**因机器出厂配置而异**：orbbec 驱动默认是 `camera`，部分机器出厂配成了 `ob_camera_head`。代码里不写死任何一个，**启动时扫一遍 ROS 图自动认出来**——正常情况下你不需要做任何配置。

启动日志会打印认到的结果，核对一眼即可：

```text
[INFO] [skill02_box_locator]: 相机命名空间 = ob_camera_head（自动探测）
```

想自己确认机器上有哪些相机话题：

```bash
ros2 topic list | grep -i color
```

**两种情况需要手动指定**——机器上有多颗相机（自动探测会选排序最靠前的那个并告警），或者想跳过探测：

```bash
export CAMERA_NS=ob_camera_head
```

设了这个变量就以它为准、不再探测；日志会显示「（环境变量 CAMERA_NS 指定，跳过探测）」。变量对 `perception01`、`skill01`、`skill02` 同时有效。

⚠ **探测依赖相机驱动已经在跑**。驱动没起时扫不到任何彩色话题，程序会告警并按默认 `camera` 继续，随后表现为「收不到图像」超时。

⚠ **先确认相机驱动是不是已经在跑**：部分机器出厂就自启了相机，此时再执行 `scripts/start_camera.sh` 会起第二个驱动抢 USB。上面那条 `ros2 topic list` 能看到彩色话题，就说明驱动已在运行，直接跳过启动脚本。

## 5. 常见问题

- **topic 列表为空**：body 没起来，或当前终端没 source 工作空间。
- **`import bodyctrl_msgs` 报错**：没 source `/home/ubuntu/ros2ws/install/setup.bash`。
- **想看整机状态**：浏览器打开 `http://<机器人IP>:8080/` 诊断看板
- **启动 body 后有关节报 `DisableMotor` 失败（手臂/头/腰/腿等）**：多半是**急停被按下**——身体电机驱动器无法使能。先松开急停、重启；若仍失败，依次查：遥控器互斥、整机断电清错、动力电源、EtherCAT/CAN 总线。
- **看到 `All devices ready.` 后 tmux 没有自动退出**：这是**正常成功**，不是卡住——body_control 是持续运行的服务，脚本故意 attach 进会话让你看日志。按 `Ctrl+B` 再按 `D` 脱离，让它后台继续跑，另开终端跑 demo。
