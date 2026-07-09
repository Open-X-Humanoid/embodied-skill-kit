# scripts —— 仓库工具脚本

[English](README.md) | **简体中文**

放**跨阶段的辅助脚本**（shell 等非示例代码），与教学示例 `atom/demos/` 分开：demos 是「拿来学」的，scripts 是「拿来用」的。

新增脚本时在下表登记一行，写清用途、在哪台机器 / 哪个用户上跑。

| 脚本 | 用途 | 在哪跑 |
|---|---|---|
| `start_body_control.sh` | 一键启动 body_control（封装《前置 · 环境配置》第 2 节手动步骤） | 机器人 x86，ubuntu 用户 |
| `start_camera.sh` | 一键启动 Orbbec 相机驱动（感知类原子如 atom25 的前置） | 机器人 Orin，nvidia 用户 |
| `start_voice.sh` | 一键启动 lyre 语音（chat 模式，语音原子 atom26~29 的前置） | 机器人 Orin，nvidia 用户 |

## start_body_control.sh —— 一键启动（快速版）

《前置 · 环境配置》第 2 节手动步骤（tmux → sudo → source → `ros2 launch`）的一键封装，见 `atom/docs/environment_setup_zh-CN.md`。想快就用它；想搞清每一步在做什么，就照手动版来——两者等价、不冲突。

```bash
chmod +x scripts/start_body_control.sh   # 首次赋可执行权限（只需一次）
./scripts/start_body_control.sh          # 在机器人 x86 上、ubuntu 用户执行
```

- 自动建 `body` tmux 会话、提权到 root、启动 body_control，并带你进会话看日志。
- 看到 `All devices ready.` 即成功；保持运行并退出界面：`Ctrl+B` 然后 `D`。
- 已在运行则直接进入、不重复启动；未装 tmux 会提示安装方法。

⚠ 手动启动与遥控器 A 键自启动二选一，不可混用。

## start_camera.sh —— 一键启动相机（快速版）

启动 Orbbec 相机驱动（Gemini 330 系列）。和 body_control 不同：相机跑在 **Orin**（不是 x86），**不需要 root、也不需要 body_control**。

```bash
chmod +x scripts/start_camera.sh    # 首次赋可执行权限（只需一次）
./scripts/start_camera.sh           # 在机器人 Orin 上、nvidia 用户执行
```

- 自动建 `cam` tmux 会话、source Orbbec workspace、执行 `ros2 launch orbbec_camera gemini_330_series.launch.py`，并带你进会话。
- 相机话题开始发布即成功；另开终端验证：`ros2 topic list | grep camera`。
- 保持运行并退出界面：`Ctrl+B` 然后 `D`；已在运行则直接进入；若找不到驱动 workspace，改脚本顶部的 `ORBBEC_WS`。
- 起来后 atom25 相机 demo 可在 Orin 本地跑，也可在 x86 跑（同一 ROS 图，需同 `ROS_DOMAIN_ID`）。

## start_voice.sh —— 一键启动语音（chat 模式）

启动 lyre 语音服务（chat 模式，含朗读/播放等）。前置见《前置 · 环境配置》(`atom/docs/environment_setup_zh-CN.md`)，用法细节见《语音·输出侧》guide（`atom/docs/atom26-27_voice_output_guide_zh-CN.md`）。跑在 **Orin**（`nvidia` 用户）。

```bash
chmod +x scripts/start_voice.sh   # 首次赋可执行权限（只需一次）
./scripts/start_voice.sh          # 在机器人 Orin 上、nvidia 用户执行
```

- 自动建 `voice` tmux 会话、source `~/ros2ws`、执行 `ros2 launch lyre chat.launch.py`。
- 语音服务/话题起来即成功：`ros2 service list | grep audio_play`、`ros2 topic list | grep audio`。
- **出厂通常已默认启动**——脚本会先检测 lyre 是否在跑，避免重复；已在跑则直接提示退出。
- 若 workspace 路径不同，改脚本顶部 `ROS2WS`。
