# 语音 · 输出侧（Voice Output）· 朗读与放音频（interaction01 / interaction02）

[English](interaction01-02_voice_output_guide.md) | **简体中文**

**一句话**：让机器人"出声"。interaction01 把一段文字读出来（TTS），interaction02 播放音频文件 / 网络 URL 并支持暂停·恢复·停止。两者都只是**调用 lyre 的 `/audio_play/*` 服务**，起一个 lyre 就能跑，不碰电机、不需麦克风。

| 配套 | 路径 |
|---|---|
| 代码（朗读） | `atom/interaction/interaction01_voice_tts_ros2.py` |
| 代码（放音频） | `atom/interaction/interaction02_voice_play_ros2.py` |
| 演示视频 | 与配套代码同名，见 `atom/interaction/assets/videos/` |

## 1. 速览

### 1.1 跑起来

**前置 · 起 lyre（chat 模式）**：语音包 **lyre**（科大讯飞）跑在 **Orin**（用户 `nvidia`），不是 x86，**不需要 body_control**；出厂通常已自启。没起或想重启时，在 Orin 仓库根目录执行：

```bash
bash scripts/start_voice.sh
# 等价于：cd ~/ros2ws && source install/setup.bash && ros2 launch lyre chat.launch.py
```

> lyre 有 `play/asr/audio/chat` 四种互斥模式，**chat（出厂默认）覆盖全部语音功能**，统一用它即可、无需切模式。ROS 基础环境见《前置 · 环境配置》(`docs/environment_setup_zh-CN.md`)。

**跑 demo**：在 Orin 上新开终端（`nvidia` 用户、仓库根目录）：

```bash
source ~/ros2ws/install/setup.bash          # 每个新终端都要，否则 import lyre_msgs 报错

python3 atom/interaction/interaction01_voice_tts_ros2.py  # 机器人读出代码里的 TEXT
python3 atom/interaction/interaction02_voice_play_ros2.py # 交互菜单：1 播文件 / 2 播URL / 3 暂停 / 4 恢复 / 5 停止 / q 退出
```

### 1.2 接口（都是服务调用）

| 服务 | 类型 | 作用 | 关键请求字段 |
|---|---|---|---|
| `/audio_play/play_text` | `PlayText` | 朗读文字 | `sid` `seq` `last` `force` `text` |
| `/audio_play/play_file` | `PlayFile` | 播 Orin 本地文件 | …`path` |
| `/audio_play/play_url` | `PlayUrl` | 播网络 URL | …`url` |
| `/audio_play/pause` | `PlayPause` | 暂停 | 空请求 |
| `/audio_play/resume` | `PlayResume` | 恢复 | 空请求 |
| `/audio_play/stop` | `PlayStop` | 停止（不可恢复） | 空请求 |

字段含义：`sid` = 唯一播放流 ID（demo 用 `interaction01_<随机>`）；`seq`/`last` = 分包序号/末包标志，单次播放填 `0`/`True`；`force` = `True` 打断当前播放 / `False` 排队；`token`/`output` = 系统内部字段，应用层**留空**。响应 `code`：`0` 成功 / `1` 参数非法 / `-1` 内部失败。

### 1.3 一个关键概念：服务 vs 话题

输出侧用的是**服务（service）**——一问一答，调用后有 `code` 明确告诉你成没成。这跟**持续订阅数据流**（如相机、传感器）用**话题（topic）**、没有返回码，正好相反。记住："让它做一件事、要回执" 用服务；"持续听它冒数据" 用话题。

## 2. 两个原子怎么用

### 2.1 interaction01 朗读（TTS）

- 改 `TEXT` 常量换朗读内容。
- 核心方法 `say(text, force)`：`wait_for_service`（5s 等服务）→ 填 `PlayText.Request`（`sid` 唯一、`token/output` 留空）→ `call_async` → `spin_until_future_complete` 等结果 → 看 `resp.code`。
- `code=0` 已接受播放；`1` 参数非法；`-1` 内部失败（常见于 lyre 不在 audio/chat 模式，无 TTS 能力）。

### 2.2 interaction02 放音频

- 改 `FILE_PATH`（Orin 上真实存在的音频）/ `URL`。
- 建了 5 个 service client，`_call()` 统一封装"等服务 → 调 → 查 code"。
- `pause`/`resume`/`stop` 是空请求；**`stop` 之后不能 `resume`**（要重新 `play_file`）；`force=True` 会打断当前音频。

## 3. 代码解读（核心）

两个原子是**同一套服务调用范式**，模块地图：

| 模块 | 代码锚点 | 职责 | 换服务要改？ |
|---|---|---|---|
| 建 client | `create_client(类型, 服务名)` | 连上某个 lyre 服务 | 改类型 + 服务名 |
| 填 request | `req.sid=…` / `req.text=…` | 装参数，`sid` 每次唯一 | 按该服务字段改 |
| 调用 + 等 | `call_async` + `spin_until_future_complete` | 异步发、等回执 | 不变 |
| 查结果 | `resp.code == 0` | 判成败 | 不变（空请求服务无 code） |

**举一反三**：想调 lyre 任何别的服务，都是这四步——`create_client` → 填 `Request` → `call_async`+`spin_until_future_complete` → 查 `code`。interaction02 里五个服务共用一个 `_call()` 就是这个道理。

## 4. 改一改，看变化

- interaction01：改 `TEXT`；把 `force=True` 改 `False`，在有音频播放时调用会**排队**而非打断。
- interaction02：改 `FILE_PATH` 指向别的文件；连续选 `1` 再选 `3/4/5` 体会暂停/恢复/停止；先播一段再选 `1` 播另一段（`force=True`）看打断效果。

## 5. 排错

- **服务 5s 超时不在**：lyre 没起，或模式不对（`play` 模式不含 TTS）。查 `ros2 service list | grep audio_play`、`ps -ef | grep lyre`。
- **`code=1` 参数非法**：`sid` 空、或必填字段没给。
- **`code=-1` 内部失败**：TTS/播放能力不可用（模式、网络、讯飞授权）。
- **`code=0` 却没声音**：音量、扬声器、或播放的文件/URL 本身有问题；`play_file` 的路径要是 **Orin 上真实存在**的文件。
