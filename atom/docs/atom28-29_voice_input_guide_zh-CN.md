# 语音 · 输入侧（Voice Input）· 听与对话（atom28 / atom29）

[English](atom28-29_voice_input_guide.md) | **简体中文**

**一句话**：让机器人"听懂"。atom28 只读订阅 ASR 话题，看机器人**听到 / 识别到什么**（纯观察，不控制）；atom29 开 / 关全双工对话**通道**（只开通道；完整"听 → 大模型 → 说"闭环另需接大模型，见第 3 节）。

> ⚠ 一句前置：麦克风在 **Orin 本地、出厂自启**，起个 lyre（chat 模式）+ 喊唤醒词「天工天工」就能识别，**不用手动起任何 mic 进程**。所以 **atom28 已真机验证可用**；atom29 能识别、但"回复"还差大模型环节（见第 3 节）。

| 配套 | 路径 |
|---|---|
| 代码（听 ASR） | `atom/demos/atom28_voice_asr_ros2.py` |
| 代码（对话 Chat） | `atom/demos/atom29_voice_chat_ros2.py` |
| 演示视频 | 与配套代码同名，见 `atom/assets/videos/` |

## 1. 速览

### 1.1 跑起来

**前置 · 起 lyre（chat 模式）**：**lyre** 跑在 **Orin**（用户 `nvidia`）、**chat 模式**、**不需要 body_control**；出厂通常已自启。没起或想重启时，在 Orin 仓库根目录 `bash scripts/start_voice.sh`。**麦克风在 Orin 本地自启**，无需额外进程。ROS 基础环境见《前置 · 环境配置》(`atom/docs/environment_setup_zh-CN.md`)。

```bash
source ~/ros2ws/install/setup.bash                 # 每个新终端都要（或 source scripts/start_voice.sh）

python3 atom/demos/atom28_voice_asr_ros2.py         # 纯订阅，一直听，Ctrl-C 退出
python3 atom/demos/atom29_voice_chat_ros2.py        # 输入 GO 确认后开启对话；按 Enter 关闭并退出
```

**先对麦克风喊「天工天工」唤醒，再说话**，日志里就会打印识别到的文字。

### 1.2 接口（都是话题）

| 话题 | 类型 | 方向 | 内容 |
|---|---|---|---|
| `/audio_asr/keyword` | `AsrKeyword` | 订阅 | 唤醒词事件：`keyword`（词）· `angle`（声源角度） |
| `/audio_asr/iat` | `AsrIat` | 订阅 | 语音转文字：`id` · `text` |
| `/audio_asr/event` | `AsrEvent` | 订阅 | 状态事件：`event`（事件码）· `arg1` |
| `/audio_chat/enable` | `std_msgs/Bool` | **发布**（仅 atom29） | `True` 开启对话 / `False` 关闭 |

> `iat` 的 `text` 字段已随 atom28 验证可用；`keyword`/`event` 的字段名仍建议 `ros2 interface show lyre_msgs/msg/AsrKeyword` 核一下。

### 1.3 一个关键概念：话题 vs 服务

输入侧用的是**话题（topic）**——持续订阅数据流、没有返回码，你被动地"收"。这跟输出侧（atom26/27）用**服务**主动"调一次拿回执"正好相反。ASR 是源源不断冒出来的识别结果，天然是话题。

## 2. 两个原子怎么用

### 2.1 atom28 听（ASR，只读、最安全）

- 三个订阅 + 三个回调，收到就打印。**不发任何指令、不驱动电机、不开关对话**，机器人行为完全不变——最适合先拿它确认"输入侧到底通没通"。
- `EVENT_NAMES` 把事件码翻译成可读名（如 `4=WAKEUP唤醒`、`13=已连接服务端`），以官方 SDK 文档为准。
- `main` 里 `rclpy.spin` 一直转，Ctrl-C 退出。

### 2.2 atom29 对话（Chat，风险最高）

- **它只开/关对话通道，本身不生成回复**（"想出要说什么"的大脑见第 3 节）。
- 发布 `/audio_chat/enable`：`enable()` 发 `True`、`disable()` 发 `False`，并同时订阅 ASR 观察机器人听到什么。
- `confirm()` 要求手动输入 `GO` 才开启——因为**开启后机器人持续聆听并回应，会影响同场所其他人，同一时刻只能一个对话会话**。
- **`finally` 里必发 `disable()`**（★退出绝不遗留"持续聆听"态）。若程序崩溃遗留开启态，手动关：

```bash
ros2 topic pub -1 /audio_chat/enable std_msgs/msg/Bool 'data: false'
```

## 3. 让 atom29 真的回复（接大模型）

atom29 只开对话通道、不含"想出要说什么"的大脑，所以你能识别、但不回复——这是**缺大脑，不是缺麦克风**。补齐它有两条路（择一，需真机核实）：

- **`/audio_llm/ask` 服务**：直接把问题丢给 lyre 的 LLM 接口，回复走 `/audio_llm/rst` 话题（可能自动 TTS 说出来）。最省事，是做成 atom30 问答的候选。
- **外部 agent**：move_box 的 `start_mwc.sh` 里单起一个 `kaiwu_agent`（接大模型、编排多轮），适合连续对话。

> 判断卡在哪：说话时 `ros2 topic echo /audio_llm/rst`——有文本 = 大脑通了、空 = 大脑没起。

## 4. 代码解读（核心）

| 模块 | 代码锚点 | 职责 | atom28 / atom29 |
|---|---|---|---|
| 订阅 ASR | `create_subscription(AsrIat, "/audio_asr/iat", …)` | 收识别结果 | 两者都有 |
| 回调打印 | `_on_iat` / `_on_keyword` / `_on_event` | 把消息打成日志 | 两者都有 |
| 开关对话 | `pub.publish(Bool(data=…))` | 开/关全双工交互 | 仅 atom29 |
| 收尾必关 | `finally: node.disable()` | 退出时关对话 | 仅 atom29（★） |

**举一反三**：想在自己程序里"感知机器人听到了什么"，订阅 `/audio_asr/iat` 即可（和 atom28 一样）；想在某个时机开启/关闭对话，发布 `/audio_chat/enable`（和 atom29 一样），且务必在退出路径上补 `False`。

## 5. 排错

- **atom28 喊了话没反应**：先确认喊了唤醒词「天工天工」；`ps -ef | grep lyre` 确认 chat 模式；`ros2 topic echo /audio_asr/iat` 看有没有数据（把"lyre 有没有发"和"demo 有没有收"分开定位）。
- **原始话题有数据、demo 不打印**：域号不一致（两个终端各 `echo $ROS_DOMAIN_ID` 对比），或 `keyword`/`event` 字段名对不上（`ros2 interface show` 核对后反馈我改 demo）。
- **atom29 能识别却不回复**：正常——缺"回复大脑"，见第 3 节，不是麦克风问题。

## 6. 状态与待办

- ✅ atom28：真机验证通过（唤醒词唤醒 + 识别语音）。
- ⏳ atom29：开关通道通、识别通；**待接上大模型回复**（`/audio_llm/ask` 或外部 agent）验证端到端，再考虑 promote。
- 待办：`ros2 interface show` 拿到 `/audio_llm/ask` 字段 → 可写 atom30 问答。
