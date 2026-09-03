# Voice · Input Side · Listen & Chat (interaction03 / interaction04)

**English** | [简体中文](interaction03-04_voice_input_guide_zh-CN.md)

**In one line**: make the robot *understand*. interaction03 read-only subscribes to the ASR topics to observe **what the robot hears / recognizes** (pure observation, no control); interaction04 turns the full-duplex chat **channel** on/off (only the channel; the full "listen → LLM → speak" loop also needs an LLM stage, see §3).

> ⚠ One prerequisite note: the microphone is **local to the Orin and auto-starts at boot**, so one lyre (chat mode) + the wake word "天工天工" is enough to recognize speech — **no manual mic process needed**. Thus **interaction03 is verified on the real robot**; interaction04 recognizes speech but the "reply" still needs an LLM stage (see §3).

| Companion | Path |
|---|---|
| Code (ASR listen) | `atom/interaction/interaction03_voice_asr_ros2.py` |
| Code (Chat) | `atom/interaction/interaction04_voice_chat_ros2.py` |
| Demo video | same name as the code, under `atom/interaction/assets/videos/` |

## 1. Overview

### 1.1 Run it

**Prerequisite · start lyre (chat mode)**: **lyre** runs on the **Orin** (user `nvidia`) in **chat mode** and needs **no body_control**; usually auto-started at boot. If not, or to restart, from the Orin repo root run `bash scripts/start_voice.sh`. The **microphone is local to the Orin and auto-starts** — no extra process. For the base ROS environment see *Prerequisite · Environment Setup* (`docs/environment_setup.md`).

```bash
source ~/ros2ws/install/setup.bash                 # every new terminal (or source scripts/start_voice.sh)

python3 atom/interaction/interaction03_voice_asr_ros2.py         # pure subscribe, keeps listening, Ctrl-C to quit
python3 atom/interaction/interaction04_voice_chat_ros2.py        # type GO to confirm & enable; Enter to disable & quit
```

**Say "天工天工" into the mic to wake, then speak** — the recognized text prints in the log.

### 1.2 Interfaces (all are topics)

| Topic | Type | Direction | Content |
|---|---|---|---|
| `/audio_asr/keyword` | `AsrKeyword` | subscribe | wake-word event: `keyword` · `angle` (sound-source angle) |
| `/audio_asr/iat` | `AsrIat` | subscribe | speech-to-text: `id` · `text` |
| `/audio_asr/event` | `AsrEvent` | subscribe | status event: `event` (code) · `arg1` |
| `/audio_chat/enable` | `std_msgs/Bool` | **publish** (interaction04 only) | `True` enable chat / `False` disable |

> `iat`'s `text` field is verified via interaction03; still worth checking `keyword`/`event` fields with `ros2 interface show lyre_msgs/msg/AsrKeyword`.

### 1.3 A key concept: topic vs service

The input side uses **topics** — you subscribe to a continuous stream with no return code; you passively *receive*. That's the opposite of the output side (interaction01/27), which uses **services** to actively "call once and get a receipt". ASR results stream out continuously, so a topic is the natural fit.

## 2. How each atom is used

### 2.1 interaction03 listen (ASR, read-only, safest)

- Three subscriptions + three callbacks that print on arrival. **Sends no commands, drives no motors, toggles no chat** — the robot's behavior is unchanged, making this the best tool to first confirm "is the input side actually working".
- `EVENT_NAMES` translates event codes to readable names (e.g. `4=WAKEUP`, `13=connected`), per the official SDK docs.
- `main` runs `rclpy.spin` forever; Ctrl-C to quit.

### 2.2 interaction04 chat (highest risk)

- **It only toggles the chat channel; it does not generate replies** (the "decide what to say" brain is in §3).
- Publishes `/audio_chat/enable`: `enable()` sends `True`, `disable()` sends `False`, while also subscribing to ASR to observe what the robot hears.
- `confirm()` requires typing `GO` before enabling — because **once enabled the robot keeps listening and responding, affecting others nearby, and only one chat session can run at a time**.
- **`finally` always sends `disable()`** (★ never leave a "keeps listening" state on exit). If the program crashes leaving it on, turn it off manually:

```bash
ros2 topic pub -1 /audio_chat/enable std_msgs/msg/Bool 'data: false'
```

## 3. Make interaction04 actually reply (wire up an LLM)

interaction04 only opens the chat channel and has no "decide what to say" brain, so it recognizes speech but won't reply — a **missing brain, not a missing mic**. Two ways to fill it (pick one, verify on the robot):

- **`/audio_llm/ask` service**: send the question straight to lyre's LLM interface; the reply comes on the `/audio_llm/rst` topic (possibly auto-spoken via TTS). Simplest — a candidate for an interaction05 Q&A.
- **External agent**: move_box's `start_mwc.sh` starts a separate `kaiwu_agent` (LLM + multi-turn orchestration), suited to continuous conversation.

> Locate the gap: while speaking, `ros2 topic echo /audio_llm/rst` — text = brain works, empty = brain not running.

## 4. Code walkthrough (core)

| Module | Code anchor | Role | interaction03 / interaction04 |
|---|---|---|---|
| subscribe ASR | `create_subscription(AsrIat, "/audio_asr/iat", …)` | receive recognition results | both |
| callback print | `_on_iat` / `_on_keyword` / `_on_event` | log the message | both |
| toggle chat | `pub.publish(Bool(data=…))` | enable/disable full-duplex | interaction04 only |
| cleanup off | `finally: node.disable()` | turn chat off on exit | interaction04 only (★) |

**Generalize**: to "sense what the robot hears" in your own program, subscribe to `/audio_asr/iat` (like interaction03); to enable/disable chat at some moment, publish `/audio_chat/enable` (like interaction04) — and always send `False` on the exit path.

## 5. Troubleshooting

- **interaction03: spoke but no reaction**: first confirm you said the wake word "天工天工"; `ps -ef | grep lyre` to confirm chat mode; `ros2 topic echo /audio_asr/iat` to check whether data flows (splits "is lyre publishing" from "is the demo receiving").
- **Raw topic has data but the demo prints nothing**: domain mismatch (compare `echo $ROS_DOMAIN_ID` in both terminals), or wrong `keyword`/`event` field names (`ros2 interface show`, then send back the real names to fix the demo).
- **interaction04 recognizes but doesn't reply**: expected — the "reply brain" is missing, see §3; not a mic problem.

## 6. Status & to-do

- ✅ interaction03: verified on the real robot (wake word + speech recognition).
- ⏳ interaction04: channel toggle and recognition work; **pending the LLM reply** (`/audio_llm/ask` or an external agent) to verify the end-to-end loop before promoting.
- To-do: `ros2 interface show` the `/audio_llm/ask` service type → then an interaction05 Q&A becomes writable.
