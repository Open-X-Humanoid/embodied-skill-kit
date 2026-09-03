# Voice · Output Side · TTS & Playback (interaction01 / interaction02)

**English** | [简体中文](interaction01-02_voice_output_guide_zh-CN.md)

**In one line**: make the robot *speak*. interaction01 reads a piece of text aloud (TTS); interaction02 plays an audio file / URL with pause·resume·stop. Both just **call lyre's `/audio_play/*` services** — one running lyre is enough, no motors, no microphone.

| Companion | Path |
|---|---|
| Code (TTS) | `atom/interaction/interaction01_voice_tts_ros2.py` |
| Code (playback) | `atom/interaction/interaction02_voice_play_ros2.py` |
| Demo video | same name as the code, under `atom/interaction/assets/videos/` |

## 1. Overview

### 1.1 Run it

**Prerequisite · start lyre (chat mode)**: the **lyre** voice package (iFlytek) runs on the **Orin** (user `nvidia`), not the x86, and needs **no body_control**; it's usually auto-started at boot. If not, or to restart, from the Orin repo root:

```bash
bash scripts/start_voice.sh
# equivalent to: cd ~/ros2ws && source install/setup.bash && ros2 launch lyre chat.launch.py
```

> lyre has four mutually-exclusive modes `play/asr/audio/chat`; **chat (factory default) covers all voice features**, so just use it — no mode switching. For the base ROS environment see *Prerequisite · Environment Setup* (`docs/environment_setup.md`).

**Run a demo**: open a new terminal on the Orin (user `nvidia`, repo root):

```bash
source ~/ros2ws/install/setup.bash          # every new terminal, else import lyre_msgs fails

python3 atom/interaction/interaction01_voice_tts_ros2.py  # robot reads the TEXT in the code
python3 atom/interaction/interaction02_voice_play_ros2.py # menu: 1 file / 2 URL / 3 pause / 4 resume / 5 stop / q quit
```

### 1.2 Interfaces (all are service calls)

| Service | Type | Purpose | Key request fields |
|---|---|---|---|
| `/audio_play/play_text` | `PlayText` | read text aloud | `sid` `seq` `last` `force` `text` |
| `/audio_play/play_file` | `PlayFile` | play a local file on the Orin | …`path` |
| `/audio_play/play_url` | `PlayUrl` | play a network URL | …`url` |
| `/audio_play/pause` | `PlayPause` | pause | empty request |
| `/audio_play/resume` | `PlayResume` | resume | empty request |
| `/audio_play/stop` | `PlayStop` | stop (not resumable) | empty request |

Field meaning: `sid` = unique playback-stream ID (the demo uses `interaction01_<random>`); `seq`/`last` = packet index / last-packet flag, use `0`/`True` for a one-shot; `force` = `True` interrupts current playback / `False` queues; `token`/`output` = internal system fields, leave **empty**. Response `code`: `0` success / `1` bad args / `-1` internal failure.

### 1.3 A key concept: service vs topic

The output side uses **services** — request/response, giving you a `code` that tells you whether it worked. That's the opposite of **continuously subscribing to a data stream** (camera, sensors), which uses **topics** and has no return code. Rule of thumb: "do one thing and give me a receipt" → service; "keep emitting data at me" → topic.

## 2. How each atom is used

### 2.1 interaction01 TTS

- Change the `TEXT` constant to change what's read.
- Core method `say(text, force)`: `wait_for_service` (5s) → fill `PlayText.Request` (unique `sid`, empty `token/output`) → `call_async` → `spin_until_future_complete` → check `resp.code`.
- `code=0` accepted; `1` bad args; `-1` internal failure (commonly: lyre not in audio/chat mode, so no TTS).

### 2.2 interaction02 playback

- Change `FILE_PATH` (a file that really exists on the Orin) / `URL`.
- Five service clients; `_call()` wraps "wait for service → call → check code".
- `pause`/`resume`/`stop` are empty requests; **after `stop` you cannot `resume`** (call `play_file` again); `force=True` interrupts current audio.

## 3. Code walkthrough (core)

Both atoms share the **same service-call pattern**. Module map:

| Module | Code anchor | Role | Change per service? |
|---|---|---|---|
| create client | `create_client(Type, name)` | connect to a lyre service | change type + name |
| fill request | `req.sid=…` / `req.text=…` | pack args; `sid` unique each time | per that service's fields |
| call + wait | `call_async` + `spin_until_future_complete` | send async, await receipt | unchanged |
| check result | `resp.code == 0` | success/failure | unchanged (empty-request services have no `code`) |

**Generalize**: to call any other lyre service, it's the same four steps — `create_client` → fill `Request` → `call_async`+`spin_until_future_complete` → check `code`. That's why interaction02's five services all share one `_call()`.

## 4. Tweak and observe

- interaction01: change `TEXT`; set `force=False` and call it while audio is playing to **queue** instead of interrupt.
- interaction02: point `FILE_PATH` at another file; pick `1` then `3/4/5` to feel pause/resume/stop; play one clip then `1` another (`force=True`) to see the interrupt.

## 5. Troubleshooting

- **Service times out at 5s**: lyre isn't up, or wrong mode (`play` mode has no TTS). Check `ros2 service list | grep audio_play`, `ps -ef | grep lyre`.
- **`code=1` bad args**: empty `sid` or a missing required field.
- **`code=-1` internal failure**: TTS/playback capability unavailable (mode, network, iFlytek auth).
- **`code=0` but no sound**: volume, speaker, or the file/URL itself; `play_file` needs a path that **really exists on the Orin**.
