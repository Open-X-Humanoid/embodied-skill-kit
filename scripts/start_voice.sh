#!/usr/bin/env bash
# 一键启动 lyre 语音（chat 模式）—— 语音原子（interaction01~29）的前置。
#
# 两种用法，跑完都能直接跑 demo：
#   1) bash   scripts/start_voice.sh   → 确保 lyre 在跑，并把你丢进一个"已 source 环境"的新 shell
#   2) source scripts/start_voice.sh   → 只给【当前终端】source 环境（不开新 shell）
#
# 为什么要这么绕：source 改的是"当前终端"的环境；而 bash 跑脚本是子进程、改不了你父终端。
# 所以 bash 模式没法直接把你现在的终端 source 上，只能"另开一个已 source 好的 shell"给你用
# （就像 start_body_control 把你送进 tmux 那样）；用完在那个 shell 里 exit 即可退回。
#
# 一劳永逸：把  source ~/ros2ws/install/setup.bash  加进 ~/.bashrc，之后每个新终端自动就绪。
#
# ⚠ 语音跑在 Orin（用户 nvidia），不是 x86。chat 模式支持全部语音功能（朗读/播放/听/对话）。

# ── 可按需修改 ──────────────────────────────────────────────
SESSION="voice"                                     # tmux 会话名
ROS2WS="$HOME/ros2ws"                                # lyre 所在 workspace
LAUNCH_CMD="ros2 launch lyre chat.launch.py"         # chat 模式：含 TTS + 播放 + ASR + 对话
SETUP="$ROS2WS/install/setup.bash"
# ────────────────────────────────────────────────────────────

# ============================================================
# 用法 2：被 source 运行 → 只给当前终端 source，然后返回。
#   放在 set -u 之前，避免把 -u 残留到用户的交互 shell。
# ============================================================
if [ "${BASH_SOURCE[0]}" != "${0}" ]; then
    if [ -f "$SETUP" ]; then
        source "$SETUP"
        echo "[OK] 已为当前终端 source: $SETUP，可直接跑 demo。"
        pgrep -f "ros2 launch lyre" >/dev/null 2>&1 || \
            echo "[提示] 未检测到 lyre 在运行；启动它用： bash scripts/start_voice.sh"
    else
        echo "[错误] 找不到 $SETUP —— 请改脚本顶部的 ROS2WS。"
    fi
    return 0
fi

# ============================================================
# 用法 1：被 bash 执行 → 确保 lyre 在跑，再把你送进一个已 source 的 shell。
# ============================================================
set -u

echo "=================================================="
echo " 一键启动 lyre 语音（chat 模式） · 在 Orin 上执行"
echo "=================================================="

if [ ! -f "$SETUP" ]; then
    echo "[错误] $SETUP 不存在。请改脚本顶部的 ROS2WS。"; exit 1
fi

# 1) 确保 lyre 在运行：已在跑就跳过；没跑就在 tmux 后台拉起
if tmux has-session -t "$SESSION" 2>/dev/null || pgrep -f "ros2 launch lyre" >/dev/null 2>&1; then
    echo "[提示] 检测到 lyre 已在运行（可能出厂自启），无需重复启动。"
else
    if ! command -v tmux >/dev/null 2>&1; then
        echo "[错误] 未安装 tmux。请先执行： sudo apt install -y tmux"; exit 1
    fi
    echo "[启动] 在 tmux 会话 '$SESSION' 里拉起 lyre ..."
    tmux new-session -d -s "$SESSION" \
        "cd '$ROS2WS' && source install/setup.bash && $LAUNCH_CMD; \
         echo; echo '[lyre 已退出，停在 shell]'; exec bash"
    echo "       看 lyre 日志： tmux attach -t $SESSION   （Ctrl+B D 脱离）"
    sleep 1
fi

# 2) 把你送进一个"已 source 环境"的交互 shell —— 直接就能跑 demo；用完 exit 退回原终端
echo "--------------------------------------------------"
echo "[就绪] 已打开一个 source 好环境的新 shell，直接跑，例如："
echo "         python3 atom/interaction/interaction01_voice_tts_ros2.py"
echo "       用完输入 exit 退回原来的终端。"
echo "--------------------------------------------------"
exec bash --rcfile <(echo "[ -f ~/.bashrc ] && source ~/.bashrc; source '$SETUP'")
