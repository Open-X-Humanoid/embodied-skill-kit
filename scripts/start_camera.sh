#!/usr/bin/env bash
# 一键启动 Orbbec 相机驱动 —— 感知类原子（如 atom25 相机）的前置。
# 用法：在机器人 Orin 上执行  ./scripts/start_camera.sh
#
# ⚠ 相机驱动跑在 Orin（用户 nvidia），不是 x86；不需要 body_control、不需要 root。
#   起来后 atom25 相机 demo 可在 Orin 本地跑，也可在 x86 跑（同一 ROS 图，需同 ROS_DOMAIN_ID）。

set -u

# ── 可按需修改 ──────────────────────────────────────────────
SESSION="cam"                                       # tmux 会话名
ORBBEC_WS="$HOME/orbbec_camera_ros2"                # Orbbec 驱动 workspace（含 install/setup.bash）
LAUNCH_CMD="ros2 launch orbbec_camera gemini_330_series.launch.py"   # Gemini 330 系列
# ────────────────────────────────────────────────────────────

echo "=================================================="
echo " 一键启动 Orbbec 相机驱动（Gemini 330）"
echo " ⚠ 在 Orin 上执行；确认相机 USB 已插好"
echo "=================================================="

# 1) 没装 tmux 直接退出并给出安装方法
if ! command -v tmux >/dev/null 2>&1; then
    echo "[错误] 未安装 tmux。请先执行： sudo apt install -y tmux"
    exit 1
fi

# 2) 已在运行则直接进入，不重复启动
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[提示] 相机会话已存在，直接进入（未重复启动）。"
    echo "       如需重启： tmux kill-session -t $SESSION  后再运行本脚本。"
    exec tmux attach -t "$SESSION"
fi

# 3) 定位 workspace：默认路径没有就自动 find 反推
if [ ! -f "$ORBBEC_WS/install/setup.bash" ]; then
    echo "[提示] $ORBBEC_WS 下没找到 install/setup.bash，尝试自动定位驱动 ..."
    launch_file=$(find "$HOME" -maxdepth 5 -name "gemini_330_series.launch.py" 2>/dev/null | head -1)
    derived=$(echo "$launch_file" | sed 's#/install/.*##')
    if [ -n "$launch_file" ] && [ -f "$derived/install/setup.bash" ]; then
        ORBBEC_WS="$derived"
        echo "[提示] 定位到 workspace： $ORBBEC_WS"
    else
        echo "[错误] 找不到已编译的 orbbec 驱动 workspace。"
        echo "       请确认已装/编译 orbbec_camera 包，或手动改本脚本顶部的 ORBBEC_WS。"
        exit 1
    fi
fi

# 4) 新建后台会话：source + launch；launch 退出后保留 shell 便于看报错
echo "[启动] 新建 tmux 会话 '$SESSION' 并拉起相机驱动 ..."
tmux new-session -d -s "$SESSION" \
    "cd '$ORBBEC_WS' && source install/setup.bash && $LAUNCH_CMD; \
     echo; echo '[相机驱动已退出，停在 shell。Ctrl+B D 脱离 / exit 关闭会话]'; exec bash"

echo "[提示] 即将进入会话；相机话题开始发布即成功。"
echo "       验证（另开终端）： ros2 topic list | grep camera"
echo "       保持运行并退出界面： Ctrl+B 然后按 D"
sleep 1
exec tmux attach -t "$SESSION"
