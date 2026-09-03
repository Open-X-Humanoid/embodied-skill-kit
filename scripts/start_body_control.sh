#!/usr/bin/env bash
# 一键启动 body_control —— 封装《前置 · 环境配置》(docs/environment_setup_zh-CN.md) 第 2 节的手动步骤。
# 用法：在机器人 x86 上、以 ubuntu 用户执行  ./start_body_control.sh
#   body_control 需要 root，脚本内部用 sudo 提权；跑在名为 body 的 tmux 会话里，SSH 断开也不退出。
#
# ⚠ 启动前确认：机器人周围无人、急停在手、遥控器保持关闭。
#   手动启动与遥控器 A 键自启动【二选一，不可混用】（会重复启动、控制冲突）。

set -u

# ── 可按需修改 ──────────────────────────────────────────────
SESSION="body"                                  # tmux 会话名
ROS2WS="/home/ubuntu/ros2ws"                     # 本体工作空间
LAUNCH_CMD="ros2 launch body_control body.launch.py"
# ────────────────────────────────────────────────────────────

echo "=================================================="
echo " 一键启动 body_control"
echo " ⚠ 确认：周围无人、急停在手、遥控器关闭（勿与 A 键自启混用）"
echo "=================================================="

# 1) 没装 tmux 直接退出并给出安装方法
if ! command -v tmux >/dev/null 2>&1; then
    echo "[错误] 未安装 tmux。请先执行： sudo apt install -y tmux"
    exit 1
fi

# 2) 已在运行则直接进入，不重复启动
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[提示] body 会话已存在，直接进入（未重复启动）。"
    echo "       如需重启： tmux kill-session -t $SESSION  后再运行本脚本。"
    exec tmux attach -t "$SESSION"
fi

# 3) 新建后台会话：提权 + source + launch；launch 退出后保留 shell 便于看报错
echo "[启动] 新建 tmux 会话 '$SESSION' 并拉起 body_control ..."
tmux new-session -d -s "$SESSION" \
    "sudo bash -c 'cd $ROS2WS && source install/setup.bash && $LAUNCH_CMD'; \
     echo; echo '[body 已退出，停在 shell。Ctrl+B D 脱离 / exit 关闭会话]'; exec bash"

echo "[提示] 即将进入会话。看到 'All devices ready.' 即启动成功。"
echo "       如提示输入密码，请输入 ubuntu 用户的 sudo 密码。"
echo "       保持 body 运行并退出界面： Ctrl+B 然后按 D"
sleep 1
exec tmux attach -t "$SESSION"
