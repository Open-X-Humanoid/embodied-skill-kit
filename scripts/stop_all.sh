#!/usr/bin/env bash
# 一键清场 —— 停掉本仓库脚本启动的机器人相关会话/进程，用于"系统乱了"时重来。
# 用法：在对应板子上执行  bash scripts/stop_all.sh
#   body/xarm 在 x86（ubuntu）；camera/voice 在 Orin（nvidia）。
# ⚠ body_control 是 root 起的，杀它要 sudo（脚本会用 sudo，可能提示输密码）。
# ⚠ 命令按常规写，进程名/路径以你机器人为准，未逐台核实。

set -u

echo "========== 一键清场 =========="

echo "-- 停 tmux 会话 --"
for s in xarm body cam voice; do
    if tmux has-session -t "$s" 2>/dev/null; then
        tmux kill-session -t "$s" && echo "  已停会话: $s"
    fi
done

echo "-- 停 launch 进程 --"
for pat in "tianyi2_moveit" "tianyi2.launch" "tianyi2_bringup" "orbbec_camera" "ros2 launch lyre"; do
    if pgrep -f "$pat" >/dev/null 2>&1; then
        pkill -f "$pat" && echo "  已停: $pat"
    fi
done

echo "-- 停 body_control（root，需 sudo）--"
if pgrep -f "body_control" >/dev/null 2>&1; then
    sudo pkill -f "body_control" && echo "  已停: body_control"
else
    echo "  (未在运行)"
fi

sleep 1
echo "-- 残留检查（应尽量为空）--"
pgrep -af "tianyi2|body_control|orbbec|ros2 launch lyre" | grep -v "grep\|stop_all" || echo "  (干净)"

echo "========== 完成。可 ros2 node list 再确认 =========="
