#!/usr/bin/env bash
# 一键启动 XARM 框架 + MoveIt 组件 —— 手臂 MoveIt 原子（atom05）的前置。
# ⚠ 本脚本命令取自知识库《天轶2.0 XARM启动》，尚未真机核实，路径/包名以你机器人为准。
#
# 两种用法：
#   1) 启动 XARM+MoveIt：      bash scripts/start_xarm.sh [real|sim]   （默认 real）
#   2) 只给【当前终端】配环境：  source scripts/start_xarm.sh            （注意是 source）
#
# ⚠ 在机器人 x86、ubuntu 用户上执行（XARM 与 body_control 同在 x86；路径 /home/ubuntu 即 x86 用户）。
# ⚠ XARM 装在 /home/ubuntu/XARM，有【自己的 install】（含 moveit_msgs / tianyi2_bringup），
#   不是 ros2ws！跑 atom05 的终端必须 source 这个，不是 ros2ws。
# ⚠ real（真机）前必须先在 x86 起 body_control（见 start_body_control.sh）。
#   sim（仿真）不需 body_control、不接真机、带 RViz —— 先用它零风险验证 atom05 最稳。

# ── 可按需修改 ──────────────────────────────────────────────
SESSION="xarm"
XARM_WS="/home/ubuntu/XARM"                          # XARM 部署位置
SETUP="$XARM_WS/install/setup.bash"
MODE="${1:-real}"                                    # real 或 sim
# ────────────────────────────────────────────────────────────

# ============================================================
# 用法 2：被 source 运行 → 只给当前终端 source，然后返回（放在 set -u 之前）
# ============================================================
if [ "${BASH_SOURCE[0]}" != "${0}" ]; then
    if [ -f "$SETUP" ]; then
        source "$SETUP"
        echo "[OK] 已为当前终端 source: $SETUP"
        echo "     现在可跑： python3 atom/demos/atom05_arm_moveit.py"
    else
        echo "[错误] 找不到 $SETUP —— XARM 部署了吗？改脚本顶部 XARM_WS。"
    fi
    return 0
fi

# ============================================================
# 用法 1：被 bash 执行 → tmux 两窗格起 XARM 本体 + MoveIt 组件
# ============================================================
set -u

if [ "$MODE" = "sim" ]; then
    BODY_LAUNCH="ros2 launch tianyi2_bringup tianyi2.launch.py gui:=true"     # 仿真 + RViz
else
    BODY_LAUNCH="ros2 launch tianyi2_bringup tianyi2.launch.py hardware:=real"
fi
MOVEIT_LAUNCH="ros2 launch tianyi2_bringup tianyi2_moveit.launch.py"

echo "=================================================="
echo " 一键启动 XARM + MoveIt   （模式: $MODE）"
echo "=================================================="

if [ ! -f "$SETUP" ]; then
    echo "[错误] $SETUP 不存在，XARM 未部署？改脚本顶部 XARM_WS。"; exit 1
fi
if ! command -v tmux >/dev/null 2>&1; then
    echo "[错误] 未装 tmux： sudo apt install -y tmux"; exit 1
fi
if [ "$MODE" = "real" ]; then
    echo "[提醒] 真机模式：确认已在 x86 起 body_control、周围无人、急停在手。"
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[提示] xarm 会话已存在，直接进入（注意：不会用这次的 '$MODE' 参数重启）。"
    echo "       想换模式/重启： bash scripts/stop_all.sh 清场后再跑。"
    exec tmux attach -t "$SESSION"
fi

# real 模式必须先起 body_control，否则 XARM 看门狗超时、臂不动
if [ "$MODE" = "real" ] && ! pgrep -f "body_control" >/dev/null 2>&1; then
    echo "[⚠ 警告] 未检测到 body_control 在运行！real 模式必须先起它。"
    echo "         请先另开终端执行： bash scripts/start_body_control.sh"
    printf "         已起好 body_control 要继续？按 Enter 继续 / Ctrl+C 取消... "; read _
fi

echo "[启动] tmux 会话 '$SESSION'：窗格0 = XARM 本体，窗格1 = MoveIt 组件"
tmux new-session -d -s "$SESSION" \
    "cd '$XARM_WS' && source install/setup.bash && $BODY_LAUNCH; \
     echo; echo '[XARM 本体已退出，停在 shell]'; exec bash"
tmux split-window -h -t "$SESSION" \
    "cd '$XARM_WS' && source install/setup.bash && \
     echo '[等 XARM 本体起来，再启动 MoveIt 组件...]' && sleep 8 && $MOVEIT_LAUNCH; \
     echo; echo '[MoveIt 组件已退出，停在 shell]'; exec bash"
tmux select-layout -t "$SESSION" even-horizontal

echo "[提示] 验证就绪（另开终端，先 source $SETUP）:"
echo "    ros2 control list_controllers          # 应含 moveit_*_arm_controller"
echo "    ros2 action list | grep move_action    # MoveIt 起来了 → atom05 才连得上"
[ "$MODE" = "real" ] && echo "    ros2 node list | grep /EAIHardware     # 真机通信节点，且日志无'看门狗超时'"
echo "  跑 demo： 新终端 source $SETUP 后 → python3 atom/demos/atom05_arm_moveit.py"
echo "  保持运行并退出界面： Ctrl+B 然后按 D"
sleep 1
exec tmux attach -t "$SESSION"
