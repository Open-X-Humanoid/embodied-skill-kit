#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原15 · 底盘（Chassis） —— 天轶 2.0 Pro 真机可运行版（思岚 MoveByAction REST）
配套讲解: atom/docs/atom15_chassis_guide.md

【为什么用 REST 而不是 /cmd_vel】
  比裸发 /cmd_vel 安全得多：
    - 限时动作：每个 MoveBy 指定 duration(ms)，到时底盘自己停，脚本崩了也不会飞车；
    - 走思岚运动控制（含其内建安全逻辑），不是开环裸速度；
    - 可软急停：DELETE /actions/:current 立刻取消当前动作。
  （裸 /cmd_vel 的 ROS2 原生变体仅供理解最底层速度控制，有飞车风险，见说明文档，不作首选。）

API（思岚 SLAMWARE 底盘 REST 接口）:
  base   : http://192.168.11.1:1448            (底盘管理口默认地址，如不通请核对底盘网络配置)
  下发   : POST   /api/core/motion/v1/actions
           body: {"action_name":"slamtec.agent.actions.MoveByAction",
                  "options":{"direction":0..3, "duration":ms}}
           direction: 0前进 1后退 2右转 3左转
  软急停 : DELETE /api/core/motion/v1/actions/:current
  位姿   : GET    /api/core/slam/v1/localization/pose   (读当前 x/y/yaw，底盘的"状态读取")

【状态读取】底盘的"状态"是它的位姿(x,y,yaw)。本脚本运动前后各读一次位姿并打印——
  既演示怎么读底盘状态，也能让你一眼看出"真的动了没"（思岚有时报成功却因离合/未使能没动）。

⚠ 前提：本脚本必须在【能连到底盘 192.168.11.x 子网】的机器人板子上跑（x86/Orin），
  不能在自己的笔记本上跑。运行前先架空或四周留 >1m、物理急停在手。依赖: requests。
"""

import sys
import time
import signal
import requests

BASE_URL = "http://192.168.11.1:1448"           # 如不通，核对 configs/nav.yaml 的 api_url
ACTIONS = f"{BASE_URL}/api/core/motion/v1/actions"
POSE    = f"{BASE_URL}/api/core/slam/v1/localization/pose"   # 读当前位姿(状态)
TIMEOUT = 5                                       # 单次 HTTP 超时(s)

DIRECTION = {"forward": 0, "backward": 1, "right": 2, "left": 3}

# —— 保守默认：每段都很短，到时自停 ——
PLAN = [
    ("forward", 1000),   # 前进 1.0s
    ("backward", 1000),  # 后退 1.0s
    ("left", 800),       # 左转 0.8s
    ("right", 800*2),    # 右转 1.6s (0.8s×2)
    ("left", 800),       # 左转 0.8s
]


def soft_stop(reason=""):
    """软急停：取消当前动作，底盘随即停止。"""
    try:
        r = requests.delete(f"{ACTIONS}/:current", timeout=TIMEOUT)
        print(f"[软急停] DELETE :current -> {r.status_code} {reason}")
    except Exception as e:
        print(f"[软急停] 失败: {e} —— 请立即按物理急停!", file=sys.stderr)


def get_pose():
    """读当前位姿 (x, y, yaw)——底盘的"状态读取"。失败返回 None（不影响运动）。"""
    try:
        r = requests.get(POSE, headers={"accept": "application/json"}, timeout=TIMEOUT)
        r.raise_for_status()
        d = r.json()
        return (d.get("x", 0.0), d.get("y", 0.0), d.get("yaw", 0.0))
    except Exception as e:
        print(f"[pose] 读取失败: {e}", file=sys.stderr)
        return None


def _signal_handler(signum, frame):
    soft_stop(f"(信号 {signum})")
    sys.exit(1)


def move_by(direction: str, duration_ms: int):
    assert direction in DIRECTION
    body = {
        "action_name": "slamtec.agent.actions.MoveByAction",
        "options": {"direction": DIRECTION[direction], "duration": int(duration_ms)},
    }
    r = requests.post(ACTIONS, json=body,
                      headers={"accept": "application/json", "Content-Type": "application/json"},
                      timeout=TIMEOUT)
    r.raise_for_status()
    action_id = r.json().get("action_id")
    print(f"  下发 {direction} {duration_ms}ms -> action_id={action_id}")
    # 动作限时自停，等它跑完(留 0.4s 余量)；期间 Ctrl-C 会触发软急停
    time.sleep(duration_ms / 1000.0 + 0.4)
    return action_id


def confirm():
    print("=" * 60)
    print("即将用思岚 MoveByAction 驱动底盘，计划:")
    for d, ms in PLAN:
        print(f"  - {d:8s} {ms} ms")
    print(f"目标底盘 API: {BASE_URL}")
    print("确认: 四周 >1m 无人无物，物理急停在手。")
    print("=" * 60)
    if input('确认请输入 GO 回车: ').strip() != "GO":
        print("已取消。")
        return False
    for i in range(3, 0, -1):
        print(f"  {i}...", flush=True)
        time.sleep(1)
    return True


def main():
    if not confirm():
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _signal_handler)
    print(f"运动前位姿 (x,y,yaw) = {get_pose()}")   # 状态读取：记下起点
    try:
        for direction, ms in PLAN:
            move_by(direction, ms)
        print("全部动作完成(均已限时自停)。")
        print(f"运动后位姿 (x,y,yaw) = {get_pose()}")  # 对比：位姿变了 = 真的动了
    except requests.exceptions.RequestException as e:
        print(f"HTTP 出错: {e}", file=sys.stderr)
        soft_stop("(HTTP 异常)")
    except Exception as e:
        print(f"出错: {e}", file=sys.stderr)
        soft_stop("(异常)")


if __name__ == "__main__":
    main()
