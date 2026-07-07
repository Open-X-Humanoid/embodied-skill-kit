#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原15 · 底盘（Chassis） —— 天轶 2.0 Pro 真机可运行版 [生产级]
（思岚 MoveByAction REST）
配套讲解: atom/docs/atom15_chassis_guide.md

API（已与生产代码 navigate_controller.py + configs/nav.yaml 核对）:
  base   : http://192.168.11.1:1448
  下发   : POST   /api/core/motion/v1/actions
  软急停 : DELETE /api/core/motion/v1/actions/:current
  位姿   : GET    /api/core/slam/v1/localization/pose   (运动前后自检用)
  direction: 0前进 1后退 2右转 3左转

★ 运动前后 pose 自检：action 回 result:0 只代表"思岚固件规划执行完毕"，
  不代表轮子真的转了。若手动推行/离合开关按下、电机未使能或急停未松，
  会"报成功却纹丝不动"。本版发完指令后比对真实位姿变化，几乎没动就告警。

⚠ 前提：在能连到底盘 192.168.11.x 子网的机器人板子上跑。
  运行前先架空或四周留 >1m、物理急停在手。依赖: requests。
"""

import sys
import time
import math
import signal

import requests

BASE_URL = "http://192.168.11.1:1448"
ACTIONS  = f"{BASE_URL}/api/core/motion/v1/actions"
POSE     = f"{BASE_URL}/api/core/slam/v1/localization/pose"
TIMEOUT  = 5    # 单次 HTTP 超时(s)
RETRIES  = 2    # HTTP 失败重试次数

DIRECTION = {"forward": 0, "backward": 1, "right": 2, "left": 3}

# 运动自检阈值：本该移动却低于此值 → 疑似离合按下/电机未使能/急停
MIN_MOVE_M  = 0.02    # 直线(前进/后退)最小期望位移(m)
MIN_ROT_RAD = 0.03    # 转向(左转/右转)最小期望转角(rad ≈ 1.7°)

PLAN = [
    ("forward",  1000),     # 前进 1.0s
    ("backward", 1000),     # 后退 1.0s
    ("left",      800),     # 左转 0.8s
    ("right", 800*2),       # 右转 0.8s * 2
    ("left", 800),          # 左转 0.8s
]

# ── 全局急停状态 ──────────────────────────────────────────────────────────────
_stop_requested = False


def soft_stop(reason: str = "") -> None:
    """软急停：取消当前动作，底盘随即停止。"""
    try:
        r = requests.delete(f"{ACTIONS}/:current", timeout=TIMEOUT)
        print(f"[软急停] DELETE :current -> HTTP {r.status_code} {reason}")
    except Exception as e:
        print(f"[软急停] HTTP 失败: {e} —— 请立即按物理急停!", file=sys.stderr)


def get_pose():
    """读当前位姿 (x, y, yaw)；失败返回 None（不影响主流程，只是跳过自检）。"""
    try:
        r = requests.get(POSE, headers={"accept": "application/json"}, timeout=TIMEOUT)
        r.raise_for_status()
        d = r.json()
        return (d.get("x", 0.0), d.get("y", 0.0), d.get("yaw", 0.0))
    except Exception as e:
        print(f"[pose] 读取失败: {e}（跳过本段运动自检）", file=sys.stderr)
        return None


def _ang_diff(a: float, b: float) -> float:
    """最小角差，归一化到 [-pi, pi]（避免 yaw 跨 ±pi 时误判）。"""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def _check_moved(direction: str, pose_before) -> None:
    """
    ★ Bug修复5: 运动前后 pose 自检。
      action 回 result:0 只代表"思岚固件规划执行完毕"，不代表轮子真的转了；
      离合(手动推行)按下/电机未使能/急停未松时会"报成功却不动"。
      用真实位姿变化戳穿这种假象——pose 是里程/定位，骗不了人。
    """
    if pose_before is None:
        return
    pose_after = get_pose()
    if pose_after is None:
        return

    x0, y0, yaw0 = pose_before
    x1, y1, yaw1 = pose_after

    if direction in ("forward", "backward"):
        moved = math.hypot(x1 - x0, y1 - y0)
        ok = moved >= MIN_MOVE_M
        thresh = f"< {MIN_MOVE_M * 100:.0f} cm"
        print(f"  [自检] 位移 {moved * 100:.1f} cm" + ("" if ok else "  ⚠"))
    else:  # left / right —— 原地转向，看 yaw 变化
        moved = abs(_ang_diff(yaw1, yaw0))
        ok = moved >= MIN_ROT_RAD
        thresh = f"< {math.degrees(MIN_ROT_RAD):.1f}°"
        print(f"  [自检] 转角 {math.degrees(moved):.1f}°" + ("" if ok else "  ⚠"))

    if not ok:
        print(f"  ⚠ 指令报成功但几乎没动（{thresh}）——疑似：手动推行/离合开关按下、"
              f"电机未使能、或急停未松开。请检查底盘后重试。", file=sys.stderr)


def _signal_handler(signum, frame) -> None:
    """
    ★ Bug修复1: 改用标志位而不是直接在信号处理器里执行副作用。
      原版在信号处理器中直接调用 soft_stop() + sys.exit()，
      若信号在 requests.post() 的 socket 等待中到达，可能造成不可预期的行为。
      改为设置标志位，由主循环检测并优雅退出。
    """
    global _stop_requested
    _stop_requested = True
    print(f"\n[信号 {signum}] 已请求急停，等待当前 HTTP 完成后退出...", file=sys.stderr)


# ★ Bug修复2: 信号注册移到 confirm() 之前（原版在 confirm() 内含 3s 倒计时，
#   期间 Ctrl-C 只会抛 KeyboardInterrupt，不会触发 soft_stop）。
#   现在在模块级注册，覆盖整个运行周期。
for _sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(_sig, _signal_handler)


def _http_post_with_retry(body: dict) -> requests.Response:
    """
    ★ Bug修复3: 带重试的 HTTP POST。
      原版单次失败直接 raise，网络瞬断就整体退出。
      加入简单重试逻辑，对 5xx / 连接错误重试 RETRIES 次。
    """
    last_exc = None
    for attempt in range(1, RETRIES + 2):
        try:
            r = requests.post(
                ACTIONS, json=body,
                headers={"accept": "application/json",
                         "Content-Type": "application/json"},
                timeout=TIMEOUT,
            )
            if r.status_code < 500:
                return r   # 4xx 也返回，让调用方决定是否 raise_for_status
            last_exc = requests.exceptions.HTTPError(
                f"HTTP {r.status_code}", response=r)
        except (requests.exceptions.ConnectionError,
                requests.exceptions.Timeout) as e:
            last_exc = e
        if attempt <= RETRIES:
            wait = 0.5 * attempt
            print(f"  [重试 {attempt}/{RETRIES}] {last_exc}，{wait}s 后重试...")
            time.sleep(wait)
    raise last_exc


def move_by(direction: str, duration_ms: int) -> None:
    """发送一次限时运动指令，等待执行完成（含 0.4s 余量）。"""
    if direction not in DIRECTION:
        raise ValueError(f"未知方向: {direction}，合法值: {list(DIRECTION)}")
    if duration_ms <= 0:
        raise ValueError(f"duration_ms 必须为正数，得到: {duration_ms}")

    body = {
        "action_name": "slamtec.agent.actions.MoveByAction",
        "options": {
            "direction": DIRECTION[direction],
            "duration":  int(duration_ms),
        },
    }
    pose_before = get_pose()   # 运动前位姿（用于事后自检）

    r = _http_post_with_retry(body)
    r.raise_for_status()

    resp = r.json()
    action_id = resp.get("action_id")
    print(f"  下发 {direction} {duration_ms}ms -> action_id={action_id}")

    # 等待动作执行（限时自停，0.4s 余量）
    wait_sec = duration_ms / 1000.0 + 0.4
    deadline = time.time() + wait_sec
    while time.time() < deadline:
        if _stop_requested:
            soft_stop("(信号中断)")
            sys.exit(1)
        time.sleep(0.05)

    # 运动前后 pose 自检：报成功却几乎没动 → 告警（离合按下/未使能/急停）
    _check_moved(direction, pose_before)


def confirm() -> bool:
    print("=" * 60)
    print("即将用思岚 MoveByAction 驱动底盘，计划:")
    for d, ms in PLAN:
        print(f"  - {d:8s} {ms} ms")
    print(f"目标底盘 API: {BASE_URL}")
    print("确认: 四周 >1m 无人无物，物理急停在手。")
    print("=" * 60)
    try:
        answer = input("确认请输入 GO 回车: ").strip()
    except (EOFError, KeyboardInterrupt):
        print("\n已取消。")
        return False
    if answer != "GO":
        print("已取消。")
        return False
    for i in range(3, 0, -1):
        if _stop_requested:
            print("急停信号，取消。")
            return False
        print(f"  {i}...", flush=True)
        time.sleep(1)
    return True


def main() -> None:
    # ★ Bug修复4: confirm() 执行时信号已注册（见模块级），
    #   倒计时期间 Ctrl-C 会设置 _stop_requested，confirm() 内检测到后返回 False。
    if not confirm():
        return

    try:
        for direction, ms in PLAN:
            if _stop_requested:
                soft_stop("(循环检测到急停标志)")
                return
            move_by(direction, ms)
        print("全部动作完成(均已限时自停)。")

    except requests.exceptions.RequestException as e:
        print(f"HTTP 出错: {e}", file=sys.stderr)
        soft_stop("(HTTP 异常)")
        sys.exit(1)
    except Exception as e:
        print(f"出错: {e}", file=sys.stderr)
        soft_stop("(未知异常)")
        sys.exit(1)


if __name__ == "__main__":
    main()
