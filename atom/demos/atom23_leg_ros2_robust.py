#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原23 · 下肢升降/下蹲（Leg，腰+腿协调）—— ROS2 原生版 [生产级]
【承重，务必先读】
配套讲解: atom/docs/atom23_leg_guide.md

升降不是纯腿单关节，而是 hip+knee+腰pitch 三关节协调（整个上身压在升降腿上）:
  腿 /leg/cmd_pos :  51=hip(髋)  52=knee(膝)
  腰 /waist/cmd_pos: 32=pitch(前倾，参与升降)  31=yaw(转腰，本 demo 保持不动)

升降方向（各关节增量近似相等）:
  下蹲(变矮): hip↓, knee↓, waist↑    抬升(变高): hip↑, knee↑, waist↓

本 demo 的安全策略（关键）:
  1) 必须先读到 /leg/status 和 /waist/status，读不到绝不运动；
  2) 只做"从当前姿态出发的极小协调位移"(默认 0.08rad) 再原路返回；
  3) 慢速、分多步插值(每 50ms 一步)，小步逼近；
  4) cur 用关节电流上限(20A)，腿才有力托住上身；
  5) 急停/异常时"带力保持当前位置"(spd=0, cur=电流上限)，绝不把电流设 0
     —— 承重关节电流设 0 = 腿失力 = 上身坠落！

⚠ 运行前: 天轶为轮式底座、横向已稳；底座停稳、升降空间内无人无物、急停在手。
  大幅度升降或配合手臂大幅伸展时才加额外防护。
"""

import sys
import threading
import time
import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition, MotorStatusMsg

LEG_CMD_TOPIC, LEG_STATUS_TOPIC     = "/leg/cmd_pos",   "/leg/status"
WAIST_CMD_TOPIC, WAIST_STATUS_TOPIC = "/waist/cmd_pos", "/waist/status"

HIP_ID         = 51
KNEE_ID        = 52
WAIST_PITCH_ID = 32
WAIST_YAW_ID   = 31

CURRENT_LIMIT = 20.0   # A，承重必须够大
STEP_DT       = 0.05   # s，控制周期
MOVE_TIME     = 4.0    # s，单程用时（越长越慢越稳）
STEP_SPEED    = 0.3    # rad/s，每步指令速度
SQUAT_DELTA   = 0.08   # rad，下蹲幅度（协调增量），很小
SAFETY_CAP    = 0.15   # rad，单关节单次位移硬上限

STATUS_TIMEOUT = 5.0   # s


class LegDemo(Node):
    def __init__(self):
        super().__init__("atom_leg_demo")
        self.leg_pub   = self.create_publisher(CmdSetMotorPosition, LEG_CMD_TOPIC,   10)
        self.waist_pub = self.create_publisher(CmdSetMotorPosition, WAIST_CMD_TOPIC, 10)

        self._lock = threading.Lock()
        self.leg_pos:   dict[int, float] = {}
        self.waist_pos: dict[int, float] = {}
        self.leg_err:   dict[int, int]   = {}
        self.waist_err: dict[int, int]   = {}
        self._last: tuple | None = None  # 最近下发的 (hip, knee, waist_pitch, yaw)

        # ★ 保存两个订阅的引用（self.leg_sub_ / self.waist_sub_）：
        #   与仓库其余示例风格统一，便于以后单独管理该订阅
        #   （rclpy 的 Node 自身已在 self._subscriptions 持有强引用，不赋值也不会被 GC 回收）。
        self.leg_sub_   = self.create_subscription(
            MotorStatusMsg, LEG_STATUS_TOPIC,   self._on_leg,   10)
        self.waist_sub_ = self.create_subscription(
            MotorStatusMsg, WAIST_STATUS_TOPIC, self._on_waist, 10)

        self.get_logger().info("下肢升降原子 demo 已启动")

    # ── 状态回调 ─────────────────────────────────────────────────────────────

    def _on_leg(self, msg: MotorStatusMsg) -> None:
        with self._lock:
            for s in msg.status:
                self.leg_pos[s.name] = s.pos
                self.leg_err[s.name] = getattr(s, 'error', 0)

    def _on_waist(self, msg: MotorStatusMsg) -> None:
        with self._lock:
            for s in msg.status:
                self.waist_pos[s.name] = s.pos
                self.waist_err[s.name] = getattr(s, 'error', 0)

    # ── 辅助 ────────────────────────────────────────────────────────────────

    def wait_status(self, timeout: float = STATUS_TIMEOUT) -> bool:
        need = [
            (self.leg_pos,   HIP_ID),
            (self.leg_pos,   KNEE_ID),
            (self.waist_pos, WAIST_PITCH_ID),
            (self.waist_pos, WAIST_YAW_ID),
        ]
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if all(i in d for d, i in need):
                    return True
        return False

    def current(self) -> tuple[float, float, float, float]:
        with self._lock:
            return (
                self.leg_pos[HIP_ID],
                self.leg_pos[KNEE_ID],
                self.waist_pos[WAIST_PITCH_ID],
                self.waist_pos[WAIST_YAW_ID],
            )

    def motor_error(self) -> tuple[bool, object]:
        with self._lock:
            for d in (self.leg_err, self.waist_err):
                for name, err in d.items():
                    if err not in (0, None):
                        return True, (name, err)
        return False, None

    def _publish(self, hip: float, knee: float,
                 waist: float, yaw: float, spd: float) -> None:
        with self._lock:
            self._last = (hip, knee, waist, yaw)

        leg = CmdSetMotorPosition(
            header=Header(stamp=self.get_clock().now().to_msg()))
        leg.cmds = [
            SetMotorPosition(
                name=HIP_ID,  pos=float(hip),  spd=float(spd), cur=CURRENT_LIMIT),
            SetMotorPosition(
                name=KNEE_ID, pos=float(knee), spd=float(spd), cur=CURRENT_LIMIT),
        ]
        wst = CmdSetMotorPosition(
            header=Header(stamp=self.get_clock().now().to_msg()))
        wst.cmds = [
            SetMotorPosition(
                name=WAIST_PITCH_ID, pos=float(waist), spd=float(spd), cur=CURRENT_LIMIT),
            SetMotorPosition(
                name=WAIST_YAW_ID,   pos=float(yaw),   spd=0.3,        cur=CURRENT_LIMIT),
        ]
        self.leg_pub.publish(leg)
        self.waist_pub.publish(wst)

    def hold(self) -> None:
        """带力保持当前(最近下发)位置 —— 承重关节的正确"停止"方式。"""
        with self._lock:
            last = self._last
        if last is None:
            self.get_logger().warn("hold(): 尚未下发过指令，无法保持位置。")
            return
        hip, knee, waist, yaw = last
        self._publish(hip, knee, waist, yaw, spd=0.0)
        self.get_logger().warn("已带力保持当前姿态（未瘫软）。")

    # ── 运动接口 ─────────────────────────────────────────────────────────────

    def move_to(self, hip_t: float, knee_t: float,
                waist_t: float, yaw_hold: float,
                move_time: float = MOVE_TIME) -> bool:
        """从当前测量姿态线性插值到目标，小步慢速；期间检查电机错误。"""
        hip0, knee0, waist0, _ = self.current()

        # 安全幅度检查
        for label, delta in (
            ("hip",   hip_t   - hip0),
            ("knee",  knee_t  - knee0),
            ("waist", waist_t - waist0),
        ):
            if abs(delta) > SAFETY_CAP:
                self.get_logger().error(
                    f"{label} 位移 {delta:+.4f} rad 超过安全上限 "
                    f"{SAFETY_CAP} rad，拒绝运动。")
                return False

        n = max(1, int(move_time / STEP_DT))
        for k in range(1, n + 1):
            err, info = self.motor_error()
            if err:
                self.get_logger().error(
                    f"检测到电机错误 {info}，立即带力保持并退出。")
                self.hold()
                return False

            r = k / n
            self._publish(
                hip0   + (hip_t   - hip0)   * r,
                knee0  + (knee_t  - knee0)  * r,
                waist0 + (waist_t - waist0) * r,
                yaw_hold,
                spd=STEP_SPEED,
            )
            # spin_once 保持状态回调活跃（原版用的 timeout_sec=0.0 可能不会真正处理消息）
            rclpy.spin_once(self, timeout_sec=STEP_DT * 0.5)
            time.sleep(STEP_DT * 0.5)

        return True


# ── 全局节点引用（用于信号处理器） ───────────────────────────────────────────
_node: LegDemo | None = None
_stop_flag = threading.Event()


def _sig(signum, frame) -> None:
    """
    ★ Bug修复2: 信号处理器改为仅设置标志位 + 调用 hold()，不调用 rclpy.shutdown()。
      原版在信号处理器中直接调用 rclpy.shutdown()，若此时主线程正在 spin_once()
      的 C++ 层内，shutdown() 会触发资源析构竞争，可能导致死锁或 segfault。
      正确做法：信号处理器只做最小操作（hold + 设标志），主循环检测标志后优雅退出。
    """
    _stop_flag.set()
    if _node is not None:
        try:
            _node.hold()
        except Exception:
            pass  # 信号上下文中不能抛异常
    print(f"\n[信号 {signum}] 已带力保持，主循环即将退出...", file=sys.stderr)


def confirm() -> bool:
    print("=" * 60)
    print(f"下肢升降 demo: 将从当前姿态下蹲约 {SQUAT_DELTA} rad(几 cm)再原路抬回。")
    print("整机重量压在腿上 —— 确认: 站稳、脚下四周无人无物、急停在手。")
    print("=" * 60)
    try:
        answer = input("确认请输入 GO 回车: ").strip()
    except (EOFError, KeyboardInterrupt):
        print("\n已取消。")
        return False
    if answer != "GO":
        print("已取消。")
        return False
    for i in range(5, 0, -1):
        if _stop_flag.is_set():
            return False
        print(f"  {i}...", flush=True)
        time.sleep(1)
    return True


def main() -> None:
    global _node

    if not confirm():
        return

    rclpy.init()
    _node = LegDemo()

    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, _sig)

    try:
        if not _node.wait_status():
            _node.get_logger().error(
                "读不到 /leg/status 或 /waist/status，为安全不运动。")
            return

        hip0, knee0, waist0, yaw0 = _node.current()
        _node.get_logger().info(
            f"起始 hip={hip0:+.4f} knee={knee0:+.4f} "
            f"waist={waist0:+.4f} yaw={yaw0:+.4f} rad")

        if _stop_flag.is_set():
            return

        d = SQUAT_DELTA
        # 下蹲: hip↓ knee↓ waist↑
        if not _node.move_to(hip0 - d, knee0 - d, waist0 + d, yaw0):
            _node.hold()
            return

        if _stop_flag.is_set():
            _node.hold()
            return

        time.sleep(1.0)

        # 抬回起点
        if not _node.move_to(hip0, knee0, waist0, yaw0):
            _node.hold()
            return

        # ★ Bug修复3: 结束时明确调用 hold()，保持带力支撑（原版同样做了，保留）。
        _node.hold()
        _node.get_logger().info("完成，已回到起始姿态并带力保持。")

    except Exception as exc:
        _node.get_logger().error(f"未处理异常: {exc}")
        if _node is not None:
            _node.hold()
        raise
    finally:
        # ★ Bug修复4: 补充 destroy_node()，原版完全缺失此调用，导致节点资源泄漏。
        if _node is not None:
            _node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
