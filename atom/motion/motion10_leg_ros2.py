#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 运控10 · 下肢升降/下蹲（Leg，腰+腿协调）—— ROS2 原生版【承重，务必先读】
配套讲解: atom/motion/docs/motion10_leg_guide.md

升降不是纯腿单关节，而是 hip+knee+腰pitch 三关节协调（整个上身压在升降腿上）:
  腿 /leg/cmd_pos :  51=hip(髋)  52=knee(膝)
  腰 /waist/cmd_pos: 32=pitch(前倾，参与升降)  31=yaw(转腰，本 demo 保持不动)
  下蹲(变矮): hip↓ knee↓ waist↑    抬升(变高): hip↑ knee↑ waist↓

安全核心（三条，缺一不可）:
  1) 先读到 /leg/status + /waist/status 才动——从当前姿态出发，不跳到标定值；
  2) 小步慢速插值(每 50ms 一步，默认 4s 走完)，只做 0.08rad 级极小位移再原路返回；
  3) 停止 = 带力保持当前位置(spd=0, cur=20A)。★绝不把 cur 设 0
     —— 承重关节 cur=0 = 腿失力 = 上身坠落砸下来！
  （生产版 motion10_leg_ros2_robust.py 另加：电机错误自检、位移硬上限、线程锁、标志位信号。）

⚠ 运行前: 天轶为轮式底座、横向已稳；底座停稳、升降空间内无人无物、急停在手。
  大幅度升降或配合手臂大幅伸展时才加额外防护。
"""

import sys
import time
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition, MotorStatusMsg

LEG_CMD,   LEG_STATUS   = "/leg/cmd_pos",   "/leg/status"
WAIST_CMD, WAIST_STATUS = "/waist/cmd_pos", "/waist/status"
HIP, KNEE, WAIST_PITCH, WAIST_YAW = 51, 52, 32, 31

CURRENT_LIMIT = 20.0   # A，承重必须够大
MOVE_TIME     = 4.0    # s，单程用时（越长越慢越稳）
STEP_DT       = 0.05   # s，每步间隔
STEP_SPEED    = 0.3    # rad/s，每步指令速度
SQUAT_DELTA   = 0.08   # rad，下蹲幅度（协调增量），很小，别乱调大


class LegDemo(Node):
    def __init__(self):
        super().__init__("atom_leg_demo")
        self.leg_pub   = self.create_publisher(CmdSetMotorPosition, LEG_CMD,   10)
        self.waist_pub = self.create_publisher(CmdSetMotorPosition, WAIST_CMD, 10)
        self.pos = {}        # {电机ID: 当前角}，腿(51/52)+腰(31/32) 合一起存
        self._last = None    # 最近下发的 (hip, knee, waist, yaw)，停止时保持它
        # 腿和腰用同一个回调——电机 ID 各不相同，存进同一个字典不冲突
        self.leg_sub_   = self.create_subscription(MotorStatusMsg, LEG_STATUS,   self._on_status, 1)
        self.waist_sub_ = self.create_subscription(MotorStatusMsg, WAIST_STATUS, self._on_status, 1)
        self.get_logger().info("下肢升降原子 demo 已启动")

    def _on_status(self, msg):
        for s in msg.status:
            self.pos[s.name] = s.pos

    def wait_status(self, timeout=3.0):
        """等到 hip/knee/waist_pitch/waist_yaw 四个当前角都读到才返回。"""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(i in self.pos for i in (HIP, KNEE, WAIST_PITCH, WAIST_YAW)):
                return True
        return False

    def _publish(self, hip, knee, waist, yaw, spd):
        """把一组目标角分别发到腿、腰两个话题；同时记下作为'保持'的目标。"""
        self._last = (hip, knee, waist, yaw)
        now = self.get_clock().now().to_msg()
        leg = CmdSetMotorPosition(header=Header(stamp=now))
        leg.cmds = [SetMotorPosition(name=HIP,  pos=float(hip),  spd=float(spd), cur=CURRENT_LIMIT),
                    SetMotorPosition(name=KNEE, pos=float(knee), spd=float(spd), cur=CURRENT_LIMIT)]
        wst = CmdSetMotorPosition(header=Header(stamp=now))
        wst.cmds = [SetMotorPosition(name=WAIST_PITCH, pos=float(waist), spd=float(spd), cur=CURRENT_LIMIT),
                    SetMotorPosition(name=WAIST_YAW,   pos=float(yaw),   spd=0.3,        cur=CURRENT_LIMIT)]
        self.leg_pub.publish(leg)
        self.waist_pub.publish(wst)

    def hold(self):
        """★带力保持最近下发的姿态(spd=0, cur=20A)——承重关节的正确'停止'，绝不瘫软。"""
        if self._last is not None:
            self._publish(*self._last, spd=0.0)
            self.get_logger().warn("已带力保持当前姿态（未瘫软）。")

    def move_to(self, hip_t, knee_t, waist_t, yaw, move_time=MOVE_TIME):
        """从当前测量姿态分多步线性插值慢慢逼近目标，绝不跳变。"""
        hip0, knee0, waist0 = self.pos[HIP], self.pos[KNEE], self.pos[WAIST_PITCH]
        n = max(1, int(move_time / STEP_DT))
        for k in range(1, n + 1):
            r = k / n
            self._publish(hip0   + (hip_t   - hip0)   * r,
                          knee0  + (knee_t  - knee0)  * r,
                          waist0 + (waist_t - waist0) * r,
                          yaw, spd=STEP_SPEED)
            # 边走边 spin，让 self.pos 保持刷新——否则本方法结束后 self.pos 会停留在
            # "刚进入这次 move_to 时"的旧值，下一次 move_to（比如回程）会读到过时的
            # 当前姿态，插值计算出错，导致该走的斜坡变成一步到位（动作变猛变快）。
            rclpy.spin_once(self, timeout_sec=STEP_DT * 0.5)
            time.sleep(STEP_DT * 0.5)


_node = None


def _sig(signum, frame):
    if _node is not None:
        _node.hold()          # ★中断也要带力保持，绝不松力坍塌
    rclpy.shutdown()
    sys.exit(1)


def confirm():
    print("=" * 60)
    print(f"下肢升降 demo: 从当前姿态下蹲约 {SQUAT_DELTA}rad(几 cm)再原路抬回。")
    print("整机重量压在腿上 —— 确认: 站稳、脚下四周无人无物、急停在手。")
    print("=" * 60)
    if input("确认请输入 GO 回车: ").strip() != "GO":
        print("已取消。")
        return False
    for i in range(5, 0, -1):
        print(f"  {i}...", flush=True)
        time.sleep(1)
    return True


def main():
    global _node
    if not confirm():
        return
    rclpy.init()
    _node = LegDemo()
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, _sig)
    try:
        if not _node.wait_status():
            _node.get_logger().error("读不到 /leg/status 或 /waist/status，为安全不运动。")
            return
        hip0, knee0, waist0, yaw0 = (_node.pos[HIP], _node.pos[KNEE],
                                     _node.pos[WAIST_PITCH], _node.pos[WAIST_YAW])
        _node.get_logger().info(f"起始 hip={hip0:+.3f} knee={knee0:+.3f} waist={waist0:+.3f}")

        d = SQUAT_DELTA
        _node.move_to(hip0 - d, knee0 - d, waist0 + d, yaw0)   # 下蹲: hip↓ knee↓ waist↑
        time.sleep(1.0)
        _node.move_to(hip0, knee0, waist0, yaw0)               # 抬回起点
        _node.hold()                                           # 结束也带力保持，不松力
        _node.get_logger().info("完成，已回到起始姿态并带力保持。")
    finally:
        if _node is not None:
            _node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
