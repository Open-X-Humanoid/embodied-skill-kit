#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原1 · 头部（Head） —— ROS2 原生版 [生产级]
让机器人点头（pitch）、摇头（yaw）。   配套讲解文档: atom/docs/atom01_head_guide.md

接口（位置模式）:
  话题: /head/cmd_pos     类型: bodyctrl_msgs/CmdSetMotorPosition
        SetMotorPosition[] cmds:  name(电机ID)  pos(rad)  spd(rad/s)  cur(最大电流A)
  电机ID: 1=roll  2=pitch(低头/抬头)  3=yaw(左右转头)   单位: 弧度(rad)
  状态反馈: /head/status   bodyctrl_msgs/MotorStatusMsg

⚠ 安全: 角度用弧度、幅度保守(±0.3rad≈±17°)、速度放慢；执行时人不要靠近机器人。
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition, MotorStatusMsg

HEAD_CMD_TOPIC    = "/head/cmd_pos"
HEAD_STATUS_TOPIC = "/head/status"

MAX_SPEED = 0.5              # rad/s
MAX_CUR   = [1.0, 1.0, 1.0] # [roll, pitch, yaw] 最大电流(A)

# 头部关节软限位（rad），避免撞机械限位
# roll=1, pitch=2, yaw=3
HEAD_SOFT_LIMITS: dict[int, tuple[float, float]] = {
    1: (-0.30, 0.30),   # roll  ±17°
    2: (-0.40, 0.40),   # pitch 低头/抬头 ±23°
    3: (-0.60, 0.60),   # yaw   左右 ±34°
}

PUB_READY_TIMEOUT = 3.0   # s
STATUS_TIMEOUT    = 5.0   # s
REACH_TOL         = 0.05  # rad, 到位容差(~3°)
REACH_TIMEOUT     = 4.0   # s, 等到位最长时间


class HeadDemo(Node):
    def __init__(self):
        super().__init__("atom_head_demo")
        self.pub = self.create_publisher(CmdSetMotorPosition, HEAD_CMD_TOPIC, 10)

        self._lock = threading.Lock()
        self.cur_pos: dict[int, float] = {}

        # ★ Bug修复1: 原版没有订阅状态反馈，导致第一条指令是盲发的（不知道头当前在哪里）。
        #   这里补上状态订阅，让生产代码能先读当前姿态、再从当前姿态出发，避免头部急剧跳变。
        #   （赋值给 self.status_sub_ 是风格约定，便于以后单独管理该订阅；
        #    rclpy 的 Node 自身已在 self._subscriptions 中持有强引用，不赋值也不会被 GC 回收。）
        self.status_sub_ = self.create_subscription(
            MotorStatusMsg, HEAD_STATUS_TOPIC, self._on_status, 10)

        self.get_logger().info(
            f"头部原子 demo 已启动，发布到 {HEAD_CMD_TOPIC}，订阅 {HEAD_STATUS_TOPIC}")

    def _on_status(self, msg: MotorStatusMsg) -> None:
        with self._lock:
            for s in msg.status:
                self.cur_pos[s.name] = s.pos

    # ── 辅助 ────────────────────────────────────────────────────────────────

    def wait_publisher_ready(self) -> bool:
        """
        ★ Bug修复2: 等待底层节点订阅连接。
          publish() 非阻塞；对端未 subscribe 时消息直接丢失，首帧头部不动也不报错。
        """
        t0 = time.time()
        while time.time() - t0 < PUB_READY_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pub.get_subscription_count() > 0:
                return True
        self.get_logger().warn(
            f"等待 {PUB_READY_TIMEOUT}s 后未检测到订阅者，消息可能丢失。")
        return False

    def wait_for_status(self, timeout: float = STATUS_TIMEOUT) -> bool:
        """等到三个头部关节都收到状态反馈。"""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if all(i in self.cur_pos for i in (1, 2, 3)):
                    return True
        self.get_logger().error(
            f"超时 {timeout}s 未读到头部完整状态反馈，拒绝运动。")
        return False

    def wait_until_reached(self, targets: dict[int, float],
                           tol: float = REACH_TOL,
                           timeout: float = REACH_TIMEOUT) -> bool:
        """
        ★ 到位检查(闭环)：轮询状态反馈，等所有目标关节进入 目标±tol 才返回。
          比"固定等 N 秒"可靠——真到位才继续；超时(负载重/未达容差)返回 False 并告警。
        """
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                if all(abs(self.cur_pos.get(mid, 1e9) - tgt) <= tol
                       for mid, tgt in targets.items()):
                    return True
        self.get_logger().warn(
            f"超时 {timeout}s 未确认到位（负载重 / 未达容差 {tol}rad）。")
        return False

    def _validate(self, roll: float, pitch: float, yaw: float) -> bool:
        """
        ★ Bug修复3: 目标角度软限位校验。
          原版直接发布任意角度，超限时会撞机械限位或触发过流保护。
        """
        for motor_id, val, name in ((1, roll, "roll"), (2, pitch, "pitch"), (3, yaw, "yaw")):
            lo, hi = HEAD_SOFT_LIMITS[motor_id]
            if not (lo <= val <= hi):
                self.get_logger().error(
                    f"头部 {name}={val:.4f} rad 超出软限位 [{lo:.4f}, {hi:.4f}]，拒绝。")
                return False
        return True

    # ── 公开接口 ─────────────────────────────────────────────────────────────

    def move_to(self, roll: float, pitch: float, yaw: float,
                hold: float = 1.5) -> bool:
        """
        移动头部到 (roll, pitch, yaw)，等到位后保持 hold 秒。
        返回 False = 安全校验未过 / 超时未到位；True = 已到位。

        ★ Bug修复4: hold 期间用 spin_once 代替 time.sleep，
          保持状态反馈回调的活跃，cur_pos 持续更新。
        """
        if not self._validate(roll, pitch, yaw):
            return False

        msg = CmdSetMotorPosition()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.cmds = [
            SetMotorPosition(name=1, pos=float(roll),  spd=MAX_SPEED, cur=MAX_CUR[0]),
            SetMotorPosition(name=2, pos=float(pitch), spd=MAX_SPEED, cur=MAX_CUR[1]),
            SetMotorPosition(name=3, pos=float(yaw),   spd=MAX_SPEED, cur=MAX_CUR[2]),
        ]
        self.pub.publish(msg)
        self.get_logger().info(
            f"头部 → roll={roll:+.3f} pitch={pitch:+.3f} yaw={yaw:+.3f} rad")

        # ★ Bug修复5: 到位检查(闭环)——发完不是"固定等 hold 秒"就完事，
        #   而是轮询状态反馈、等三关节真正进入目标±容差；返回值即"是否到位"。
        reached = self.wait_until_reached({1: roll, 2: pitch, 3: yaw})

        # 到位后再保持 hold 秒便于观察姿态（spin_once 保活回调，不用 sleep）
        deadline = time.time() + hold
        while rclpy.ok():
            # remaining 可能因竞态变负（条件判断与此行之间时间已越过 deadline）；
            # rclpy 对负 timeout_sec 会永久阻塞，必须先判空再 spin。
            remaining = deadline - time.time()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

        return reached


# ── 主程序 ────────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = HeadDemo()
    try:
        node.wait_publisher_ready()

        # ★ Bug修复5: 先读当前状态，而不是盲发 (0,0,0)。
        #   原版直接发 move_to(0,0,0)，若头当前在大角度处，会产生急剧跳变。
        if not node.wait_for_status():
            return

        with node._lock:
            cur_r = node.cur_pos.get(1, 0.0)
            cur_p = node.cur_pos.get(2, 0.0)
            cur_y = node.cur_pos.get(3, 0.0)
        node.get_logger().info(
            f"头部当前姿态: roll={cur_r:+.3f} pitch={cur_p:+.3f} yaw={cur_y:+.3f} rad")

        # 从当前姿态回正，再做演示动作
        node.move_to(0.0, 0.0, 0.0)
        node.move_to(0.0,  0.25, 0.0)   # 低头
        node.move_to(0.0, -0.15, 0.0)   # 抬头
        node.move_to(0.0,  0.0,  0.3)   # 向左转头
        node.move_to(0.0,  0.0, -0.3)   # 向右转头
        node.move_to(0.0,  0.0,  0.0)   # 回正

        node.get_logger().info("演示完成。")

    except KeyboardInterrupt:
        node.get_logger().warn("用户 Ctrl-C 中断。")
    except Exception as exc:
        node.get_logger().error(f"未处理异常: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
