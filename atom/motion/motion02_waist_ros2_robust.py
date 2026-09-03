#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 运控2 · 腰部（Waist） —— ROS2 原生版 [生产级]，演示转腰（yaw）
配套讲解: atom/motion/docs/motion02_waist_guide.md

接口:
  控制: /waist/cmd_pos   bodyctrl_msgs/CmdSetMotorPosition
  反馈: /waist/status    bodyctrl_msgs/MotorStatusMsg
  腰部 2 个自由度:
     31 = yaw   绕竖直轴转腰(左右拧身)   ← 本 demo 演示这个，不影响身高/平衡
     32 = pitch 前倾俯仰(与下肢升降耦合)  ← 本 demo 只"保持"它，不主动动

⚠ 安全: 读到 /waist/status 才动(不盲发)；幅度 ±0.2rad≈±11°；急停在手。
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition, MotorStatusMsg

WAIST_CMD_TOPIC    = "/waist/cmd_pos"
WAIST_STATUS_TOPIC = "/waist/status"

WAIST_YAW_ID   = 31
WAIST_PITCH_ID = 32
CURRENT_LIMIT  = 20.0   # A，承重需要足够力矩
SPEED          = 0.5    # rad/s
YAW_AMP        = 0.2   # rad，左右各转这么多

# 软限位（rad）
YAW_LIMITS   = (-0.5, 0.5)    # ±29°
PITCH_LIMITS = (-0.3, 0.3)    # ±17°，pitch 与腿耦合，范围保守

PUB_READY_TIMEOUT = 3.0   # s
STATUS_TIMEOUT    = 5.0   # s


class WaistDemo(Node):
    def __init__(self):
        super().__init__("atom_waist_demo")
        self.pub = self.create_publisher(CmdSetMotorPosition, WAIST_CMD_TOPIC, 10)

        self._lock = threading.Lock()
        self.cur_pos: dict[int, float] = {}
        self.cur_err: dict[int, int] = {}

        # ★ 保存订阅引用（self.status_sub_）：与仓库其余示例风格统一，便于以后单独管理该订阅
        #   （rclpy 的 Node 自身已在 self._subscriptions 持有强引用，不赋值也不会被 GC 回收）。
        self.status_sub_ = self.create_subscription(
            MotorStatusMsg, WAIST_STATUS_TOPIC, self._on_status, 10)

        self.get_logger().info(
            f"腰部原子 demo 已启动，发布到 {WAIST_CMD_TOPIC}，订阅 {WAIST_STATUS_TOPIC}")

    def _on_status(self, msg: MotorStatusMsg) -> None:
        with self._lock:
            for s in msg.status:
                self.cur_pos[s.name] = s.pos
                # ★ Bug修复2: 同步记录电机错误码。
                #   原版只记录 pos，忽略 error 字段，无法在运动中检测电机故障。
                self.cur_err[s.name] = getattr(s, 'error', 0)

    # ── 辅助 ────────────────────────────────────────────────────────────────

    def wait_publisher_ready(self) -> bool:
        """
        ★ Bug修复3: 等待底层节点订阅连接，防止首条消息丢失。
        """
        t0 = time.time()
        while time.time() - t0 < PUB_READY_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pub.get_subscription_count() > 0:
                return True
        self.get_logger().warn(
            f"等待 {PUB_READY_TIMEOUT}s 后未检测到订阅者，消息可能丢失。")
        return False

    def wait_status(self, ids: list[int], timeout: float = STATUS_TIMEOUT) -> bool:
        """等到指定电机 ID 全部收到状态反馈。"""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if all(i in self.cur_pos for i in ids):
                    return True
        self.get_logger().error(
            f"超时 {timeout}s 未读到腰部状态，为安全起见不运动。")
        return False

    def _check_motor_errors(self) -> bool:
        """检查是否有电机错误码，有则返回 True。"""
        with self._lock:
            errors = {k: v for k, v in self.cur_err.items() if v not in (0, None)}
        if errors:
            self.get_logger().error(f"检测到电机错误码: {errors}，拒绝运动。")
            return True
        return False

    def _validate(self, yaw: float, pitch: float) -> bool:
        """软限位校验。"""
        if not (YAW_LIMITS[0] <= yaw <= YAW_LIMITS[1]):
            self.get_logger().error(
                f"yaw={yaw:.4f} 超出软限位 {YAW_LIMITS}，拒绝。")
            return False
        if not (PITCH_LIMITS[0] <= pitch <= PITCH_LIMITS[1]):
            self.get_logger().error(
                f"pitch={pitch:.4f} 超出软限位 {PITCH_LIMITS}，拒绝。")
            return False
        return True

    # ── 公开接口 ─────────────────────────────────────────────────────────────

    def command(self, yaw: float, pitch_hold: float, hold: float = 1.5) -> bool:
        """
        yaw=目标转腰角; pitch_hold=把 pitch 保持在此角度(通常=当前测量值)。
        返回 False 表示安全校验或电机错误未通过。

        ★ Bug修复4: hold 期间用 spin_once 代替 time.sleep。
        """
        if self._check_motor_errors():
            return False
        if not self._validate(yaw, pitch_hold):
            return False

        msg = CmdSetMotorPosition()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.cmds = [
            SetMotorPosition(
                name=WAIST_YAW_ID, pos=float(yaw), spd=SPEED, cur=CURRENT_LIMIT),
            SetMotorPosition(
                name=WAIST_PITCH_ID, pos=float(pitch_hold), spd=SPEED, cur=CURRENT_LIMIT),
        ]
        self.pub.publish(msg)
        self.get_logger().info(
            f"腰 yaw→{yaw:+.3f} (pitch 保持 {pitch_hold:+.3f}) rad")

        deadline = time.time() + hold
        while rclpy.ok():
            # remaining 可能因竞态变负（条件判断与此行之间时间已越过 deadline）；
            # rclpy 对负 timeout_sec 会永久阻塞，必须先判空再 spin。
            remaining = deadline - time.time()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

        return True


# ── 主程序 ────────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = WaistDemo()
    try:
        node.wait_publisher_ready()

        if not node.wait_status([WAIST_YAW_ID, WAIST_PITCH_ID]):
            return

        with node._lock:
            yaw0   = node.cur_pos[WAIST_YAW_ID]
            pitch0 = node.cur_pos[WAIST_PITCH_ID]
        node.get_logger().info(
            f"当前 yaw={yaw0:+.3f}, pitch={pitch0:+.3f} rad")

        if not node.command(yaw0 + YAW_AMP, pitch0):
            return
        if not node.command(yaw0 - YAW_AMP, pitch0):
            return
        node.command(yaw0, pitch0)

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
