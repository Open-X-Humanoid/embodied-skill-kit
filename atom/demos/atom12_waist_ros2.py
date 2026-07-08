#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原12 · 腰部（Waist） —— ROS2 原生版，演示转腰（yaw）
配套讲解: atom/docs/atom12_waist_guide.md

【版本说明】本文件是 ROS2 原生实现（/waist/cmd_pos 位置模式直发）。若走 xRocs 等封装，接口可能不同。

接口:
  控制: /waist/cmd_pos   bodyctrl_msgs/CmdSetMotorPosition
  反馈: /waist/status    bodyctrl_msgs/MotorStatusMsg
  腰部 2 个自由度:
     31 = yaw   绕竖直轴转腰(左右拧身)   ← 本 demo 演示这个，不影响身高/平衡
     32 = pitch 前倾俯仰(与下肢升降耦合)  ← 本 demo 只"保持"它，不主动动

  为什么这么设计: pitch(32) 和腿部一起决定整机高度与重心，单独乱动有倾覆风险；
  yaw(31) 只是上身水平拧转，是腰部里最安全的演示。要动 pitch 请用 atom23_leg_ros2.py（协调升降）。

⚠ 安全: 读到 /waist/status 才动(不盲发)；幅度 ±0.2rad≈±11°；急停在手。
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition, MotorStatusMsg

WAIST_CMD_TOPIC = "/waist/cmd_pos"
WAIST_STATUS_TOPIC = "/waist/status"

WAIST_YAW_ID = 31
WAIST_PITCH_ID = 32
CURRENT_LIMIT = 20.0     # A，承重需要足够力矩（取自工程配置 leg.current_limit）
SPEED = 0.5              # rad/s
YAW_AMP = 0.2            # rad，左右各转这么多


class WaistDemo(Node):
    def __init__(self):
        super().__init__("atom_waist_demo")
        self.pub = self.create_publisher(CmdSetMotorPosition, WAIST_CMD_TOPIC, 1)
        self.cur_pos = {}
        self.status_sub_ = self.create_subscription(MotorStatusMsg, WAIST_STATUS_TOPIC, self._on_status, 1)
        self.get_logger().info(f"腰部原子 demo 已启动，发布到 {WAIST_CMD_TOPIC}")

    def _on_status(self, msg: MotorStatusMsg):
        for s in msg.status:
            self.cur_pos[s.name] = s.pos

    def wait_status(self, ids, timeout=3.0):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(i in self.cur_pos for i in ids):
                return True
        return False

    def command(self, yaw: float, pitch_hold: float, hold: float = 1.5):
        """yaw=目标转腰角; pitch_hold=把 pitch 保持在此角度(通常=当前测量值)。"""
        msg = CmdSetMotorPosition()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.cmds = [
            SetMotorPosition(name=WAIST_YAW_ID,   pos=float(yaw),        spd=SPEED, cur=CURRENT_LIMIT),
            SetMotorPosition(name=WAIST_PITCH_ID, pos=float(pitch_hold), spd=SPEED, cur=CURRENT_LIMIT),
        ]
        self.pub.publish(msg)
        self.get_logger().info(f"腰 yaw->{yaw:+.2f} (pitch 保持 {pitch_hold:+.2f})")
        time.sleep(hold)


def main():
    rclpy.init()
    node = WaistDemo()
    try:
        if not node.wait_status([WAIST_YAW_ID, WAIST_PITCH_ID]):
            node.get_logger().error("读不到 /waist/status，为安全起见不运动。请确认主控在运行。")
            return
        yaw0 = node.cur_pos[WAIST_YAW_ID]
        pitch0 = node.cur_pos[WAIST_PITCH_ID]   # 全程保持 pitch 不变
        node.get_logger().info(f"当前 yaw={yaw0:+.3f}, pitch={pitch0:+.3f}")

        node.command(yaw0 + YAW_AMP, pitch0)    # 向一侧转
        node.command(yaw0 - YAW_AMP, pitch0)    # 向另一侧转
        node.command(yaw0,           pitch0)    # 回到起点
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
