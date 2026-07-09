#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原1 · 头部（Head） —— ROS2 原生版
让机器人点头（pitch）、摇头（yaw），并演示如何读取头部当前角度。
配套讲解文档: atom/docs/atom01_head_guide.md

【版本说明】本文件是 ROS2 原生实现（rclpy + bodyctrl_msgs 话题直发）。
  若你的机器人走 xRocs / xArm 等封装接口，话题名或调用方式可能不同，届时另见对应变体。

接口:
  下发(位置模式): /head/cmd_pos   类型: bodyctrl_msgs/CmdSetMotorPosition
        SetMotorPosition[] cmds:  name(电机ID)  pos(rad)  spd(rad/s)  cur(最大电流A)
  读取(状态反馈): /head/status    类型: bodyctrl_msgs/MotorStatusMsg
        MotorStatus[] status:     name(电机ID)  pos(当前角rad)
  电机ID: 1=roll  2=pitch(低头/抬头)  3=yaw(左右转头)   单位: 弧度(rad)

⚠ 安全: 角度用弧度、幅度保守(±0.3rad≈±17°)、速度放慢；执行时人不要靠近机器人。
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition, MotorStatusMsg

HEAD_CMD_TOPIC = "/head/cmd_pos"
HEAD_STATUS_TOPIC = "/head/status"
MAX_SPEED = 0.5             # rad/s，慢一点更安全
MAX_CUR = [1.0, 1.0, 1.0]  # 三个关节最大电流(A)


class HeadDemo(Node):
    def __init__(self):
        super().__init__("atom_head_demo")
        self.pub = self.create_publisher(CmdSetMotorPosition, HEAD_CMD_TOPIC, 10)

        # —— 读取当前角度 —— 订阅状态话题，回调里把每个电机的当前角存进 cur_pos
        self.cur_pos = {}   # {电机ID: 当前角度(rad)}
        self.status_sub_ = self.create_subscription(
            MotorStatusMsg, HEAD_STATUS_TOPIC, self._on_status, 10)

        self.get_logger().info(f"头部原子 demo 已启动，发布到 {HEAD_CMD_TOPIC}")

    def _on_status(self, msg: MotorStatusMsg):
        """每收到一帧 /head/status，刷新三个电机的当前角度。"""
        for s in msg.status:              # 每个电机一条
            self.cur_pos[s.name] = s.pos  # s.name=电机ID, s.pos=当前角(rad)

    def move_to(self, roll: float, pitch: float, yaw: float, hold: float = 1.5):
        """移动到 (roll, pitch, yaw) 弧度并保持 hold 秒。"""
        msg = CmdSetMotorPosition()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.cmds = [
            SetMotorPosition(name=1, pos=float(roll),  spd=MAX_SPEED, cur=MAX_CUR[0]),
            SetMotorPosition(name=2, pos=float(pitch), spd=MAX_SPEED, cur=MAX_CUR[1]),
            SetMotorPosition(name=3, pos=float(yaw),   spd=MAX_SPEED, cur=MAX_CUR[2]),
        ]
        self.pub.publish(msg)
        self.get_logger().info(f"头部 -> roll={roll:.2f} pitch={pitch:.2f} yaw={yaw:.2f}")
        time.sleep(hold)


def main():
    rclpy.init()
    node = HeadDemo()
    try:
        # —— 读一次当前角度 —— status 是被动推送的，先 spin 收几帧才拿得到
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.cur_pos:
                break
        node.get_logger().info(f"当前头部角度(rad): {node.cur_pos}")

        node.move_to(0.0, 0.0, 0.0)      # 回正
        node.move_to(0.0, 0.25, 0.0)     # 低头（点头）
        node.move_to(0.0, -0.15, 0.0)    # 抬头
        node.move_to(0.0, 0.0, 0.3)      # 向左转头（摇头）
        node.move_to(0.0, 0.0, -0.3)     # 向右转头
        node.move_to(0.0, 0.0, 0.0)      # 回正
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
