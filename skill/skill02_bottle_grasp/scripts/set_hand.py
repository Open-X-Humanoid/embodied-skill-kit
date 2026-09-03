#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill02_bottle_grasp/scripts · 灵巧手手动下发：命令行传6个数，直接发一条 JointState

一句话
  把6个张合值从命令行传进来，发到 /inspire_hand/ctrl/left_hand——用来在真机上快速试
  不同的手指组合（比如标定抓瓶子的固定手型），不用每次改代码。

手指ID对照（顺序固定）：
  1=小指  2=无名指  3=中指  4=食指  5=拇指弯  6=拇指旋
  1/2/3/4/5：0.0=握紧 1.0=张开；6拇指旋：0/1 是转出/转入两个极限方向，不是张合语义。

用法
  python3 skill/skill02_bottle_grasp/scripts/set_hand.py <小指> <无名> <中指> <食指> <拇弯> <拇旋>
  例：python3 .../set_hand.py 0.1 0.1 0.1 0.5 0.1 0.0

⚠ 不做闭环验证：/inspire_hand/state 是否真实反映手指位置存疑，这个脚本发完就完事，
  实际效果靠肉眼看手，别指望读 state 回来确认。
"""

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

HAND_CMD_TOPIC = "/inspire_hand/ctrl/left_hand"
FINGER_NAMES = ["1", "2", "3", "4", "5", "6"]


def parse_args():
    if len(sys.argv) != 7:
        print(f"用法: python3 {sys.argv[0]} <小指> <无名> <中指> <食指> <拇弯> <拇旋>  （6个0~1的数）")
        sys.exit(1)
    try:
        values = [float(a) for a in sys.argv[1:7]]
    except ValueError:
        print("❌ 6个参数都必须是数字")
        sys.exit(1)
    for name, v in zip(FINGER_NAMES, values):
        if not (0.0 <= v <= 1.0):
            print(f"⚠ 手指{name}的值 {v} 超出 [0,1]，仍照发（简洁版不夹值，自己确认合理）")
    return values


class SetHand(Node):
    def __init__(self):
        super().__init__("bottle_grasp_set_hand")
        self.pub = self.create_publisher(JointState, HAND_CMD_TOPIC, 10)

    def send(self, values):
        t0 = self.get_clock().now()
        while self.pub.get_subscription_count() == 0:      # 等手驱动订上，否则消息丢弃
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - t0).nanoseconds > 3e9:
                self.get_logger().warn(
                    f"{HAND_CMD_TOPIC} 无订阅者——手驱动(inspire_hand)起了吗？仍尝试发送一次")
                break
        msg = JointState()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.name = FINGER_NAMES
        msg.position = values
        self.pub.publish(msg)
        pose = dict(zip(FINGER_NAMES, values))
        self.get_logger().info(f"已发手型：{pose}")
        t0 = self.get_clock().now()                        # 给手指运动留 1.5s
        while (self.get_clock().now() - t0).nanoseconds < 1.5e9:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    values = parse_args()
    rclpy.init()
    node = SetHand()
    try:
        node.send(values)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
