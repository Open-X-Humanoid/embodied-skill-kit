#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原3 · 灵巧手（Hand，因时 Inspire 五指） —— ROS2 原生版
让灵巧手张开、握拳，并演示如何读取手指当前状态。   配套讲解: atom/docs/atom03_hand_guide.md

【版本说明】本文件是 ROS2 原生实现（topic 控制）。因时灵巧手另有 service 接口可设力矩/速度
  （见 SDK）；若走 xRocs 等封装，话题/服务名可能不同，届时另见对应变体。

接口（topic 控制）:
  下发: /inspire_hand/ctrl/left_hand    （右手: /inspire_hand/ctrl/right_hand）
  类型: sensor_msgs/JointState
        name[]     手指ID字符串:  "1"小指 "2"无名指 "3"中指 "4"食指 "5"拇指弯 "6"拇指旋
        position[] 张合百分比 (0.0=握紧/合, 1.0=完全张开)   ← 注意是百分比，不是弧度
  状态反馈: /inspire_hand/state/left_hand   sensor_msgs/JointState（position 也是百分比）

⚠ 安全: 手指别夹到东西/人；建议握到 0.1 而不是 0.0，给机械留点余量。
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

HAND_CMD_TOPIC   = "/inspire_hand/ctrl/left_hand"    # 左手；右手改成 right_hand
HAND_STATE_TOPIC = "/inspire_hand/state/left_hand"
FINGER_NAMES = ["1", "2", "3", "4", "5", "6"]        # 小指 无名指 中指 食指 拇指弯 拇指旋


class HandDemo(Node):
    def __init__(self):
        super().__init__("atom_hand_demo")
        self.pub = self.create_publisher(JointState, HAND_CMD_TOPIC, 10)

        # —— 读取当前状态 —— 订阅状态话题，回调里把 6 个手指的当前张合百分比存起来
        self.cur_pos = None   # list[float]：6 个手指当前张合(0~1)；None=还没收到
        self.state_sub_ = self.create_subscription(
            JointState, HAND_STATE_TOPIC, self._on_state, 10)

        self.get_logger().info(f"灵巧手原子 demo 已启动，发布到 {HAND_CMD_TOPIC}")

    def _on_state(self, msg: JointState):
        """每收到一帧 /inspire_hand/state，刷新当前手指张合百分比。"""
        self.cur_pos = list(msg.position)   # position[i] = 第 i 个手指当前张合(0~1)

    def set_open_ratio(self, ratios, hold: float = 2.0):
        """ratios: 6 个手指的张合百分比 (0~1)。"""
        assert len(ratios) == 6
        msg = JointState()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.name = FINGER_NAMES
        msg.position = [float(r) for r in ratios]
        self.pub.publish(msg)
        self.get_logger().info(f"手指张合 -> {ratios}")
        time.sleep(hold)


def main():
    rclpy.init()
    node = HandDemo()
    try:
        # —— 读一次当前手指状态 —— 状态是被动推送的，先 spin 收几帧才拿得到
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.cur_pos is not None:
                break
        node.get_logger().info(f"当前手指张合: {node.cur_pos}")

        node.set_open_ratio([1.0] * 6)                  # 完全张开
        node.set_open_ratio([0.1] * 6)                  # 握拳（留 0.1 余量，别到 0.0 顶死）
        node.set_open_ratio([1.0] * 6)                  # 再张开
        # 也可单指控制，例如只弯食指(id4)、其余张开:
        node.set_open_ratio([1.0, 1.0, 1.0, 0.1, 1.0, 1.0])
        node.set_open_ratio([1.0] * 6)                  # 收尾张开
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
