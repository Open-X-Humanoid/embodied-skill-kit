#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原25 · 相机（Camera） —— ROS2 原生版
读取 Orbbec 相机一帧 RGB 图并保存。   配套讲解文档: atom/docs/atom25_camera_guide.md

接口（只读订阅，不驱动任何电机）:
  RGB  话题: /camera/color/image_raw   类型: sensor_msgs/Image   编码: bgr8
  分辨率示例 1280x720 @30fps（以相机驱动实际配置为准）
  ⚠ QoS: 相机图像常以 BEST_EFFORT 发布，必须用 qos_profile_sensor_data 订阅才收得到；
         用默认(RELIABLE)订阅遇到 BEST_EFFORT 驱动会一帧都收不到。

⚠ 安全: 本示例只订阅、读取图像，机器人不会动。
  前提: 相机驱动(orbbec launch)已启动；依赖 cv_bridge / opencv-python。
"""

import os
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

RGB_TOPIC = "/camera/color/image_raw"
OUT_DIR   = "atom/assets/camera_captures"

FRAME_TIMEOUT = 5.0   # s 等第一帧图像


class CameraDemo(Node):
    def __init__(self):
        super().__init__("atom_camera_demo")
        self.bridge = CvBridge()
        self.rgb = None
        # 相机图像用 sensor_data QoS(BEST_EFFORT)订阅——兼容任何发布者，避免 QoS 不匹配收不到帧
        self.sub_ = self.create_subscription(Image, RGB_TOPIC, self._on_rgb, qos_profile_sensor_data)
        self.get_logger().info(f"相机原子 demo 已启动，订阅 {RGB_TOPIC}")

    def _on_rgb(self, msg: Image) -> None:
        # imgmsg_to_cv2 把 sensor_msgs/Image 转成 OpenCV 的 numpy 数组(H,W,3)
        self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main() -> None:
    rclpy.init()
    node = CameraDemo()
    try:
        # 等第一帧到达（图像是被动推送的，要 spin 才会触发回调）
        t0 = time.time()
        while rclpy.ok() and node.rgb is None and time.time() - t0 < FRAME_TIMEOUT:
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.rgb is None:
            node.get_logger().error(
                f"{FRAME_TIMEOUT}s 内没收到图像，检查相机驱动是否启动、话题名是否一致。")
            return

        h, w = node.rgb.shape[:2]
        node.get_logger().info(f"收到一帧 RGB 图: {w}x{h}")

        os.makedirs(OUT_DIR, exist_ok=True)
        out = os.path.join(OUT_DIR, "atom25_rgb.jpg")
        cv2.imwrite(out, node.rgb)
        node.get_logger().info(f"已保存到 {out}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
