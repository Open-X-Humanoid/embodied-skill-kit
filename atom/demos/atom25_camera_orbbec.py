#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原25 · 相机（Camera） —— ROS2 原生版
读取 Orbbec 相机一帧 RGB 图并保存。   配套讲解文档: atom/docs/atom25_camera_guide.md

接口（只读订阅，不驱动任何电机）:
  RGB  话题: /<相机命名空间>/color/image_raw   类型: sensor_msgs/Image   编码: bgr8
       命名空间启动时自动探测（orbbec 默认 camera，部分机器 ob_camera_head）；
       想强制指定就 export CAMERA_NS=<命名空间>，见 detect_camera_ns()
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

OUT_DIR   = "atom/assets/camera_captures"

FALLBACK_NS = "camera"            # 探测不到时的兜底（orbbec 驱动默认命名空间）
COLOR_SUFFIX = "/color/image_raw"

FRAME_TIMEOUT = 5.0   # s 等第一帧图像


def detect_camera_ns(node, timeout_s=5.0):
    """确定相机话题的命名空间，返回不带斜杠的字符串（如 "camera"）。

    话题命名空间【因机器出厂配置而异】：orbbec 驱动默认 `camera`，部分机器出厂配成
    `ob_camera_head`。写死任何一个，换台机器就一帧都收不到——而且没有报错，只有超时，
    很难查。所以这里启动时扫一次 ROS 图自己认出来。三档逐级回落：
      1) 环境变量 CAMERA_NS 有值 → 直接用（多相机、或想强制指定时）
      2) 扫 ROS 图里的 */color/image_raw 话题 → 抠出命名空间（正常路径）
      3) 扫不到（驱动没起）→ 回落 `camera`，让下面的超时提示照常出现
    """
    forced = os.getenv("CAMERA_NS", "").strip().strip("/")
    if forced:
        node.get_logger().info(f"相机命名空间 = {forced}（环境变量 CAMERA_NS 指定，跳过探测）")
        return forced

    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    found = []
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        found = sorted(
            name[: -len(COLOR_SUFFIX)].strip("/")
            for name, types in node.get_topic_names_and_types()
            if name.endswith(COLOR_SUFFIX) and "sensor_msgs/msg/Image" in types
        )
        if found:
            break
        rclpy.spin_once(node, timeout_sec=0.2)      # 让发现机制转起来，别空转 CPU

    if not found:
        node.get_logger().warn(
            f"{timeout_s:.0f}s 内没扫到 */color/image_raw 话题——相机驱动起了吗？"
            f"暂按默认 `{FALLBACK_NS}` 继续。")
        return FALLBACK_NS
    if len(found) > 1:
        node.get_logger().warn(f"扫到多颗相机 {found}，自动选第一个 `{found[0]}`；"
                               "要用别的就 export CAMERA_NS=<命名空间>。")
    node.get_logger().info(f"相机命名空间 = {found[0]}（自动探测）")
    return found[0]


class CameraDemo(Node):
    def __init__(self):
        super().__init__("atom_camera_demo")
        self.bridge = CvBridge()
        self.rgb = None
        # 相机图像用 sensor_data QoS(BEST_EFFORT)订阅——兼容任何发布者，避免 QoS 不匹配收不到帧
        rgb_topic = f"/{detect_camera_ns(self)}{COLOR_SUFFIX}"
        self.sub_ = self.create_subscription(Image, rgb_topic, self._on_rgb, qos_profile_sensor_data)
        self.get_logger().info(f"相机原子 demo 已启动，订阅 {rgb_topic}")

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
