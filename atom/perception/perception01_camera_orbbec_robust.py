#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 感知1 · 相机（Camera） —— ROS2 原生版 [生产级]
读取 Orbbec 相机的 RGB + 深度 + 内参，保存三件套并打印关键信息。
配套讲解文档: atom/perception/docs/perception01_camera_guide.md

接口（只读订阅，不驱动任何电机）:
  RGB   话题: /camera/color/image_raw    类型: sensor_msgs/Image       编码: bgr8
  深度  话题: /camera/depth/image_raw    类型: sensor_msgs/Image       编码: 16UC1  单位: 毫米(mm)
  内参  话题: /camera/color/camera_info  类型: sensor_msgs/CameraInfo  k: 3x3 行优先展平
  分辨率示例 1280x720 @30fps（以相机驱动实际配置为准）
  ⚠ QoS: 相机图像常以 BEST_EFFORT 发布，用 qos_profile_sensor_data 订阅才收得到（默认 RELIABLE 遇 BEST_EFFORT 驱动收不到帧）

⚠ 安全: 本示例只订阅、读取图像，机器人不会动；可安全反复运行。
  前提: 相机驱动(orbbec launch)已启动；依赖 cv_bridge / opencv-python / numpy。
  深度图与彩色图分辨率可能不同，中心深度按各自尺寸取，不能用彩色的宽高去索引深度。
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

RGB_TOPIC        = "/camera/color/image_raw"
DEPTH_TOPIC      = "/camera/depth/image_raw"
COLOR_INFO_TOPIC = "/camera/color/camera_info"

RGB_ENCODING   = "bgr8"
DEPTH_ENCODING = "16UC1"        # 单位 mm

FRAME_TIMEOUT = 5.0             # s 等图像/内参到齐
OUT_DIR       = "atom/perception/assets/camera_captures"


class CameraDemo(Node):
    def __init__(self):
        super().__init__("atom_camera_demo")
        self.bridge = CvBridge()

        self.rgb = None
        self.depth = None
        self.intrinsic = None    # 3x3 相机内参 K

        # ★ 保留订阅对象引用（尾部下划线，风格约定；rclpy 的 Node 内部已持强引用，
        #   赋值只为统一风格、便于按需单独管理该订阅）。
        # 相机话题用 sensor_data QoS(BEST_EFFORT)订阅——兼容任何发布者，避免 QoS 不匹配收不到帧
        self.rgb_sub_   = self.create_subscription(Image, RGB_TOPIC, self._on_rgb, qos_profile_sensor_data)
        self.depth_sub_ = self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.info_sub_  = self.create_subscription(CameraInfo, COLOR_INFO_TOPIC, self._on_info, qos_profile_sensor_data)

        self.get_logger().info(
            f"相机原子 demo 已启动，订阅 {RGB_TOPIC} / {DEPTH_TOPIC} / {COLOR_INFO_TOPIC}")

    # ── 回调 ────────────────────────────────────────────────────────────────
    def _on_rgb(self, msg: Image) -> None:
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding=RGB_ENCODING)
        except Exception as e:
            self.get_logger().warn(f"RGB 解码失败: {e}")

    def _on_depth(self, msg: Image) -> None:
        try:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding=DEPTH_ENCODING)
        except Exception as e:
            self.get_logger().warn(f"深度解码失败: {e}")

    def _on_info(self, msg: CameraInfo) -> None:
        # CameraInfo.k 是长度 9 的行优先展平 3x3 内参矩阵
        self.intrinsic = np.array(msg.k, dtype=float).reshape(3, 3)

    # ── 辅助 ────────────────────────────────────────────────────────────────
    def wait_for_frames(self, need_depth: bool = True,
                         timeout: float = FRAME_TIMEOUT) -> bool:
        """
        等到 RGB、内参（可选深度）都到齐。
        图像是被动推送的，必须持续 spin 才会触发回调。
        """
        deadline = time.time() + timeout
        while rclpy.ok():
            # remaining 可能因竞态变负；rclpy 对负 timeout_sec 会永久阻塞，先判空再 spin。
            remaining = deadline - time.time()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
            ok_rgb   = self.rgb is not None
            ok_info  = self.intrinsic is not None
            ok_depth = (self.depth is not None) or (not need_depth)
            if ok_rgb and ok_info and ok_depth:
                return True
        return False


def main() -> None:
    rclpy.init()
    node = CameraDemo()
    try:
        if not node.wait_for_frames():
            miss = []
            if node.rgb is None:       miss.append("RGB图像")
            if node.depth is None:     miss.append("深度图")
            if node.intrinsic is None: miss.append("相机内参")
            node.get_logger().error(
                f"{FRAME_TIMEOUT}s 内未收齐: {', '.join(miss)}。"
                f"检查相机驱动(orbbec launch)是否已启动、话题名是否一致。")
            return

        rgb, depth = node.rgb, node.depth
        rh, rw = rgb.shape[:2]
        dh, dw = depth.shape[:2]
        node.get_logger().info(
            f"RGB: {rw}x{rh}   深度: {dw}x{dh} (dtype={depth.dtype})")

        # 打印内参 K
        fx, fy = node.intrinsic[0, 0], node.intrinsic[1, 1]
        cx, cy = node.intrinsic[0, 2], node.intrinsic[1, 2]
        node.get_logger().info(
            f"内参 K: fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")

        # 深度统计（0 表示无效/无返回，需剔除）
        valid = depth[depth > 0]
        if valid.size:
            node.get_logger().info(
                f"深度有效像素 {valid.size}/{depth.size}, "
                f"范围 {int(valid.min())}~{int(valid.max())} mm, "
                f"中位 {int(np.median(valid))} mm")
        # 画面中心点深度（按深度图自身尺寸取，勿用彩色宽高）
        center_d = int(depth[dh // 2, dw // 2])
        node.get_logger().info(f"画面中心深度: {center_d} mm")

        # 保存三件套：RGB.jpg + 原始16位深度.png + 伪彩色深度.jpg
        os.makedirs(OUT_DIR, exist_ok=True)
        rgb_path       = os.path.join(OUT_DIR, "perception01_rgb.jpg")
        depth_path     = os.path.join(OUT_DIR, "perception01_depth.png")        # png 保留 16 位
        depth_vis_path = os.path.join(OUT_DIR, "perception01_depth_color.jpg")

        cv2.imwrite(rgb_path, rgb)
        cv2.imwrite(depth_path, depth)

        # 伪彩色：按有效最大值归一化到 0-255 再上色，无效点(0)置黑
        dmax = float(valid.max()) if valid.size else 1.0
        dnorm = np.clip(depth.astype(np.float32) / dmax * 255.0, 0, 255).astype(np.uint8)
        depth_color = cv2.applyColorMap(dnorm, cv2.COLORMAP_JET)
        depth_color[depth == 0] = 0
        cv2.imwrite(depth_vis_path, depth_color)

        node.get_logger().info(
            f"已保存: {rgb_path} / {depth_path} / {depth_vis_path}")
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
