#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill02_bottle_grasp · 诊断工具：把 box_locator 的候选像素画在 RGB 图上，肉眼核对边界

一句话
  收一帧 RGB + 一帧深度 + bottle_locator 发的种子高度，跑一次跟 box_locator.py 完全一样的
  "筛高度 + 筛水平范围"逻辑，把通过筛选的像素直接标绿点存成 jpg——不用信任何数字，直接
  看图核对：标出来的区域是不是精确覆盖箱子顶面、有没有多圈进邻近的桌子/背景、边缘对不对。

为什么直接标像素、不画拟合出的矩形
  拟合矩形是在 base 系(米)算出来的，要画回图像像素需要反向投影（base→head→相机→像素），
  这里跳过这步复杂度——候选像素在筛选时本来就是从图像坐标(u,v)取样的，直接标记最直接，
  也最能看出"筛选逻辑本身选没选对区域"这个最想验证的问题；拟合矩形准不准是下一层的事，
  先确认候选点本身没选错、没漏、没混进不该有的区域。

跑在哪（Orin，nvidia 用户；依赖 bottle_locator.py 同时在跑，借它发的箱顶高度当种子）
  python3 skill/skill02_bottle_grasp/bottle_locator.py   # 另一个终端，先跑起来
  python3 skill/skill02_bottle_grasp/debug_box_overlay.py

安全：只收一帧、存图、退出，不发任何运动指令，零风险，可反复运行。
输出：skill/skill02_bottle_grasp/captures/box_overlay.jpg
"""

import sys
import json
from pathlib import Path

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

import config as C
import camera_ns
import pose_math as PM

try:
    from tf2_ros import Buffer, TransformListener
except ImportError:
    print("❌ 找不到 tf2_ros。")
    sys.exit(1)

OUT_DIR = Path(__file__).resolve().parent / "captures"
OUT_FILE = OUT_DIR / "box_overlay.jpg"


class BoxOverlayDebug(Node):
    def __init__(self):
        super().__init__("skill02_box_overlay_debug")
        self.fx, self.fy, self.cx, self.cy = self._load_intrinsics()
        self.cam_to_head = self._load_extrinsics()
        self._intrinsics_scaled = False

        self.rgb = None
        self.depth = None
        self._bottle_xyz = None
        ns = camera_ns.resolve(self)          # 相机命名空间：自动探测，CAMERA_NS 可覆盖
        self.rgb_sub_ = self.create_subscription(
            Image, f"/{ns}/color/image_raw", self._on_rgb, qos_profile_sensor_data)
        self.depth_sub_ = self.create_subscription(
            Image, f"/{ns}/depth/image_raw", self._on_depth, qos_profile_sensor_data)
        self.bottle_sub_ = self.create_subscription(
            PoseStamped, C.TARGET_TOPIC, self._on_bottle, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    # ── 标定文件加载（同 box_locator.py）───────────────────────
    def _load_intrinsics(self):
        with open(C.INTRINSICS_FILE, encoding="utf-8") as f:
            m = json.load(f)["camera"]
        k = m["matrix"]
        return k[0][0], k[1][1], k[0][2], k[1][2]

    def _load_extrinsics(self):
        with open(C.EXTRINSICS_FILE, encoding="utf-8") as f:
            groups = json.load(f)["groups"]
        if C.EXTRINSICS_GROUP not in groups:
            print(f"❌ 外参文件里没有组 {C.EXTRINSICS_GROUP}，可选：{list(groups)}")
            sys.exit(1)
        return np.asarray(groups[C.EXTRINSICS_GROUP]["extrinsics_cam_to_head_roll_link_matrix"],
                           dtype=np.float64)

    def _match_intrinsics(self, w, h):
        ew, eh = C.EXPECTED_SIZE
        if (w, h) == (ew, eh) or self._intrinsics_scaled:
            return
        sx, sy = w / ew, h / eh
        self.fx, self.cx = self.fx * sx, self.cx * sx
        self.fy, self.cy = self.fy * sy, self.cy * sy
        self._intrinsics_scaled = True
        self.get_logger().warn(f"图像分辨率 {w}x{h} ≠ 内参标定 {ew}x{eh}，内参已等比缩放")

    # ── 回调 ────────────────────────────────────────────────────
    def _on_rgb(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]
        elif msg.encoding != "bgr8":
            self.get_logger().error(f"不支持的 RGB 编码 {msg.encoding}（期望 bgr8/rgb8）")
            return
        self._match_intrinsics(msg.width, msg.height)
        self.rgb = img

    def _on_depth(self, msg: Image):
        self.depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    def _on_bottle(self, msg: PoseStamped):
        p = msg.pose.position
        self._bottle_xyz = np.array([p.x, p.y, p.z])

    # ── base 系变换（同 box_locator.py）─────────────────────────
    def _to_base_points(self, p_cam):
        ones = np.ones((p_cam.shape[0], 1))
        p_head = (np.hstack([p_cam, ones]) @ self.cam_to_head.T)[:, :3]
        try:
            tr = self.tf_buffer.lookup_transform(C.BASE_FRAME, C.CALIB_PARENT_FRAME, Time())
        except Exception as e:
            self.get_logger().error(f"TF({C.BASE_FRAME}←{C.CALIB_PARENT_FRAME}) 查询失败："
                                     f"{type(e).__name__}（body/XARM 起了吗）")
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        R = PM.quat_to_mat([q.x, q.y, q.z, q.w])
        return p_head @ R.T + np.array([t.x, t.y, t.z])

    # ── 跟 box_locator.locate_box() 完全一样的筛选，多留一份原始像素坐标 ──
    def compute_candidates(self):
        h, w = self.depth.shape
        us = np.arange(0, w, C.BOX_SCAN_STRIDE)
        vs = np.arange(0, h, C.BOX_SCAN_STRIDE)
        uu, vv = np.meshgrid(us, vs)
        uu, vv = uu.ravel(), vv.ravel()

        z_mm = self.depth[vv, uu].astype(np.float64)
        keep = z_mm > 0
        uu, vv, z_mm = uu[keep], vv[keep], z_mm[keep]
        if uu.size == 0:
            return None, None

        z = z_mm / 1000.0
        x = (uu - self.cx) * z / self.fx
        y = (vv - self.cy) * z / self.fy
        p_cam = np.stack([x, y, z], axis=1)

        p_base = self._to_base_points(p_cam)
        if p_base is None:
            return None, None

        top_z = float(self._bottle_xyz[2])
        mask = np.abs(p_base[:, 2] - top_z) < C.BOX_HEIGHT_TOL
        dxy = p_base[:, :2] - self._bottle_xyz[:2]
        mask &= np.linalg.norm(dxy, axis=1) < C.BOX_SEARCH_RADIUS
        return uu[mask], vv[mask]

    def save_overlay(self, uu, vv):
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        img = self.rgb.copy()
        for u, v in zip(uu, vv):
            cv2.circle(img, (int(u), int(v)), 2, (0, 255, 0), -1)   # 绿点标记候选像素
        cv2.imwrite(str(OUT_FILE), img)
        self.get_logger().info(f"已存: {OUT_FILE}（{len(uu)} 个候选像素，绿点标记，"
                                "看是否精确覆盖箱顶、有没有漏/多）")


def _wait_for(node, attr, timeout, label):
    t0 = node.get_clock().now()
    while rclpy.ok() and getattr(node, attr) is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - t0).nanoseconds > timeout * 1e9:
            node.get_logger().error(f"超时未收到 {label}")
            return False
    return True


def main():
    rclpy.init()
    node = BoxOverlayDebug()
    try:
        if not _wait_for(node, "rgb", 5.0, "RGB"):
            return
        if not _wait_for(node, "depth", 5.0, "深度"):
            return
        if not _wait_for(node, "_bottle_xyz", 10.0,
                          "bottle_locator 种子（bottle_locator.py 在跑吗？瓶子在视野里吗？）"):
            return
        uu, vv = node.compute_candidates()
        if uu is None or uu.size == 0:
            node.get_logger().warn(
                "候选像素为0——检查 BOX_HEIGHT_TOL/BOX_SEARCH_RADIUS 是否太严，或箱子是否在视野内")
            return
        node.save_overlay(uu, vv)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
