#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill02_bottle_grasp · 阶段1b：借瓶子已测出的支撑面高度，反推箱子顶面轮廓（纯感知）

一句话
  订 bottle_locator 发的 /skill02/target_point（拿"箱顶高度"这个已知种子值，不用
  重新猜）+ 深度图整帧反投影到 base 系 → 筛出"高度落在箱顶±容差内 且 水平位置离瓶子不
  太远"的点（这些点物理上就是箱子顶面本身）→ 对这批点在 base 系水平面上求带旋转角的
  最小包围矩形 → 得到箱子的长、宽、朝向、水平位置，全部是测出来的，不是猜的。
  箱子往下延伸多深用固定值（config.BOX_DOWN_MARGIN），不做实测。

为什么箱子顶面比瓶身好测
  箱子是不透明哑光大平面，是深度相机的最佳案例——跟透明瓶身完全反过来。不需要"借支撑面
  反推"这种绕弯子的做法，直接对深度点云做"筛高度 + 求轮廓"这种纯古典几何就够了，不需要
  额外上开放集检测器识别"箱子/纸箱"这个类别。

跑在哪（Orin，nvidia 用户；依赖 bottle_locator.py 同时在跑，借它发的箱顶高度当种子）
  python3 skill/skill02_bottle_grasp/bottle_locator.py   # 另一个终端，先跑起来
  python3 skill/skill02_bottle_grasp/box_locator.py

安全：本阶段纯感知，不发任何运动指令，零风险，可反复运行。

⚠ 待真机验证的假设
  1) 箱子顶面在相机视野内且大部分未被瓶子/杂物遮挡——遮挡太多会导致候选点不够（见
     BOX_MIN_POINTS）。
  2) BOX_SEARCH_RADIUS 假设箱子 footprint 不会离谱地大、且附近没有同高度的其它平面
     （比如旁边桌子跟箱顶一样高）——真机验证时留意有没有把邻近家具误当箱子边缘。
  3) 拟合出的矩形朝向角只用来估计安全路径点，不追求精确对齐箱子真实棱边。
  4) 本节点全程持续运行、逐帧刷新检测结果，对"画面里出现的到底是箱子还是机器人自己的
     手/臂"没有分辨能力——下游 grasp_bottle.py 只在动臂前拍一次快照就冻结，见该文件
     wait_box() 的说明；如果直接用这个话题做别的用途，注意手臂本身可能污染检测结果。
"""

import sys
import json

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion

import config as C
import pose_math as PM

try:
    from tf2_ros import Buffer, TransformListener
except ImportError:
    print("❌ 找不到 tf2_ros。box_locator 依赖 base 系水平面假设，TF 是硬依赖，不能跳过。")
    sys.exit(1)


class BoxLocator(Node):
    def __init__(self):
        super().__init__("skill02_box_locator")
        self.fx, self.fy, self.cx, self.cy = self._load_intrinsics()
        self.cam_to_head = self._load_extrinsics()          # 4×4：相机系 → head_roll_link 系
        self._intrinsics_scaled = False

        self.depth = None            # 最新一帧深度（numpy, HxW, uint16, mm）
        self._bottle_xyz = None      # bottle_locator 给的种子：瓶子中心(base系, m)
        self.depth_sub_ = self.create_subscription(
            Image, C.DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.bottle_sub_ = self.create_subscription(
            PoseStamped, C.TARGET_TOPIC, self._on_bottle, 10)

        self.pose_pub = self.create_publisher(PoseStamped, C.BOX_TOPIC, 10)
        self.size_pub = self.create_publisher(Vector3, C.BOX_SIZE_TOPIC, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._last_print_ns = 0
        self.timer = self.create_timer(C.BOX_INFER_PERIOD_S, self.step)
        self.get_logger().info(
            f"box_locator 启动 | 订 {C.DEPTH_TOPIC} + {C.TARGET_TOPIC}（借箱顶高度当种子）"
            f" → 发 {C.BOX_TOPIC} + {C.BOX_SIZE_TOPIC}")

    # ── 标定文件加载 ───────────────────────────────────────────
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
        g = groups[C.EXTRINSICS_GROUP]
        if not g.get("calibration_accurate", False):
            print(f"⚠ 组 {C.EXTRINSICS_GROUP} 标注 calibration_accurate=False，谨慎使用")
        return np.asarray(g["extrinsics_cam_to_head_roll_link_matrix"], dtype=np.float64)

    def _match_intrinsics(self, w, h):
        ew, eh = C.EXPECTED_SIZE
        if (w, h) == (ew, eh) or self._intrinsics_scaled:
            return
        sx, sy = w / ew, h / eh
        self.fx, self.cx = self.fx * sx, self.cx * sx
        self.fy, self.cy = self.fy * sy, self.cy * sy
        self._intrinsics_scaled = True
        self.get_logger().warn(f"图像分辨率 {w}x{h} ≠ 内参标定 {ew}x{eh}，内参已等比缩放")

    # ── 回调：只存数据，重活留给定时器 step() ───────────────────
    def _on_depth(self, msg: Image):
        self._match_intrinsics(msg.width, msg.height)
        self.depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    def _on_bottle(self, msg: PoseStamped):
        p = msg.pose.position
        self._bottle_xyz = np.array([p.x, p.y, p.z])

    # ── base 系变换：相机系批量点 → head_roll_link → base ──────────
    def _to_base_points(self, p_cam):
        """p_cam: N×3 numpy。先乘固定外参到 head_roll_link，再查一次 TF 转 base，
        整帧点复用同一个 TF 快照。返回 (N×3 base系点, 是否成功)。"""
        ones = np.ones((p_cam.shape[0], 1))
        p_head = (np.hstack([p_cam, ones]) @ self.cam_to_head.T)[:, :3]
        try:
            tr = self.tf_buffer.lookup_transform(C.BASE_FRAME, C.CALIB_PARENT_FRAME, Time())
        except Exception as e:
            self.get_logger().warn(f"TF({C.BASE_FRAME}←{C.CALIB_PARENT_FRAME}) 查询失败："
                                    f"{type(e).__name__}（body/XARM 起了吗），本帧跳过")
            return None, False
        t = tr.transform.translation
        q = tr.transform.rotation
        R = PM.quat_to_mat([q.x, q.y, q.z, q.w])
        p_base = p_head @ R.T + np.array([t.x, t.y, t.z])
        return p_base, True

    # ── 定位主逻辑：整帧反投影 → 高度+水平范围筛选 → 求最小包围矩形 ──
    def locate_box(self):
        h, w = self.depth.shape
        us = np.arange(0, w, C.BOX_SCAN_STRIDE)
        vs = np.arange(0, h, C.BOX_SCAN_STRIDE)
        uu, vv = np.meshgrid(us, vs)
        uu, vv = uu.ravel(), vv.ravel()

        z_mm = self.depth[vv, uu].astype(np.float64)
        keep = z_mm > 0
        uu, vv, z_mm = uu[keep], vv[keep], z_mm[keep]
        if uu.size == 0:
            self.get_logger().warn("整帧深度全无效，本帧跳过")
            return None

        z = z_mm / 1000.0
        x = (uu - self.cx) * z / self.fx
        y = (vv - self.cy) * z / self.fy
        p_cam = np.stack([x, y, z], axis=1)

        p_base, ok = self._to_base_points(p_cam)
        if not ok:
            return None

        top_z = float(self._bottle_xyz[2])
        mask = np.abs(p_base[:, 2] - top_z) < C.BOX_HEIGHT_TOL
        dxy = p_base[:, :2] - self._bottle_xyz[:2]
        mask &= np.linalg.norm(dxy, axis=1) < C.BOX_SEARCH_RADIUS
        pts = p_base[mask]
        if pts.shape[0] < C.BOX_MIN_POINTS:
            self.get_logger().warn(
                f"箱顶候选点只有 {pts.shape[0]} 个（<{C.BOX_MIN_POINTS}），本帧跳过"
                "（箱顶被遮挡太多？BOX_HEIGHT_TOL/BOX_SEARCH_RADIUS 太严？）")
            return None

        # 带旋转角的最小包围矩形，直接对 2D 点集求解
        pts2d = pts[:, :2].astype(np.float32)
        (cx, cy), (rw, rh), angle_deg = cv2.minAreaRect(pts2d)
        length = max(rw, rh) + 2 * C.BOX_XY_MARGIN
        width = min(rw, rh) + 2 * C.BOX_XY_MARGIN
        yaw = np.radians(angle_deg)     # cv2 角度约定不唯一，但矩形本身跟±90°/宽长互换等价

        center = np.array([cx, cy, top_z])
        return center, (length, width), yaw, pts.shape[0]

    # ── 发布 + 打印验收 ─────────────────────────────────────────
    def publish(self, center, size_xy, yaw, n_pts):
        length, width = size_xy
        # 下边界从检测出的顶面往下延伸 BOX_DOWN_MARGIN；发布的中心取顶面与下边界的中点
        box_center_z = center[2] - C.BOX_DOWN_MARGIN / 2.0

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = C.BASE_FRAME
        out.pose.position.x, out.pose.position.y, out.pose.position.z = (
            center[0], center[1], box_center_z)
        out.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=float(np.sin(yaw / 2)), w=float(np.cos(yaw / 2)))
        self.pose_pub.publish(out)
        self.size_pub.publish(Vector3(x=float(length), y=float(width), z=float(C.BOX_DOWN_MARGIN)))

        self._print_throttled(
            f"箱子 候选点={n_pts} 长×宽={length*100:.1f}×{width*100:.1f}cm "
            f"朝向={np.degrees(yaw):+.0f}° | 顶面高度={center[2]:+.3f}m | "
            f"中心 base系=({center[0]:+.3f},{center[1]:+.3f},{box_center_z:+.3f}) "
            f"尺寸(长,宽,高)=({length:.2f},{width:.2f},{C.BOX_DOWN_MARGIN:.2f})m")

    def _print_throttled(self, text, period_s=1.0):
        now = self.get_clock().now().nanoseconds
        if now - self._last_print_ns >= period_s * 1e9:
            self._last_print_ns = now
            self.get_logger().info(text)

    # ── 定时器主循环 ─────────────────────────────────────────────
    def step(self):
        if self.depth is None:
            return
        if self._bottle_xyz is None:
            self._print_throttled("还没收到 bottle_locator 的种子高度——bottle_locator.py 在跑吗？")
            return
        result = self.locate_box()
        if result is None:
            return
        center, size_xy, yaw, n_pts = result
        self.publish(center, size_xy, yaw, n_pts)


def main():
    rclpy.init()
    node = BoxLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("用户中断")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
