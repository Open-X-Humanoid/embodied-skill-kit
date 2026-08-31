#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
技能示例 · skill01 · 手指点按 · 阶段1：tag 定位（纯感知，不动机器人）
配套讲解：skill/skill01_finger_tap/docs/skill01_finger_tap_guide.md

一句话
  订相机图像 → pyapriltags 检测 AprilTag（tag36h11, id=2）→ 内参+实测边长(4cm)解出
  tag 中心在相机系的 3D 坐标 pose_t → 乘手眼标定外参得 head_roll_link 系坐标 →
  发布 PoseStamped（位置+卡面朝向，frame=head_roll_link）。TF 可用时同时打印 base 系坐标，便于验收。

为什么发 head_roll_link 系、不发相机系
  手眼外参（本目录 extrinsics.json，标定值）比 URDF 的 CAD 值准约 1.3°/4mm——
  标定矩阵管"相机→head_roll_link"这一节，head_roll_link→base 交给 TF 活链条
  （头/腰转动照样对）。下游 finger_tap 只需一次 lookup_transform(base←head_roll_link)。

跑在哪（Orin，nvidia 用户——相机同板图像不出板；pyapriltags 已随 move_box 装好）
  1) 起相机： bash scripts/start_camera.sh          （atom25 同款）
  2) 本节点： python3 skill/skill01_finger_tap/tag_locator.py

接口
  Sub  IMAGE_TOPIC（见 config.py）  sensor_msgs/Image           相机彩色图（bgr8/rgb8 均可）
  Pub  /skill01/target_point     geometry_msgs/PoseStamped    tag 中心+卡面朝向，frame=head_roll_link
  TF   base ← head_roll_link        仅用于验收打印；查不到不影响发布

阶段1验收（手持卡片前后左右挪，看 1Hz 打印）
  1) 三套坐标随卡片同向变化；2) 相机系 z ≈ 卡片到相机实测距离；3) base 系数值量级合理（误差<2cm）

⚠ 待真机核实：IMAGE_TOPIC 名；分辨率与内参匹配（不匹配会自动缩放内参并告警，注意看日志）
⚠ 外参分组：EXTRINSICS_GROUP 默认头部回零组；demo 时头俯仰不同（如低头看桌面）须换组，否则偏 ~1°
安全：本阶段纯感知，不发任何运动指令，零风险。
"""

import sys
import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped

import config as C
import pose_math as PM

# 此处在 rclpy.init 之前，无节点可用，只能用 print；两个库分开报，缺哪个一目了然
try:
    import cv2
except ImportError:
    print("❌ 找不到 cv2（OpenCV）。Orin 上一般随系统有；确实缺则 pip3 install opencv-python")
    sys.exit(1)
try:
    from pyapriltags import Detector
except ImportError:
    print("❌ 找不到 pyapriltags（宿主机没装；move_box 的装在 docker 里，宿主机不可见）")
    print("   安装： pip3 install pyapriltags")
    sys.exit(1)

try:
    from tf2_ros import Buffer, TransformListener
    import tf2_geometry_msgs  # noqa: F401  注册 PointStamped 的 do_transform 支持
    from tf2_geometry_msgs import do_transform_point
    TF_OK = True
except ImportError:
    TF_OK = False  # 没有 tf2 也能跑：只是不打印 base 系坐标


class TagLocator(Node):
    def __init__(self):
        super().__init__("skill01_tag_locator")
        # ── 标定：内参 + 手眼外参 ──
        self.fx, self.fy, self.cx, self.cy, self.dist = self._load_intrinsics()
        self.cam_to_head = self._load_extrinsics()          # 4×4：相机系 → head_roll_link 系
        self._intrinsics_scaled = False                     # 分辨率不匹配只缩放/告警一次
        self._undistort_maps = None                         # 去畸变映射表缓存（按分辨率建一次）

        # ── 检测器（参数借鉴 move_box 实际项目）──
        self.detector = Detector(families=C.TAG_FAMILY, nthreads=1, quad_decimate=1.0,
                                 quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25)

        # ── ROS 接口（发完整位姿：position=中心，orientation 的 z 轴=卡面朝外法线）──
        self.pose_pub = self.create_publisher(PoseStamped, C.TARGET_TOPIC, 10)
        self.image_sub_ = self.create_subscription(Image, C.IMAGE_TOPIC, self._on_image, 5)
        if TF_OK:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_print_ns = 0                             # 打印限频到 1Hz，发布不限
        self.get_logger().info(
            f"tag 定位启动 | 家族={C.TAG_FAMILY} id={C.TAG_ID} 边长={C.TAG_SIZE}m | "
            f"订 {C.IMAGE_TOPIC} → 发 {C.TARGET_TOPIC}（frame={C.CALIB_PARENT_FRAME}）")

    # ── 标定文件加载 ─────────────────────────────────────────────
    def _load_intrinsics(self):
        """读内参 json：3×3 矩阵取 fx/fy/cx/cy，畸变系数用于检测前去畸变。"""
        with open(C.INTRINSICS_FILE, encoding="utf-8") as f:
            m = json.load(f)["camera"]
        k = m["matrix"]
        return k[0][0], k[1][1], k[0][2], k[1][2], np.asarray(m["dist_coeffs"], dtype=np.float64)

    def _load_extrinsics(self):
        """读手眼外参 json：按 EXTRINSICS_GROUP 取"相机→head_roll_link"的 4×4 矩阵。"""
        with open(C.EXTRINSICS_FILE, encoding="utf-8") as f:
            groups = json.load(f)["groups"]
        if C.EXTRINSICS_GROUP not in groups:
            print(f"❌ 外参文件里没有组 {C.EXTRINSICS_GROUP}，可选：{list(groups)}")
            sys.exit(1)
        g = groups[C.EXTRINSICS_GROUP]
        if not g.get("calibration_accurate", False):
            print(f"⚠ 组 {C.EXTRINSICS_GROUP} 标注 calibration_accurate=False，谨慎使用")
        return np.asarray(g["extrinsics_cam_to_head_roll_link_matrix"], dtype=np.float64)

    # ── 图像处理 ─────────────────────────────────────────────────
    def _to_gray(self, msg: Image):
        """Image 消息 → 灰度图（不依赖 cv_bridge；bgr8/rgb8 都处理）。"""
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding in ("bgr8", "rgb8"):
            # 灰度化只做亮度加权，bgr/rgb 权重次序影响可忽略，统一按 BGR 转
            return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if msg.encoding == "mono8":
            return img[:, :, 0] if img.ndim == 3 else img
        self.get_logger().error(f"不支持的图像编码 {msg.encoding}（期望 bgr8/rgb8/mono8）")
        return None

    def _match_intrinsics(self, w, h):
        """实际分辨率≠标定分辨率时，等比缩放内参（只做一次并告警）。"""
        ew, eh = C.EXPECTED_SIZE
        if (w, h) == (ew, eh) or self._intrinsics_scaled:
            return
        sx, sy = w / ew, h / eh
        self.fx, self.cx = self.fx * sx, self.cx * sx
        self.fy, self.cy = self.fy * sy, self.cy * sy
        self._intrinsics_scaled = True
        self.get_logger().warn(
            f"图像分辨率 {w}x{h} ≠ 内参标定 {ew}x{eh}，内参已等比缩放（深度精度可能略降）")

    def _undistort(self, gray):
        """去畸变（映射表按分辨率缓存一次；借鉴 move_box 的做法）。"""
        if self._undistort_maps is None:
            h, w = gray.shape[:2]
            K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])
            self._undistort_maps = cv2.initUndistortRectifyMap(
                K, self.dist, None, K, (w, h), cv2.CV_16SC2)
            self.get_logger().info(f"去畸变映射表已缓存（{w}x{h}）")
        m1, m2 = self._undistort_maps
        return cv2.remap(gray, m1, m2, interpolation=cv2.INTER_LINEAR)

    # ── 主回调：检测 → 定位 → 发布 ───────────────────────────────
    def _on_image(self, msg: Image):
        gray = self._to_gray(msg)
        if gray is None:
            return
        self._match_intrinsics(msg.width, msg.height)
        gray = self._undistort(gray)

        dets = self.detector.detect(
            gray, estimate_tag_pose=True,
            camera_params=(self.fx, self.fy, self.cx, self.cy), tag_size=C.TAG_SIZE)
        det = next((d for d in dets if d.tag_id == C.TAG_ID), None)
        if det is None:
            self._print_throttled(f"未检测到 tag id={C.TAG_ID}（画面里有 {len(dets)} 个其它 tag）"
                                  if dets else f"未检测到 tag id={C.TAG_ID}")
            return

        # 相机系中心点 + 姿态 → 手眼外参 → head_roll_link 系
        p_cam = np.asarray(det.pose_t, dtype=np.float64).reshape(3)
        R_cam = np.asarray(det.pose_R, dtype=np.float64)
        # 卡面朝外法线：取 tag z 轴，若背向相机则翻转（不同库对 tag z 朝向约定不一，
        # 用"朝向相机方向(-p_cam)"判定并翻转，翻转时连 y 一起翻保持右手系）
        if float(np.dot(R_cam[:, 2], -p_cam)) < 0:
            R_cam = R_cam @ np.diag([1.0, -1.0, -1.0])
        p_head = (self.cam_to_head @ np.append(p_cam, 1.0))[:3]
        R_head = self.cam_to_head[:3, :3] @ R_cam
        q_head = PM.mat_to_quat(R_head)

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = C.CALIB_PARENT_FRAME
        out.pose.position.x, out.pose.position.y, out.pose.position.z = p_head
        (out.pose.orientation.x, out.pose.orientation.y,
         out.pose.orientation.z, out.pose.orientation.w) = q_head
        self.pose_pub.publish(out)

        # 验收打印（1Hz）：相机系 / head_roll_link 系 / base 系（TF 可用时）
        base_txt = "base系=（TF 不可用）"
        if TF_OK:
            try:
                tr = self.tf_buffer.lookup_transform(
                    C.BASE_FRAME, C.CALIB_PARENT_FRAME, Time())
                pt = PointStamped()
                pt.header.frame_id = C.CALIB_PARENT_FRAME
                pt.point.x, pt.point.y, pt.point.z = p_head
                p_base = do_transform_point(pt, tr).point
                base_txt = f"base系=({p_base.x:+.3f}, {p_base.y:+.3f}, {p_base.z:+.3f})"
            except Exception as e:
                base_txt = f"base系=查询失败({type(e).__name__}，body/XARM 起了吗)"
        self._print_throttled(
            f"tag{C.TAG_ID} 相机系=({p_cam[0]:+.3f}, {p_cam[1]:+.3f}, {p_cam[2]:+.3f}) "
            f"head_roll系=({p_head[0]:+.3f}, {p_head[1]:+.3f}, {p_head[2]:+.3f}) {base_txt}  "
            f"[margin={det.decision_margin:.0f}]")

    def _print_throttled(self, text, period_s=1.0):
        now = self.get_clock().now().nanoseconds
        if now - self._last_print_ns >= period_s * 1e9:
            self._last_print_ns = now
            self.get_logger().info(text)


def main():
    rclpy.init()
    node = TagLocator()
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
