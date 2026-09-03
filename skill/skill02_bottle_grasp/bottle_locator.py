#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill02_bottle_grasp · 阶段1：YOLO(分割) 检测瓶子 + 瓶底边界曲线拟合圆心定位（纯感知）

一句话
  订 RGB + 深度 → YOLO-seg 检测 "bottle"，拿逐像素 mask → 提取 mask 底部边界曲线（每列最下沿
  像素，排除左右切线区）→ 每个边界点借【支撑面】深度反投影成 base 系 3D 点 → 用 mask 中段
  宽度独立测一个半径 → 以这个半径为约束，对边界点做"固定半径"最小二乘拟合出圆心 → 圆心即
  瓶子中轴线在支撑面上的位置估计，发布。

为什么用"固定半径"拟合圆心，而不是单点估计或自由拟合
  只用一个像素点（比如检测框底边中点）定位，本质是瓶底圆弧上离相机最近的那一点，不是
  圆心，误差没有约束。
  用 mask 提取整条可见的瓶底边界弧，多点参与更可信；但对着这一小段近侧弧【自由】拟合
  圆心+半径（3自由度）在深度方向上是病态的（弧越短，圆心沿弧法线方向的解越不稳）。
  半径单独从"左右宽度"方向测（这个方向不受近侧弧病态问题影响，更可信），只把圆心
  (2自由度) 交给拟合——相当于给每个边界点一条"离圆心距离=r"的约束，多点联立定圆心，
  等价于已知半径的三边定位，比自由拟合良态得多。

为什么不直接测瓶身本体的深度
  透明塑料瓶身让结构光/双目深度算法大概率测不到有效值。换个思路：只借瓶子接触的
  【支撑面】（桌子/箱子，这类表面深度一直很干净）的深度，反推瓶子边界弧上每个点的
  3D 位置，不测瓶身本体。

跑在哪（相机与本节点在 Orin/nvidia；本体在 x86/ubuntu，两板须同 ROS_DOMAIN_ID）
  0) 环境：pip install ultralytics opencv-python
  1) 起本体(x86)： bash scripts/start_body_control.sh  然后  bash scripts/start_xarm.sh real
                   —— 本节点输出 base 系坐标，靠本体提供的 TF 算出来，必须先起
  2) 起相机(Orin)： bash scripts/start_camera.sh
  3) 本节点(Orin)： python3 skill/skill02_bottle_grasp/bottle_locator.py

安全：本阶段纯感知，不发任何运动指令，零风险，可反复运行。

⚠ 待真机验证的假设（第一次跑务必对照打印值人工核实，不要直接信）
  1) RGB/深度像素坐标一一对应——需要用同一个矩形框叠在 RGB 和深度伪彩色图上比对确认，
     换相机型号/话题要重新验证。
  2) EXTRINSICS_GROUP 沿用头部回零姿态标定的外参，见 config.py 注释。
  3) 圆心拟合假设瓶子脚下这一小片支撑面是水平的（base 系 z 不变）——真实箱子/桌面有轻微
     不平整属于噪声范畴，但明显倾斜的支撑面会系统性带偏拟合，人工核对时留意。
  4) 拟合给出的是"瓶子中轴线在支撑面上的落点"，不是抓握点——抓握点还要在此基础上加一个
     偏移，由 grasp_bottle.py 的 TCP_OFFSET 承担。
"""

import sys
import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

import config as C
import camera_ns
import pose_math as PM

try:
    import cv2
except ImportError:
    print("❌ 找不到 cv2（OpenCV）。pip3 install opencv-python")
    sys.exit(1)
try:
    from ultralytics import YOLO
except ImportError:
    print("❌ 找不到 ultralytics。pip3 install ultralytics")
    sys.exit(1)
try:
    from tf2_ros import Buffer, TransformListener
except ImportError:
    print("❌ 找不到 tf2_ros。圆心拟合依赖 base 系水平面假设，TF 是硬依赖，不能跳过。")
    sys.exit(1)


def fit_center_fixed_radius(points_xy, r, iters=C.FIT_ITERS):
    """给定一批"应落在半径 r 圆周上"的 2D 点，求圆心 (cx,cy)。
    高斯-牛顿迭代最小化 Σ(‖Pi-C‖-r)²，只有 2 个未知数（半径已知、不参与拟合）——
    这是本节点相比"自由拟合圆(3自由度)"更良态的关键：短弧对自由拟合的半径方向病态，
    但半径固定后，每个点退化成一条"离圆心距离=r"的约束，等价于已知半径的三边定位。"""
    pts = np.asarray(points_xy, dtype=float)
    c = pts.mean(axis=0)                      # 初值：点集质心（够用，问题良态、收敛快）
    for _ in range(iters):
        d = pts - c                           # N×2，圆心指向各点的向量
        dist = np.linalg.norm(d, axis=1)
        dist = np.where(dist < 1e-6, 1e-6, dist)
        resid = dist - r                      # N，径向残差
        J = -d / dist[:, None]                # N×2，d(resid)/dc
        JTJ = J.T @ J
        JTr = J.T @ resid
        try:
            delta = np.linalg.solve(JTJ, JTr)
        except np.linalg.LinAlgError:
            break
        c = c - delta
    return c


class BottleLocator(Node):
    def __init__(self):
        super().__init__("skill02_bottle_locator")
        self.fx, self.fy, self.cx, self.cy = self._load_intrinsics()
        self.cam_to_head = self._load_extrinsics()          # 4×4：相机系 → head_roll_link 系
        self._intrinsics_scaled = False

        self.get_logger().info(f"加载 YOLO 模型 {C.YOLO_MODEL} ...")
        self.model = YOLO(C.YOLO_MODEL)
        self.bottle_cls_id = self._resolve_class_id(C.TARGET_CLASS_NAME)

        self.rgb = None          # 最新一帧 RGB（numpy, HxWx3, bgr）
        self.depth = None        # 最新一帧深度（numpy, HxW, uint16, mm）
        ns = camera_ns.resolve(self)          # 相机命名空间：自动探测，CAMERA_NS 可覆盖
        self.rgb_topic = f"/{ns}/color/image_raw"
        self.depth_topic = f"/{ns}/depth/image_raw"
        self.rgb_sub_ = self.create_subscription(
            Image, self.rgb_topic, self._on_rgb, qos_profile_sensor_data)
        self.depth_sub_ = self.create_subscription(
            Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)

        self.pose_pub = self.create_publisher(PoseStamped, C.TARGET_TOPIC, 10)
        self.diam_pub = self.create_publisher(Float32, C.DIAMETER_TOPIC, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._last_print_ns = 0
        self.timer = self.create_timer(C.INFER_PERIOD_S, self.step)   # 推理频率与图像帧率解耦
        self.get_logger().info(
            f"bottle_locator 启动 | 类别={C.TARGET_CLASS_NAME}(id={self.bottle_cls_id}) "
            f"conf>={C.CONF_THRES} | 订 {self.rgb_topic} + {self.depth_topic} → 发 {C.TARGET_TOPIC}"
            f"（frame={C.BASE_FRAME}）")

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

    def _resolve_class_id(self, name):
        names = self.model.names   # {id: name}
        for k, v in names.items():
            if v == name:
                return k
        self.get_logger().error(f"YOLO 模型里没有类别 '{name}'，可选：{list(names.values())}")
        sys.exit(1)

    def _match_intrinsics(self, w, h):
        ew, eh = C.EXPECTED_SIZE
        if (w, h) == (ew, eh) or self._intrinsics_scaled:
            return
        sx, sy = w / ew, h / eh
        self.fx, self.cx = self.fx * sx, self.cx * sx
        self.fy, self.cy = self.fy * sy, self.cy * sy
        self._intrinsics_scaled = True
        self.get_logger().warn(f"图像分辨率 {w}x{h} ≠ 内参标定 {ew}x{eh}，内参已等比缩放")

    # ── 图像回调：只转 numpy 存起来，重活留给定时器 step() ─────────
    def _on_rgb(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]          # → bgr（YOLO/cv2 的习惯通道序）
        elif msg.encoding != "bgr8":
            self.get_logger().error(f"不支持的 RGB 编码 {msg.encoding}（期望 bgr8/rgb8）")
            return
        self._match_intrinsics(msg.width, msg.height)
        self.rgb = img

    def _on_depth(self, msg: Image):
        # 16UC1，单位 mm。假设与 RGB 像素坐标一一对应（见文件头注释）。
        self.depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    # ── 检测：YOLO-seg 出框+mask，取置信度最高的一个 ────────────────
    def detect_bottle(self):
        """返回 (x1,y1,x2,y2,conf,mask_polygon) 或 None。mask_polygon 是原图像素坐标下的
        轮廓多边形 (N,2)，来自 res.masks.xy——ultralytics 已经把它缩放到原图分辨率了。"""
        res = self.model.predict(self.rgb, classes=[self.bottle_cls_id],
                                  conf=C.CONF_THRES, verbose=False)[0]
        if len(res.boxes) == 0:
            return None
        if res.masks is None:
            self.get_logger().error("模型没有输出 mask——确认 YOLO_MODEL 是 -seg 版本")
            return None
        if len(res.boxes) > 1:
            self.get_logger().warn(f"检测到 {len(res.boxes)} 个候选 bottle，取置信度最高的一个"
                                    "（场景里有多个瓶子/类瓶子物体时要加筛选逻辑，当前版本没做）")
        boxes = res.boxes
        best = int(np.argmax(boxes.conf.cpu().numpy()))
        x1, y1, x2, y2 = boxes.xyxy[best].cpu().numpy()
        conf = float(boxes.conf[best])
        poly = np.asarray(res.masks.xy[best], dtype=np.float64)
        return float(x1), float(y1), float(x2), float(y2), conf, poly

    # ── mask → 瓶底边界曲线（每列最下沿像素，排除左右切线区）─────────
    def extract_bottom_boundary(self, poly, x1, x2):
        """把多边形轮廓栅格化成二值 mask，逐列找最下沿（最大 v）——这条曲线沿瓶底可见弧走，
        排除左右各 BOUNDARY_EDGE_MARGIN_FRAC 比例——那部分是瓶身侧面切线轮廓，不是瓶底圆弧，
        混进来会带偏拟合（侧面轮廓点离真实圆心的距离不等于半径）。
        返回一串 (u,v) 整数像素坐标。"""
        h, w = self.depth.shape
        canvas = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(canvas, [poly.reshape(-1, 1, 2).astype(np.int32)], 1)

        margin = C.BOUNDARY_EDGE_MARGIN_FRAC * (x2 - x1)
        u_lo, u_hi = int(round(x1 + margin)), int(round(x2 - margin))
        pts = []
        for u in range(max(0, u_lo), min(w, u_hi), C.BOUNDARY_COL_STRIDE):
            col = canvas[:, u]
            rows = np.nonzero(col)[0]
            if rows.size == 0:
                continue
            pts.append((u, int(rows.max())))   # 该列最下沿像素
        return pts, canvas

    # ── mask 中段宽度 → 半径（独立测量，不依赖圆心拟合）──────────────
    def measure_radius_px(self, canvas, x1, y1, x2, y2):
        """在 mask 包围盒高度的中段（RADIUS_ROW_BAND）取每行宽度的中位数，
        换算成像素半径。中段能避开上方瓶颈收缩和下方贴地边缘的噪声。"""
        h_box = y2 - y1
        r0, r1 = C.RADIUS_ROW_BAND
        v_lo = int(round(y1 + r0 * h_box))
        v_hi = int(round(y1 + r1 * h_box))
        widths = []
        for v in range(max(0, v_lo), min(canvas.shape[0], v_hi)):
            row = canvas[v, int(x1):int(x2)]
            cols = np.nonzero(row)[0]
            if cols.size > 0:
                widths.append(cols.max() - cols.min())
        if not widths:
            return None
        return float(np.median(widths)) / 2.0

    # ── 深度采样：单个边界点邻域取中位数 ─────────────────────────
    def _sample_depth_m(self, u, v):
        v = v - C.CONTACT_PIXEL_INSET
        half = C.CONTACT_WINDOW // 2
        h, w = self.depth.shape
        v0, v1 = max(0, v - half), min(h, v + half + 1)
        u0, u1 = max(0, u - half), min(w, u + half + 1)
        patch = self.depth[v0:v1, u0:u1]
        valid = patch[patch > 0]
        if valid.size == 0:
            return None
        return float(np.median(valid)) / 1000.0

    def _backproject(self, u, v, z):
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return np.array([x, y, z])

    # ── base 系变换：相机系批量点 → head_roll_link → base ───────────
    def _to_base_points(self, p_cam_list):
        """p_cam_list: N×3 numpy。先乘固定外参到 head_roll_link，再查一次 TF 转 base，
        对这批点复用同一个 TF 快照（同一帧内头部姿态视为不变，没必要逐点查）。
        返回 (N×3 base系点, 是否成功)。"""
        p_cam = np.asarray(p_cam_list, dtype=float)
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

    # ── 定位主逻辑：边界曲线 → base系点云 → 半径 + 圆心拟合 ──────────
    def locate(self, box):
        x1, y1, x2, y2, conf, poly = box
        boundary, canvas = self.extract_bottom_boundary(poly, x1, x2)
        if len(boundary) < C.MIN_BOUNDARY_POINTS:
            self.get_logger().warn(
                f"有效边界点只有 {len(boundary)} 个（<{C.MIN_BOUNDARY_POINTS}），本帧跳过"
                "（瓶子太小/太远、或被遮挡一大半？）")
            return None

        radius_px = self.measure_radius_px(canvas, x1, y1, x2, y2)
        if radius_px is None:
            self.get_logger().warn("半径测量区间内没有有效宽度，本帧跳过")
            return None

        p_cam_list = []
        for u, v in boundary:
            z = self._sample_depth_m(u, v)
            if z is None:
                continue
            p_cam_list.append(self._backproject(u, v, z))
        if len(p_cam_list) < C.MIN_BOUNDARY_POINTS:
            self.get_logger().warn(
                f"边界点里深度有效的只剩 {len(p_cam_list)} 个（<{C.MIN_BOUNDARY_POINTS}），本帧跳过")
            return None

        p_base, ok = self._to_base_points(p_cam_list)
        if not ok:
            return None

        z_mean = p_base[:, 2].mean()      # 支撑面高度：假设这一小片区域水平，取均值
        # 半径换算用相机系深度（跟像素/焦距同一套坐标系，物理意义清楚），不用 base 系 z
        z_cam_med = float(np.median([p[2] for p in p_cam_list]))
        radius_m = radius_px * z_cam_med / self.fx

        center_xy = fit_center_fixed_radius(p_base[:, :2], radius_m)
        center = np.array([center_xy[0], center_xy[1], z_mean])
        return center, radius_m, conf, len(p_cam_list)

    # ── 发布 + 打印验收 ─────────────────────────────────────────
    def publish(self, center_base, radius_m, conf, n_pts):
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = C.BASE_FRAME
        out.pose.position.x, out.pose.position.y, out.pose.position.z = center_base
        out.pose.orientation.w = 1.0     # YOLO 不给朝向；接近方向留给下游运控按位置自己算
        self.pose_pub.publish(out)
        self.diam_pub.publish(Float32(data=float(radius_m * 2.0)))

        self._print_throttled(
            f"瓶子 conf={conf:.2f} 边界点数={n_pts} 直径估计={radius_m*2*100:.1f}cm | "
            f"圆心 base系=({center_base[0]:+.3f},{center_base[1]:+.3f},{center_base[2]:+.3f})")

    def _print_throttled(self, text, period_s=1.0):
        now = self.get_clock().now().nanoseconds
        if now - self._last_print_ns >= period_s * 1e9:
            self._last_print_ns = now
            self.get_logger().info(text)

    # ── 定时器主循环：有新帧就跑一次检测+定位 ───────────────────
    def step(self):
        if self.rgb is None or self.depth is None:
            return
        box = self.detect_bottle()
        if box is None:
            self._print_throttled("未检测到 bottle（画面里有瓶子吗？光照/角度是否正常？）")
            return
        result = self.locate(box)
        if result is None:
            return
        center, radius_m, conf, n_pts = result
        self.publish(center, radius_m, conf, n_pts)


def main():
    rclpy.init()
    node = BottleLocator()
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
