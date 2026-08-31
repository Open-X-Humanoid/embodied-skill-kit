#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
技能示例 · skill01 · 手指点按 · 看到卡片 → 伸手到位 → 食指点按 → 收手归位

配套讲解：skill/skill01_finger_tap/docs/skill01_finger_tap_guide.md

一句话
  收 tag_locator 发的目标点（head_roll_link 系）→ 取 10 帧中位数压抖动 → TF 变换到 base 系
  → SAFE_BOX 校验（默认关）→ 使能+切控制器 → 末端到卡前 APPROACH_OFFSET(8cm) → 闭环补刀
  压误差 → 点按（前进 8+PRESS_DEPTH，柔顺吸收过冲）→ 回车确认点按姿态 → 退回 → 关节空间归位 READY。
  阶段3 点按由 config.PRESS_ENABLE 开关（False=只伸手到 8cm 不接触）。

日志怎么读（阶段用"阶段N/4"，动作用圈号①-⑤，各带 [控制器｜关节/末端]；细节走 debug）
  阶段1 看   ：tag 中心/法线 → 指尖目标（指尖=left_index_2 实际位姿 + PAD_LOCAL_OFFSET 局部偏移）
  阶段2 就位 ：起始手腕 → ①伸手[MoveIt·末端] → ②微调[endpose QP·末端] → 就位完成
              ★偏差一律拆成"左右/上下/前后 cm"，直接对上要调的参数：
                左右→AIM_BIAS_BASE 的 y、上下→z、前后→重新标定 PAD_LOCAL_OFFSET（见 config 注释）
  阶段3 点按 ：③按下[endpose QP·末端] → 检查点按姿态(回车确认)
  阶段4 归位 ：④退回停驻[endpose QP·末端] → ⑤归位[jointspace QP·关节] + 手张开
  三个控制器：MoveIt / endpose QP（都末端·给 tcp 坐标）、jointspace QP（关节·给关节角）
  ★末端动作打两行（坐标均 base 系）：手腕 tcp（位置+rpy，给运控参考）+ 指尖指肚（位置，← 核心）。
    · 运控只控制 tcp；指尖刚性挂在 tcp 前 ~17cm、被"带着走"，tcp 是手段、指尖才是目的。
    · 指尖指肚 = left_index_2 实际位姿(TF 读，含旋转) 叠加局部固定偏移 PAD_LOCAL_OFFSET（随姿态
      刚体旋转）——才是真接触面。
    · 指尖不重复给 rpy：指尖姿态=手的姿态，看 tcp 那行的 rpy 即可。
  想看控制器切换/逐段路点/内部数学等细节：跑时加 --ros-args --log-level debug

双后端（config.ARM_BACKEND 一行切换）
  "qp"    （对照用）QP 末端控制器分段小步喂（atom08 已验证套路）。QP 是局部跟踪，
           目标离当前太远会被拒（dis_err_bound），故拆成 ≤QP_STEP(4cm) 的路点依次下发。
  "moveit"（当前默认）MoveIt 一步规划（atom06 套路）。99999 的【已实锤】主因：MoveIt 限位表比 URDF 紧
           （如 shoulder_yaw_l MoveIt=±1.5、URDF/QP=±2.96），QP/遥控把关节停在 MoveIt 界外
           → 起点非法秒拒（xarm.1 日志 grep 'outside bounds' 点名关节，用 QP 挪回即恢复，
           atom05 guide 排错表有完整处置）。此外姿态过约束也会 99999（放宽 ori_tol 可辨）。
           SRDF 碰撞矩阵不同步为未证实假设（已报 XARM 团队）。排除起点越界后本后端可用。

两段法（qp 后端；杜绝"保持垂臂手腕朝向跨大范围 → 臂/手扭曲"）
  第一段  关节空间(jointspace QP) → READY_JOINTS 预备姿态：大范围位移在关节空间完成，
          每关节直插目标角、不经 IK，不会拧麻花；且 QP 碰撞球不含手，预备位由人确认安全。
  第二段  末端空间(endpose QP) 分段接近：只走最后 20~30cm，姿态=预备位自身朝向，手腕几乎不动。
  ★录制 READY_JOINTS：用 jointspace action 增量把臂调到"手朝卡片方向、离卡 20~30cm、
    姿态自然"的位置 → 跑 atom05 抄它打印的『当前左臂角』（按名字序，即 J1..J7）→ 填入 config。

一次快照语义（安全设计）
  启动时取一次目标点后不再更新——绝不追踪移动目标。执行中挪卡片，手臂仍去原定点。

到位闭环修正（TIP_CORRECT，2026-07-24 实测定的方案）
  症状：同一目标三连跑 Δ=1.5/2.7/3.7cm 方向各异，而感知三次一字不差——散布全在执行侧。
  病根：MoveIt 姿态容差(±17°方向/±34°自旋)×腕→指尖 17cm 杠杆 = 指尖随机平移 ±2~3cm。
  修法：到位后 TF 实测【指肚】Δ → QP 短程平移 −Δ（对齐指肚、姿态保持当前实际值不动）→ 重测，
  |Δ|≤CORRECT_TOL 收工，最多 CORRECT_MAX 轮。就是人"发现偏了补一刀"的动作。

意外恢复（--recover；段失败/Ctrl-C/进程死掉，臂停在半空时用）
  每次动臂前出发位姿已落盘 _last_start_pose.json。收臂：
    python3 skill/skill01_finger_tap/finger_tap.py --recover
  流程：QP 慢速原路退回出发位 →（若配置）READY_JOINTS 归位；动前回车确认。
  无落盘记录且无 READY_JOINTS 时不自动动作；停止运行，由现场人员按安全规程处置。

⚠ 安全（执行前逐条确认）
  1) 卡片固定在静物上（纸箱/桌沿），★人退出手臂可达范围——目标点就是卡片，别拿在手里
  2) 动前打印目标并要求回车确认——SAFE_BOX 默认关闭（config 置 None），回车前
     ★人工核对打印坐标是否合理，这是感知算错时唯一的拦截点；要软件闸可在 config 重开
  3) QP 慢速 VEL_LIMITS / MoveIt VEL_SCALE=0.1 + arm_mode 0 柔顺兜底；急停在手

跑在哪（x86，ubuntu 用户——XARM 接口在这板；Orin 上 tag_locator 须同时在跑）
  0) sudo systemctl stop teleop_robot          # 遥控服务占 /arm/cmd_pos，不停则使能失败
  1) bash scripts/start_body_control.sh        # 另终端
  2) bash scripts/start_xarm.sh real           # XARM 本体（qp 后端不需要 MoveIt 组件）
  3) python3 skill/skill01_finger_tap/finger_tap.py

接口
  Sub  /skill01/target_point   geometry_msgs/PoseStamped    目标点（position=中心, orientation.z轴=法线; frame=head_roll_link）
  Pub  /inspire_hand/ctrl/left_hand  sensor_msgs/JointState     摆点按手型（食指伸直其余蜷起，见 atom03）
  TF   base ← head_roll_link / base ← left_tcp_link            目标变换 / 读末端起点
  qp后端     Action /endpose_single_arm_qp_L_controller/endPosSingleTarget（atom08 同款）
             Service .../set_parameters                          设 vel_limits 慢速
  moveit后端 Action /move_action（atom06 同款）
  Service 使能（候选自动探测）+ switch/list_controllers          同 atom06/08 已验证套路
"""

import sys
import copy
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import config as C
import pose_math as PM

try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (MotionPlanRequest, Constraints, PlanningOptions,
                                 PositionConstraint, OrientationConstraint)
    from shape_msgs.msg import SolidPrimitive
    from eai_manipulator_msgs.action import EndPosSingleTarget, JointSpace
    from eai_manipulator_msgs.msg import ArmTargetPose
    from controller_manager_msgs.srv import SwitchController, ListControllers
    from std_srvs.srv import SetBool
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from tf2_ros import Buffer, TransformListener
    from tf2_geometry_msgs import do_transform_point
except ImportError:
    print("❌ 找不到 moveit_msgs / eai_manipulator_msgs / tf2 等。请先 source XARM 环境：")
    print("   source /home/ubuntu/XARM/install/setup.bash")
    sys.exit(1)

MOVE_ACTION = "/move_action"
QP_ACTION = f"/{C.QP_CONTROLLER}/endPosSingleTarget"
SWITCH_SRV = "/controller_manager/switch_controller"
LIST_SRV = "/controller_manager/list_controllers"
ENABLE_SRV_CANDIDATES = ["/EAIHardware/set_arm_enable", "/moveit_controller_enable"]
# 出发位姿落盘文件：--recover 的退回依据（进程内变量会随失败/崩溃一起丢，必须落盘）
START_POSE_FILE = Path(__file__).resolve().parent / "_last_start_pose.json"


class FingerTap(Node):
    def __init__(self):
        super().__init__("skill01_finger_tap")
        self.moveit_client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.qp_client = ActionClient(self, EndPosSingleTarget, QP_ACTION)
        self.js_client = ActionClient(self, JointSpace, f"/{C.JOINTSPACE_CONTROLLER}/jointspace")
        self.switch_cli = self.create_client(SwitchController, SWITCH_SRV)
        self.list_cli = self.create_client(ListControllers, LIST_SRV)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._samples = []          # [(位置xyz, 法线xyz)]，法线=卡面朝外方向
        self.target_sub_ = self.create_subscription(
            PoseStamped, C.TARGET_TOPIC, self._on_target, 10)
        self.hand_pub = self.create_publisher(JointState, C.HAND_CMD_TOPIC, 10)
        self._cur_joints = {}       # name→角度(rad)，_on_joints 刷新，读起始关节角用
        self.joint_sub_ = self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10)

    def _on_joints(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._cur_joints[name] = pos

    def read_arm_joints(self, timeout=2.0):
        """读当前左臂 7 关节角(rad)，按 C.ARM_JOINT_NAMES 顺序；读不到返回 None（日志用）。"""
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(nm in self._cur_joints for nm in C.ARM_JOINT_NAMES):
                return [round(self._cur_joints[nm], 3) for nm in C.ARM_JOINT_NAMES]
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                return None

    @staticmethod
    def _fmt_ee(pos, quat):
        """末端位姿格式化：'(x,y,z)m rpy(r,p,y)°'（rpy 换算成度，仅供人看轨迹）。"""
        r = np.degrees(PM.quat_to_rpy(quat))
        return (f"({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})m "
                f"rpy({r[0]:+.0f},{r[1]:+.0f},{r[2]:+.0f})°")

    @staticmethod
    def _fmt_xyz(pos):
        """只格式化位置：'(x,y,z)m'（指尖用；指尖姿态=手腕姿态，看 tcp 的 rpy 即可）。"""
        return f"({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})m"

    def read_pad_pos(self, timeout=2.0):
        """读当前指肚(真接触面)在 base 的位置 = TAP_LINK(left_index_2) 实际位姿(TF 读，含旋转)
        叠加局部系固定偏移 PAD_LOCAL_OFFSET（网格标定，随手指实际姿态刚体旋转——姿态怎么变
        都物理正确，不像旧版假设"沿 tcp→TAP_LINK 连线方向"外推）。读不到返回 None。"""
        if not C.TAP_LINK:
            return None
        t0 = self.get_clock().now()
        while rclpy.ok():
            try:
                tr = self.tf_buffer.lookup_transform(C.BASE_FRAME, C.TAP_LINK, Time())
                t, q = tr.transform.translation, tr.transform.rotation
                pos = np.array([t.x, t.y, t.z])
                quat = [q.x, q.y, q.z, q.w]
                return pos + PM.rotate_vec(quat, C.PAD_LOCAL_OFFSET)
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                return None

    def stage(self, text):
        """打印醒目的阶段分隔行——让 log 一眼分清"现在到第几步"。
        细节（控制器切换/逐段路点/内部数学）走 logger.debug，默认不显示，需要时开 debug 看。"""
        self.get_logger().info(f"──────── {text} ────────")

    # ── ① 收目标：取 N 帧中位数（一次快照，之后不再更新）─────────────
    def _on_target(self, msg: PoseStamped):
        if len(self._samples) < C.N_SAMPLES:
            o = msg.pose.orientation
            n = PM.rotate_vec([o.x, o.y, o.z, o.w], [0.0, 0.0, 1.0])   # 姿态 z 轴 = 法线
            p = msg.pose.position
            self._samples.append(((p.x, p.y, p.z), tuple(n)))

    def wait_target(self, timeout=10.0):
        """等满 N 帧求中位数，返回 (中心 PointStamped, 法线 np 单位向量)，都在 head_roll_link 系。"""
        self.get_logger().info(f"等待 {C.TARGET_TOPIC} 凑满 {C.N_SAMPLES} 帧（tag_locator 在 Orin 跑着吗）...")
        t0 = self.get_clock().now()
        while rclpy.ok() and len(self._samples) < C.N_SAMPLES:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                self.get_logger().error(
                    f"超时未收到目标（收到 {len(self._samples)}/{C.N_SAMPLES}）。"
                    "自查：Orin 上 tag_locator 在跑？卡片在视野里？两板同 ROS_DOMAIN_ID？"
                    "（注意：新版 tag_locator 发 PoseStamped，两边代码要一起 git pull）")
                return None, None
        pts = np.asarray([s[0] for s in self._samples])
        nrm = np.asarray([s[1] for s in self._samples])
        spread = np.max(np.ptp(pts, axis=0))
        if spread > 0.05:
            self.get_logger().warn(
                f"目标点抖动 {spread:.3f}m 偏大（卡片太远/太小时位姿解会抖；已用中位数抗离群，"
                "仍建议卡片放近到 0.5m 内）")
        avg = np.median(pts, axis=0)          # 中位数：个别离群帧不带偏结果
        n = np.median(nrm, axis=0)
        n = n / np.linalg.norm(n)
        out = PointStamped()
        out.header.frame_id = C.CALIB_PARENT_FRAME
        out.point.x, out.point.y, out.point.z = map(float, avg)
        return out, n

    # ── ② TF：head_roll_link → base（点用平移+旋转，法线只旋转）───────
    def to_base(self, pt: PointStamped, normal, timeout=5.0):
        t0 = self.get_clock().now()
        while rclpy.ok():
            try:
                tr = self.tf_buffer.lookup_transform(C.BASE_FRAME, pt.header.frame_id, Time())
                q = tr.transform.rotation
                n_base = PM.rotate_vec([q.x, q.y, q.z, q.w], normal)
                return do_transform_point(pt, tr), n_base
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                self.get_logger().error(f"TF 超时：{C.BASE_FRAME} ← {pt.header.frame_id}。XARM/body 起了吗？")
                return None, None

    # ── 指尖补偿：查静态 TF 拿 tcp→TAP_LINK 偏移，叠加局部偏移得 tcp→指肚，tcp 目标按姿态后撤 ──
    def pad_offset(self, timeout=2.0):
        """返回 tcp 系下 EE_LINK→指肚(pad) 的固定偏移向量（np, m）；查不到返回 None。
        组成：查 EE_LINK→TAP_LINK 静态 TF（灵巧手经 fixed 关节挂在 tcp 下，恒定，只查一次即可），
        再叠加 TAP_LINK 局部系下的固定偏移 PAD_LOCAL_OFFSET（先转到 tcp 系再相加）。"""
        if not C.TAP_LINK:
            return None
        t0 = self.get_clock().now()
        while rclpy.ok():
            try:
                tr = self.tf_buffer.lookup_transform(C.EE_LINK, C.TAP_LINK, Time())
                t, q = tr.transform.translation, tr.transform.rotation
                o_tap = np.array([t.x, t.y, t.z])
                quat_tap = [q.x, q.y, q.z, q.w]
                return o_tap + PM.rotate_vec(quat_tap, C.PAD_LOCAL_OFFSET)
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                self.get_logger().warn(
                    f"查不到 {C.EE_LINK}→{C.TAP_LINK} 的 TF——本次不做指尖补偿，手可能冲过目标！"
                    "（TAP_LINK 名字对吗？ros2 run tf2_ros tf2_echo left_tcp_link left_index_2 核实）")
                return None

    def build_approach(self, p, n):
        """由卡片中心 p 与法线 n 算 (tcp 目标点, 目标姿态, 指肚目标)：
        接近方向按 ORIENT_MODE 取——"level"=法线水平投影（手指水平指向卡片，人按按钮姿态，
        相对 base 确定可复现）；"tag"=感知法线原样（斜卡垂直接近，姿态随感知漂）。
        先算"指肚应停的接近点"（中心沿接近方向退 APPROACH_OFFSET），
        再把 tcp 目标按 tcp→指肚 偏移后撤 R·o —— 让指肚而非腕端落在接近点（修灵巧手过冲）。"""
        if C.ORIENT_MODE == "level":
            h = np.array([n[0], n[1], 0.0])
            if np.linalg.norm(h) < 1e-6:                   # 法线近乎竖直的兜底：取正前方
                h = np.array([-1.0, 0.0, 0.0])
            n = h / np.linalg.norm(h)
            self.get_logger().debug(f"姿态模式 level：接近方向=法线水平投影 ({n[0]:+.3f}, {n[1]:+.3f}, 0)")
        center = np.array([p.x, p.y, p.z]) + np.asarray(C.AIM_BIAS_BASE, dtype=float)
        if any(C.AIM_BIAS_BASE):
            self.get_logger().debug(
                f"应用固定瞄准偏置 {C.AIM_BIAS_BASE}（吸收手眼外参/帧-指肚固定差）→ "
                f"修正后瞄准中心 ({center[0]:+.3f}, {center[1]:+.3f}, {center[2]:+.3f})")
        # ★pad_goal（指肚应停点）是开环与闭环的【统一基准】：tcp_goal=pad_goal−R·o、
        #   report/correct 也比它——不再经过"骨架点目标"这层中转（PAD_LOCAL_OFFSET 已经是
        #   指肚相对 TAP_LINK 的完整偏移，无需像旧版标量外推那样先退回骨架点再合成）。
        pad_goal = center + n * C.APPROACH_OFFSET          # 指肚应停这（卡前 APPROACH_OFFSET）
        quat = PM.quat_from_zaxis(n, spin=C.HAND_SPIN)
        o = self.pad_offset()
        if o is None:
            return pad_goal, quat, pad_goal
        world_o = np.asarray(PM.rotate_vec(list(quat), list(o)))   # 偏移转到 base 系
        tcp_goal = pad_goal - world_o
        self.get_logger().debug(
            f"指尖补偿：tcp→指肚 偏移长度 {np.linalg.norm(o):.3f}m（tcp系 {np.round(o, 3)}），"
            f"tcp 目标已按姿态后撤（含 PAD_LOCAL_OFFSET 指肚偏移）")
        return tcp_goal, quat, pad_goal

    def report_pad_error(self, pad_goal, label=""):
        """到位误差报告（对齐指肚版）：读当前指肚(真接触面)实际位置(base)与 pad_goal 比，
        打印并返回 Δ=指肚实际−目标。指肚 = TAP_LINK(left_index_2) 实际位姿 叠加局部固定偏移
        PAD_LOCAL_OFFSET。比对齐骨架点更贴接触点：手腕朝向每次不同→指肚随之上下挪，只有
        直接盯指肚才能把这块散布吸收掉。这个 Δ 只含【规划+执行】环节；感知(tag+外参)误差
        要靠尺量指肚↔卡面对比 8cm 分离出来。读不到返回 None。"""
        if not C.TAP_LINK:
            return None
        pad = self.read_pad_pos()
        if pad is None:
            return None
        d = pad - np.asarray(pad_goal, dtype=float)
        # 偏差拆成"左右/上下/前后"（对应 base 的 y/z/x），直接对上要调的参数
        self.get_logger().info(
            f"  {label}偏差 {np.linalg.norm(d)*100:.1f}cm ＝ 左右 {d[1]*100:+.1f} / "
            f"上下 {d[2]*100:+.1f} / 前后 {d[0]*100:+.1f} cm")
        self.get_logger().debug(
            f"  预期指肚=({pad_goal[0]:+.3f},{pad_goal[1]:+.3f},{pad_goal[2]:+.3f}) "
            f"实际指肚=({pad[0]:+.3f},{pad[1]:+.3f},{pad[2]:+.3f})")
        return d

    # ── ③ SAFE_BOX 白名单：出盒拒动（config.SAFE_BOX=None 时关闭）──
    def check_safe(self, p):
        if C.SAFE_BOX is None:
            self.get_logger().warn(
                "SAFE_BOX 已关闭：不校验目标点范围。回车确认前请人工核对上面打印的坐标是否合理"
                "（感知算错时没有软件闸拦截）")
            return True
        for axis in ("x", "y", "z"):
            lo, hi = C.SAFE_BOX[axis]
            v = getattr(p.point, axis)
            if not (lo <= v <= hi):
                self.get_logger().error(
                    f"目标点 {axis}={v:+.3f} 超出 SAFE_BOX {C.SAFE_BOX[axis]}，拒绝执行。"
                    "卡片放对位置了吗？确是新位置就改 config.SAFE_BOX")
                return False
        return True

    # ── 摆点按手型：只伸食指(+拇指)，其余蜷起（抬臂前调）──────────────
    def set_point_pose(self):
        """把灵巧手摆成"点按手型"：食指伸直、小/无名/中指蜷起、拇指伸展。
        抬臂/接近前调，其它手指不再蹭卡片所在的板面。食指保持伸直=TF/PAD_LOCAL_OFFSET 前提。"""
        if not C.HAND_POSE_ENABLE:
            return
        t0 = self.get_clock().now()
        while self.hand_pub.get_subscription_count() == 0:      # 等手驱动订上，否则消息丢弃
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - t0).nanoseconds > 3e9:
                self.get_logger().warn(
                    f"{C.HAND_CMD_TOPIC} 无订阅者——手驱动(inspire_hand)起了吗？本次跳过摆手型")
                return
        ids = list(C.POINT_POSE.keys())
        msg = JointState()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.name = ids
        msg.position = [float(C.POINT_POSE[i]) for i in ids]
        self.hand_pub.publish(msg)
        self.get_logger().info(f"摆点按手型：食指伸直、其余蜷起 {C.POINT_POSE}")
        t0 = self.get_clock().now()                             # 给手指运动留 1.5s
        while (self.get_clock().now() - t0).nanoseconds < 1.5e9:
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_hand_open(self):
        """恢复手张开（归位时用）：所有手指置 1.0。HAND_POSE_ENABLE=False 或手驱动未在线则跳过。"""
        if not C.HAND_POSE_ENABLE or self.hand_pub.get_subscription_count() == 0:
            return
        ids = list(C.POINT_POSE.keys())
        msg = JointState()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.name = ids
        msg.position = [1.0] * len(ids)
        self.hand_pub.publish(msg)
        self.get_logger().debug("手已恢复张开")

    # ── 使能 + 切控制器（按后端选目标控制器，自动避让占臂者）────────
    def enable_arm(self):
        for srv in ENABLE_SRV_CANDIDATES:
            cli = self.create_client(SetBool, srv)
            if not cli.wait_for_service(timeout_sec=3.0):
                self.destroy_client(cli)
                continue
            future = cli.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            resp = future.result()
            if resp is not None and resp.success:
                self.get_logger().info(f"✓ 手臂已使能（{srv}）")
                return True
            self.get_logger().warn(f"{srv} 使能未成功，试下一个候选...")
        self.get_logger().warn("使能未确认（若上个 demo 已使能过则不影响；否则先 stop_all.sh 清场）")
        return True   # 使能是持续状态，之前使能过时重复调会被拒——不据此中止

    def activate_arm_controller(self, want=None):
        """激活指定控制器（默认按后端选）；先停掉占本臂的其它 active 控制器。"""
        if want is None:
            want = C.QP_CONTROLLER if C.ARM_BACKEND == "qp" else C.MOVEIT_CONTROLLER
        if not self.switch_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"{SWITCH_SRV} 不在。XARM 起了吗？")
            return False
        to_stop, already = [], False
        if self.list_cli.wait_for_service(timeout_sec=3.0):
            future = self.list_cli.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            resp = future.result()
            if resp is not None:
                for c in resp.controller:
                    if c.state != "active":
                        continue
                    if c.name == want:
                        already = True
                        continue
                    if any(ci.split("/")[0] in C.ARM_JOINT_NAMES for ci in c.claimed_interfaces):
                        to_stop.append(c.name)
        if already and not to_stop:
            self.get_logger().debug(f"{want} 已激活且无冲突")
            return True
        req = SwitchController.Request()
        req.activate_controllers = [] if already else [want]
        req.deactivate_controllers = to_stop
        req.strictness = SwitchController.Request.BEST_EFFORT
        if to_stop:
            self.get_logger().debug(f"先停占臂控制器: {to_stop}")
        future = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        ok = future.result() is not None and future.result().ok
        self.get_logger().debug(f"激活 {want}: {'成功' if ok else '未确认(可能已激活)'}")
        return True

    def set_vel_limits(self, force=False):
        """QP 后端慢速：endpose 和 jointspace 两个控制器都设（预备段默认速度快，必须压下来）。
        force=True 时不看后端（--recover 恢复恒走 QP，即使 config 配的是 moveit）。"""
        if (C.ARM_BACKEND != "qp" and not force) or C.VEL_LIMITS is None:
            return
        p = Parameter(name="vel_limits", value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(x) for x in C.VEL_LIMITS]))
        for ctrl in (C.QP_CONTROLLER, C.JOINTSPACE_CONTROLLER):
            cli = self.create_client(SetParameters, f"/{ctrl}/set_parameters")
            if not cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(f"{ctrl}/set_parameters 不在，跳过其调速")
                self.destroy_client(cli)
                continue
            future = cli.call_async(SetParameters.Request(parameters=[p]))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            resp = future.result()
            ok = resp is not None and resp.results and resp.results[0].successful
            self.get_logger().debug(f"{ctrl} vel_limits={C.VEL_LIMITS[0]}: {'✓' if ok else '未确认'}")

    def goto_ready(self, prefix=""):
        """关节空间走到预备姿态（大范围位移不经 IK，杜绝手腕/手扭曲）。
        prefix：日志前缀（归位时传 "⑤ "，开场留空）。"""
        if not self.js_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"jointspace action 不在。{C.JOINTSPACE_CONTROLLER} 激活了吗？")
            return False
        cur_j = self.read_arm_joints()
        target_j = [round(x, 3) for x in C.READY_JOINTS]
        if cur_j is not None:
            self.get_logger().info(
                f"{prefix}[jointspace QP｜关节] 回预备姿态：关节(rad) {cur_j} → {target_j}")
        else:
            self.get_logger().info(
                f"{prefix}[jointspace QP｜关节] 回预备姿态：目标关节(rad) {target_j}")
        goal = JointSpace.Goal(target_positions=[float(x) for x in C.READY_JOINTS])
        send_future = self.js_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("预备姿态目标被拒（超限位？控制器没激活？）")
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        res = result_future.result()
        if res is None or not res.result.success:
            self.get_logger().error(f"预备姿态失败：{res.result.result_msg if res else '超时'}")
            return False
        self.get_logger().info("✓ 到预备姿态")
        return True

    # ── 读当前末端位姿 ──────────────────────────────────────────────
    def read_ee_pose(self, timeout=5.0):
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tr = self.tf_buffer.lookup_transform(C.BASE_FRAME, C.EE_LINK, Time()).transform
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (
                    tr.translation.x, tr.translation.y, tr.translation.z)
                pose.orientation = tr.rotation
                return pose
            except Exception:
                pass
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                self.get_logger().error(f"超时未读到 {C.EE_LINK} 位姿（TF）")
                return None

    # ── qp 后端：单段下发（atom08 同款）+ 分段接近 ──────────────────
    def qp_move_once(self, pose, quiet=False):
        """把一个末端位姿发给 QP 控制器（阻塞等结果）。段间距离须 ≤ dis_err_bound。
        quiet=True（点按用）：失败降为 info——触卡面推不动而超时是预期，不当报错。"""
        if not self.qp_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"QP action {QP_ACTION} 不在。{C.QP_CONTROLLER} 激活了吗？")
            return False
        target = ArmTargetPose()
        target.header.frame_id = C.BASE_FRAME
        target.target = pose
        target.from_frame = C.BASE_FRAME
        target.to_frame = C.EE_LINK
        target.offset_x = target.offset_y = target.offset_z = 0.0
        goal = EndPosSingleTarget.Goal(target=target)
        send_future = self.qp_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("QP 段目标被拒（超误差限？控制器没激活？）")
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        res = result_future.result()
        if res is None or not res.result.success:
            msg = res.result.result_msg if res is not None else "超时"
            if quiet:
                self.get_logger().debug(f"QP 段未完成（点按触阻力，预期内）：{msg}")
            else:
                self.get_logger().error(f"QP 段执行失败：{msg}")
            return False
        return True

    def move_segmented(self, target_xyz, target_quat, contact_ok=False):
        """从当前末端分段走到目标：位置线性插值(≤QP_STEP/段) + 姿态 slerp 渐变(≤ORI_STEP/段)。
        target_quat: [x,y,z,w]。姿态和位置同步到位，手腕不突变、不会被 QP 姿态误差限拒。
        contact_ok=True（点按前进用）：某段触阻力失败=已按到卡面，停止前进并返回 True（不报错、
        不提示 --recover）。返回值：走完 True；contact_ok 下触阻力停下也 True；接近段失败 False。"""
        cur = self.read_ee_pose()
        if cur is None:
            return False
        p0 = np.array([cur.position.x, cur.position.y, cur.position.z])
        p1 = np.asarray(target_xyz, dtype=float)
        q0 = np.array([cur.orientation.x, cur.orientation.y,
                       cur.orientation.z, cur.orientation.w])
        q1 = np.asarray(target_quat, dtype=float)
        dist = float(np.linalg.norm(p1 - p0))
        ang = PM.quat_angle(q0, q1)
        n_seg = max(1, int(np.ceil(dist / C.QP_STEP)), int(np.ceil(ang / C.ORI_STEP)))
        self.get_logger().debug(
            f"分段移动：距离 {dist:.3f}m + 转角 {np.degrees(ang):.0f}° → {n_seg} 段")
        for i in range(1, n_seg + 1):
            t = i / n_seg
            wp = p0 + (p1 - p0) * t
            wq = PM.slerp(q0, q1, t)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = wp
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = wq
            self.get_logger().debug(f"  段 {i}/{n_seg} → ({wp[0]:+.3f}, {wp[1]:+.3f}, {wp[2]:+.3f})")
            if not self.qp_move_once(pose, quiet=contact_ok):
                if contact_ok:
                    self.get_logger().info("  触到卡面（endpose QP 阻力，正常），停止前进")
                    return True
                self.get_logger().error(
                    f"第 {i}/{n_seg} 段失败，停在当前位置。收臂："
                    "python3 skill/skill01_finger_tap/finger_tap.py --recover")
                return False
        return True

    # ── 到位闭环修正：按实测 Δ 反向平移，姿态不动 ───────────────────
    def correct_tip(self, pad_goal, delta):
        """病根：MoveIt 姿态容差(±17°/±34°)×腕→指尖 17cm 杠杆 → 每次手腕朝向不同，指肚
        （TAP_LINK 位姿 + PAD_LOCAL_OFFSET 局部偏移）随之随机散布 ±2~3cm。修法（人补一刀
        的做法）：TF 实测指肚 Δ → 位置平移 −Δ、姿态保持当前实际值（不给容差第二次机会）→ 重测。
        ★对齐指肚而非骨架点：平移把指尖悬臂 + 手腕朝向差一起吸收（骨架点已对齐、指肚仍散
        就是这来的）。QP 短程执行（2~3cm 一两段，是 QP 舒适区；长扫掠才会拖手撞身）。"""
        self.set_vel_limits(force=True)          # 修正恒走 QP，moveit 后端也要压速
        if delta is not None and float(np.linalg.norm(delta)) > C.CORRECT_TOL:
            self.get_logger().info(
                f"② [endpose QP｜末端] 微调：偏差偏大，闭环补刀（对齐指肚，"
                f"最多 {C.CORRECT_MAX} 轮，姿态不动、只平移补 Δ）...")
        for i in range(C.CORRECT_MAX):
            if delta is None or float(np.linalg.norm(delta)) <= C.CORRECT_TOL:
                break
            cur = self.read_ee_pose()
            if cur is None:
                return
            target_xyz = (cur.position.x - float(delta[0]),
                          cur.position.y - float(delta[1]),
                          cur.position.z - float(delta[2]))
            cur_quat = [cur.orientation.x, cur.orientation.y,
                        cur.orientation.z, cur.orientation.w]
            self.get_logger().debug(
                f"闭环修正 {i + 1}/{C.CORRECT_MAX}：平移 −Δ="
                f"({-delta[0]:+.3f},{-delta[1]:+.3f},{-delta[2]:+.3f})（姿态不动）")
            self.activate_arm_controller(C.QP_CONTROLLER)
            if not self.move_segmented(target_xyz, cur_quat):
                self.get_logger().warn("修正段失败，保持当前位置（不影响后续退回）")
                return
            # 等 0.4s 让臂落稳再测：QP action"目标已接受"就返回、臂还在追（实测指令
            # 平移 3.5cm 时 90ms 后测只挪了 1.1cm）；顺带 spin 让 TF 缓冲吃到新数据
            t0 = self.get_clock().now()
            while rclpy.ok() and (self.get_clock().now() - t0).nanoseconds < 0.4e9:
                rclpy.spin_once(self, timeout_sec=0.05)
            delta = self.report_pad_error(pad_goal, label="微调后")
        if delta is not None:
            fin = float(np.linalg.norm(delta)) * 100
            tag = "✓ 已到位" if fin <= C.CORRECT_TOL * 100 else "（已用满修正轮数）"
            self.get_logger().info(f"就位完成：最终偏差 {fin:.1f}cm {tag}")

    # ── 按下前摆正手掌到名义水平朝向 ──────────────────────────────
    def level_wrist(self, target_quat):
        """把手腕摆到名义朝向（水平·掌心朝下），消掉 MoveIt/QP 残留的手掌倾斜。
        ★2026-08：本函数当初是为了消掉旧 pad 模型（沿 tcp→index_2 连线做标量外推，假设手指
        笔直）的姿态相关误差——那个近似只在名义姿态下标定准，手掌一歪指肚就离卡面差几毫米。
        现在 read_pad_pos 已改用"TAP_LINK 实际姿态 + 局部固定偏移 PAD_LOCAL_OFFSET"，偏移随
        手指刚体旋转，理论上姿态怎么变都物理正确，本函数存在的必要性存疑，但尚未真机重新
        验证，config.LEVEL_WRIST 暂保留默认开启（见 config 注释）。
        只转朝向、tcp 位置不变（指肚多在指向轴附近，转动带来的位移小；随后的位置闭环再补）。"""
        cur = self.read_ee_pose()
        if cur is None:
            return
        cur_quat = [cur.orientation.x, cur.orientation.y,
                    cur.orientation.z, cur.orientation.w]
        ang = np.degrees(PM.quat_angle(np.asarray(cur_quat),
                                       np.asarray(target_quat, dtype=float)))
        if ang < 2.0:                       # 已经基本水平，免得多走一段
            self.get_logger().debug(f"手掌已接近名义朝向（偏 {ang:.0f}°），跳过摆正")
            return
        self.get_logger().info(
            f"② [endpose QP｜末端] 摆正手掌：转到名义水平朝向（当前偏 {ang:.0f}°，"
            f"消 pad 模型的姿态相关误差）...")
        self.activate_arm_controller(C.QP_CONTROLLER)
        self.set_vel_limits(force=True)
        self.move_segmented([cur.position.x, cur.position.y, cur.position.z],
                            list(target_quat))

    # ── 阶段3：点按（③前进按下 → 回车确认姿态）；退回归到阶段4 ──────────
    def press_only(self, n):
        """③ 按下 + 回车确认。从停驻位沿 −n（指向卡片）前进 APPROACH_OFFSET+PRESS_DEPTH：
        指肚触卡面并过冲 PRESS_DEPTH（柔顺吸收）；姿态全程不动。触卡推不动而超时=已按到。
        ★按到后停住不退，等人工回车确认点按姿态（最关键的检查点）。
        ★按下方向与伸手一致：level 模式取法线水平投影，忽略法线俯仰噪声，按下走水平、不上下漂。
        返回停驻位 (xyz np, quat list) 供阶段4 退回用；读不到起点返回 (None, None)。"""
        n = np.asarray(n, dtype=float)
        if C.ORIENT_MODE == "level":            # 与 build_approach 同款水平投影
            h = np.array([n[0], n[1], 0.0])
            if np.linalg.norm(h) > 1e-6:
                n = h / np.linalg.norm(h)
        stand = self.read_ee_pose()
        if stand is None:
            return None, None
        sp = np.array([stand.position.x, stand.position.y, stand.position.z])
        sq = [stand.orientation.x, stand.orientation.y,
              stand.orientation.z, stand.orientation.w]
        advance = C.APPROACH_OFFSET + C.PRESS_DEPTH
        press_xyz = sp - n * advance            # 沿 −n 前进（指向卡片，水平）
        pad0 = self.read_pad_pos()
        self.get_logger().info(
            f"③ [endpose QP｜末端] 按下（前进 {advance * 100:.1f}cm = "
            f"{C.APPROACH_OFFSET * 100:.0f}cm 到卡面 + {C.PRESS_DEPTH * 100:.1f}cm 过冲，柔顺吸收）：")
        self.get_logger().info(
            f"   手腕 tcp(base系) {self._fmt_ee(sp, sq)} → {self._fmt_ee(press_xyz, sq)}")
        if pad0 is not None:
            pad1 = pad0 + (press_xyz - sp)          # 纯平移：指肚位移 = tcp 位移
            self.get_logger().info(
                f"   指尖 指肚(base系) {self._fmt_xyz(pad0)} → {self._fmt_xyz(pad1)}"
                f"   ← 核心：指肚推进到卡面")
        self.activate_arm_controller(C.QP_CONTROLLER)
        self.set_vel_limits(force=True)
        self.move_segmented(press_xyz, sq, contact_ok=True)     # 触阻力停下也算按到
        self.get_logger().info("★ 已按到，保持按下中——请检查点按姿态（最关键的一步）")
        input("检查完点按姿态 → 回车退回并归位（Ctrl-C 中断→臂停在按下位，用 --recover 收臂）...")
        return sp, sq

    def retract_to_standoff(self, sp, sq):
        """④ 退回停驻位：沿原路把指肚从卡面撤回到按下前的停驻位（归位前先安全离开卡面）。"""
        cur = self.read_ee_pose()
        self.get_logger().info("④ [endpose QP｜末端] 退回停驻位：")
        if cur is not None:
            cur_pos = np.array([cur.position.x, cur.position.y, cur.position.z])
            cur_q = [cur.orientation.x, cur.orientation.y,
                     cur.orientation.z, cur.orientation.w]
            pad0 = self.read_pad_pos()
            self.get_logger().info(
                f"   手腕 tcp(base系) {self._fmt_ee(cur_pos, cur_q)} → {self._fmt_ee(sp, sq)}")
            if pad0 is not None:
                pad1 = pad0 + (np.asarray(sp, dtype=float) - cur_pos)   # 纯平移：指肚位移=tcp位移
                self.get_logger().info(
                    f"   指尖 指肚(base系) {self._fmt_xyz(pad0)} → {self._fmt_xyz(pad1)}"
                    f"   ← 核心：指肚撤离卡面")
        else:
            self.get_logger().info(f"   手腕 tcp(base系) → {self._fmt_ee(sp, sq)} 停驻位")
        self.activate_arm_controller(C.QP_CONTROLLER)
        return self.move_segmented((sp[0], sp[1], sp[2]), sq)

    # ── 出发位姿落盘（--recover 的退回依据）─────────────────────────
    def save_start_pose(self, pose):
        """动臂前把出发位姿写进 JSON：段失败/Ctrl-C/进程死掉后，--recover 据此退回。"""
        try:
            START_POSE_FILE.write_text(json.dumps({
                "stamp": time.time(),
                "xyz": [pose.position.x, pose.position.y, pose.position.z],
                "quat": [pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w]}))
        except Exception as e:
            self.get_logger().warn(f"出发位姿落盘失败（不影响本次执行，只影响 --recover）：{e}")

    # ── moveit 后端：一步规划（atom06 同款；XARM 修复 99999 后可用）──
    def _pose_goal(self, pose, pos_tol=0.01, ori_tol=0.05, ori_tol_z=None):
        """位置球约束 + 逐轴姿态约束。ori_tol_z 单独放开 z 轴（绕指向轴的自旋）——
        点按任务只要求指尖方向对准卡面（x/y 紧），绕指向轴转多少无所谓（z 松），
        reach_check 实测：全紧会 99999 过约束、全松会给贴限位的扭曲解，逐轴才是正解。"""
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = C.BASE_FRAME
        pc.link_name = C.EE_LINK
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol]))
        pc.constraint_region.primitive_poses.append(
            Pose(position=pose.position, orientation=Quaternion(w=1.0)))
        pc.weight = 1.0
        c.position_constraints.append(pc)
        oc = OrientationConstraint()
        oc.header.frame_id = C.BASE_FRAME
        oc.link_name = C.EE_LINK
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol if ori_tol_z is None else ori_tol_z
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def moveit_move_to_pose(self, pose, ori_tol=0.3, ori_tol_z=None):
        """ori_tol_z 默认取 config.SPIN_TOL：自旋围绕 HAND_SPIN 名义值±适度容差——
        全放开(3.14)实测 MoveIt 会随便挑解、掌心外翻；全卡死(0.05)又 99999 过约束。"""
        if ori_tol_z is None:
            ori_tol_z = C.SPIN_TOL
        if not self.moveit_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"MoveGroup action {MOVE_ACTION} 不在。MoveIt 组件起了吗？")
            return False
        req = MotionPlanRequest()
        req.group_name = C.GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = C.VEL_SCALE
        req.max_acceleration_scaling_factor = C.ACC_SCALE
        req.goal_constraints.append(self._pose_goal(pose, ori_tol=ori_tol, ori_tol_z=ori_tol_z))
        goal = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=False))
        send_future = self.moveit_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("目标被拒绝/发送超时")
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        res = result_future.result()
        if res is None:
            self.get_logger().error("执行结果超时(60s)")
            return False
        code = res.result.error_code.val
        if code == 1:
            return True
        self.get_logger().error(
            f"MoveIt 失败 error_code={code}（99999 首查起点越界：tmux capture-pane -t xarm.1 -p -J -S -400"
            f" | grep -i 'outside bounds' 点名关节后用 QP 挪回，见 atom05 guide 排错表；其次查姿态过约束）")
        return False


def recover(node):
    """--recover：段失败/中断后收臂。优先按 _last_start_pose.json QP 慢速原路退回出发位，
    再（若配置了 READY_JOINTS）关节空间归位。恒走 QP，不管 config 后端是什么。"""
    log = node.get_logger()
    data = None
    if START_POSE_FILE.exists():
        try:
            data = json.loads(START_POSE_FILE.read_text())
        except Exception:
            log.warn(f"{START_POSE_FILE.name} 损坏，忽略")
    if data is None and C.READY_JOINTS is None:
        log.error("没有可退回的目标：无出发位记录（旧代码跑的？）且 READY_JOINTS 未配置。"
                  "自动恢复已停止；请由现场人员按安全规程处置，勿继续运行。")
        return
    if data is not None:
        age_h = (time.time() - data["stamp"]) / 3600.0
        xyz = data["xyz"]
        log.info(f"退回目标=上次出发位 ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f})"
                 f"（{age_h:.1f} 小时前记录）")
        if age_h > 12:
            log.warn("记录超过 12 小时，可能不是本次意外的出发位——确认合理再回车")
    input("★确认臂周边无人、退回路径无障碍 → 回车开始收臂（Ctrl-C 取消）...")
    node.enable_arm()
    node.set_vel_limits(force=True)
    if data is not None:
        node.activate_arm_controller(C.QP_CONTROLLER)
        if not node.move_segmented(data["xyz"], data["quat"]):
            log.error("退回出发位失败，臂仍停在半路——自动恢复已停止；"
                      "请由现场人员按安全规程处置。")
            return
        log.info("✓ 已退回出发位")
    if C.READY_JOINTS is not None:
        node.activate_arm_controller(C.JOINTSPACE_CONTROLLER)
        if node.goto_ready():
            log.info("✓ 已归位 READY，恢复完成")
    elif data is not None:
        log.info("READY_JOINTS 未配置，恢复止步于出发位（建议尽快录 READY，见文件头）")


def main():
    rclpy.init()
    node = FingerTap()
    log = node.get_logger()
    if "--recover" in sys.argv:
        try:
            recover(node)
        except KeyboardInterrupt:
            log.warn("用户中断")
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return
    log.info("════════ skill01 · 手指点按 ════════")
    log.info(f"流程：①看到卡片 → ②伸手到卡前 {C.APPROACH_OFFSET * 100:.0f}cm 并修正 → "
             f"③点按 → ④收手归位")
    input("★安全确认：卡片已固定、人已退出手臂范围、急停在手 → 回车继续（Ctrl-C 取消）...")

    try:
        # ═══ 阶段1 · 看：收 tag_locator(Orin) 发来的中心+法线，变换到 base 系 ═══
        node.stage("阶段1/4 · 看：读取 tag 位置")
        pt_head, n_head = node.wait_target()
        if pt_head is None:
            return
        pt_base, n = node.to_base(pt_head, n_head)
        if pt_base is None:
            return
        p = pt_base.point
        if n[0] > 0:                    # 法线应指向机器人（base -x 方向）；反了就翻转
            log.warn("法线朝向异常（背向机器人），已自动翻转")
            n = -n
        log.info(f"tag 中心(base系, m) = ({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})")
        log.info(f"tag 法线(base系)    = ({n[0]:+.3f}, {n[1]:+.3f}, {n[2]:+.3f})（卡面朝向，决定手怎么对准）")

        if not node.check_safe(pt_base):
            return

        approach_xyz, target_quat, pad_goal = node.build_approach(p, n)
        log.info(f"→ 指尖(指肚)要停到：tag 前方 {C.APPROACH_OFFSET * 100:.0f}cm 处（沿法线方向）")
        log.info(f"  指尖 指肚目标(base系, m) = "
                 f"({pad_goal[0]:+.3f}, {pad_goal[1]:+.3f}, {pad_goal[2]:+.3f})   ← 核心：真接触点要到这")
        log.info("  指肚 = TF 里 left_index_2 实际位姿 叠加局部固定偏移 PAD_LOCAL_OFFSET（随姿态刚体旋转）")
        log.info(f"  控制器实际控手腕 tcp，换算后 tcp 目标(base系, m) = "
                 f"({approach_xyz[0]:+.3f}, {approach_xyz[1]:+.3f}, {approach_xyz[2]:+.3f})   ← 手段：tcp 带指肚过去")
        input("核对以上坐标合理 → 回车开始伸手（Ctrl-C 取消）...")

        # ═══ 阶段2 · 就位：使能→摆手型→回预备姿态→伸到卡前→闭环修正 ═══
        node.stage("阶段2/4 · 就位：伸手到卡前并修正误差")
        node.enable_arm()
        node.set_vel_limits()
        node.set_point_pose()           # 先蜷起其余手指，避免抬臂时蹭板面

        # 先回预备姿态（大范围位移在关节空间走，不经 IK、不扭曲；起点每次一致防漂移）
        if C.READY_JOINTS is not None:
            if not node.activate_arm_controller(C.JOINTSPACE_CONTROLLER):
                return
            if not node.goto_ready():
                return
        else:
            log.warn("READY_JOINTS 未配置——起点不确定，会导致姿态累计漂移/扭曲！录制方法见文件头")

        node.activate_arm_controller()          # 切末端控制器（qp/moveit）
        start = node.read_ee_pose()
        if start is None:
            return
        sp = start.position
        log.info(f"起始手腕 tcp(base系, m) = ({sp.x:+.3f}, {sp.y:+.3f}, {sp.z:+.3f})")
        node.save_start_pose(start)     # 落盘：中途失败后 --recover 据此退回

        start_quat = [start.orientation.x, start.orientation.y,
                      start.orientation.z, start.orientation.w]
        pad0 = node.read_pad_pos()
        log.info(f"① [{'endpose QP' if C.ARM_BACKEND == 'qp' else 'MoveIt'}｜末端] 伸手：")
        log.info(f"   手腕 tcp(base系) {node._fmt_ee([sp.x, sp.y, sp.z], start_quat)} → "
                 f"{node._fmt_ee(approach_xyz, target_quat)}")
        if pad0 is not None:
            log.info(f"   指尖 指肚(base系) {node._fmt_xyz(pad0)} → {node._fmt_xyz(pad_goal)}"
                     f"   ← 核心：伸到卡前 {C.APPROACH_OFFSET * 100:.0f}cm")
        if C.ARM_BACKEND == "qp":
            ok = node.move_segmented(approach_xyz, target_quat)
        else:
            target = copy.deepcopy(start)
            target.position.x, target.position.y, target.position.z = approach_xyz
            (target.orientation.x, target.orientation.y,
             target.orientation.z, target.orientation.w) = target_quat
            ok = node.moveit_move_to_pose(target)
        if ok:
            if C.LEVEL_WRIST:
                node.level_wrist(target_quat)     # 先摆正手掌到名义水平，消 pad 模型姿态相关误差
            delta = node.report_pad_error(pad_goal, label="伸手到位")   # 预期 vs 实际（指肚）
            if C.TIP_CORRECT:
                node.correct_tip(pad_goal, delta)     # ② 微调：endpose QP 闭环对齐指肚

            # ═══ 阶段3 · 点按：前进按下 → 保持 → 退回停驻位 ═══
            if C.PRESS_ENABLE:
                input(f"★点按确认：卡片固定、路径无人、急停在手 → 回车按下"
                      f"（前进 {(C.APPROACH_OFFSET + C.PRESS_DEPTH) * 100:.1f}cm = 到卡面 + 过冲 "
                      f"{C.PRESS_DEPTH * 100:.1f}cm；config.PRESS_ENABLE=False 可只伸手不按；"
                      f"Ctrl-C 中断→臂停原位，用 --recover 收臂）...")
                node.stage("阶段3/4 · 点按")
                stand_xyz, stand_q = node.press_only(n)   # ③按下+回车确认；退回归到阶段4
            else:
                input("检查完 → 回车返回（未开启点按）...")
                stand_xyz = stand_q = None

            node.stage("阶段4/4 · 收手归位")
            # ④ 先退回停驻位：把指肚从卡面撤回（沿原路，安全离开卡面），再做归位。
            if stand_xyz is not None:
                node.retract_to_standoff(stand_xyz, stand_q)
            # 归位：
            #   有 READY_JOINTS → 直接关节空间归位。跳过"末端返回出发位"——那步会因 QP 把
            #     关节留在 MoveIt 界外，导致回 moveit 规划起点越界报 99999（功能无害但刷红、
            #     且与 goto_ready 重复）。关节空间归位不吃 MoveIt 限位，稳。
            #   无 READY_JOINTS → 只能用末端返回出发位（退离卡片）。
            if C.READY_JOINTS is not None:
                node.activate_arm_controller(C.JOINTSPACE_CONTROLLER)
                node.goto_ready(prefix="⑤ ")
                node.set_hand_open()
                log.info("✓ 全部完成：臂回预备姿态、手已张开")
            else:
                log.info(f"⑤ [{'endpose QP' if C.ARM_BACKEND == 'qp' else 'MoveIt'}｜末端] "
                         f"返回出发位：手腕 tcp → ({sp.x:+.3f},{sp.y:+.3f},{sp.z:+.3f})")
                node.activate_arm_controller()   # 修正/点按切到 QP，退回前恢复后端控制器
                if C.ARM_BACKEND == "qp":
                    node.move_segmented((sp.x, sp.y, sp.z), start_quat)
                else:
                    node.moveit_move_to_pose(start)
                node.set_hand_open()
                log.info("✓ 全部完成：已返回出发位、手已张开")
    except KeyboardInterrupt:
        log.warn("用户中断")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
