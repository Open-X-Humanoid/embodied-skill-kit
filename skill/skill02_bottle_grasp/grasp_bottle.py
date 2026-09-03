#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill02_bottle_grasp · 阶段2：看到瓶子 → 伸手 → 包裹式抓握 → 搬运放下 → 收手归位

一句话
  收 bottle_locator 发的瓶子位置（凑几帧取中位数压抖动）→ 算抓握目标（瓶子位置 − TCP_OFFSET，
  拖动示教实测的3D偏移）→ 接近分三步（①aMoveIt到安全中间点 ①bQP下探到停驻点(顺带摆正
  姿态) ③QP精修平移）→ 手闭合到标定好的抓握手型 → 竖直提起确认 → wsad+回车 交互式微调
  水平位置（纯开环，自己看着挪，别挪出支撑面）→ 下降放下 → 松手 → 先升到安全中间点、
  再沿固定方向退回停驻位 → 关节空间归位 READY。

不依赖 MoveIt 的碰撞检测
  没有把箱子注册成 MoveIt 的碰撞体：MoveIt 自带的避障是否精确检测手部几何存疑，作为
  教程代码不希望引入这种不确定性。安全性完全靠自己算的路径点（build_intermediate_point，
  箱子数据只喂给这个函数，不喂给 MoveIt）——理解这一点很重要：安全来自"选对了端点"，
  不是"MoveIt 帮忙躲开了障碍"。

设计依据
  瓶子旋转对称，不需要按目标方向算接近姿态。姿态整个固定死（GRASP_ORIENT，全程不变），
  只有"抓握点位置"随瓶子位置变——平移方向也固定（GRASP_DIR，是"手部姿态=GRASP_ORIENT
  时缺口朝向的 base 系方向"，真机目视标定出来的常量，不是算出来的）。

接近分三步，不是简单一步到位
  ①a MoveIt 到安全中间点：大范围移动，会绕障，但姿态容差松、落地姿态不精确；中间点特意
    选在箱子 y 方向最远处再加一点余量（build_intermediate_point），把这段"不确定会不会
    贴着箱子扫过"的大范围移动限制在明显甩开箱子的地方。
  ①b QP 下探到停驻点：从中间点只调 y/z（x 不变），QP 是精确执行（不像 MoveIt 有容差内
    随便挑解的问题），顺带就把姿态收到了精确的 GRASP_ORIENT，不需要再单独一步"摆正姿态"。
  ③ QP 精修平移：姿态已锁死，沿 GRASP_DIR 走最后一段到精确抓握位置——这时平移方向才可信。
  MoveIt 独自做不到"姿态精确复现"，QP 独自做不到"大范围绕障"，两者分工缺一不可。

没有真正意义上的闭环修正——这是本技能最大的局限之一，务必知道
  手指是 fixed 关节，没有标定过"缺口相对 tcp"的偏移帧，没有能测的"缺口/瓶子有没有真的
  对齐"这个反馈量。当前只能核对"tcp 有没有精确到达下发目标"（QP 本身该精确执行，这只是
  兜底诊断，不是误差修正）。精度完全依赖开环：感知准 + TCP_OFFSET/GRASP_DIR 标定准。

手没有力反馈
  手型闭合量是真机试出来的固定值（config.HAND_GRASP_POSE），不是靠力控收敛的——闭合到
  这个值就停，不判断"有没有真的抓稳"，跟点按任务同理，靠人工确认。

跑在哪（x86，ubuntu 用户；Orin 上 bottle_locator.py + box_locator.py 须同时在跑）
  0) sudo systemctl stop teleop_robot
  1) bash scripts/start_body_control.sh
  2) bash scripts/start_xarm.sh real
  3) source /home/ubuntu/XARM/install/setup.bash
  4) python3 skill/skill02_bottle_grasp/grasp_bottle.py

⚠ 待真机标定的常量（config.py 里标了 None/待定的那些，标完才能完整跑通）
  READY_JOINTS、GRASP_ORIENT、GRASP_DIR、TCP_OFFSET、STANDOFF_MARGIN——标定前跑这个脚本，
  会在用到的地方明确报错退出，不会带着 None 稀里糊涂往下走。

⚠ 局限性与安全隐患（完整说明见 docs/skill02_bottle_grasp_guide_zh-CN.md）
  抓握手型、TCP_OFFSET/GRASP_DIR/GRASP_ORIENT/STANDOFF_MARGIN 都是针对当前这一个瓶子+
  这一副手指装配标定出来的固定值，换瓶子/换手都要重新标定；搬运阶段纯开环，全靠人眼
  判断是否安全，没有碰撞检测；箱子数据不参与 MoveIt 规划，安全性完全依赖我们自己选的
  路径点，不是真正的实时避障。

⚠ 安全（执行前逐条确认）
  1) 箱子/瓶子摆好、人退出手臂可达范围
  2) 每步 input() 回车确认，回车前核对打印坐标（安全中间点/停驻点处也各有一次确认——
     大范围移动的落点是算出来的估计值、不是实测确认，这是感知/标定算错时最后一道人工
     防线，不要嫌麻烦跳过）
  3) QP 慢速 VEL_LIMITS + MoveIt VEL_SCALE=0.1，急停在手
  4) 出意外臂停半空：python3 .../grasp_bottle.py --recover

接口
  Sub  /skill02/target_point  geometry_msgs/PoseStamped     瓶子位置(frame=base)
  Sub  /skill02/diameter_m    std_msgs/Float32               瓶子直径估计(m)（当前未用于
                                                                    控制，只做日志参考）
  Sub  /skill02/box_pose      geometry_msgs/PoseStamped     箱子位姿(frame=base)，
                                                                    只喂给 build_intermediate_point
                                                                    算安全中间点
  Sub  /skill02/box_size      geometry_msgs/Vector3          箱子尺寸(长,宽,高)，同上
  Pub  /inspire_hand/ctrl/left_hand  sensor_msgs/JointState       手型下发
"""

import sys
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float32

import config as C
import pose_math as PM

try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (MotionPlanRequest, Constraints, PlanningOptions,
                                 PositionConstraint, OrientationConstraint, JointConstraint)
    from shape_msgs.msg import SolidPrimitive
    from eai_manipulator_msgs.action import EndPosSingleTarget
    from eai_manipulator_msgs.msg import ArmTargetPose
    from controller_manager_msgs.srv import SwitchController, ListControllers
    from std_srvs.srv import SetBool
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from tf2_ros import Buffer, TransformListener
except ImportError:
    print("❌ 找不到 moveit_msgs / eai_manipulator_msgs / tf2 等。请先 source XARM 环境：")
    print("   source /home/ubuntu/XARM/install/setup.bash")
    sys.exit(1)

MOVE_ACTION = "/move_action"
QP_ACTION = f"/{C.QP_CONTROLLER}/endPosSingleTarget"
SWITCH_SRV = "/controller_manager/switch_controller"
LIST_SRV = "/controller_manager/list_controllers"
ENABLE_SRV_CANDIDATES = ["/EAIHardware/set_arm_enable", "/moveit_controller_enable"]
START_POSE_FILE = Path(__file__).resolve().parent / "_last_start_pose.json"


class GraspBottle(Node):
    def __init__(self):
        super().__init__("skill02_grasp_bottle")
        self.moveit_client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.qp_client = ActionClient(self, EndPosSingleTarget, QP_ACTION)
        self.switch_cli = self.create_client(SwitchController, SWITCH_SRV)
        self.list_cli = self.create_client(ListControllers, LIST_SRV)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._target_samples = []       # 瓶子位置采样(base系xyz)，wait_target 用
        self._diameter = None
        self._box_pose = None           # (xyz, yaw) base系，box_locator 给——一次快照后冻结
        self._box_size = None           # (长,宽,高)——同上
        self._box_snapshot_locked = False  # wait_box() 首次拿到数据后置 True，此后回调不再覆盖
        self.target_sub_ = self.create_subscription(
            PoseStamped, C.TARGET_TOPIC, self._on_target, 10)
        self.diam_sub_ = self.create_subscription(
            Float32, C.DIAMETER_TOPIC, self._on_diameter, 10)
        self.box_pose_sub_ = self.create_subscription(
            PoseStamped, C.BOX_TOPIC, self._on_box_pose, 10)
        self.box_size_sub_ = self.create_subscription(
            Vector3, C.BOX_SIZE_TOPIC, self._on_box_size, 10)

        self.hand_pub = self.create_publisher(JointState, C.HAND_CMD_TOPIC, 10)
        self._cur_joints = {}
        self.joint_sub_ = self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10)

    # ── 订阅回调：只存最新值 ─────────────────────────────────────
    def _on_target(self, msg: PoseStamped):
        if len(self._target_samples) < C.N_SAMPLES:
            p = msg.pose.position
            self._target_samples.append((p.x, p.y, p.z))

    def _on_diameter(self, msg: Float32):
        self._diameter = float(msg.data)

    def _on_box_pose(self, msg: PoseStamped):
        if self._box_snapshot_locked:       # 已拍过快照——之后手/臂进画面污染检测也不再理会
            return
        p, q = msg.pose.position, msg.pose.orientation
        yaw = 2.0 * np.arctan2(q.z, q.w)     # 只编码了 yaw，见 box_locator.publish()
        self._box_pose = (np.array([p.x, p.y, p.z]), yaw)

    def _on_box_size(self, msg: Vector3):
        if self._box_snapshot_locked:
            return
        self._box_size = (msg.x, msg.y, msg.z)

    def _on_joints(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._cur_joints[name] = pos

    # ── 收目标：凑 N 帧取中位数，源头已经是 base 系，不用再做坐标变换 ──
    def wait_target(self, timeout=10.0):
        self.get_logger().info(f"等待 {C.TARGET_TOPIC} 凑满 {C.N_SAMPLES} 帧...")
        t0 = self.get_clock().now()
        while rclpy.ok() and len(self._target_samples) < C.N_SAMPLES:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                self.get_logger().error(
                    f"超时未收到目标（收到 {len(self._target_samples)}/{C.N_SAMPLES}）。"
                    "bottle_locator.py 在跑吗？瓶子在视野里吗？")
                return None
        pts = np.asarray(self._target_samples)
        spread = np.max(np.ptp(pts, axis=0))
        if spread > 0.05:
            self.get_logger().warn(f"目标点抖动 {spread:.3f}m 偏大，已用中位数抗离群")
        return np.median(pts, axis=0)

    def wait_box(self, timeout=10.0):
        """等 box_locator 发的箱子数据到位，拍一次快照后锁定——不是给 MoveIt 碰撞体用的，
        是给 build_intermediate_point() 算安全中间点用的，全程靠我们自己算路径点，不依赖
        MoveIt 检测箱子。
        ★一次快照语义：拿到数据后立刻锁定（_box_snapshot_locked=True），此后 box_locator
        再怎么更新都不会覆盖——原因是 box_locator 全程持续跑，对"画面里出现的是箱子还是
        自己的手臂"没有分辨能力；撤回阶段手/瓶经过箱子上方时若被误当箱子点云，会让
        box_max_y 突然暴涨。跟瓶子位置用 wait_target() 只采样一次是同一套设计哲学。
        本方法之后可重复调用，会直接返回同一份冻结数据。"""
        if self._box_snapshot_locked:
            return self._box_pose, self._box_size
        t0 = self.get_clock().now()
        while rclpy.ok() and (self._box_pose is None or self._box_size is None):
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                self.get_logger().warn(
                    "超时未收到箱子数据（box_locator.py 在跑吗？）——安全中间点算不出来，"
                    "接近/撤回会跳过这层保护直接走单段路线，注意安全")
                return None, None
        self._box_snapshot_locked = True
        return self._box_pose, self._box_size

    # ── 读状态 ───────────────────────────────────────────────────
    def read_arm_joints(self, timeout=2.0):
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(nm in self._cur_joints for nm in C.ARM_JOINT_NAMES):
                return [round(self._cur_joints[nm], 3) for nm in C.ARM_JOINT_NAMES]
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                return None

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

    @staticmethod
    def _fmt_ee(pos, quat):
        r = np.degrees(PM.quat_to_rpy(quat))
        return (f"({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})m "
                f"rpy({r[0]:+.0f},{r[1]:+.0f},{r[2]:+.0f})°")

    # ── 使能 + 切控制器 ──────────────────────────────────────────
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
        self.get_logger().warn("使能未确认（若之前已使能则不影响；否则先 stop_all.sh 清场）")
        return True

    def activate_arm_controller(self, want=None):
        if want is None:
            want = C.MOVEIT_CONTROLLER
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
            return True
        req = SwitchController.Request()
        req.activate_controllers = [] if already else [want]
        req.deactivate_controllers = to_stop
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        ok = future.result() is not None and future.result().ok
        self.get_logger().debug(f"激活 {want}: {'成功' if ok else '未确认(可能已激活)'}")
        return True

    def set_vel_limits(self):
        if C.VEL_LIMITS is None:
            return
        p = Parameter(name="vel_limits", value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(x) for x in C.VEL_LIMITS]))
        for ctrl in (C.QP_CONTROLLER,):   # 关节空间大范围移动走 MoveIt(goto_ready)，不用
                                           # jointspace_arm_L_controller，这里不用给它调速
            cli = self.create_client(SetParameters, f"/{ctrl}/set_parameters")
            if not cli.wait_for_service(timeout_sec=3.0):
                self.destroy_client(cli)
                continue
            future = cli.call_async(SetParameters.Request(parameters=[p]))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    # ── 灵巧手 ───────────────────────────────────────────────────
    def set_hand_pose(self, pose_dict, label=""):
        t0 = self.get_clock().now()
        while self.hand_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - t0).nanoseconds > 3e9:
                self.get_logger().warn(f"{C.HAND_CMD_TOPIC} 无订阅者，跳过摆手型")
                return
        ids = list(pose_dict.keys())
        msg = JointState()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.name = ids
        msg.position = [float(pose_dict[i]) for i in ids]
        self.hand_pub.publish(msg)
        self.get_logger().info(f"手型{label}：{pose_dict}")
        t0 = self.get_clock().now()
        while (self.get_clock().now() - t0).nanoseconds < 1.5e9:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── 关节空间：回预备姿态（走 MoveIt 关节目标，不是 jointspace QP）───
    def goto_ready(self, prefix=""):
        """回预备姿态，走 MoveIt 关节空间目标，不是 jointspace_arm_L_controller——QP（不管
        关节空间还是末端空间）完全不查 MoveIt 的 planning scene，即使规划场景里有障碍物，
        QP 对它也形同虚设。给关节角目标而不是笛卡尔位姿目标，不用经过 IK 反解，保留了
        "不会拧麻花"这个关节空间移动的优点，同时还能享受 MoveIt 自身的碰撞检测（自碰撞等）。
        ⚠代价：这条腿要过 MoveIt 更紧的关节限位表检查（QP 对此免疫）——READY_JOINTS 若还
        没被 MoveIt 校验过起点合法性，建议先单独调用这个函数验证一次，别直接信任整条
        流程都没问题。"""
        cur_j = self.read_arm_joints()
        target_j = [round(x, 3) for x in C.READY_JOINTS]
        self.get_logger().info(
            f"{prefix}[MoveIt｜关节] 回预备姿态：{cur_j or '(当前角未读到)'} → {target_j}")
        if not self.moveit_move_to_joints(C.READY_JOINTS):
            return False
        self.get_logger().info(f"{prefix}✓ 到预备姿态")
        return True

    # ── QP：单段下发 + 分段接近 ──────────────────────────────────
    def qp_move_once(self, pose):
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
            self.get_logger().error(f"QP 段执行失败：{res.result.result_msg if res else '超时'}")
            return False
        return True

    def move_segmented(self, target_xyz, target_quat):
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
        for i in range(1, n_seg + 1):
            t = i / n_seg
            wp = p0 + (p1 - p0) * t
            wq = PM.slerp(q0, q1, t)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = wp
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = wq
            if not self.qp_move_once(pose):
                self.get_logger().error(
                    f"第 {i}/{n_seg} 段失败，停在当前位置。收臂："
                    "python3 skill/skill02_bottle_grasp/grasp_bottle.py --recover")
                return False
        return True

    # ── 提起后交互式搬运：wsad+回车 单步平移，q 结束 ─────────────
    def interactive_jog_place(self, quat):
        """提起后手动微调水平位置。w=+x(远离机器人/前) s=-x(后) a=+y(机器人左) d=-y(机器人右)，
        同 ROS body frame 惯例（x前/y左/z上），z 不在这里动——高度已经在提起那一步定了，
        放下由调用方单独处理。累计水平位移超过 PLACE_JOG_MAX_TOTAL 就拒绝该次移动，防止
        连续误按把手带到很远/意外的地方。返回最终 xyz；读不到当前位姿返回 None。
        ★进函数先等 0.4s 再读基准位姿：提起动作刚结束时读到的可能是柔顺模式下还没完全
        落稳的瞬态值（新指令下发瞬间有轻微下沉），不等的话第一步平移会把这个偏差也带上，
        表现为"按第一个键手臂先跟着往下掉2~3cm"。"""
        t0 = self.get_clock().now()
        while rclpy.ok() and (self.get_clock().now() - t0).nanoseconds < 0.4e9:
            rclpy.spin_once(self, timeout_sec=0.05)
        cur = self.read_ee_pose()
        if cur is None:
            return None
        xyz = np.array([cur.position.x, cur.position.y, cur.position.z])
        origin_xy = xyz[:2].copy()
        step_map = {"w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1)}
        print("提起完成，wsad+回车 微调水平位置(w前/s后/a左/d右)，q 结束并下降放下：")
        while True:
            try:
                key = input("> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            if key == "q":
                break
            if key not in step_map:
                print("❌ 只认 w/a/s/d/q")
                continue
            sx, sy = step_map[key]
            new_xyz = xyz + np.array([sx, sy, 0.0]) * C.PLACE_JOG_STEP
            if float(np.linalg.norm(new_xyz[:2] - origin_xy)) > C.PLACE_JOG_MAX_TOTAL:
                print(f"❌ 累计水平位移会超过安全上限 {C.PLACE_JOG_MAX_TOTAL}m，拒绝，先用反方向退一点")
                continue
            if self.move_segmented(new_xyz, quat):
                xyz = new_xyz
                d = float(np.linalg.norm(xyz[:2] - origin_xy))
                self.get_logger().info(f"  → xyz={xyz.round(3)}（累计水平位移 {d*100:.1f}cm）")
            else:
                print("⚠ 这一步没走成，位置可能没变，再试一次或者按 q 用当前实际位置结束")
        return xyz

    # ── MoveIt：一步规划 ─────────────────────────────────────────
    def _pose_goal(self, pose, pos_tol=0.02, ori_tol=0.3):
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
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def moveit_move_to_pose(self, pose, ori_tol=0.3):
        """粗定位专用：姿态容差故意给松（ori_tol 默认 0.3rad≈17°），落地姿态不精确，
        靠后面 QP 摆正姿态那一步收拾——不要在这里收紧容差去追求精确，那是 MoveIt 的病根
        （容差内随便挑解），追求精确应该交给 QP。"""
        if not self.moveit_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"MoveGroup action {MOVE_ACTION} 不在。MoveIt 组件起了吗？")
            return False
        req = MotionPlanRequest()
        req.group_name = C.GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = C.VEL_SCALE
        req.max_acceleration_scaling_factor = C.ACC_SCALE
        req.goal_constraints.append(self._pose_goal(pose, ori_tol=ori_tol))
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
            f"MoveIt 失败 error_code={code}（99999 首查起点越界，见 motion04 guide 排错表）")
        return False

    def _joint_goal(self, target_joints, tol=0.01):
        """7 个 JointConstraint：给关节角目标而不是笛卡尔位姿目标，MoveIt 不用做 IK，
        只做碰撞检测+关节空间规划。"""
        c = Constraints()
        for name, val in zip(C.ARM_JOINT_NAMES, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = jc.tolerance_below = tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def moveit_move_to_joints(self, target_joints):
        """给 MoveIt 一个关节角目标（用于 goto_ready，大范围移动交给 MoveIt 规划——
        QP 关节空间控制器不查 MoveIt 场景，做不到这件事；见 goto_ready 的详细说明）。"""
        if not self.moveit_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"MoveGroup action {MOVE_ACTION} 不在。MoveIt 组件起了吗？")
            return False
        req = MotionPlanRequest()
        req.group_name = C.GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = C.VEL_SCALE
        req.max_acceleration_scaling_factor = C.ACC_SCALE
        req.goal_constraints.append(self._joint_goal(target_joints))
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
            f"MoveIt 关节目标失败 error_code={code}（99999 首查起点越界，见 motion04 guide 排错表）")
        return False

    # ── 抓握几何：瓶子位置 → (停驻点, 最终tcp目标, 瓶子实测位置) ──
    def build_grasp_targets(self, bottle_xyz):
        """最终 tcp 目标 = 瓶子实测位置 − TCP_OFFSET（拖动示教实测的完整3D偏移，包含抓握
        高度/前伸距离/任何侧向偏移，不假设偏移只沿一个轴，见 config.py 标定说明）。
        预抓停驻点 = 最终目标 沿 -GRASP_DIR 再退 STANDOFF_MARGIN（同一条直线，中间用 QP
        短程平移衔接，姿态全程不变）。"""
        final = np.asarray(bottle_xyz, dtype=float) - np.asarray(C.TCP_OFFSET, dtype=float)
        d = np.asarray(C.GRASP_DIR, dtype=float)
        d = d / np.linalg.norm(d)
        standoff = final - d * C.STANDOFF_MARGIN
        return standoff, final

    def check_position(self, target_xyz, label=""):
        """诊断用，不是修正：核对 tcp 有没有精确到达下发目标（见 config.py 里的说明，
        这不是"抓握准不准"的反馈，只是"QP 有没有执行到位"的兜底检查）。"""
        cur = self.read_ee_pose()
        if cur is None:
            return
        p = np.array([cur.position.x, cur.position.y, cur.position.z])
        d = float(np.linalg.norm(p - np.asarray(target_xyz, dtype=float)))
        tag = "✓" if d <= C.POSITION_CHECK_TOL else "⚠"
        self.get_logger().info(f"  {tag} {label}tcp 实际位置偏离下发目标 {d*100:.1f}cm")

    # ── 安全中间点：箱子 y 方向最远处再加余量（见 config.py 标定说明）────
    def _box_max_y(self):
        """箱子在 base 系 y 方向能延伸到的最大值——轴对齐包围盒公式，对 length/width
        具体哪个对应哪条边不敏感（交换两者同时把 yaw 转90°，结果不变），箱子转多少度
        都适用。读的是 self._box_pose/self._box_size，wait_box() 拍过快照后这两个字段
        已锁定不再随 box_locator 更新，见 _on_box_pose/_on_box_size 里的
        _box_snapshot_locked 判断——避免撤回阶段手/瓶经过箱子上方被误判进箱子点云，把
        box_max_y 算爆。拿不到箱子数据（没跑 box_locator，或 wait_box 还没成功拍到快照）
        返回 None。"""
        if self._box_pose is None or self._box_size is None:
            return None
        (_, cy, _), yaw = self._box_pose
        length, width, _ = self._box_size
        half_extent_y = (length / 2.0) * abs(np.sin(yaw)) + (width / 2.0) * abs(np.cos(yaw))
        return cy + half_extent_y

    def build_intermediate_point(self, anchor_xyz):
        """由某个已知安全点（标定出来的 standoff，或撤回时现算的 retreat_xyz）推出一个
        更安全的中间点：x/水平深度、z/高度都沿用 anchor 的（不做竖直抬高——会让某个关节
        顶到限位），y 朝箱子 y 方向最远处（box_max_y + INTERMEDIATE_Y_MARGIN）甩，让
        "大范围移动"的落点在水平方向明显甩开箱子，不用去猜箱子具体哪个角在哪、也不需要
        看见箱子完整轮廓。拿不到箱子数据返回 None（调用方应跳过这一步、直接走原来的单段
        路线，不能因为拿不到箱子数据就卡住整个流程）。
        只在 anchor 的 y 还没到目标线时才朝它移动——如果 anchor 的 y 已经 ≥ 目标线
        （比如搬运时把瓶子挪到了箱子左边缘外），再朝目标线走反而是往箱子方向带（退回时
        anchor 是搬运后现算的 retreat_xyz，y 范围不受控，跟标定死的 standoff_xyz 不一样，
        这个条件是会真的触发的，不是理论上的边界情况）。这种情况下直接保留 anchor 自己
        的 y，不再往箱子那侧凑。"""
        box_max_y = self._box_max_y()
        if box_max_y is None:
            return None
        target_line = box_max_y + C.INTERMEDIATE_Y_MARGIN
        target_y = target_line if anchor_xyz[1] < target_line else anchor_xyz[1]
        return np.array([anchor_xyz[0], target_y, anchor_xyz[2]])

    # ── 三段式接近：①a MoveIt到安全中间点 ①b QP下探到停驻点(顺带摆正姿态) ③QP精修 ──
    def approach_three_stage(self, standoff_xyz, final_xyz, quat):
        intermediate_xyz = self.build_intermediate_point(standoff_xyz)
        self.activate_arm_controller(C.MOVEIT_CONTROLLER)
        if intermediate_xyz is not None:
            self.get_logger().info(
                "①a [MoveIt｜末端] 粗定位到安全中间点（箱子y方向最远处再加余量，"
                "避免大范围移动贴着箱子扫过；姿态容差放松）...")
            target = Pose()
            target.position.x, target.position.y, target.position.z = intermediate_xyz
            (target.orientation.x, target.orientation.y,
             target.orientation.z, target.orientation.w) = quat
            if not self.moveit_move_to_pose(target):
                return False
            self.check_position(intermediate_xyz, label="中间点 ")
            # 中间点是算出来的估计值，不是实测确认——大范围移动落点前人工核对一次，
            # 是感知/标定出错时的最后一道防线。
            input("检查手臂是否安全停在中间点、没有碰到箱子 → 回车继续到停驻点"
                  "（Ctrl-C 中断→--recover 收臂）...")

            self.get_logger().info(
                "①b [QP｜末端] 由中间点下探到预抓停驻点（x不变，只调y/z；QP 精确执行，"
                "顺带把姿态收到 GRASP_ORIENT，不用再单独摆正）...")
            self.activate_arm_controller(C.QP_CONTROLLER)
            self.set_vel_limits()
            if not self.move_segmented(standoff_xyz, quat):
                return False
        else:
            self.get_logger().warn(
                "拿不到箱子数据（box_locator.py 在跑吗？），跳过安全中间点，"
                "直接 MoveIt 粗定位到停驻点——没有任何箱子避障，注意观察")
            target = Pose()
            target.position.x, target.position.y, target.position.z = standoff_xyz
            (target.orientation.x, target.orientation.y,
             target.orientation.z, target.orientation.w) = quat
            if not self.moveit_move_to_pose(target):
                return False
            self.activate_arm_controller(C.QP_CONTROLLER)
            self.set_vel_limits()
        self.check_position(standoff_xyz, label="到停驻点后 ")
        input("检查手臂是否安全停在预抓停驻点 → 回车继续精修到抓握位"
              "（Ctrl-C 中断→--recover 收臂）...")

        self.get_logger().info("③ [QP｜末端] 精修平移到最终抓握位置（沿 GRASP_DIR，姿态不动）...")
        if not self.move_segmented(final_xyz, quat):
            return False
        self.check_position(final_xyz, label="精修后 ")
        return True

    # ── 落盘/恢复 ────────────────────────────────────────────────
    def save_start_pose(self, pose):
        try:
            START_POSE_FILE.write_text(json.dumps({
                "stamp": time.time(),
                "xyz": [pose.position.x, pose.position.y, pose.position.z],
                "quat": [pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w]}))
        except Exception as e:
            self.get_logger().warn(f"出发位姿落盘失败（不影响本次执行）：{e}")


def recover(node):
    log = node.get_logger()
    data = None
    if START_POSE_FILE.exists():
        try:
            data = json.loads(START_POSE_FILE.read_text())
        except Exception:
            log.warn(f"{START_POSE_FILE.name} 损坏，忽略")
    if data is None and C.READY_JOINTS is None:
        log.error("没有可退回的目标：无出发位记录，且 READY_JOINTS 未配置。")
        return
    if data is not None:
        age_h = (time.time() - data["stamp"]) / 3600.0
        xyz = data["xyz"]
        log.info(f"退回目标=上次出发位 ({xyz[0]:+.3f}, {xyz[1]:+.3f}, {xyz[2]:+.3f})"
                 f"（{age_h:.1f} 小时前记录）")
    input("★确认臂周边无人、退回路径无障碍 → 回车开始收臂（Ctrl-C 取消）...")
    node.enable_arm()
    node.set_vel_limits()
    if data is not None:
        node.activate_arm_controller(C.QP_CONTROLLER)
        if not node.move_segmented(data["xyz"], data["quat"]):
            log.error("退回出发位失败，臂仍停在半路")
            return
        log.info("✓ 已退回出发位")
    if C.READY_JOINTS is not None:
        node.activate_arm_controller(C.MOVEIT_CONTROLLER)   # goto_ready 走 MoveIt 关节目标
        if node.goto_ready():
            log.info("✓ 已归位 READY，恢复完成")
    node.set_hand_pose(C.HAND_OPEN_POSE, label="（复位张开）")


def _check_constants(log):
    """标定前跑这个脚本会在这里明确报错退出，不会带着 None 稀里糊涂往下走。"""
    missing = [name for name in ("READY_JOINTS", "GRASP_ORIENT", "GRASP_DIR", "TCP_OFFSET")
               if getattr(C, name) is None]
    if missing:
        log.error(f"以下常量还没标定（config.py 里是 None）：{missing}——先完成真机标定再跑。"
                   "只想测感知（wait_target/wait_box）可以先跳过这个检查自行调用对应函数。")
        return False
    return True


def main():
    rclpy.init()
    node = GraspBottle()
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

    log.info("════════ skill02_bottle_grasp · 抓瓶子 ════════")
    if not _check_constants(log):
        node.destroy_node()
        rclpy.shutdown()
        return
    input("★安全确认：箱子/瓶子已固定、人已退出手臂范围、急停在手 → 回车继续（Ctrl-C 取消）...")

    try:
        # ═══ 阶段1 · 看：多帧聚合拿瓶子位置 ═══
        node.get_logger().info("──────── 阶段1 · 看：读取瓶子位置 ────────")
        bottle_xyz = node.wait_target()
        if bottle_xyz is None:
            return
        diam_txt = f"  直径估计={node._diameter*100:.1f}cm" if node._diameter else ""
        log.info(f"瓶子位置(base系,m) = ({bottle_xyz[0]:+.3f},{bottle_xyz[1]:+.3f},"
                 f"{bottle_xyz[2]:+.3f}){diam_txt}")

        standoff_xyz, final_xyz = node.build_grasp_targets(bottle_xyz)
        quat = C.GRASP_ORIENT
        log.info(f"  预抓停驻点 tcp {node._fmt_ee(standoff_xyz, quat)}")
        log.info(f"  最终闭合点 tcp {node._fmt_ee(final_xyz, quat)}")
        input("核对以上坐标合理 → 回车开始（Ctrl-C 取消）...")

        # ═══ 阶段2 · 就位：使能→摆开手→回预备→三段式接近 ═══
        # 箱子数据只喂给 build_intermediate_point 算安全中间点，不注册进 MoveIt，全程
        # 靠我们自己选路径点避障，不依赖/不假装依赖 MoveIt 检测到了箱子。
        node.get_logger().info("──────── 阶段2 · 就位 ────────")
        node.enable_arm()
        node.set_vel_limits()
        node.wait_box()   # 提前拍一次箱子快照并锁定（此后手/臂进画面也不会再污染这份数据）；超时只警告不阻塞
        node.set_hand_pose(C.HAND_OPEN_POSE, label="（预抓开手）")

        if not node.activate_arm_controller(C.MOVEIT_CONTROLLER):   # goto_ready 走 MoveIt 关节目标
            return
        if not node.goto_ready():
            return

        node.activate_arm_controller(C.QP_CONTROLLER)
        start = node.read_ee_pose()
        if start is None:
            return
        node.save_start_pose(start)

        if not node.approach_three_stage(standoff_xyz, final_xyz, quat):
            log.error("接近失败，臂可能停在半路。收臂：--recover")
            return

        # ═══ 阶段3 · 抓握：闭合手 → 提起 → 回车确认 ═══
        node.get_logger().info("──────── 阶段3 · 抓握 ────────")
        input("★闭合手确认：瓶子应已在缺口中 → 回车闭合抓握（Ctrl-C 中断→--recover 收臂）...")
        node.set_hand_pose(C.HAND_GRASP_POSE, label="（抓握闭合）")

        cur = node.read_ee_pose()
        if cur is not None:
            lift_xyz = (cur.position.x, cur.position.y, cur.position.z + C.LIFT_HEIGHT)
            lift_quat = [cur.orientation.x, cur.orientation.y,
                         cur.orientation.z, cur.orientation.w]
            log.info(f"提起 {C.LIFT_HEIGHT*100:.0f}cm 确认抓稳...")
            node.move_segmented(lift_xyz, lift_quat)
        input("★检查是否抓稳 → 回车开始搬运（Ctrl-C 中断→臂停在提起位，用 --recover 收臂）...")

        # ═══ 阶段3b · 搬运：wsad 微调水平位置 → 下降放下 ═══
        # ⚠ 纯开环：没有视觉验证新位置下方是否有支撑面，wsad 挪的时候自己看着办，
        #   别挪出箱子边缘再下降，否则瓶子会悬空摔下去。
        placed_xy_z = node.interactive_jog_place(quat)
        if placed_xy_z is None:
            log.error("读不到当前位姿，放弃搬运，臂停在提起位。收臂：--recover")
            return
        # 下降基准重新实时读一次，不直接信 interactive_jog_place 内部记录的 placed_xy_z——
        # 这个记录值是"我们下发过的目标"，不是"手臂现在真实在哪"，柔顺模式下两者可能有
        # 瞬态/漂移误差，下降量会跟着算错、导致下压过头。
        actual = node.read_ee_pose()
        if actual is None:
            log.error("下降前读不到当前位姿，放弃搬运，臂停在搬运位。收臂：--recover")
            return
        cur_xyz = (actual.position.x, actual.position.y, actual.position.z)
        down_xyz = (cur_xyz[0], cur_xyz[1], cur_xyz[2] - C.LIFT_HEIGHT)
        log.info(f"下降 {C.LIFT_HEIGHT*100:.0f}cm 放下（基准 {node._fmt_ee(cur_xyz, quat)}）...")
        node.move_segmented(down_xyz, quat)
        input("★确认已放稳 → 回车松手并归位（Ctrl-C 中断→臂停在放置位，用 --recover 收臂）...")
        node.set_hand_pose(C.HAND_OPEN_POSE, label="（松开）")
        input("★确认手已松开、瓶子已放稳、退回路径无障碍 → 回车开始收臂"
              "（Ctrl-C 中断→臂停在放置位，用 --recover 收臂）...")

        # ═══ 阶段4 · 收手归位：退回停驻位（从新放置点算，不是旧 standoff_xyz）→ 关节空间归位 ═══
        node.get_logger().info("──────── 阶段4 · 收手归位 ────────")
        d_unit = np.asarray(C.GRASP_DIR, dtype=float)
        d_unit = d_unit / np.linalg.norm(d_unit)
        retreat_xyz = np.asarray(down_xyz, dtype=float) - d_unit * C.STANDOFF_MARGIN
        node.activate_arm_controller(C.QP_CONTROLLER)   # 退回前显式恢复，防中途被切走
        node.move_segmented(retreat_xyz, quat)
        # 回程同样先经过安全中间点再走大范围的关节空间归位（用 retreat_xyz 现算，不是用
        # 去程那个 standoff_xyz——放置位置可能因为搬运挪过，退回点也跟着变了）
        intermediate_back = node.build_intermediate_point(retreat_xyz)
        if intermediate_back is not None:
            node.get_logger().info("先升到安全中间点，再回预备姿态...")
            node.move_segmented(intermediate_back, quat)
            input("检查手臂是否安全停在中间点、没有碰到箱子 → 回车继续回预备姿态"
                  "（Ctrl-C 中断→--recover 收臂）...")
        node.activate_arm_controller(C.MOVEIT_CONTROLLER)   # goto_ready 走 MoveIt 关节目标
        node.goto_ready(prefix="⑤ ")
        log.info("✓ 全部完成：臂回预备姿态、手已张开")
    except KeyboardInterrupt:
        log.warn("用户中断")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
