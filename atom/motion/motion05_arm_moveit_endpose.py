#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 运控5 · 手臂（Arm）· XARM MoveIt 末端（笛卡尔）控制
配套讲解：atom/motion/docs/motion05_arm_moveit_endpose_guide.md

一句话
  给 MoveIt 一个"末端位姿"目标（位置 x/y/z + 姿态四元数），它规划无碰撞轨迹把 TCP 送到那儿。
  和 motion04 的区别：motion04 给"关节角"目标；motion05 给"末端在空间里的位姿"目标（更贴近"手要去哪")。

控制器互斥（重要）
  moveit/jointspace/endpose 各族控制器都抢同一条臂的关节接口，同一时刻只能一个 active
  （如刚跑过 QP demo 的控制器还 active，moveit 就激活不了、执行报 CONTROL_FAILED -4）。
  本脚本激活前自动查询并停掉占用本臂的其它 active 控制器，再 STRICT 切换（失败如实报错）。
  流程：从 TF 读当前 TCP 位姿 → 小幅平移 → 复位。严格超时校验参考 motion04_arm_moveit_robust.py。

运行前提（同 motion04：x86 / ubuntu；先 body_control → XARM 本体 → MoveIt 组件；本脚本自动使能+切控制器）
  一键前置： bash scripts/start_xarm.sh real
  跑 demo 只需基础 ROS 2（~/.bashrc 已自动 source）；import 失败才 source /home/ubuntu/XARM/install/setup.bash

接口（标准 MoveIt2）
  Action   /move_action    moveit_msgs/action/MoveGroup   目标=末端位姿约束(位置+姿态)，规划并执行
  Service  <使能服务>      std_srvs/SetBool               real 模式使能（sim 无）
           ★名随 XARM 版本变：/moveit_controller_enable（新）或 /EAIHardware/set_arm_enable（旧），
             运行时逐个探测、调失败自动换下一个（见 ENABLE_SRV_CANDIDATES）
  Service  /controller_manager/switch_controller   激活 moveit_left_arm_controller
  Service  /controller_manager/list_controllers    查占用本臂的控制器（自动避让用）
  TF       base → left_tcp_link             读当前末端 TCP 位姿作起点

⚠ 待真机核实（首次跑必查）
  1) EE_LINK：末端 link 名（这里用 URDF 里的 left_tcp_link）
  2) BASE_FRAME：规划/参考基坐标系（用 `base`；
     核实用 `ros2 run tf2_tools view_frames` 或
     `ros2 param get /move_group robot_description` 里 planning_frame 核实）
  3) XARM 的 MoveIt 是否支持标准 pose-goal 规划（若只认路点/JSON，改用官方 waypoint 接口）

⚠ 安全: 手臂力矩大，先仿真；本 demo 只让末端小幅平移(默认 +z 5cm)再复位，可复位、低速。
"""

import sys
import copy
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Quaternion

try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (MotionPlanRequest, Constraints, PlanningOptions,
                                 PositionConstraint, OrientationConstraint)
    from shape_msgs.msg import SolidPrimitive
    from controller_manager_msgs.srv import SwitchController, ListControllers
    from std_srvs.srv import SetBool
    from tf2_ros import Buffer, TransformListener
except ImportError:
    print("❌ 找不到 moveit_msgs / tf2_ros 等。请先 source XARM 环境：")
    print("   source /home/ubuntu/XARM/install/setup.bash")
    sys.exit(1)

# ── 机器人相关常量（★待真机核实）──
GROUP = "left_arm"
EE_LINK = "left_tcp_link"                 # 末端 link（URDF 有此 link）
BASE_FRAME = "base"                       # 参考基坐标系
MOVEIT_CONTROLLER = "moveit_left_arm_controller"
JOINT_NAMES = [                           # 左臂 7 关节名（判断"谁占着本臂"用）
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
]
MOVE_ACTION = "/move_action"
SWITCH_SRV = "/controller_manager/switch_controller"
LIST_SRV = "/controller_manager/list_controllers"
# 使能服务名随 XARM 版本变化；两个都是 std_srvs/SetBool，运行时逐个探测、调失败自动换下一个。
ENABLE_SRV_CANDIDATES = ["/moveit_controller_enable", "/EAIHardware/set_arm_enable"]

DEMO_DELTA_XYZ = [0.0, 0.0, 0.05]         # 末端小幅平移(m)：默认 +z 上抬 5cm，可复位
VEL_SCALE = 0.1
ACC_SCALE = 0.1
# ─────────────────────────────────────────────────────────────────


class ArmEndposeDemo(Node):
    def __init__(self):
        super().__init__("atom_arm_endpose_demo")
        self.client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.switch_cli = self.create_client(SwitchController, SWITCH_SRV)
        self.list_cli = self.create_client(ListControllers, LIST_SRV)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def enable_arm(self):
        """使能手臂（real 模式必需）。sim 无此服务，自动跳过。
        使能服务名随 XARM 版本不同（见 ENABLE_SRV_CANDIDATES），调失败自动换下一个。"""
        tried = False   # 是否至少调到过一个使能服务（区分"都不在(sim)"和"都失败"）
        for srv in ENABLE_SRV_CANDIDATES:
            cli = self.create_client(SetBool, srv)
            if not cli.wait_for_service(timeout_sec=3.0):
                self.destroy_client(cli)
                continue
            tried = True
            future = cli.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            resp = future.result()
            if resp is not None and resp.success:
                self.get_logger().info(f"✓ 手臂已使能（{srv}）")
                return
            msg = resp.message if resp is not None else "超时"
            self.get_logger().warn(f"{srv} 使能未成功（{msg}），尝试下一个候选...")
        if tried:
            if self._already_enabled():
                self.get_logger().info("臂已是使能状态（arm_enable=1）——重复使能被拒不影响运行，继续")
                return
            self.get_logger().error(
                f"所有使能服务都失败（候选 {ENABLE_SRV_CANDIDATES}）且 arm_enable≠1。"
                "自查：有程序占 /arm/cmd_pos？（bash scripts/stop_all.sh + sudo systemctl stop teleop_robot 清场）")
        else:
            self.get_logger().warn(f"使能服务都不在（候选 {ENABLE_SRV_CANDIDATES}）：sim 或 XARM 未起，跳过使能")

    def _already_enabled(self):
        """兜底：查 /EAIHardware/debug 的 arm_enable——使能是跨进程持续的硬件状态，
        上个 demo 已使能时本次重复使能常被拒/超时，但臂其实可用。"""
        try:
            from eai_manipulator_msgs.srv import Info
        except ImportError:
            return False
        cli = self.create_client(Info, "/EAIHardware/debug")
        if not cli.wait_for_service(timeout_sec=2.0):
            self.destroy_client(cli)
            return False
        future = cli.call_async(Info.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        resp = future.result()
        return resp is not None and "arm_enable: 1" in resp.info

    def _scan_arm_controllers(self, timeout=5.0):
        """查 controller_manager：返回 (MoveIt控制器是否已active, 占用本臂关节、需先停的其它active控制器名)。
        moveit/jointspace/endpose 各族控制器互斥——只要 claim 了本臂任一关节接口就要先停。"""
        if not self.list_cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"{LIST_SRV} 不在，无法自动避让，按“本臂未被占用”处理")
            return (False, [])
        future = self.list_cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        resp = future.result()
        if resp is None:
            self.get_logger().warn("查询控制器超时，按“本臂未被占用”处理")
            return (False, [])
        already_active, conflict = False, []
        for c in resp.controller:
            if c.state != "active":
                continue
            if c.name == MOVEIT_CONTROLLER:
                already_active = True
                continue
            if any(ci.split("/")[0] in JOINT_NAMES for ci in c.claimed_interfaces):
                conflict.append(c.name)
        return (already_active, conflict)

    def activate_moveit_controller(self):
        """激活 MoveIt 控制器（不激活执行会 CONTROL_FAILED -4）：
        先停掉占用本臂的其它 active 控制器（如刚跑过 QP demo 残留的），再 STRICT 切换（失败如实报错）。"""
        if not self.switch_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"{SWITCH_SRV} 不在，跳过自动切换（请手动 ros2 control switch_controllers）")
            return
        already_active, to_stop = self._scan_arm_controllers()
        if already_active and not to_stop:
            self.get_logger().info(f"{MOVEIT_CONTROLLER} 已激活且无冲突，无需切换")
            return
        req = SwitchController.Request()
        req.activate_controllers = [] if already_active else [MOVEIT_CONTROLLER]
        req.deactivate_controllers = to_stop
        req.strictness = SwitchController.Request.STRICT
        if to_stop:
            self.get_logger().info(f"先停占用本臂的控制器: {to_stop}")
        future = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        ok = resp is not None and resp.ok
        self.get_logger().info(f"切到 {MOVEIT_CONTROLLER}: {'成功' if ok else '失败(检查控制器名/资源占用)'}")

    def read_ee_pose(self, timeout=5.0):
        """用 TF 读当前末端 TCP 在 BASE_FRAME 下的位姿（Pose）。读不到返回 None。"""
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tr = self.tf_buffer.lookup_transform(BASE_FRAME, EE_LINK, Time()).transform
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (
                    tr.translation.x, tr.translation.y, tr.translation.z)
                pose.orientation = tr.rotation
                return pose
            except Exception:
                pass
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                self.get_logger().error(
                    f"超时未读到 {EE_LINK} 位姿（TF）。BASE_FRAME={BASE_FRAME} / EE_LINK 对吗？")
                return None

    def _pose_goal(self, pose, pos_tol=0.01, ori_tol=0.05):
        """把一个末端目标位姿转成 MoveGroup 的约束（位置=小球容差 + 姿态=四元数容差），作用在 EE_LINK 上。"""
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = BASE_FRAME
        pc.link_name = EE_LINK
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol]))
        pc.constraint_region.primitive_poses.append(
            Pose(position=pose.position, orientation=Quaternion(w=1.0)))
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = BASE_FRAME
        oc.link_name = EE_LINK
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        return c

    def move_to_pose(self, pose, vel=VEL_SCALE, acc=ACC_SCALE):
        """规划并把末端运动到目标位姿（阻塞等结果）。返回 True/False。"""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"MoveGroup action {MOVE_ACTION} 不在。MoveIt 组件起了吗？")
            return False
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc
        req.goal_constraints.append(self._pose_goal(pose))

        goal = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=False))
        p = pose.position
        self.get_logger().info(f"规划末端 → ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("目标被拒绝/发送超时（EE_LINK/组名？位姿不可达？MoveGroup 在吗？）")
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        res = result_future.result()
        if res is None:
            self.get_logger().error("执行结果超时(60s)。")
            return False
        code = res.result.error_code.val
        if code == 1:
            self.get_logger().info("✓ 规划并执行成功")
            return True
        self.get_logger().error(f"MoveIt 失败 error_code={code}（-4=控制器没激活, -1=IK/规划失败, 详见 guide）")
        return False


def main():
    rclpy.init()
    node = ArmEndposeDemo()
    log = node.get_logger()
    log.info(f"运控5 · 手臂 MoveIt 末端控制 | 组={GROUP} EE={EE_LINK} 平移={DEMO_DELTA_XYZ}m vel={VEL_SCALE}")
    input("确认 XARM 本体 + MoveIt 组件已启动、臂周围无人无物 → 按 Enter 继续（Ctrl-C 取消）...")

    try:
        node.enable_arm()                    # ③ real 模式使能
        node.activate_moveit_controller()    # ④ 切 moveit 控制器

        start = node.read_ee_pose()
        if start is None:
            return
        p = start.position
        log.info(f"当前末端 TCP = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")

        target = copy.deepcopy(start)        # 姿态不变，只小幅平移
        target.position.x += DEMO_DELTA_XYZ[0]
        target.position.y += DEMO_DELTA_XYZ[1]
        target.position.z += DEMO_DELTA_XYZ[2]

        log.info("Step 1: 末端小幅平移 ...")
        if node.move_to_pose(target):
            log.info("Step 2: 回到起始位姿 ...")
            node.move_to_pose(start)
    except KeyboardInterrupt:
        log.warn("用户中断")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
