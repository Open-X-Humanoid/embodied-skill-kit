#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原6 · 手臂（Arm）· XARM MoveIt 末端（笛卡尔）控制
配套讲解：atom/docs/atom06_arm_moveit_endpose_guide.md

一句话
  给 MoveIt 一个"末端位姿"目标（位置 x/y/z + 姿态四元数），它规划无碰撞轨迹把 TCP 送到那儿。
  和 atom05 的区别：atom05 给"关节角"目标；atom06 给"末端在空间里的位姿"目标（更贴近"手要去哪")。

简洁版 vs robust
  本文件=简洁版：BEST_EFFORT 激活 moveit 控制器；从 TF 读当前 TCP 位姿 → 小幅平移 → 复位。
  遇控制器冲突/严格校验参考 atom05_arm_moveit_robust.py 的做法（后续可补 atom06 robust）。

运行前提（同 atom05：x86 / ubuntu；先 body_control → XARM 本体 → MoveIt 组件；本脚本自动使能+切控制器）
  一键前置： bash scripts/start_xarm.sh sim   /   source scripts/start_xarm.sh
  ★ 跑前 source 的是 XARM： source /home/ubuntu/XARM/install/setup.bash

接口（标准 MoveIt2）
  Action   /move_action    moveit_msgs/action/MoveGroup   目标=末端位姿约束(位置+姿态)，规划并执行
  Service  /EAIHardware/set_arm_enable      real 模式使能（sim 无）
  Service  /controller_manager/switch_controller   激活 moveit_left_arm_controller
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
    from controller_manager_msgs.srv import SwitchController
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
MOVE_ACTION = "/move_action"
SWITCH_SRV = "/controller_manager/switch_controller"
ENABLE_SRV = "/EAIHardware/set_arm_enable"

DEMO_DELTA_XYZ = [0.0, 0.0, 0.05]         # 末端小幅平移(m)：默认 +z 上抬 5cm，可复位
VEL_SCALE = 0.1
ACC_SCALE = 0.1
# ─────────────────────────────────────────────────────────────────


class ArmEndposeDemo(Node):
    def __init__(self):
        super().__init__("atom_arm_endpose_demo")
        self.client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.switch_cli = self.create_client(SwitchController, SWITCH_SRV)
        self.enable_cli = self.create_client(SetBool, ENABLE_SRV)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def enable_arm(self):
        """使能手臂（real 模式必需）。sim 无此服务，自动跳过。"""
        if not self.enable_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"{ENABLE_SRV} 不在（sim 或 XARM 未起），跳过使能")
            return
        future = self.enable_cli.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        if resp is not None and resp.success:
            self.get_logger().info("✓ 手臂已使能")
        else:
            self.get_logger().error("使能失败（可能有别的程序占 /arm/cmd_pos，先 bash scripts/stop_all.sh 清场）")

    def activate_moveit_controller(self):
        """激活 MoveIt 控制器（不激活执行会 CONTROL_FAILED -4）。简洁版 BEST_EFFORT。"""
        if not self.switch_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"{SWITCH_SRV} 不在，跳过自动切换（请手动 ros2 control switch_controllers）")
            return
        req = SwitchController.Request()
        req.activate_controllers = [MOVEIT_CONTROLLER]
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        ok = future.result() is not None and future.result().ok
        self.get_logger().info(f"激活 {MOVEIT_CONTROLLER}: {'成功' if ok else '未确认(可能已激活)'}")

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
    log.info(f"原6 · 手臂 MoveIt 末端控制 | 组={GROUP} EE={EE_LINK} 平移={DEMO_DELTA_XYZ}m vel={VEL_SCALE}")
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
