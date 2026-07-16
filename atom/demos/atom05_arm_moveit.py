#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原5 · 手臂（Arm）· XARM MoveIt 关节运动
配套讲解：atom/docs/atom05_arm_moveit_guide.md

一句话
  用标准 MoveIt2 的 move_group，让手臂从当前角小幅、可复位地运动到目标关节角
  （读当前 → 规划无碰撞轨迹 → 执行 → 回原位）。不像 atom04 硬发角度，这里是"说去哪、它自己规划怎么去"。

简洁版 vs robust
  · 本文件=简洁版：看懂原理即可，假设 XARM 刚起、本臂控制器无冲突，直接 BEST_EFFORT 激活 moveit 控制器。
  · atom05_arm_moveit_robust.py=生产版：切控制器前自动停占用本臂的控制器 + STRICT 切换 + 严格超时校验。

运行前提（x86 / ubuntu；官方 5 步 SOP，缺一步臂不动，后两步本脚本自动做）
  1) 起 body_control（x86，真机必需；仿真跳过）
  2) 起 XARM 本体：   ros2 launch tianyi2_bringup tianyi2.launch.py hardware:=real   （仿真: gui:=true）
  3) 起 MoveIt 组件： ros2 launch tianyi2_bringup tianyi2_moveit.launch.py
  4) ③使能手臂  ④切 moveit 控制器（本脚本自动）
  ★ 跑前 source 的是 XARM，不是 ros2ws： source /home/ubuntu/XARM/install/setup.bash
     一键前置： bash scripts/start_xarm.sh sim   /   source scripts/start_xarm.sh

接口（标准 MoveIt2 + XARM 使能）
  Action   /move_action                            moveit_msgs/action/MoveGroup   送目标关节角、规划并执行
  Service  /EAIHardware/set_arm_enable             std_srvs/SetBool               real 模式使能（sim 无）
  Service  /controller_manager/switch_controller                                  激活 moveit_*_arm_controller
  Topic    /joint_states                           sensor_msgs/JointState         读当前关节角作起点

两个必知坑（real 模式）
  1) 不使能 → 规划/执行都报成功，但 XARM 不发 cmd_pos → 物理臂不动（头号坑）。
  2) 不切 moveit 控制器 → 执行 error_code=-4 CONTROL_FAILED。
  sim 无 /EAIHardware，使能自动跳过；动作在 RViz / joint_states 看。

已验证（天轶2.0 真机）
  GROUP=left_arm、下面 7 个关节名、moveit_left_arm_controller、/move_action、使能服务 均实测可用。
  换机器人/右臂自查： ros2 control list_controllers | grep -i moveit
                     ros2 topic echo --once --field name /joint_states

安全
  手臂力矩大：臂周围无人无物、急停在手、建议先仿真；首次 VEL/ACC_SCALE=0.1，本 demo 从当前角小幅可复位。
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import (MotionPlanRequest, Constraints,
                                 JointConstraint, PlanningOptions)
    from controller_manager_msgs.srv import SwitchController
    from std_srvs.srv import SetBool
except ImportError:
    print("❌ 找不到 moveit_msgs / controller_manager_msgs。请先 source XARM 环境：")
    print("   source /home/ubuntu/XARM/install/setup.bash")
    sys.exit(1)

# ── 机器人相关常量（天轶2.0 已实测；换机器人/右臂时自查）──
GROUP = "left_arm"                       # 规划组名
MOVEIT_CONTROLLER = "moveit_left_arm_controller"   # MoveIt 执行用的控制器
JOINT_NAMES = [                          # 左臂 7 关节（顺序即目标角顺序）
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
]
MOVE_ACTION = "/move_action"
SWITCH_SRV = "/controller_manager/switch_controller"
ENABLE_SRV = "/EAIHardware/set_arm_enable"

DEMO_DELTA = [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0]   # 只动肘俯仰 +0.3rad(~17°)，可复位
VEL_SCALE = 0.1
ACC_SCALE = 0.1
# ─────────────────────────────────────────────────────────────────


class ArmMoveItDemo(Node):
    def __init__(self):
        super().__init__("atom_arm_moveit_demo")
        self.client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.switch_cli = self.create_client(SwitchController, SWITCH_SRV)
        self.enable_cli = self.create_client(SetBool, ENABLE_SRV)
        self.cur = {}   # {joint_name: position}
        self.sub_ = self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.cur[name] = pos

    def enable_arm(self):
        """使能手臂（real 模式必需：使能后 XARM 才向 body_control 发指令）。sim 无此服务，自动跳过。"""
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
        """激活 MoveIt 控制器（不激活执行会 CONTROL_FAILED -4）。
        简洁版直接 BEST_EFFORT 激活；若本臂被别的控制器占着，用 _robust 版自动避让。"""
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

    def read_current(self, joint_names, timeout=5.0):
        """读当前关节角（按 joint_names 顺序返回 list）；读不全返回 None。"""
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(n in self.cur for n in joint_names):
                return [self.cur[n] for n in joint_names]
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                self.get_logger().error("超时未读到全部关节角（/joint_states）。XARM 起了吗？")
                return None

    def move_to_joints(self, joint_names, target, vel=VEL_SCALE, acc=ACC_SCALE, tol=0.01):
        """规划并运动到目标关节角（阻塞等结果）。返回 True/False。"""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"MoveGroup action {MOVE_ACTION} 不在。MoveIt 组件起了吗？")
            return False
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc
        constraints = Constraints()
        for name, pos in zip(joint_names, target):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(pos),
                tolerance_above=tol, tolerance_below=tol, weight=1.0))
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=False)

        self.get_logger().info(f"规划 → {GROUP} 到 {[round(x, 3) for x in target]}")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("目标被拒绝/发送超时（组名/关节名？MoveGroup 在吗？）")
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
        self.get_logger().error(f"MoveIt 失败 error_code={code}（-4=控制器没激活, -15=组名错，详见 guide）")
        return False


def main():
    rclpy.init()
    node = ArmMoveItDemo()
    log = node.get_logger()
    log.info(f"原5 · 手臂 MoveIt 关节运动 | 规划组={GROUP} vel={VEL_SCALE} acc={ACC_SCALE} 演示增量={DEMO_DELTA}")
    input("确认 XARM 本体 + MoveIt 组件已启动、臂周围无人无物 → 按 Enter 继续（Ctrl-C 取消）...")

    try:
        node.enable_arm()                    # ③ real 模式使能（不使能臂不动）
        node.activate_moveit_controller()    # ④ 切 moveit 控制器

        start = node.read_current(JOINT_NAMES)
        if start is None:
            return
        log.info(f"当前左臂角 = {[round(x, 3) for x in start]}")
        target = [s + d for s, d in zip(start, DEMO_DELTA)]

        log.info("Step 1: 从当前角做小幅运动 ...")
        if node.move_to_joints(JOINT_NAMES, target):
            log.info("Step 2: 回到起始角 ...")
            node.move_to_joints(JOINT_NAMES, start)
    except KeyboardInterrupt:
        log.warn("用户中断")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
