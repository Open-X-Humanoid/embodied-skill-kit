#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原8 · 手臂（Arm）· XARM QP 末端（笛卡尔）控制
配套讲解：atom/docs/atom08_arm_qp_endpose_guide.md

一句话
  给 XARM 的 QP 末端控制器（endpose_single_arm_qp_L_controller）一个"末端位姿"目标，
  它做局部响应式跟踪把 TCP 送过去，自带防碰撞。
  和 atom06 的区别：atom06 是 MoveIt"一次规划整条轨迹"；QP 是"在线小步跟踪"——
  目标离当前太远会被直接拒绝（误差限 dis_err_bound/ori_err_bound），只能小幅增量喂。

控制器互斥（重要）
  moveit/jointspace/endpose 各族控制器都抢同一条臂的关节接口，同一时刻只能一个 active
  （例如刚跑过 atom07 的 jointspace 控制器还 active，本控制器就激活不了、臂不动）。
  本脚本激活前自动查询并停掉占用本臂的其它 active 控制器，再 STRICT 切换（失败如实报错）。

运行前提（x86 / ubuntu；同 atom07——QP 不需要 MoveIt 组件）
  1) 起 body_control（真机必需；仿真跳过）
  2) 起 XARM 本体：   ros2 launch tianyi2_bringup tianyi2.launch.py hardware:=real   （仿真: gui:=true）
  3) ③使能手臂  ④切 QP 末端控制器（本脚本自动）
  ★ 跑前 source 的是 XARM： source /home/ubuntu/XARM/install/setup.bash
     一键前置： bash scripts/start_xarm.sh sim   /   source scripts/start_xarm.sh

接口（XARM 原生 QP 末端控制器）
  Action   /endpose_single_arm_qp_L_controller/endPosSingleTarget
           eai_manipulator_msgs/action/EndPosSingleTarget
           目标=ArmTargetPose（from_frame→to_frame 的期望位姿），result.success 报成败
           （另有流式 topic /endposetarget_L，二选一）
  Service  <使能服务>                                std_srvs/SetBool    real 模式使能（sim 无）
           ★名随 XARM 版本变：/EAIHardware/set_arm_enable（QP 原生）或 /moveit_controller_enable，
             运行时逐个探测、调失败自动换下一个（见 ENABLE_SRV_CANDIDATES）
  Service  /controller_manager/switch_controller                          激活 endpose_single_arm_qp_L_controller
  Service  /controller_manager/list_controllers                           查占用本臂的控制器（自动避让用）
  Service  /endpose_single_arm_qp_L_controller/set_parameters             改 vel_limits 调速（见 VEL_LIMITS）
  TF       base → left_tcp_link                      读当前末端 TCP 位姿作起点

⚠ 待真机核实（首次跑必查）
  1) FROM_FRAME=base：XARM 手册对天轶的建议值（也可 waist_yaw_link）；TO_FRAME=left_tcp_link
  2) 5cm 增量是否在 dis_err_bound 误差限内（超限会拒绝执行并报警，改小 DEMO_DELTA_XYZ 重试）

⚠ 安全
  本控制器自带防碰撞（身体 2 球 + 手臂 7 球模型），但仍需：臂周围无人无物、急停在手、先仿真。
  本 demo 姿态不变、末端小幅平移（默认 +z 上抬 5cm）再复位，可复位、低速。
"""

import sys
import copy
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose

try:
    from eai_manipulator_msgs.action import EndPosSingleTarget
    from eai_manipulator_msgs.msg import ArmTargetPose
    from controller_manager_msgs.srv import SwitchController, ListControllers
    from std_srvs.srv import SetBool
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from tf2_ros import Buffer, TransformListener
except ImportError:
    print("❌ 找不到 eai_manipulator_msgs / tf2_ros 等。请先 source XARM 环境：")
    print("   source /home/ubuntu/XARM/install/setup.bash")
    sys.exit(1)

# ── 机器人相关常量（★待真机核实；换右臂 L→R、left→right）──
QP_CONTROLLER = "endpose_single_arm_qp_L_controller"  # QP 末端控制器（切换/action 前缀同名）
FROM_FRAME = "base"                                   # 参考基坐标系（XARM 手册天轶建议值）
TO_FRAME = "left_tcp_link"                            # 末端 TCP link
JOINT_NAMES = [                                       # 左臂 7 关节名（判断"谁占着本臂"用）
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
]
QP_ACTION = f"/{QP_CONTROLLER}/endPosSingleTarget"
SWITCH_SRV = "/controller_manager/switch_controller"
LIST_SRV = "/controller_manager/list_controllers"
PARAM_SRV = f"/{QP_CONTROLLER}/set_parameters"        # 改控制器参数（调速用）
# 使能服务名随 XARM 版本变；QP 原生的旧服务放首位，新服务作后备（它顺带切 moveit 控制器，
# 但随后本脚本会把控制器切回 QP，不冲突）。调失败自动试下一个。
ENABLE_SRV_CANDIDATES = ["/EAIHardware/set_arm_enable", "/moveit_controller_enable"]

DEMO_DELTA_XYZ = [0.0, 0.0, 0.05]                     # 末端小幅平移(m)：默认 +z 上抬 5cm，可复位
# 调速主旋钮：QP 速度是控制器参数（不像 MoveIt 在下发时给），越小越慢。7 关节速度上限 rad/s。
# 设 None 则不改、用控制器当前值。手册：vel_limits 不应超过物理机构执行能力，只往小调是安全方向。
VEL_LIMITS = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
# ─────────────────────────────────────────────────────────────────


class ArmQpEndposeDemo(Node):
    def __init__(self):
        super().__init__("atom_arm_qp_endpose_demo")
        self.client = ActionClient(self, EndPosSingleTarget, QP_ACTION)
        self.switch_cli = self.create_client(SwitchController, SWITCH_SRV)
        self.list_cli = self.create_client(ListControllers, LIST_SRV)
        self.param_cli = self.create_client(SetParameters, PARAM_SRV)
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
        """查 controller_manager：返回 (本控制器是否已active, 占用本臂关节、需先停的其它active控制器名)。
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
            if c.name == QP_CONTROLLER:
                already_active = True
                continue
            if any(ci.split("/")[0] in JOINT_NAMES for ci in c.claimed_interfaces):
                conflict.append(c.name)
        return (already_active, conflict)

    def activate_qp_controller(self):
        """激活 QP 控制器：先停掉占用本臂的其它 active 控制器（各族互斥），再 STRICT 切换（失败如实报错）。"""
        if not self.switch_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"{SWITCH_SRV} 不在，跳过自动切换（请手动 ros2 control switch_controllers）")
            return
        already_active, to_stop = self._scan_arm_controllers()
        if already_active and not to_stop:
            self.get_logger().info(f"{QP_CONTROLLER} 已激活且无冲突，无需切换")
            return
        req = SwitchController.Request()
        req.activate_controllers = [] if already_active else [QP_CONTROLLER]
        req.deactivate_controllers = to_stop
        req.strictness = SwitchController.Request.STRICT
        if to_stop:
            self.get_logger().info(f"先停占用本臂的控制器: {to_stop}")
        future = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        ok = resp is not None and resp.ok
        self.get_logger().info(f"切到 {QP_CONTROLLER}: {'成功' if ok else '失败(检查控制器名/资源占用)'}")

    def set_vel_limits(self, vel):
        """改控制器的 vel_limits（7 元素 rad/s）调速——QP 速度是控制器参数，越小越慢。
        vel=None 则跳过；本质是调另一个节点(控制器)的参数服务，等价命令行 ros2 param set。"""
        if vel is None:
            return
        if not self.param_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"{PARAM_SRV} 不在，跳过调速（可改用命令行 ros2 param set）")
            return
        p = Parameter(
            name="vel_limits",
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(x) for x in vel]))
        future = self.param_cli.call_async(SetParameters.Request(parameters=[p]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        if resp is not None and resp.results and resp.results[0].successful:
            self.get_logger().info(f"✓ vel_limits 设为 {vel}（越小越慢）")
        else:
            reason = resp.results[0].reason if (resp and resp.results) else "超时/无响应"
            self.get_logger().warn(
                f"设 vel_limits 未成功（{reason}）——该控制器可能不允许运行时改此参数，"
                f"退回命令行：ros2 param set {QP_CONTROLLER} vel_limits '[...]'")

    def read_ee_pose(self, timeout=5.0):
        """用 TF 读当前末端 TCP 在 FROM_FRAME 下的位姿（Pose）。读不到返回 None。"""
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tr = self.tf_buffer.lookup_transform(FROM_FRAME, TO_FRAME, Time()).transform
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = (
                    tr.translation.x, tr.translation.y, tr.translation.z)
                pose.orientation = tr.rotation
                return pose
            except Exception:
                pass
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                self.get_logger().error(
                    f"超时未读到 {TO_FRAME} 位姿（TF）。FROM_FRAME={FROM_FRAME} / TO_FRAME 对吗？")
                return None

    def move_to_pose(self, pose):
        """把末端目标位姿发给 QP 控制器（阻塞等结果）。返回 True/False。"""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"QP action {QP_ACTION} 不在。{QP_CONTROLLER} 激活了吗？")
            return False
        target = ArmTargetPose()
        target.header.frame_id = FROM_FRAME
        target.target = pose
        target.from_frame = FROM_FRAME
        target.to_frame = TO_FRAME
        target.offset_x = target.offset_y = target.offset_z = 0.0

        goal = EndPosSingleTarget.Goal(target=target)
        p = pose.position
        self.get_logger().info(f"QP 末端目标 → ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})  [{FROM_FRAME}→{TO_FRAME}]")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("目标被拒绝/发送超时（离当前太远超误差限？控制器没激活？）")
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        res = result_future.result()
        if res is None:
            self.get_logger().error("执行结果超时(60s)。")
            return False
        if res.result.success:
            self.get_logger().info(f"✓ 到位（{res.result.result_msg or 'success'}）")
            return True
        self.get_logger().error(
            f"QP 失败：{res.result.result_msg}（目标超 dis/ori_err_bound 误差限？改小增量重试，详见 guide）")
        return False


def main():
    rclpy.init()
    node = ArmQpEndposeDemo()
    log = node.get_logger()
    log.info(f"原8 · 手臂 QP 末端控制 | 控制器={QP_CONTROLLER} {FROM_FRAME}→{TO_FRAME} 平移={DEMO_DELTA_XYZ}m")
    input("确认 XARM 本体已启动（QP 不需要 MoveIt 组件）、臂周围无人无物 → 按 Enter 继续（Ctrl-C 取消）...")

    try:
        node.enable_arm()                    # ③ real 模式使能（不使能臂不动）
        node.activate_qp_controller()        # ④ 切 QP 末端控制器（停 moveit）
        node.set_vel_limits(VEL_LIMITS)      # ⑤ 调速（越小越慢；None 则用控制器当前值）

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
