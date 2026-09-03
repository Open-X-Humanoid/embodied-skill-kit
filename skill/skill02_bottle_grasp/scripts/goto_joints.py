#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill02_bottle_grasp/scripts · 手臂关节角三合一：看 / 录 / 走

一句话
  一个脚本包办候选姿态的整条链路——拖动示教时实时看关节角(--watch)、取一次当前值
  (--show)、把录下的 7 个角走过去(传 7 个数)。用来在真机上快速验证一个姿态好不好使，
  不用改 config.py 再跑整套抓取流程。

三种模式
  --watch   2Hz 单行刷新，拖动示教边拖边看，Ctrl-C 输出可粘贴的一行   不动臂
  --show    读一次当前角就退出                                        不动臂
  J1..J7    走到指定关节角（走 MoveIt，动前等回车确认）              ★会动臂

为什么走 MoveIt 而不是 QP 关节控制器
  QP（不管关节空间还是末端空间）【完全不查 MoveIt 的 planning scene】，规划场景里有障碍
  物对它形同虚设。走 MoveIt 关节目标则能享受自碰撞检测；同时因为给的是关节角而不是笛卡尔
  位姿，不用过 IK 反解，保留了「不会拧麻花」这个关节空间移动的优点。与 grasp_bottle.py
  的 goto_ready() 同一条路径、同一套限速。
  ⚠代价：要过 MoveIt 更紧的关节限位表检查（QP 对此免疫）。报 error_code=99999 时首先怀疑
  【起点越界】——当前某个关节已在 MoveIt 限位之外，而不是目标够不到。见 motion04 guide 排错表。

用法（x86，ubuntu 用户；本终端须已 source XARM）
  source /home/ubuntu/XARM/install/setup.bash
  python3 skill/skill02_bottle_grasp/scripts/goto_joints.py --watch      # 拖动示教录姿态
  python3 skill/skill02_bottle_grasp/scripts/goto_joints.py --show       # 取一次当前角
  python3 skill/skill02_bottle_grasp/scripts/goto_joints.py J1 ... J7    # 走过去
  例（config 里的 READY_JOINTS）：
    python3 .../goto_joints.py 0.2 0.065 0.157 -0.373 0.005 0.089 -0.135

拖动示教配合 --watch（切重力补偿那几条服务调用见 motion04/motion07 guide）：
  ⚠ 切重力补偿的瞬间手臂会变软，必须【扶稳】；手里抓着重物时尤其危险（重力补偿只补偿
    手臂自重、不补偿负载），录姿态前先把负载取下来。

前提：body_control + XARM（含 MoveIt 组件）已启动；teleop_robot/xsys 已停（否则抢 /arm/cmd_pos）。

⚠ 安全
  1. 本脚本【会让手臂真实运动】。执行前臂周围无人无物、急停在手。
  2. 动之前会打印「当前角 → 目标角 → 逐关节变化」并停下等回车，核对幅度合理再放行。
  3. MoveIt 会规划路径并做自碰撞检测，但【不认识环境里的箱子/桌子/工件】——本仓库全程没
     把它们注册成碰撞体。大范围移动前自己看清路径。
  4. 手里抓着东西时格外当心：负载不在任何模型里，路径贴着底盘/箱子走可能撞到。
"""

import sys
from pathlib import Path

# config.py 在技能根目录，本脚本在 scripts/ 子目录里——把上一级加进模块搜索路径，
# 这样从仓库根目录跑 `python3 skill/skill02_bottle_grasp/scripts/goto_joints.py` 也能 import。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from controller_manager_msgs.srv import SwitchController, ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints, JointConstraint

import config as C

MOVE_ACTION = "/move_action"
SWITCH_SRV = "/controller_manager/switch_controller"
LIST_SRV = "/controller_manager/list_controllers"
ENABLE_SRV_CANDIDATES = ["/EAIHardware/set_arm_enable", "/moveit_controller_enable"]

JOINT_TOL = 0.01        # 单关节到位容差(rad)，与 grasp_bottle.goto_ready 一致


MODE_SHOW = "show"      # 读一次当前角就退出
MODE_WATCH = "watch"    # 2Hz 持续刷新，Ctrl-C 退出并输出最后一帧


def parse_args():
    """返回 (模式, 目标关节角)。模式为 MODE_SHOW/MODE_WATCH 时目标为 None。"""
    args = sys.argv[1:]
    if args and args[0] == "--show":
        return MODE_SHOW, None
    if args and args[0] == "--watch":
        return MODE_WATCH, None
    if len(args) != 7:
        print(f"用法: python3 {sys.argv[0]} J1 J2 J3 J4 J5 J6 J7   （7 个关节角，单位 rad，会动臂）")
        print(f"      python3 {sys.argv[0]} --show                  （读一次当前角就退出，不动臂）")
        print(f"      python3 {sys.argv[0]} --watch                 （2Hz 持续刷新，拖动示教用，不动臂）")
        sys.exit(1)
    try:
        return None, [float(a) for a in args]
    except ValueError:
        print("7 个参数都必须是数字")
        sys.exit(1)


class GotoJoints(Node):
    def __init__(self):
        super().__init__("skill02_goto_joints")
        self._cur_joints = {}
        self.js_sub_ = self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.moveit_client = ActionClient(self, MoveGroup, MOVE_ACTION)
        self.switch_cli = self.create_client(SwitchController, SWITCH_SRV)
        self.list_cli = self.create_client(ListControllers, LIST_SRV)

    def _on_js(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._cur_joints[name] = pos

    def read_arm_joints(self, timeout=5.0):
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(nm in self._cur_joints for nm in C.ARM_JOINT_NAMES):
                return [round(self._cur_joints[nm], 4) for nm in C.ARM_JOINT_NAMES]
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                return None

    def enable_arm(self):
        """使能服务名随 XARM 版本改过，逐个候选试（同 grasp_bottle.py）。"""
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
        self.get_logger().warn("使能未确认（若之前已使能则不影响）")
        return True

    def activate_moveit_controller(self):
        """切到 MoveIt 控制器，顺带停掉占着【本臂】的其它控制器（别误伤右臂/头/腰）。"""
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
        if to_stop:
            self.get_logger().info(f"已停占臂控制器 {to_stop}，切到 {want}")
        return True

    def move_to(self, target_joints):
        if not self.moveit_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"MoveGroup action {MOVE_ACTION} 不在。MoveIt 组件起了吗？")
            return False
        req = MotionPlanRequest()
        req.group_name = C.GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = C.VEL_SCALE      # 与主流程同一套限速(0.1，慢)
        req.max_acceleration_scaling_factor = C.ACC_SCALE
        c = Constraints()
        for name, val in zip(C.ARM_JOINT_NAMES, target_joints):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = jc.tolerance_below = JOINT_TOL
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

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
            f"MoveIt 关节目标失败 error_code={code}"
            "（99999 首查【起点越界】：当前某关节已在 MoveIt 限位外，先用 QP 挪回去再试；"
            "见 motion04 guide 排错表）")
        return False


def watch(node):
    """--watch：2Hz 单行原地刷新，拖动示教时边拖边看；Ctrl-C 退出并输出最后一帧。"""
    print("拖动手臂到理想姿态；满意时按 Ctrl-C，会输出可直接粘贴的一行\n")
    last = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            cur = [round(node._cur_joints[nm], 4) for nm in C.ARM_JOINT_NAMES] \
                if all(nm in node._cur_joints for nm in C.ARM_JOINT_NAMES) else None
            if cur is None:
                continue
            last = cur
            # 单行刷新：拖动时数字实时变，不刷屏
            print(f"\r当前 J1..J7 = {last}      ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    if last is None:
        print("\n未收到关节数据（body_control/XARM 起了吗？）")
    else:
        print(f"\n\n★ 可直接粘贴：\nREADY_JOINTS = {last}\n")


def main():
    mode, target = parse_args()
    rclpy.init()
    node = GotoJoints()
    try:
        if mode == MODE_WATCH:                   # --watch：持续刷新，不动臂
            watch(node)
            return

        cur = node.read_arm_joints()
        if cur is None:
            node.get_logger().error("读不到 /joint_states 的左臂关节角——body_control/XARM 起了吗？")
            return
        node.get_logger().info(f"当前 J1..J7 = {cur}")
        if mode == MODE_SHOW:                    # --show：读一次就退出，不动臂
            print(f"\n★ 可直接粘贴：\nREADY_JOINTS = {cur}\n")
            return

        delta = [round(t - c, 4) for t, c in zip(target, cur)]
        node.get_logger().info(f"目标 J1..J7 = {[round(t, 4) for t in target]}")
        node.get_logger().info(f"逐关节变化 = {delta}（最大 {max(abs(d) for d in delta):.3f} rad）")
        input("★确认臂周围无人无物、路径无障碍、急停在手 → 回车开始移动（Ctrl-C 取消）...")

        node.enable_arm()
        node.activate_moveit_controller()
        if node.move_to(target):
            node.get_logger().info("✓ 已到目标关节角")
            after = node.read_arm_joints()
            if after is not None:
                node.get_logger().info(f"到位后 J1..J7 = {after}")
        else:
            node.get_logger().error("移动失败，臂停在当前位置")
    except KeyboardInterrupt:
        node.get_logger().warn("用户中断")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
