#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原7 · 手臂（Arm）· XARM QP 关节空间控制
配套讲解：atom/docs/atom07_arm_qp_guide.md

一句话
  把 7 个目标关节角交给 XARM 的 QP 关节控制器（jointspace_arm_L_controller），
  它在线做速度/加速度/jerk 平滑与限位检查后执行。
  和 atom05 的区别：atom05 是 MoveIt"一次规划整条轨迹再执行"；QP 是"响应式在线跟踪"，
  收到新目标立即平滑跟过去，适合目标不断变化的场景（如视觉伺服）。

控制器互斥（重要）
  moveit/jointspace/endpose 各族控制器都抢同一条臂的关节接口，同一时刻只能一个 active
  （例如刚跑过 atom08 的 endpose 控制器还 active，本控制器就激活不了）。
  本脚本激活前自动查询并停掉占用本臂的其它 active 控制器，再 STRICT 切换（失败如实报错）。

运行前提（x86 / ubuntu；比 atom05 少一步——QP 不需要 MoveIt 组件）
  1) 起 body_control（真机必需；仿真跳过）
  2) 起 XARM 本体：   ros2 launch tianyi2_bringup tianyi2.launch.py hardware:=real   （仿真: gui:=true）
  3) ③使能手臂  ④切 QP 控制器（本脚本自动）
  ★ 跑前 source 的是 XARM： source /home/ubuntu/XARM/install/setup.bash
     一键前置： bash scripts/start_xarm.sh sim   /   source scripts/start_xarm.sh

接口（XARM 原生 QP 控制器）
  Action   /jointspace_arm_L_controller/jointspace   eai_manipulator_msgs/action/JointSpace
           目标=7 个关节角(rad)，result.success 报成败（另有流式 topic /jointspace_commands_L，二选一）
  Service  <使能服务>                                std_srvs/SetBool    real 模式使能（sim 无）
           ★名随 XARM 版本变：/EAIHardware/set_arm_enable（QP 原生）或 /moveit_controller_enable，
             运行时逐个探测、调失败自动换下一个（见 ENABLE_SRV_CANDIDATES）
  Service  /controller_manager/switch_controller                          激活 jointspace_arm_L_controller
  Service  /controller_manager/list_controllers                           查占用本臂的控制器（自动避让用）
  Topic    /joint_states                             sensor_msgs/JointState   读当前关节角作起点

⚠ 待真机核实（首次跑必查）
  1) 目标数组顺序：按手册示例推断为 J1肩俯仰→J7腕旋转（同 JOINT_NAMES 顺序）
  2) action 与 topic 只能用其一；用 action 时确保没人往 /jointspace_commands_L 发流

⚠ 安全
  本控制器【无防碰撞】（手册明示；防自碰版是 jointspace_arm_qpik_L_controller，但不支持 action）。
  臂周围无人无物、急停在手、先仿真；本 demo 从当前角小幅可复位（默认肘 +0.3rad）。
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState

try:
    from eai_manipulator_msgs.action import JointSpace
    from controller_manager_msgs.srv import SwitchController, ListControllers
    from std_srvs.srv import SetBool
except ImportError:
    print("❌ 找不到 eai_manipulator_msgs / controller_manager_msgs。请先 source XARM 环境：")
    print("   source /home/ubuntu/XARM/install/setup.bash")
    sys.exit(1)

# ── 机器人相关常量（★待真机核实；换右臂 L→R）──
QP_CONTROLLER = "jointspace_arm_L_controller"        # QP 关节控制器（切换/action 前缀同名）
JOINT_NAMES = [                                      # 左臂 7 关节（顺序即目标数组顺序；也用于判断"谁占着本臂"）
    "shoulder_pitch_l_joint", "shoulder_roll_l_joint", "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint", "elbow_yaw_l_joint", "wrist_pitch_l_joint", "wrist_roll_l_joint",
]
QP_ACTION = f"/{QP_CONTROLLER}/jointspace"
SWITCH_SRV = "/controller_manager/switch_controller"
LIST_SRV = "/controller_manager/list_controllers"
# 使能服务名随 XARM 版本变；QP 原生的旧服务放首位，新服务作后备（它顺带切 moveit 控制器，
# 但随后本脚本会把控制器切回 QP，不冲突）。调失败自动试下一个。
ENABLE_SRV_CANDIDATES = ["/EAIHardware/set_arm_enable", "/moveit_controller_enable"]

DEMO_DELTA = [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0]     # 只动肘俯仰 +0.3rad(~17°)，可复位
# ─────────────────────────────────────────────────────────────────


class ArmQpJointDemo(Node):
    def __init__(self):
        super().__init__("atom_arm_qp_joint_demo")
        self.client = ActionClient(self, JointSpace, QP_ACTION)
        self.switch_cli = self.create_client(SwitchController, SWITCH_SRV)
        self.list_cli = self.create_client(ListControllers, LIST_SRV)
        self.cur = {}   # {joint_name: position}
        self.sub_ = self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.cur[name] = pos

    def enable_arm(self):
        """使能手臂（real 模式必需：使能后 XARM 才向 body_control 发指令）。sim 无此服务，自动跳过。
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

    def move_to_joints(self, target):
        """把 7 个目标关节角发给 QP 控制器（阻塞等结果）。返回 True/False。"""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"QP action {QP_ACTION} 不在。{QP_CONTROLLER} 激活了吗？")
            return False
        goal = JointSpace.Goal(target_positions=[float(x) for x in target])
        self.get_logger().info(f"QP 目标 → {[round(x, 3) for x in target]}")
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("目标被拒绝/发送超时（超限位？控制器没激活？）")
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        res = result_future.result()
        if res is None:
            self.get_logger().error("执行结果超时(60s)。")
            return False
        if res.result.success:
            # ★实测坑：本控制器为在线跟踪型，action 返回=目标已接受，不等于已到位
            #   （立即返回后若马上发下一目标，会覆盖上一目标 → 臂几乎不动）。
            #   所以还要轮询 /joint_states 等实际角度收敛。
            self.get_logger().info(f"目标已接受（{res.result.result_msg or 'success'}），等待实际到位...")
            return self.wait_reached(target)
        self.get_logger().error(
            f"QP 失败：{res.result.result_msg}（600101/600102=超上/下限位, 600100=优化不可行，详见 guide）")
        return False

    def wait_reached(self, target, tol=0.05, timeout=15.0):
        """轮询 /joint_states，等全部关节进入 target±tol(rad) 或超时。"""
        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            cur = [self.cur.get(n) for n in JOINT_NAMES]
            if all(c is not None and abs(c - t) < tol for c, t in zip(cur, target)):
                self.get_logger().info("✓ 实际到位（/joint_states 收敛）")
                return True
            if (self.get_clock().now() - t0).nanoseconds > timeout * 1e9:
                self.get_logger().warn(
                    f"等待到位超时（{timeout}s）。当前={[None if c is None else round(c, 3) for c in cur]}，"
                    "臂可能未使能或目标被后续指令覆盖")
                return False


def main():
    rclpy.init()
    node = ArmQpJointDemo()
    log = node.get_logger()
    log.info(f"原7 · 手臂 QP 关节控制 | 控制器={QP_CONTROLLER} 演示增量={DEMO_DELTA}")
    input("确认 XARM 本体已启动（QP 不需要 MoveIt 组件）、臂周围无人无物 → 按 Enter 继续（Ctrl-C 取消）...")

    try:
        node.enable_arm()                    # ③ real 模式使能（不使能臂不动）
        node.activate_qp_controller()        # ④ 切 QP 控制器（停 moveit）

        start = node.read_current(JOINT_NAMES)
        if start is None:
            return
        log.info(f"当前左臂角 = {[round(x, 3) for x in start]}")
        target = [s + d for s, d in zip(start, DEMO_DELTA)]

        log.info("Step 1: 从当前角做小幅运动 ...")
        if node.move_to_joints(target):
            log.info("Step 2: 回到起始角 ...")
            node.move_to_joints(start)
    except KeyboardInterrupt:
        log.warn("用户中断")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
