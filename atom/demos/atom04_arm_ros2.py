#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原4 · 手臂（Arm） —— ROS2 原生版
在关节空间里，慢速、小幅地移动一条 7 自由度手臂的单个关节。  配套讲解: atom/docs/atom04_arm_guide.md

【版本说明】本文件是 ROS2 原生实现（/arm/cmd_pos 位置模式直发），这是最底层的关节位置控制。
  工程实战中手臂通常走 xArm 的 QP / MoveIt 封装（带避障、逆解、力控），接口与本文件不同，另见对应变体。

接口:
  话题: /arm/cmd_pos     类型: bodyctrl_msgs/CmdSetMotorPosition
        SetMotorPosition[] cmds:  name(电机ID)  pos(rad)  spd(rad/s)  cur(最大电流A)
  电机ID: 左臂 11~17，右臂 21~27（1=肩 … 7=腕）   单位: 弧度(rad)
  状态反馈: /arm/status   bodyctrl_msgs/MotorStatusMsg

⚠ 安全（务必先读）:
  1) 手臂力矩大、范围大；先确保臂周围无人无物，急停在手边。
  2) 第一次跑请只动 1~2 个关节、幅度 < 0.3rad、速度 0.2rad/s。
  3) 先订阅 /arm/status 读到当前角度，从当前角度小步增量，不要直接发大角度。
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition, MotorStatusMsg

ARM_CMD_TOPIC = "/arm/cmd_pos"
ARM_STATUS_TOPIC = "/arm/status"

DEMO_JOINT_ID = 12   # 左臂第 2 个关节（右臂同理用 22）
SPEED = 0.2          # rad/s，越小越安全
MAX_CUR = 1.0        # A


class ArmDemo(Node):
    def __init__(self):
        super().__init__("atom_arm_demo")
        self.pub = self.create_publisher(CmdSetMotorPosition, ARM_CMD_TOPIC, 10)
        self.cur_pos = {}  # {motor_id: pos}
        self.status_sub_ = self.create_subscription(MotorStatusMsg, ARM_STATUS_TOPIC, self._on_status, 1)
        self.get_logger().info(f"手臂原子 demo 已启动，发布到 {ARM_CMD_TOPIC}")

    def _on_status(self, msg: MotorStatusMsg):
        for s in msg.status:
            self.cur_pos[s.name] = s.pos

    def xarm_owns_arm(self):
        """互斥处理（实测坑）：XARM 已使能时它以 250Hz 高频流持续控制手臂，
        直发 /arm/cmd_pos 会被瞬间覆盖 → 臂看似不动且无报错。
        检测到使能即【自动关闭】接管手臂（XARM 系 demo 下次跑时会自动重新使能，双向零手动）。
        自动关闭失败才报错退出。返回 True=仍被 XARM 占有、应退出。"""
        try:
            from eai_manipulator_msgs.srv import Info
        except ImportError:
            return False   # 无 XARM 消息包的环境，跳过检查
        cli = self.create_client(Info, "/EAIHardware/debug")
        if not cli.wait_for_service(timeout_sec=1.0):
            self.destroy_client(cli)
            return False   # XARM 未起，无冲突
        future = cli.call_async(Info.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        resp = future.result()
        if resp is None or "arm_enable: 1" not in resp.info:
            return False   # 未使能，无冲突
        # XARM 使能中 → 自动关掉，交接控制权（等价手动 ros2 service call ... "{data: false}"）
        from std_srvs.srv import SetBool
        self.get_logger().info("检测到 XARM 已使能（会覆盖直发指令），自动关闭 XARM 使能、接管手臂...")
        dis = self.create_client(SetBool, "/EAIHardware/set_arm_enable")
        if dis.wait_for_service(timeout_sec=2.0):
            future = dis.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            r = future.result()
            if r is not None and r.success:
                self.get_logger().info("✓ 已关闭 XARM 使能，直发接管（下次跑 XARM 系 demo 会自动重新使能）")
                return False
        self.get_logger().error(
            "自动关闭 XARM 使能失败，直发会被覆盖，本 demo 退出。手动关闭后重试：\n"
            '  ros2 service call /EAIHardware/set_arm_enable std_srvs/srv/SetBool "{data: false}"')
        return True

    def wait_for_status(self, motor_id, timeout=3.0):
        """等到读到该关节当前角度，作为运动起点。"""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if motor_id in self.cur_pos:
                return self.cur_pos[motor_id]
        # ⚠ 兜底假设 0.0 有风险：若手臂实际在大角度处，后续会从 0.0 起算导致跳变。
        #   这是简洁版的简化；生产版 robust 改为返回 None 并拒绝运动——真机勿依赖此兜底。
        self.get_logger().warn("未读到 /arm/status，假设当前角度为 0.0（有跳变风险，见 robust 版）")
        return 0.0

    def move_joint(self, motor_id: int, target_pos: float, hold: float = 2.0):
        msg = CmdSetMotorPosition()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.cmds = [SetMotorPosition(name=int(motor_id), pos=float(target_pos),
                                     spd=SPEED, cur=MAX_CUR)]
        self.pub.publish(msg)
        self.get_logger().info(f"关节 {motor_id} -> {target_pos:.3f} rad")
        time.sleep(hold)


def main():
    rclpy.init()
    node = ArmDemo()
    try:
        if node.xarm_owns_arm():     # XARM 使能中会覆盖直发指令（互斥），明确报错退出
            return
        start = node.wait_for_status(DEMO_JOINT_ID)
        node.get_logger().info(f"关节 {DEMO_JOINT_ID} 当前角度 = {start:.3f} rad")
        # 从当前角度做一个 ±0.1rad 的小幅来回，再回到起点
        node.move_joint(DEMO_JOINT_ID, start + 0.1)
        node.move_joint(DEMO_JOINT_ID, start - 0.1)
        node.move_joint(DEMO_JOINT_ID, start)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
