#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原33 · 手臂力位混合模式（Force-Position Hybrid） —— ROS2 原生版
在关节空间演示力位混合接口：在位置目标基础上叠加速度/力矩前馈，
通过 kp/kd 决定关节"刚度"——小值柔顺（可被外力推开），大值刚硬。
配套讲解: atom/docs/atom33_arm_force_position_guide.md

【版本说明】本文件是 ROS2 原生实现（/arm/cmd_ctrl 力位混合模式直发）。
  工程实战中手臂通常走 xArm 的 QP / MoveIt 封装（带避障、逆解、力控），接口与本文件不同，另见对应变体。

接口:
  控制: /arm/cmd_ctrl   bodyctrl_msgs/CmdMotorCtrl
        MotorCtrl[] cmds:  name(电机ID) kp kd pos(目标rad) spd(前馈rad/s) tor(前馈Nm)
        控制律: τ = kp·(pos-θ) + kd·(spd-θ̇) + tor
  回位:  /arm/cmd_pos   bodyctrl_msgs/CmdSetMotorPosition   （测试结束后用它送回起点）
  反馈: /arm/status     bodyctrl_msgs/MotorStatusMsg
  电机 ID: 左臂 11~17，右臂 21~27（1=肩 … 7=腕）   单位: 弧度(rad)

⚠ 安全（务必先读）:
  1) 手臂力矩大、范围大；确保臂周围无人无物，急停在手边。
  2) 保持期内可用手轻推关节感受柔顺性，但仍要小心——kp/kd 越大关节越"硬"。
  3) 目标偏移量 CTRL_DELTA 默认 +0.5rad，按左臂 J2（DEMO_JOINT_ID=12，
     侧展限位约 -0.26~2.61）的外展方向设计；换关节前请对照该关节机械范围重算。
  4) 左右臂 J2 关节角方向镜像：右臂 22 的侧展在负方向（限位约 -2.61~0.26）。
     只改 DEMO_JOINT_ID=22 而不改 CTRL_DELTA 符号，会向体内收、有撞躯干风险；
     改测右臂时应同时设 CTRL_DELTA=-0.5（并核对目标仍在限位内）。
"""

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import (
    CmdSetMotorPosition, SetMotorPosition,
    CmdMotorCtrl, MotorCtrl,
    MotorStatusMsg,
)

ARM_STATUS_TOPIC   = "/arm/status"
ARM_CMD_POS_TOPIC  = "/arm/cmd_pos"    # 位置模式，仅用于测试结束后回初始位
ARM_CMD_CTRL_TOPIC = "/arm/cmd_ctrl"   # 力位混合模式

DEMO_JOINT_ID = 12    # 左臂 J2（肩侧展）；改右臂 22 时须同时取反 CTRL_DELTA，见上方安全说明 4)
RETURN_SPEED  = 0.2   # rad/s，回初始位时的限速
RETURN_CUR    = 1.0   # A，    回初始位时的最大电流

CTRL_KP    = 15.0   # 位置增益（合法范围 0~2000；小值=柔顺，大值=刚硬）
CTRL_KD    = 1.5    # 速度增益（合法范围 0~300）
CTRL_DELTA = 0.5    # 目标位移增量 (rad)；左臂外展用正值，右臂 J2 外展须用负值
CTRL_HOLD  = 5.0    # 保持时间 (s)，便于手动测试柔顺性


class ArmForcePositionDemo(Node):
    def __init__(self):
        super().__init__("atom_arm_force_position_demo")
        self.pub_pos  = self.create_publisher(CmdSetMotorPosition, ARM_CMD_POS_TOPIC,  1)
        self.pub_ctrl = self.create_publisher(CmdMotorCtrl,        ARM_CMD_CTRL_TOPIC, 1)
        self.cur_pos: dict[int, float] = {}
        self.status_sub_ = self.create_subscription(
            MotorStatusMsg, ARM_STATUS_TOPIC, self._on_status, 1)
        self.get_logger().info("手臂力位混合 demo 已启动")

    def _on_status(self, msg: MotorStatusMsg) -> None:
        for s in msg.status:
            self.cur_pos[s.name] = s.pos

    def wait_for_status(self, motor_id: int, timeout: float = 3.0) -> float:
        """等到读到该关节当前角度，作为运动起点。"""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if motor_id in self.cur_pos:
                return self.cur_pos[motor_id]
        self.get_logger().warn(
            "未读到 /arm/status，假设当前角度为 0.0（有跳变风险，见 atom04 robust 版）")
        return 0.0

    def _spin_hold(self, duration: float) -> None:
        """保持期内持续 spin（而非 time.sleep 阻塞），让 _on_status 保持更新。"""
        deadline = time.time() + duration
        while rclpy.ok():
            remaining = deadline - time.time()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

    def _return_to(self, motor_id: int, pos: float, hold: float = 2.5) -> None:
        """用位置模式将关节送回指定角度。"""
        msg = CmdSetMotorPosition()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.cmds = [SetMotorPosition(
            name=int(motor_id), pos=float(pos),
            spd=RETURN_SPEED, cur=RETURN_CUR)]
        self.pub_pos.publish(msg)
        self.get_logger().info(f"[回位] 关节 {motor_id} -> {pos:.3f} rad，等待 {hold}s")
        time.sleep(hold)

    def move_ctrl(self, motor_id: int, start_pos: float) -> None:
        """
        力位混合模式：/arm/cmd_ctrl
        发 CmdMotorCtrl，字段 name / kp / kd / pos（目标位置）/ spd（前馈速度）/ tor（前馈力矩）。
        控制律: τ = kp*(pos-cur_pos) + kd*(spd-cur_spd) + tor
        小 kp/kd 时关节柔顺（可被外力推开），大 kp/kd 时刚硬。
        """
        target = start_pos + CTRL_DELTA
        self.get_logger().info(
            f"[力位混合] 关节 {motor_id}: kp={CTRL_KP}, kd={CTRL_KD}, "
            f"target={target:.3f} rad（当前={start_pos:.3f}）")

        msg = CmdMotorCtrl()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.cmds = [MotorCtrl(
            name=int(motor_id),
            kp=float(CTRL_KP),
            kd=float(CTRL_KD),
            pos=float(target),
            spd=0.0,
            tor=0.0,
        )]
        self.pub_ctrl.publish(msg)
        self.get_logger().info(f"[力位混合] 已发指令，保持 {CTRL_HOLD}s（可用手轻推关节感受柔顺性）")
        self._spin_hold(CTRL_HOLD)

        now = self.cur_pos.get(motor_id, start_pos)
        delta = now - start_pos
        self.get_logger().info(
            f"[力位混合] 当前角度={now:+.4f} rad，Δ={delta:+.4f} rad"
            f"（{math.degrees(delta):+.2f}°），目标 Δ=+{CTRL_DELTA:.4f} rad"
            f"（{math.degrees(CTRL_DELTA):+.2f}°）")

        self._return_to(motor_id, start_pos)


def main() -> None:
    rclpy.init()
    node = ArmForcePositionDemo()
    try:
        start = node.wait_for_status(DEMO_JOINT_ID)
        node.get_logger().info(f"关节 {DEMO_JOINT_ID} 当前角度 = {start:.3f} rad")
        node.move_ctrl(DEMO_JOINT_ID, start)
    except KeyboardInterrupt:
        node.get_logger().warn("用户 Ctrl-C 中断。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
