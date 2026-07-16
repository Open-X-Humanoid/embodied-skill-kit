#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原33 · 手臂力位混合模式（Force-Position Hybrid） —— ROS2 原生版 [生产级]
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
  3) 力位混合模式是持续生效的：一旦发出指令，关节会一直被驱动向目标位置，
     直到收到新指令。本文件用 try/finally 确保 Ctrl-C/异常中断时也会尝试回位，
     但仍建议急停在手边，不要依赖代码兜底。
  4) 左右臂 J2 关节角方向镜像：CTRL_DELTA 默认 +0.5 按左臂 12 外展设计；
     只改 DEMO_JOINT_ID=22 而不取反增量，会向体内收、有撞躯干风险。
     改测右臂时应同时设 CTRL_DELTA=-0.5（并核对目标仍在限位内）。
"""

import math
import threading
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

# kp/kd 合法范围（MotorCtrl 字段）
KP_RANGE = (0.0, 2000.0)
KD_RANGE = (0.0, 300.0)

# 单次运动允许的最大位移量（安全上限，与 atom04_robust 的 MAX_DELTA_RAD 同一用途）
MAX_DELTA_RAD = 1.0  # rad，大于默认 CTRL_DELTA=0.5，仅用于兜底改错参数

# 关节角度软限位（rad），取自 tianyi2.0 URDF 硬限位、截断到两位小数，与
# atom04_arm_ros2_robust.py 的 JOINT_SOFT_LIMITS 保持一致（同一具身，共用限位表）。
JOINT_SOFT_LIMITS: dict[int, tuple[float, float]] = {
    11: (-2.96, 2.96), 12: (-0.26, 2.61), 13: (-2.96, 2.96),
    14: (-2.61, 0.26), 15: (-2.96, 2.96), 16: (-0.78, 1.04), 17: (-1.65, 1.30),
    21: (-2.96, 2.96), 22: (-2.61, 0.26), 23: (-2.96, 2.96),
    24: (-2.61, 0.26), 25: (-2.96, 2.96), 26: (-0.78, 1.04), 27: (-1.30, 1.65),
}

# 关节错误码（MotorStatus.error）
MOTOR_ERROR_DESC: dict[int, str] = {
    1: "关节电机过温", 2: "过流", 3: "电压过低", 4: "关节mos过温",
    5: "堵转", 6: "电压过高", 7: "缺相", 8: "编码器错误",
    33072: "设备掉线", 33073: "关节位置超限",
}

PUB_READY_TIMEOUT = 3.0   # 等待底层控制节点订阅的超时 (s)
STATUS_TIMEOUT    = 5.0   # 等待状态反馈的超时 (s)


class ArmForcePositionDemo(Node):
    def __init__(self):
        super().__init__("atom_arm_force_position_demo")

        # 发布者队列深度用 10（而非简洁版的 1），降低首包在 DDS 匹配阶段丢失的概率
        self.pub_pos  = self.create_publisher(CmdSetMotorPosition, ARM_CMD_POS_TOPIC,  10)
        self.pub_ctrl = self.create_publisher(CmdMotorCtrl,        ARM_CMD_CTRL_TOPIC, 10)

        self._lock = threading.Lock()
        self.cur_pos: dict[int, float] = {}
        self.cur_err: dict[int, int] = {}
        self.status_sub_ = self.create_subscription(
            MotorStatusMsg, ARM_STATUS_TOPIC, self._on_status, 10)

        self.get_logger().info(
            f"手臂力位混合 demo 已启动，发布到 {ARM_CMD_CTRL_TOPIC}，订阅 {ARM_STATUS_TOPIC}")

    # ── 内部回调 ─────────────────────────────────────────────────────────────

    def _on_status(self, msg: MotorStatusMsg) -> None:
        # ★ Bug修复1: 同步记录电机错误码（简洁版只存了 pos）。
        #   保持期内一旦这个关节报过流/堵转等故障码，能第一时间感知到，而不是
        #   等关节"看起来不太对"才去猜原因。
        with self._lock:
            for s in msg.status:
                self.cur_pos[s.name] = s.pos
                self.cur_err[s.name] = getattr(s, "error", 0)

    # ── 辅助方法 ─────────────────────────────────────────────────────────────

    def wait_publisher_ready(self, pub, topic_name: str) -> bool:
        """
        ★ Bug修复2: 等待底层控制节点的订阅者连接后再发布。
          publish() 是非阻塞的，若对端尚未 subscribe，消息直接丢失，简洁版对此毫无提示。
        """
        t0 = time.time()
        while time.time() - t0 < PUB_READY_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.1)
            if pub.get_subscription_count() > 0:
                return True
        self.get_logger().warn(
            f"等待 {PUB_READY_TIMEOUT}s 后仍未检测到 {topic_name} 的订阅者——"
            "请确认 body_control 已启动。消息可能丢失。")
        return False

    def wait_for_status(self, motor_id: int) -> float | None:
        """
        ★ Bug修复3: 状态读取加超时，超时返回 None 而不是"假设 0.0"。
          简洁版读不到状态时会默认当前角度为 0.0，若关节实际停在大角度处，
          之后基于错误起点算出的目标位置可能是一次危险的大位移。
        """
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < STATUS_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if motor_id in self.cur_pos:
                    return self.cur_pos[motor_id]
        self.get_logger().error(
            f"超时 {STATUS_TIMEOUT}s 未收到 {ARM_STATUS_TOPIC}，"
            f"关节 {motor_id} 角度未知——为安全起见拒绝运动。")
        return None

    def _validate_gains(self, kp: float, kd: float) -> bool:
        """
        ★ Bug修复4: kp/kd 范围校验（简洁版直接裸发，改参数时手滑填错没有兜底）。
        """
        kp_lo, kp_hi = KP_RANGE
        kd_lo, kd_hi = KD_RANGE
        if not (kp_lo <= kp <= kp_hi):
            self.get_logger().error(f"kp={kp} 超出合法范围 [{kp_lo}, {kp_hi}]，拒绝。")
            return False
        if not (kd_lo <= kd <= kd_hi):
            self.get_logger().error(f"kd={kd} 超出合法范围 [{kd_lo}, {kd_hi}]，拒绝。")
            return False
        return True

    def _validate_target(self, motor_id: int, current: float, target: float) -> bool:
        """
        ★ Bug修复5: 运动前做双重安全校验（与 atom04_arm_ros2_robust.py 同一套路）。
          (a) 软限位检查：目标角度是否在允许范围内。
          (b) 单步幅度检查：与当前角度的差值是否过大（防止跳变）。
        """
        if motor_id in JOINT_SOFT_LIMITS:
            lo, hi = JOINT_SOFT_LIMITS[motor_id]
            if not (lo <= target <= hi):
                self.get_logger().error(
                    f"关节 {motor_id}: 目标 {target:.4f} rad 超出软限位 "
                    f"[{lo:.4f}, {hi:.4f}]，拒绝。")
                return False
        delta = abs(target - current)
        if delta > MAX_DELTA_RAD:
            self.get_logger().error(
                f"关节 {motor_id}: 目标 {target:.4f} 距当前 {current:.4f} "
                f"差值 {delta:.4f} rad > 上限 {MAX_DELTA_RAD} rad，拒绝。")
            return False
        return True

    def _spin_hold_watch(self, motor_id: int, duration: float, label: str) -> bool:
        """
        ★ Bug修复6: 保持期内持续 spin 并实时监视 error（简洁版的 _spin_hold 只
          负责让 cur_pos 保持更新，不检查 error）。一旦这个关节报出非 0 错误码
          （过流/堵转……），立刻打印告警并提前结束保持，返回 False；调用方据此
          提前回位，而不是继续傻等到 CTRL_HOLD 满时长。
        """
        deadline = time.time() + duration
        with self._lock:
            last_err = self.cur_err.get(motor_id, 0)
        while rclpy.ok():
            remaining = deadline - time.time()
            if remaining <= 0.0:
                return True
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))
            with self._lock:
                err = self.cur_err.get(motor_id, 0)
            if err and err != last_err:
                self.get_logger().error(
                    f"[{label}] 关节 {motor_id} 上报 error={err}"
                    f"（{MOTOR_ERROR_DESC.get(err, '未知错误码')}）——"
                    "提前结束保持并回位。")
                return False
            last_err = err
        return False   # rclpy 已停止（如 Ctrl-C），视为未正常完成

    def _return_to(self, motor_id: int, pos: float, hold: float = 2.5) -> None:
        """用位置模式将关节送回指定角度；保持期同样用 spin_once 而非 time.sleep。"""
        msg = CmdSetMotorPosition()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.cmds = [SetMotorPosition(
            name=int(motor_id), pos=float(pos),
            spd=RETURN_SPEED, cur=RETURN_CUR)]
        self.pub_pos.publish(msg)
        self.get_logger().info(f"[回位] 关节 {motor_id} -> {pos:.3f} rad，等待 {hold}s")
        deadline = time.time() + hold
        while rclpy.ok():
            remaining = deadline - time.time()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

    # ── 公开接口 ─────────────────────────────────────────────────────────────

    def move_ctrl(self, motor_id: int, start_pos: float) -> bool:
        """
        力位混合模式：/arm/cmd_ctrl
        发 CmdMotorCtrl，字段 name / kp / kd / pos（目标位置）/ spd（前馈速度）/ tor（前馈力矩）。
        返回 True 表示保持期内一切正常并已回位；返回 False 表示校验未通过（未
        发送任何指令）或保持期内检测到错误码（已提前回位）。

        ★ Bug修复7: 发指令后用 try/finally 包住"保持 + 判断"，保证无论保持期
          正常结束、提前因 error 中止，还是被 Ctrl-C/异常打断，都会执行
          finally 里的回位——力位混合模式一旦发出指令就会持续生效，指令发出
          之后半途退出程序，关节会停留在"被力控但没人管"的状态，比位置模式
          风险更高，值得专门用 finally 兜底。
        """
        if not self._validate_gains(CTRL_KP, CTRL_KD):
            return False

        target = start_pos + CTRL_DELTA
        if not self._validate_target(motor_id, start_pos, target):
            return False

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

        try:
            ok = self._spin_hold_watch(motor_id, CTRL_HOLD, "力位混合")

            with self._lock:
                now = self.cur_pos.get(motor_id, start_pos)
            delta = now - start_pos
            self.get_logger().info(
                f"[力位混合] 当前角度={now:+.4f} rad，Δ={delta:+.4f} rad"
                f"（{math.degrees(delta):+.2f}°），目标 Δ=+{CTRL_DELTA:.4f} rad"
                f"（{math.degrees(CTRL_DELTA):+.2f}°）")
            return ok
        finally:
            self._return_to(motor_id, start_pos)


# ── 主程序 ────────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = ArmForcePositionDemo()
    try:
        node.wait_publisher_ready(node.pub_ctrl, ARM_CMD_CTRL_TOPIC)
        node.wait_publisher_ready(node.pub_pos, ARM_CMD_POS_TOPIC)

        start = node.wait_for_status(DEMO_JOINT_ID)
        if start is None:
            return

        node.get_logger().info(f"关节 {DEMO_JOINT_ID} 当前角度 = {start:.3f} rad，开始演示。")

        if node.move_ctrl(DEMO_JOINT_ID, start):
            node.get_logger().info("演示完成。")
        else:
            node.get_logger().warn("演示未正常完成（校验未通过或保持期内检测到异常），已尝试回位。")

    except KeyboardInterrupt:
        node.get_logger().warn("用户 Ctrl-C 中断。")
    except Exception as exc:
        node.get_logger().error(f"未处理异常: {exc}")
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
