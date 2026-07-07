#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原4 · 手臂（Arm） —— ROS2 原生版 [生产级]
在关节空间里，慢速、小幅地移动一条 7 自由度手臂的单个关节。
配套讲解: atom/docs/atom04_arm_guide.md

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
  3) 程序会先订阅 /arm/status 读到当前角度，从当前角度小步增量，不会盲发大角度。
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition, MotorStatusMsg

ARM_CMD_TOPIC    = "/arm/cmd_pos"
ARM_STATUS_TOPIC = "/arm/status"

DEMO_JOINT_ID = 12   # 左臂第 2 个关节（右臂同理用 22）
SPEED         = 0.2  # rad/s，越小越安全
MAX_CUR       = 1.0  # A

# 单次 move_joint 允许的最大位移量（安全上限，可根据实际硬件调整）
MAX_DELTA_RAD = 0.5  # rad

# 关节角度软限位（rad），取自 tianyi2.0 URDF 硬限位、截断到两位小数（略严于硬限位）。
# 限位以 URDF 为准；需要更大安全余量时可再向内收（如各留 0.05~0.1 rad）。
# 左臂 11~17 / 右臂 21~27，顺序: 肩俯仰/肩侧展/上臂旋转/肘弯/前臂旋转/腕弯/腕旋。
# 格式: {motor_id: (lower, upper)}
JOINT_SOFT_LIMITS: dict[int, tuple[float, float]] = {
    11: (-2.96, 2.96), 12: (-0.26, 2.61), 13: (-2.96, 2.96),
    14: (-2.61, 0.26), 15: (-2.96, 2.96), 16: (-0.78, 1.04), 17: (-1.65, 1.30),
    21: (-2.96, 2.96), 22: (-2.61, 0.26), 23: (-2.96, 2.96),
    24: (-2.61, 0.26), 25: (-2.96, 2.96), 26: (-0.78, 1.04), 27: (-1.30, 1.65),
}

# 等待底层控制节点订阅的超时（s）
PUB_READY_TIMEOUT = 3.0
# 等待状态反馈的超时（s）
STATUS_TIMEOUT = 5.0


class ArmDemo(Node):
    def __init__(self):
        super().__init__("atom_arm_demo")

        # 发布者（队列深度 10，防止首包在 DDS 匹配阶段丢失）
        self.pub = self.create_publisher(CmdSetMotorPosition, ARM_CMD_TOPIC, 10)

        # ★ 保存订阅引用（self.status_sub_）：与仓库其余示例风格统一，
        #   也便于以后单独管理/销毁该订阅。
        #   注：rclpy 的 Node.create_subscription 内部会把返回对象存入 self._subscriptions
        #   （强引用）；只有 CallbackGroup 那层是弱引用。只要 Node 存活，不赋值给 self 也不会被 GC。
        self._lock = threading.Lock()
        self.cur_pos: dict[int, float] = {}
        self.status_sub_ = self.create_subscription(
            MotorStatusMsg, ARM_STATUS_TOPIC, self._on_status, 10)

        self.get_logger().info(
            f"手臂原子 demo 已启动，发布到 {ARM_CMD_TOPIC}，订阅 {ARM_STATUS_TOPIC}")

    # ── 内部回调 ─────────────────────────────────────────────────────────────

    def _on_status(self, msg: MotorStatusMsg) -> None:
        with self._lock:
            for s in msg.status:
                self.cur_pos[s.name] = s.pos

    # ── 辅助方法 ─────────────────────────────────────────────────────────────

    def wait_publisher_ready(self) -> bool:
        """
        ★ Bug修复2: 等待底层控制节点的订阅者连接后再发布。
          publish() 是非阻塞的，若对端尚未 subscribe，消息直接丢失。
          此处轮询直到至少一个订阅者出现。
        """
        t0 = time.time()
        while time.time() - t0 < PUB_READY_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pub.get_subscription_count() > 0:
                return True
        self.get_logger().warn(
            f"等待 {PUB_READY_TIMEOUT}s 后仍未检测到订阅者——"
            "请确认底层控制节点已启动。消息可能丢失。")
        return False

    def wait_for_status(self, motor_id: int) -> float | None:
        """
        等待读到指定关节的当前角度并返回。
        超时返回 None（调用方应拒绝后续运动）。
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

    def _validate_target(self, motor_id: int, current: float, target: float) -> bool:
        """
        ★ Bug修复3: 运动前做双重安全校验。
          (a) 软限位检查：目标角度是否在允许范围内。
          (b) 单步幅度检查：与当前角度的差值是否过大（防止跳变）。
        """
        # (a) 软限位
        if motor_id in JOINT_SOFT_LIMITS:
            lo, hi = JOINT_SOFT_LIMITS[motor_id]
            if not (lo <= target <= hi):
                self.get_logger().error(
                    f"关节 {motor_id}: 目标 {target:.4f} rad 超出软限位 "
                    f"[{lo:.4f}, {hi:.4f}]，拒绝。")
                return False
        # (b) 单步幅度
        delta = abs(target - current)
        if delta > MAX_DELTA_RAD:
            self.get_logger().error(
                f"关节 {motor_id}: 目标 {target:.4f} 距当前 {current:.4f} "
                f"差值 {delta:.4f} rad > 上限 {MAX_DELTA_RAD} rad，拒绝。")
            return False
        return True

    # ── 公开接口 ─────────────────────────────────────────────────────────────

    def move_joint(self, motor_id: int, target_pos: float,
                   hold: float = 2.0) -> bool:
        """
        向指定关节发送位置指令，然后保持 hold 秒。
        返回 True 表示指令已发出，False 表示安全校验未通过（运动被拒绝）。

        ★ Bug修复4: hold 期间用 spin_once 轮询代替 time.sleep。
          原来 time.sleep 会阻塞整个线程，导致 _on_status 回调无法触发，
          cur_pos 在 hold 期间不再更新。
        """
        with self._lock:
            current = self.cur_pos.get(motor_id)

        if current is None:
            self.get_logger().error(
                f"关节 {motor_id} 当前角度未知（尚未收到状态反馈），拒绝运动。")
            return False

        if not self._validate_target(motor_id, current, target_pos):
            return False

        msg = CmdSetMotorPosition()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.cmds = [SetMotorPosition(
            name=int(motor_id),
            pos=float(target_pos),
            spd=float(SPEED),
            cur=float(MAX_CUR),
        )]
        self.pub.publish(msg)
        self.get_logger().info(
            f"关节 {motor_id}: {current:+.4f} → {target_pos:+.4f} rad "
            f"(Δ={target_pos - current:+.4f}, hold={hold}s)")

        # 保持期间继续 spin，让 _on_status 保持更新
        deadline = time.time() + hold
        while rclpy.ok():
            # remaining 可能因竞态变负（条件判断与此行之间时间已越过 deadline）；
            # rclpy 对负 timeout_sec 会永久阻塞，必须先判空再 spin。
            remaining = deadline - time.time()
            if remaining <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

        return True


# ── 主程序 ────────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = ArmDemo()
    try:
        # 等待底层控制节点订阅后再发布（防止首包丢失）
        node.wait_publisher_ready()

        # 读取关节当前角度——失败则安全退出
        start = node.wait_for_status(DEMO_JOINT_ID)
        if start is None:
            return

        node.get_logger().info(
            f"关节 {DEMO_JOINT_ID} 当前角度 = {start:+.4f} rad，开始演示。")

        # 从当前角度做 ±0.1 rad 小幅来回，再回到起点
        if not node.move_joint(DEMO_JOINT_ID, start + 0.1):
            return
        if not node.move_joint(DEMO_JOINT_ID, start - 0.1):
            return
        node.move_joint(DEMO_JOINT_ID, start)

        node.get_logger().info("演示完成。")

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
