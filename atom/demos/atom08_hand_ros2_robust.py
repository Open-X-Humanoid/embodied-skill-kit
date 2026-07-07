#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原8 · 灵巧手（Hand，因时 Inspire 五指） —— ROS2 原生版 [生产级]
让灵巧手张开、握拳，并读取手指当前状态。   配套讲解: atom/docs/atom08_hand_guide.md

接口（topic 控制）:
  话题: /inspire_hand/ctrl/left_hand   （右手: /inspire_hand/ctrl/right_hand）
  类型: sensor_msgs/JointState
        name[]     手指ID字符串: "1"小指 "2"无名指 "3"中指 "4"食指 "5"拇指弯 "6"拇指旋
        position[] 张合百分比 (0.0=握紧/合, 1.0=完全张开)   ← 注意是百分比，不是弧度
  状态反馈: /inspire_hand/state/left_hand   sensor_msgs/JointState

⚠ 安全: 手指别夹到东西/人；建议握到 0.1 而不是 0.0，给机械留点余量。
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

HAND_CMD_TOPIC   = "/inspire_hand/ctrl/left_hand"     # 左手；右手改成 right_hand
HAND_STATE_TOPIC = "/inspire_hand/state/left_hand"
FINGER_NAMES     = ["1", "2", "3", "4", "5", "6"]    # 小指 无名指 中指 食指 拇指弯 拇指旋
NUM_FINGERS      = len(FINGER_NAMES)

PUB_READY_TIMEOUT = 3.0   # s
STATUS_TIMEOUT    = 5.0   # s


class HandDemo(Node):
    def __init__(self):
        super().__init__("atom_hand_demo")
        self.pub = self.create_publisher(JointState, HAND_CMD_TOPIC, 10)

        self._lock = threading.Lock()
        self.cur_pos: list[float] | None = None
        self._warned_state_len = False   # 状态长度不符只告警一次，避免刷屏

        # ★ 保存订阅引用（self.state_sub_）：与仓库其余示例风格统一，便于以后单独管理该订阅
        #   （rclpy 的 Node 自身已在 self._subscriptions 持有强引用，不赋值也不会被 GC 回收）。
        self.state_sub_ = self.create_subscription(
            JointState, HAND_STATE_TOPIC, self._on_state, 10)

        self.get_logger().info(
            f"灵巧手原子 demo 已启动，发布到 {HAND_CMD_TOPIC}，订阅 {HAND_STATE_TOPIC}")

    def _on_state(self, msg: JointState) -> None:
        with self._lock:
            if len(msg.position) == NUM_FINGERS:
                self.cur_pos = list(msg.position)
            elif not self._warned_state_len:
                # ★ 长度不符时告警（而非静默丢弃）：否则 wait_for_status 会超时，
                #   让人误以为"没收到状态"，其实是消息格式和预期的 6 指不同。
                self._warned_state_len = True
                self.get_logger().warn(
                    f"{HAND_STATE_TOPIC} 的 position 长度为 {len(msg.position)}，"
                    f"预期 {NUM_FINGERS}——请核对该手状态消息格式。")

    # ── 辅助 ────────────────────────────────────────────────────────────────

    def wait_publisher_ready(self) -> bool:
        """
        ★ Bug修复2: 等待底层灵巧手节点订阅连接再发送首条指令。
        """
        t0 = time.time()
        while time.time() - t0 < PUB_READY_TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pub.get_subscription_count() > 0:
                return True
        self.get_logger().warn(
            f"等待 {PUB_READY_TIMEOUT}s 后未检测到灵巧手订阅者，消息可能丢失。")
        return False

    def wait_for_status(self, timeout: float = STATUS_TIMEOUT) -> bool:
        """等到收到第一帧手部状态反馈。"""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if self.cur_pos is not None:
                    return True
        self.get_logger().error(
            f"超时 {timeout}s 未收到灵巧手状态反馈，请确认灵巧手节点已启动。")
        return False

    @staticmethod
    def _clamp_ratios(ratios: list[float]) -> list[float]:
        """将每个手指值夹在 [0.0, 1.0]，防止超范围损坏手指。"""
        return [max(0.0, min(1.0, float(r))) for r in ratios]

    def _validate_ratios(self, ratios: list[float]) -> bool:
        """
        ★ Bug修复3: 替换 assert 为显式校验。
          assert 在 python -O（优化模式）下会被完全跳过，参数错误无声通过。
          另外，原版没有检查值域 [0,1]——超出范围的值会损坏灵巧手。
        """
        if len(ratios) != NUM_FINGERS:
            self.get_logger().error(
                f"ratios 长度应为 {NUM_FINGERS}，实际为 {len(ratios)}，拒绝。")
            return False
        out_of_range = [
            (i + 1, r) for i, r in enumerate(ratios) if not (0.0 <= r <= 1.0)
        ]
        if out_of_range:
            self.get_logger().warn(
                f"以下手指值超出 [0,1]，将被夹限: {out_of_range}")
        return True

    # ── 公开接口 ─────────────────────────────────────────────────────────────

    def set_open_ratio(self, ratios: list[float], hold: float = 2.0) -> bool:
        """
        设置 6 个手指的张合百分比 (0~1)，保持 hold 秒。

        ★ Bug修复4: hold 期间用 spin_once 代替 time.sleep，
          状态反馈回调在等待期间保持活跃。
        """
        if not self._validate_ratios(ratios):
            return False

        clamped = self._clamp_ratios(ratios)

        msg = JointState()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.name = FINGER_NAMES
        msg.position = clamped
        self.pub.publish(msg)
        self.get_logger().info(
            f"手指张合 → {[f'{v:.2f}' for v in clamped]}")

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
    node = HandDemo()
    try:
        node.wait_publisher_ready()

        if not node.wait_for_status():
            return

        with node._lock:
            node.get_logger().info(
                f"当前手指状态: {[f'{v:.2f}' for v in (node.cur_pos or [])]}")

        node.set_open_ratio([1.0] * 6)                       # 完全张开
        node.set_open_ratio([0.1] * 6)                       # 握拳（留 0.1 余量）
        node.set_open_ratio([1.0] * 6)                       # 再张开
        node.set_open_ratio([1.0, 1.0, 1.0, 0.1, 1.0, 1.0]) # 只弯食指(id4)
        node.set_open_ratio([1.0] * 6)                       # 收尾张开

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
