#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原32 · 电源状态（Power） —— ROS2 原生版
订阅电池电量/电压/电流及急停按键状态，以 1 Hz 在终端打印摘要。
配套讲解: atom/docs/atom32_power_guide.md

【版本说明】本文件是 ROS2 原生实现（rclpy 订阅 body_control 发布的 /power/* 话题）。
  若你的机器人走其他封装接口，话题名可能不同，届时另见对应变体。

接口（只读）:
  话题: /power/battery/status    类型: bodyctrl_msgs/PowerBatteryStatus
        字段: master_battery_power/voltage/current（大电池 电量%/电压V/电流A）
              little_battery_power/voltage/current（小电池）
              battery_installed（0x01小|0x02大|0x03两者）
              battery_working  （0x01小|0x10大，当前工作中的电池）
              master_battery_current 负值=放电，正值=充电
  话题: /power/board/key_status  类型: bodyctrl_msgs/PowerBoardKeyStatus
        字段: is_estop.data（急停是否被按下）
              is_remote_estop.data（软急停）
              is_power_on.data（电源是否正常）
              work_time（上电以来工作时间，单位 s）
  上报频率: 1 Hz（两个话题均为 1 Hz）

⚠ 安全（务必先读）:
  本原子仅订阅传感器数据，对机器人无任何运动指令，安全风险极低。
  急停已按下时（is_estop=True）请先解除急停再运行其他控制 demo。
"""

import rclpy
from rclpy.node import Node
from bodyctrl_msgs.msg import PowerBatteryStatus, PowerBoardKeyStatus

BATTERY_TOPIC  = "/power/battery/status"
KEY_STATUS_TOPIC = "/power/board/key_status"


class PowerDemo(Node):
    def __init__(self):
        super().__init__("atom_power_demo")
        self._battery: PowerBatteryStatus | None = None
        self._key:     PowerBoardKeyStatus | None = None

        self.sub_bat_ = self.create_subscription(
            PowerBatteryStatus, BATTERY_TOPIC, self._cb_battery, 10)
        self.sub_key_ = self.create_subscription(
            PowerBoardKeyStatus, KEY_STATUS_TOPIC, self._cb_key, 10)

        # 1 Hz 打印（与话题上报频率一致）
        self.timer_ = self.create_timer(1.0, self._on_print_timer)
        self.get_logger().info(
            f"电源 demo 已启动，订阅 {BATTERY_TOPIC} 和 {KEY_STATUS_TOPIC}。"
            "Ctrl-C 退出。")

    # ── 回调 ──────────────────────────────────────────────────────────────────

    def _cb_battery(self, msg: PowerBatteryStatus) -> None:
        self._battery = msg

    def _cb_key(self, msg: PowerBoardKeyStatus) -> None:
        self._key = msg

    # ── 格式化辅助 ────────────────────────────────────────────────────────────

    @staticmethod
    def _charge_str(current: float) -> str:
        """根据电流正负判断充放电状态。"""
        if current > 0.05:
            return "充电中"
        if current < -0.05:
            return "放电中"
        return "待机"

    # ── 定时打印 ──────────────────────────────────────────────────────────────

    def _on_print_timer(self) -> None:
        self._log_battery()
        self._log_key()

    def _log_battery(self) -> None:
        if self._battery is None:
            self.get_logger().warn(
                f"[电池] 尚未收到数据（确认 body_control 已启动，"
                f"ros2 topic hz {BATTERY_TOPIC}）")
            return
        b = self._battery
        self.get_logger().info(
            f"[电池] 大电池: {b.master_battery_power:.1f}%  "
            f"{b.master_battery_voltage:.2f}V  "
            f"{b.master_battery_current:+.2f}A  "
            f"({self._charge_str(b.master_battery_current)})  |  "
            f"小电池: {b.little_battery_power:.1f}%  "
            f"{b.little_battery_voltage:.2f}V  "
            f"{b.little_battery_current:+.2f}A")

    def _log_key(self) -> None:
        if self._key is None:
            self.get_logger().warn(
                f"[急停] 尚未收到数据（{KEY_STATUS_TOPIC}）")
            return
        k = self._key
        estop = "⚠ 已按下" if k.is_estop.data else "未按下"
        remote = "已触发" if k.is_remote_estop.data else "未触发"
        power  = "正常" if k.is_power_on.data else "异常"
        self.get_logger().info(
            f"[急停] 硬急停: {estop}  软急停: {remote}  供电: {power}")


# ── 主程序 ────────────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = PowerDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("用户 Ctrl-C 中断。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
