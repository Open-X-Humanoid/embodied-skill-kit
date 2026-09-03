#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 感知2 · 六维力传感器（6-DoF F/T Sensor） —— ROS2 原生版
订阅左右腕部六维力话题，实时打印三轴力（Fx/Fy/Fz，N）和三轴力矩（Tx/Ty/Tz，Nm）。
配套讲解: atom/perception/docs/perception02_arm_6dof_guide.md

【版本说明】本文件是 ROS2 原生实现（rclpy 订阅 body_control 发布的 /arm_6dof_* 话题）。
  若你的机器人走其他封装接口，话题名可能不同，届时另见对应变体。

接口（只读）:
  话题: /arm_6dof_left    类型: geometry_msgs/WrenchStamped
        /arm_6dof_right   类型: geometry_msgs/WrenchStamped
  字段: wrench.force  (x/y/z，单位 N，腕部三轴受力)
        wrench.torque (x/y/z，单位 Nm，腕部三轴力矩)
  上报频率: 1000 Hz（本示例降采样至 PRINT_HZ 打印，避免刷屏）

⚠ 安全（务必先读）:
  本原子仅订阅传感器数据，对机器人无任何运动指令，安全风险极低。
  确认 body_control 已启动后再运行；若话题无数据，先查看排错章节。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

LEFT_TOPIC  = "/arm_6dof_left"
RIGHT_TOPIC = "/arm_6dof_right"

PRINT_HZ = 2.0   # 终端打印频率（原始 1000 Hz，降采样后方便观察）


class FTSensorDemo(Node):
    def __init__(self):
        super().__init__("atom_ft_sensor_demo")
        self._latest: dict[str, WrenchStamped | None] = {
            "left":  None,
            "right": None,
        }
        self.sub_left_  = self.create_subscription(
            WrenchStamped, LEFT_TOPIC,  self._cb_left,  10)
        self.sub_right_ = self.create_subscription(
            WrenchStamped, RIGHT_TOPIC, self._cb_right, 10)
        self.timer_ = self.create_timer(1.0 / PRINT_HZ, self._on_print_timer)
        self.get_logger().info(
            f"六维力 demo 已启动，订阅 {LEFT_TOPIC} 和 {RIGHT_TOPIC}"
            f"，以 {PRINT_HZ} Hz 打印（原始 1000 Hz）。Ctrl-C 退出。")

    def _cb_left(self, msg: WrenchStamped) -> None:
        self._latest["left"] = msg

    def _cb_right(self, msg: WrenchStamped) -> None:
        self._latest["right"] = msg

    def _log_wrench(self, side: str, msg: WrenchStamped | None) -> None:
        if msg is None:
            self.get_logger().warn(f"[{side}] 尚未收到数据（确认 body_control 已启动）")
            return
        f = msg.wrench.force
        t = msg.wrench.torque
        self.get_logger().info(
            f"[{side}] "
            f"F=({f.x:+7.3f}, {f.y:+7.3f}, {f.z:+7.3f}) N  "
            f"T=({t.x:+7.3f}, {t.y:+7.3f}, {t.z:+7.3f}) Nm")

    def _on_print_timer(self) -> None:
        self._log_wrench("左腕", self._latest["left"])
        self._log_wrench("右腕", self._latest["right"])


def main() -> None:
    rclpy.init()
    node = FTSensorDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("用户 Ctrl-C 中断。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
