#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
skill01_finger_tap · 相机话题命名空间解析（自动探测 + 环境变量覆盖）

为什么需要它
  相机驱动发布的话题命名空间【因机器出厂配置而异】：orbbec 驱动默认 `camera`，部分机器
  出厂配成 `ob_camera_head`。代码里写死任何一个，换台机器就一帧都收不到，而且现象是
  「静默无数据」——没有报错，只有超时，很难查。所以改成启动时扫一次 ROS 图自己认出来。

优先级（三档，逐级回落）
  1) 环境变量 CAMERA_NS 有值 → 直接用，不探测（机器上有多颗相机、或想强制指定时用）
  2) 扫 ROS 图里所有 */color/image_raw 话题 → 抠出命名空间（正常路径，不用配任何东西）
  3) 扫不到（相机没起）→ 回落 orbbec 默认 `camera` 并告警，让后续超时提示照常出现

⚠ 探测依赖 ROS 图发现，需要相机驱动【已经在跑】。刚起驱动就跑本节点可能扫不到，
  故留了 timeout_s 秒的重试窗口。
"""

import os

import rclpy

FALLBACK_NS = "camera"          # 探测不到时的兜底（orbbec 驱动默认命名空间）
_COLOR_SUFFIX = "/color/image_raw"


def resolve(node, timeout_s=5.0):
    """返回相机话题的命名空间（不带斜杠，如 "ob_camera_head"）。

    node 需已建好（rclpy.init 之后）。拼话题名的活儿留给调用方，因为彩色/深度后缀
    各节点用得不一样。
    """
    forced = os.getenv("CAMERA_NS", "").strip().strip("/")
    if forced:
        node.get_logger().info(f"相机命名空间 = {forced}（环境变量 CAMERA_NS 指定，跳过探测）")
        return forced

    deadline = node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
    found = []
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        found = sorted(
            name[: -len(_COLOR_SUFFIX)].strip("/")
            for name, types in node.get_topic_names_and_types()
            if name.endswith(_COLOR_SUFFIX) and "sensor_msgs/msg/Image" in types
        )
        if found:
            break
        rclpy.spin_once(node, timeout_sec=0.2)      # 让发现机制转起来，别空转 CPU

    if not found:
        node.get_logger().warn(
            f"{timeout_s:.0f}s 内没扫到任何 */color/image_raw 话题——相机驱动起了吗？"
            f"暂按默认 `{FALLBACK_NS}` 继续；确知命名空间可 export CAMERA_NS=<命名空间> 跳过探测。")
        return FALLBACK_NS

    if len(found) > 1:
        node.get_logger().warn(
            f"扫到多颗相机 {found}，自动选了第一个 `{found[0]}`；"
            "要用别的就 export CAMERA_NS=<命名空间> 指定。")
    node.get_logger().info(f"相机命名空间 = {found[0]}（自动探测）")
    return found[0]
