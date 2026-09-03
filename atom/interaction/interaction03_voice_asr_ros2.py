#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 交互3 · 语音·听（ASR）—— ROS2 原生版
只订阅 lyre 的 ASR 话题，观察"机器人听到了什么"——纯只读，不发布任何控制。
配套讲解: atom/interaction/docs/interaction03-04_voice_input_guide.md（输入侧：interaction03 听 + interaction04 对话）

接口（只读订阅，感知类，不驱动电机）:
  /audio_asr/keyword  lyre_msgs/AsrKeyword  唤醒词事件: keyword(词) · angle(声源角度)
  /audio_asr/iat      lyre_msgs/AsrIat       语音转文本: id · text
  /audio_asr/event    lyre_msgs/AsrEvent     状态事件: event(事件码) · arg1
  ⚠ 前提: 起 lyre(chat) 即可；麦克风在 Orin 本地出厂自启，无需手动起 mic 进程。
    先对麦克风喊唤醒词「天工天工」再说话，/audio_asr/* 才出识别结果。

⚠ 安全: 本示例只订阅、观察，不发任何指令、不开关对话，机器人行为不变。
  前提: lyre 已启动为 chat 模式（见《语音·输入侧》guide / 《前置·环境配置》）。喊「天工天工」唤醒后说话来观察。
"""

import rclpy
from rclpy.node import Node
from lyre_msgs.msg import AsrKeyword, AsrIat, AsrEvent

# AsrEvent 事件码（以官方 SDK 文档为准）
EVENT_NAMES = {2: "ERROR出错", 3: "STATE服务状态", 4: "WAKEUP唤醒", 5: "SLEEP休眠",
               10: "PRE_SLEEP准备休眠", 13: "已连接服务端", 14: "与服务端断开"}


class AsrListener(Node):
    def __init__(self):
        super().__init__("atom_voice_asr")
        self.kw_sub_  = self.create_subscription(AsrKeyword, "/audio_asr/keyword", self._on_keyword, 10)
        self.iat_sub_ = self.create_subscription(AsrIat,     "/audio_asr/iat",     self._on_iat,     10)
        self.ev_sub_  = self.create_subscription(AsrEvent,   "/audio_asr/event",   self._on_event,   10)
        self.get_logger().info("语音·听 已启动，订阅 /audio_asr/*；对机器人喊唤醒词 + 说话来观察。")

    def _on_keyword(self, msg):
        self.get_logger().info(f"[唤醒词] {msg.keyword!r}  声源角度={msg.angle}°")

    def _on_iat(self, msg):
        self.get_logger().info(f"[转文本] id={msg.id!r} text={msg.text!r}")

    def _on_event(self, msg):
        name = EVENT_NAMES.get(msg.event, f"未知({msg.event})")
        self.get_logger().info(f"[事件] {name} arg1={msg.arg1}")


def main():
    rclpy.init()
    node = AsrListener()
    try:
        rclpy.spin(node)     # 一直听，Ctrl-C 退出
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
