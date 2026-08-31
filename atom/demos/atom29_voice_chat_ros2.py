#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原29 · 语音·对话（Chat）—— ROS2 原生版【风险最高，务必先读】
发布 /audio_chat/enable 开/关全双工对话通道，并订阅 ASR 观察。
⚠ 本原子只开/关通道、不含"生成回复"的大脑：完整"听→大模型→说"闭环另需 LLM agent 或 /audio_llm/ask 服务。
配套讲解: atom/docs/atom28-29_voice_input_guide.md（输入侧：atom28 听 + atom29 对话）

接口:
  发布: /audio_chat/enable  std_msgs/Bool   True=开启交互 / False=关闭
  订阅: /audio_asr/keyword · iat · event    （观察机器人听到/识别到什么）
  ⚠ 仅 chat 模式生效（出厂默认）。查模式: ps -ef | grep "ros2 launch lyre"

⚠ 风险（最高的语音接口）:
  - 开启后机器人持续聆听并回应，会影响同场所其他人；同一时刻只能一个对话会话。
  - 开启前告知现场同事；程序退出必发 False 关闭（finally 保证）。
  - 若崩溃遗留开启态，手动关:
      ros2 topic pub -1 /audio_chat/enable std_msgs/msg/Bool 'data: false'
  前提: lyre 已启动且为 chat 模式（麦克风 Orin 本地自启，无需手动起）。喊「天工天工」唤醒后说话。
    ⚠ 能识别但不回复 = 缺"回复大脑"(/audio_llm/ask 或外部 agent)，非麦克风问题；见《语音·输入侧》guide 第 3 节。
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from lyre_msgs.msg import AsrKeyword, AsrIat, AsrEvent

ENABLE_TOPIC = "/audio_chat/enable"


class ChatToggle(Node):
    def __init__(self):
        super().__init__("atom_voice_chat")
        self.pub = self.create_publisher(Bool, ENABLE_TOPIC, 1)
        self.kw_sub_  = self.create_subscription(AsrKeyword, "/audio_asr/keyword", self._on_kw,  10)
        self.iat_sub_ = self.create_subscription(AsrIat,     "/audio_asr/iat",     self._on_iat, 10)
        self.ev_sub_  = self.create_subscription(AsrEvent,   "/audio_asr/event",   self._on_ev,  10)

    def enable(self):  self.pub.publish(Bool(data=True));  self.get_logger().info(">> 语音交互已开启")
    def disable(self): self.pub.publish(Bool(data=False)); self.get_logger().info(">> 语音交互已关闭")

    def _on_kw(self, msg):  self.get_logger().info(f"[唤醒词] {msg.keyword!r} angle={msg.angle}°")
    def _on_iat(self, msg): self.get_logger().info(f"[转文本] {msg.text!r}")
    def _on_ev(self, msg):  self.get_logger().info(f"[事件] event={msg.event} arg1={msg.arg1}")


def confirm():
    print("=" * 60)
    print("即将开启语音交互 (/audio_chat/enable=True) —— 风险最高的语音接口。")
    print("确认: 当前 lyre 为 chat 模式；现场无人正在和机器人对话；已告知同事。")
    print("=" * 60)
    return input("确认请输入 GO 回车: ").strip() == "GO"


def main():
    if not confirm():
        print("已取消。")
        return
    rclpy.init()
    node = ChatToggle()
    stop = threading.Event()

    def wait_enter():
        input("\n已开启，对机器人喊唤醒词 + 说话观察日志。按 Enter 关闭并退出: ")
        stop.set()

    try:
        node.enable()
        threading.Thread(target=wait_enter, daemon=True).start()
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.disable()          # ★ 退出必关，绝不遗留"持续聆听"态
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("已退出，语音交互已关闭。")


if __name__ == "__main__":
    main()
