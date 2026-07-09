#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 原26 · 语音·朗读（TTS）—— ROS2 原生版
调用 lyre 语音包的 PlayText 服务，让机器人把一段文字读出来。
配套讲解: atom/docs/atom26-27_voice_output_guide.md（输出侧：atom26 朗读 + atom27 放音频）

接口（服务调用，不驱动电机）:
  服务: /audio_play/play_text   类型: lyre_msgs/srv/PlayText
    请求: sid(播放流ID,唯一) · seq(分包序号,单次填0) · last(是否末包,单次True)
          · force(True=打断当前播放 / False=排队) · text(要读的文字)
          · token/output(系统内部字段,应用层留空)
    响应: code(0成功 / 1参数非法 / -1失败) · sid · message
  ⚠ 仅 audio / chat 模式可用（出厂 chat，直接可用）；play 模式不含 TTS。

⚠ 安全: 声音从扬声器播出，注意音量/场合；force=True 会打断当前正在播放的音频。
  前提: lyre 已启动为 chat 模式，在 Orin 上跑（见《语音·输出侧》guide / 《前置·环境配置》）。
"""

import uuid
import rclpy
from rclpy.node import Node
from lyre_msgs.srv import PlayText

PLAY_TEXT = "/audio_play/play_text"
TEXT = "你好，我是天轶机器人。"     # 改这里换朗读内容


class TtsClient(Node):
    def __init__(self):
        super().__init__("atom_voice_tts")
        self.cli = self.create_client(PlayText, PLAY_TEXT)

    def say(self, text, force=True):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"服务 {PLAY_TEXT} 不在（5s 超时）。确认 lyre 已起、且为 audio/chat 模式："
                "ros2 service list | grep audio_play")
            return None
        req = PlayText.Request()
        req.sid = f"atom26_{uuid.uuid4().hex[:8]}"   # 唯一播放流 ID
        req.seq, req.last, req.force = 0, True, force
        req.text = text
        req.token = req.output = ""                  # 系统内部字段，留空
        self.get_logger().info(f"朗读: {text!r} (force={force})")
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result() if future.done() else None


def main():
    rclpy.init()
    node = TtsClient()
    try:
        resp = node.say(TEXT, force=True)
        if resp is None:
            node.get_logger().error("调用失败/超时，见上方日志。")
        elif resp.code == 0:
            node.get_logger().info(f"已接受播放。sid={resp.sid!r} msg={resp.message!r}")
        else:
            node.get_logger().error(
                f"服务返回错误 code={resp.code} msg={resp.message!r}"
                "（1=参数非法，-1=内部失败/TTS 不可用）")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
