#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
原子示例 · 交互2 · 语音·放音频 —— ROS2 原生版
调用 lyre 播放控制：播本地文件 / 网络 URL，暂停 / 恢复 / 停止。
配套讲解: atom/interaction/docs/interaction01-02_voice_output_guide.md（输出侧：interaction01 朗读 + interaction02 放音频）

接口（服务调用）:
  /audio_play/play_file  PlayFile   播 Orin 本地文件   请求: sid/seq/last/force/path
  /audio_play/play_url   PlayUrl    播网络 URL         请求: sid/seq/last/force/url
  /audio_play/pause      PlayPause  暂停   (空请求)
  /audio_play/resume     PlayResume 恢复   (空请求)
  /audio_play/stop       PlayStop   停止(不可恢复) (空请求)
  play_file/url 响应: code(0成功 / 1参数错 / -1失败) · sid · message
  ⚠ play/audio/chat 模式可用（asr 是纯识别、无音频播放）。

⚠ 安全: 扬声器播音，注意音量/场合；force=True 打断当前音频；stop 后不能 resume。
  前提: lyre 已启动为 chat 模式（见《语音·输出侧》guide）。play_file 需 Orin 上有该音频文件。
"""

import uuid
import rclpy
from rclpy.node import Node
from lyre_msgs.srv import PlayFile, PlayUrl, PlayStop, PlayPause, PlayResume

FILE_PATH = "/home/nvidia/test.mp3"                 # 改成你 Orin 上的音频文件
URL       = "http://localhost:8000/test.mp3"        # 改成可访问的音频 URL


class PlayClient(Node):
    def __init__(self):
        super().__init__("atom_voice_play")
        self.file_   = self.create_client(PlayFile,   "/audio_play/play_file")
        self.url_    = self.create_client(PlayUrl,    "/audio_play/play_url")
        self.pause_  = self.create_client(PlayPause,  "/audio_play/pause")
        self.resume_ = self.create_client(PlayResume, "/audio_play/resume")
        self.stop_   = self.create_client(PlayStop,   "/audio_play/stop")

    def _call(self, cli, req, name):
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"服务 {name} 不在（5s 超时）。lyre 起了吗？")
            return
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        resp = future.result() if future.done() else None
        if resp is None:
            self.get_logger().error(f"{name} 超时/失败。")
        elif hasattr(resp, "code") and resp.code != 0:
            self.get_logger().error(f"{name} 返回错误 code={resp.code} msg={resp.message!r}")
        else:
            self.get_logger().info(f"{name} OK")

    def play_file(self, path):
        req = PlayFile.Request(); req.sid = f"interaction02_{uuid.uuid4().hex[:8]}"
        req.seq, req.last, req.force, req.path = 0, True, True, path
        self._call(self.file_, req, "PlayFile")

    def play_url(self, url):
        req = PlayUrl.Request(); req.sid = f"interaction02_{uuid.uuid4().hex[:8]}"
        req.seq, req.last, req.force, req.url = 0, True, True, url
        self._call(self.url_, req, "PlayUrl")

    def pause(self):  self._call(self.pause_,  PlayPause.Request(),  "PlayPause")
    def resume(self): self._call(self.resume_, PlayResume.Request(), "PlayResume")
    def stop(self):   self._call(self.stop_,   PlayStop.Request(),   "PlayStop")


MENU = """
── 放音频菜单 ──────────────────────────────
  1 播本地文件   2 播URL   3 暂停   4 恢复   5 停止   q 退出
────────────────────────────────────────────"""


def main():
    rclpy.init()
    node = PlayClient()
    ops = {"1": lambda: node.play_file(FILE_PATH), "2": lambda: node.play_url(URL),
           "3": node.pause, "4": node.resume, "5": node.stop}
    try:
        while True:
            print(MENU)
            c = input("选择: ").strip().lower()
            if c == "q":
                break
            op = ops.get(c)
            op() if op else print("无效输入")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
