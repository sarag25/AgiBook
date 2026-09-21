#!/usr/bin/env python3
"""
ROS 2 node that records the simulation cameras to mp4 in SIMULATED time (smooth video even at RTF ~0.1).
Each frame goes at round((stamp - t0) * fps), gaps are filled with the last frame.
Outputs in out_dir: <prefix>_main, _tcp_right, _tcp_left, _head, _combo (main + picture-in-picture windows).
`ros2 run agibot_x2_pkg_py record_video`   (sim launched with video:=true, Ctrl+C closes the files)
`ros2 run agibot_x2_pkg_py record_video --ros-args -p prefix:=presa_it -p fps:=25`
"""

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Stream:
    """
    One mp4 fed by a topic, indexed in simulated time
    """

    def __init__(self, path, fps):
        """
        Writer is opened lazily on the first frame (size taken from it)
        """
        self.path, self.fps = path, fps
        self.writer = None
        self.t0 = None
        self.written = 0          # frames written (index of the next one)
        self.last = None
        self.dropped_gaps = 0

    def push(self, frame, stamp_s, fill_from=None):
        """
        Write the frame at its simulated-time slot, filling gaps with the last frame
        """
        if self.t0 is None:
            self.t0 = stamp_s
        if self.writer is None:
            h, w = frame.shape[:2]
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
            if not self.writer.isOpened():
                raise RuntimeError(f"cv2.VideoWriter cannot open {self.path}")
        target = int(round((stamp_s - self.t0) * self.fps))
        # fill gaps with the last frame (or the new one if it is the first)
        filler = self.last if self.last is not None else frame
        gap = target - self.written
        if gap > 1:
            self.dropped_gaps += 1
        while self.written < target:
            self.writer.write(filler)
            self.written += 1
        if target >= self.written:      # target == written: write the new frame
            self.writer.write(frame)
            self.written += 1
        self.last = frame

    def close(self):
        """
        Release the writer and return the video duration in s
        """
        if self.writer is not None:
            self.writer.release()
        return self.written / self.fps if self.fps else 0.0


class RecordVideo(Node):
    """
    Records main, TCP and head cameras plus a combo video with picture-in-picture windows
    """

    def __init__(self):
        """
        Declare parameters, open the streams and subscribe to the cameras
        """
        super().__init__("record_video")
        self.declare_parameter("main_topic", "/video_camera/image")
        self.declare_parameter("pip_topic", "/tcp_camera_right/image")          # right TCP
        self.declare_parameter("pip_left_topic", "/tcp_camera_left/image")     # left TCP
        self.declare_parameter("tcp_left_pip", True)   # left TCP window (the idle arm window stays on its last frame)
        self.declare_parameter("head_topic", "/head_camera/image")
        self.declare_parameter("head_pip", True)    # head camera window
        self.declare_parameter("fps", 25.0)
        self.declare_parameter("out_dir", "videos")
        self.declare_parameter("prefix", time.strftime("sim_%Y%m%d_%H%M%S"))
        self.declare_parameter("pip_width", 0.32)   # fraction of the main video width
        g = lambda n: self.get_parameter(n).value
        self.fps = float(g("fps"))
        self.pip_w = float(g("pip_width"))
        self.head_pip = bool(g("head_pip"))
        self.tcp_left_pip = bool(g("tcp_left_pip"))
        out_dir = os.path.expanduser(str(g("out_dir")))
        os.makedirs(out_dir, exist_ok=True)
        prefix = os.path.join(out_dir, str(g("prefix")))
        keys = ("main", "tcp_right", "combo") + (("head",) if self.head_pip else ()) \
            + (("tcp_left",) if self.tcp_left_pip else ())
        self.paths = {k: f"{prefix}_{k}.mp4" for k in keys}
        self.streams = {k: Stream(p, self.fps) for k, p in self.paths.items()}
        self.bridge = CvBridge()
        self.last_pip = None
        self.last_head = None
        self.last_pip_left = None
        self.n = {"main": 0, "tcp_right": 0, "head": 0, "tcp_left": 0}
        self.create_subscription(Image, str(g("main_topic")), self._main_cb, 10)
        self.create_subscription(Image, str(g("pip_topic")), self._pip_cb, 10)
        if self.head_pip:
            self.create_subscription(Image, str(g("head_topic")), self._head_cb, 10)
        if self.tcp_left_pip:
            self.create_subscription(Image, str(g("pip_left_topic")), self._pip_left_cb, 10)
        msg = f"Registro {g('main_topic')} (+ PiP {g('pip_topic')}"
        if self.tcp_left_pip:
            msg += f" + PiP {g('pip_left_topic')}"
        msg += f" + testa {g('head_topic')})" if self.head_pip else ")"
        self.get_logger().info(
            f"{msg} at {self.fps:.0f} fps of simulated time -> "
            f"{prefix}_{{{','.join(keys)}}}.mp4. Ctrl+C to close.")
        self.create_timer(5.0, self._report)

    @staticmethod
    def _stamp(msg):
        """
        Header stamp in seconds
        """
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _to_bgr(self, msg):
        """
        Convert an Image message to a BGR array
        """
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _pip_cb(self, msg):
        """
        Right TCP camera frame
        """
        frame = self._to_bgr(msg)
        self.last_pip = frame
        self.n["tcp_right"] += 1
        self.streams["tcp_right"].push(frame, self._stamp(msg))

    def _pip_left_cb(self, msg):
        """
        Left TCP camera frame
        """
        frame = self._to_bgr(msg)
        self.last_pip_left = frame
        self.n["tcp_left"] += 1
        self.streams["tcp_left"].push(frame, self._stamp(msg))

    def _head_cb(self, msg):
        """
        Head camera frame: it fires only on triggers, so its window shows the last photo taken
        """
        # rgbd camera is rgb8, the others are already bgr8
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") \
            if msg.encoding != "rgb8" else cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg), cv2.COLOR_RGB2BGR)
        self.last_head = frame
        self.n["head"] += 1
        self.streams["head"].push(frame, self._stamp(msg))

    def _main_cb(self, msg):
        """
        Main camera frame: drives the combo video, windows use the latest frame of each camera
        """
        frame = self._to_bgr(msg)
        t = self._stamp(msg)
        self.n["main"] += 1
        self.streams["main"].push(frame, t)
        pips = [(self.last_pip, "br", self.pip_w)]
        if self.tcp_left_pip:
            pips.append((self.last_pip_left, "tr", self.pip_w))
        if self.head_pip:
            pips.append((self.last_head, "bl", self.pip_w))
        self.streams["combo"].push(self.compose(frame, pips), t)

    @staticmethod
    def compose(main, pips, margin=16):
        """
        Main frame with one or more white-bordered windows
        pips: list of (frame_or_None, corner 'br'|'bl'|'tr'|'tl', width_fraction)
        """
        out = main.copy()
        H, W = out.shape[:2]
        for pip, corner, pip_w in pips:
            if pip is None:
                continue
            w = int(W * pip_w)
            h = int(round(w * pip.shape[0] / pip.shape[1]))
            small = cv2.resize(pip, (w, h), interpolation=cv2.INTER_AREA)
            x0 = margin if "l" in corner else W - w - margin
            y0 = margin if "t" in corner else H - h - margin
            cv2.rectangle(out, (x0 - 3, y0 - 3), (x0 + w + 2, y0 + h + 2), (255, 255, 255), -1)
            out[y0:y0 + h, x0:x0 + w] = small
        return out

    def _report(self):
        """
        Log received frame counts and recorded duration
        """
        m = self.streams["main"]
        dur = m.written / self.fps if m.t0 is not None else 0.0
        parts = f"regista {self.n['main']}, tcp destra {self.n['tcp_right']}"
        if self.tcp_left_pip:
            parts += f", tcp sinistra {self.n['tcp_left']}"
        if self.head_pip:
            parts += f", testa {self.n['head']} (solo sugli scatti: normale se pochi)"
        self.get_logger().info(
            f"frames received: {parts}; video: {dur:.1f} s simulated" + ("" if self.n["main"] else
            " - no frames from the main camera: launched with video:=true?"))

    def close(self):
        """
        Close all streams and delete empty files
        """
        for k, s in self.streams.items():
            dur = s.close()
            if s.written:
                self.get_logger().info(f"{self.paths[k]}: {dur:.1f} s, {s.written} frame"
                                       + (f", {s.dropped_gaps} gaps filled" if s.dropped_gaps else ""))
            elif os.path.exists(self.paths[k]):
                os.remove(self.paths[k])


def main(args=None):
    """
    Spin until Ctrl+C, then close the video files
    """
    rclpy.init(args=args)
    node = RecordVideo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
