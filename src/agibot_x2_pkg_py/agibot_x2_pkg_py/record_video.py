#!/usr/bin/env python3
"""
record_video - registra il video della simulazione IN TEMPO SIMULATO
(2026-09-13, README "VIDEO PER LA PRESENTAZIONE").

Problema: senza GPU la simulazione va a RTF ~0.1 e ogni feed a schermo va a
1-2 fps reali: registrare lo schermo produce un video a scatti. Le camere
di Gazebo pero' pubblicano frame con timestamp in tempo SIMULATO a cadenza
costante (20 Hz la camera regista, 15 Hz la TCP). Questo nodo ricostruisce
la linea temporale simulata: ogni frame va nel mp4 alla posizione
round((stamp - t0) * fps), i buchi vengono riempiti ripetendo l'ultimo
frame. Risultato: un video fluido a `fps` fotogrammi al secondo di tempo
simulato, a prescindere da quanto e' lento il PC.

Output (in out_dir, default ./videos):
  <prefix>_main.mp4   camera regista (main_topic)
  <prefix>_pip.mp4    camera TCP da sola (pip_topic)
  <prefix>_combo.mp4  regista + TCP in picture-in-picture (angolo in basso a destra)
Il video combo e' guidato dai frame della regista; per la finestrella si
usa l'ultimo frame TCP arrivato (le due camere hanno lo stesso clock
simulato, quindi restano sincronizzate).

Uso (con la sim lanciata con video:=true):
  ros2 run agibot_x2_pkg_py record_video               # Ctrl+C per chiudere i file
  ros2 run agibot_x2_pkg_py record_video --ros-args -p prefix:=presa_it -p fps:=25
Non serve ffmpeg: usa cv2.VideoWriter (mp4v).
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
    """Un mp4 alimentato da un topic, indicizzato in tempo simulato."""

    def __init__(self, path, fps):
        self.path, self.fps = path, fps
        self.writer = None
        self.t0 = None
        self.written = 0          # frame gia' scritti (indice del prossimo)
        self.last = None
        self.dropped_gaps = 0

    def push(self, frame, stamp_s, fill_from=None):
        if self.t0 is None:
            self.t0 = stamp_s
        if self.writer is None:
            h, w = frame.shape[:2]
            self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
            if not self.writer.isOpened():
                raise RuntimeError(f"cv2.VideoWriter non apre {self.path}")
        target = int(round((stamp_s - self.t0) * self.fps))
        # riempi i buchi con l'ultimo frame (o con il nuovo se e' il primo)
        filler = self.last if self.last is not None else frame
        gap = target - self.written
        if gap > 1:
            self.dropped_gaps += 1
        while self.written < target:
            self.writer.write(filler)
            self.written += 1
        if target >= self.written:      # target == written: scrivi il frame nuovo
            self.writer.write(frame)
            self.written += 1
        self.last = frame

    def close(self):
        if self.writer is not None:
            self.writer.release()
        return self.written / self.fps if self.fps else 0.0


class RecordVideo(Node):

    def __init__(self):
        super().__init__("record_video")
        self.declare_parameter("main_topic", "/video_camera/image")
        self.declare_parameter("pip_topic", "/tcp_camera_right/image")
        self.declare_parameter("fps", 25.0)
        self.declare_parameter("out_dir", "videos")
        self.declare_parameter("prefix", time.strftime("sim_%Y%m%d_%H%M%S"))
        self.declare_parameter("pip_width", 0.32)   # frazione della larghezza del video main
        g = lambda n: self.get_parameter(n).value
        self.fps = float(g("fps"))
        self.pip_w = float(g("pip_width"))
        out_dir = os.path.expanduser(str(g("out_dir")))
        os.makedirs(out_dir, exist_ok=True)
        prefix = os.path.join(out_dir, str(g("prefix")))
        self.paths = {k: f"{prefix}_{k}.mp4" for k in ("main", "pip", "combo")}
        self.streams = {k: Stream(p, self.fps) for k, p in self.paths.items()}
        self.bridge = CvBridge()
        self.last_pip = None
        self.n = {"main": 0, "pip": 0}
        self.create_subscription(Image, str(g("main_topic")), self._main_cb, 10)
        self.create_subscription(Image, str(g("pip_topic")), self._pip_cb, 10)
        self.get_logger().info(
            f"Registro {g('main_topic')} (+ PiP {g('pip_topic')}) a {self.fps:.0f} fps di tempo "
            f"simulato -> {prefix}_{{main,pip,combo}}.mp4. Ctrl+C per chiudere.")
        self.create_timer(5.0, self._report)

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _to_bgr(self, msg):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _pip_cb(self, msg):
        frame = self._to_bgr(msg)
        self.last_pip = frame
        self.n["pip"] += 1
        self.streams["pip"].push(frame, self._stamp(msg))

    def _main_cb(self, msg):
        frame = self._to_bgr(msg)
        t = self._stamp(msg)
        self.n["main"] += 1
        self.streams["main"].push(frame, t)
        self.streams["combo"].push(self.compose(frame, self.last_pip, self.pip_w), t)

    @staticmethod
    def compose(main, pip, pip_w=0.32, margin=16):
        """main con pip (se presente) in basso a destra, bordo bianco."""
        out = main.copy()
        if pip is None:
            return out
        H, W = out.shape[:2]
        w = int(W * pip_w)
        h = int(round(w * pip.shape[0] / pip.shape[1]))
        small = cv2.resize(pip, (w, h), interpolation=cv2.INTER_AREA)
        x0, y0 = W - w - margin, H - h - margin
        cv2.rectangle(out, (x0 - 3, y0 - 3), (x0 + w + 2, y0 + h + 2), (255, 255, 255), -1)
        out[y0:y0 + h, x0:x0 + w] = small
        return out

    def _report(self):
        m = self.streams["main"]
        dur = m.written / self.fps if m.t0 is not None else 0.0
        self.get_logger().info(
            f"frame ricevuti: regista {self.n['main']}, tcp {self.n['pip']}; "
            f"video: {dur:.1f} s simulati" + ("" if self.n["main"] else
            " - nessun frame dalla regista: launch con video:=true?"))

    def close(self):
        for k, s in self.streams.items():
            dur = s.close()
            if s.written:
                self.get_logger().info(f"{self.paths[k]}: {dur:.1f} s, {s.written} frame"
                                       + (f", {s.dropped_gaps} buchi riempiti" if s.dropped_gaps else ""))
            elif os.path.exists(self.paths[k]):
                os.remove(self.paths[k])


def main(args=None):
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
