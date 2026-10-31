#!/usr/bin/env python3
"""Python 3.8-compatible dual RealSense RGB ZMQ publisher for G1 Orin.

Publishes the same msgpack payload shape as gear_sonic.camera.composed_camera:
  {"timestamps": {"ego_view": t, "head": t}, "images": {"ego_view": b64jpg, "head": b64jpg}}

Default mapping:
  ego_view <- neck camera 242622070951
  head     <- stock onboard 254522070308
"""
from __future__ import annotations

import argparse
import base64
import signal
import sys
import threading
import time

import cv2
import msgpack
import numpy as np
import pyrealsense2 as rs
import zmq


def encode_jpeg_b64(rgb_image, quality=80):
    # OpenCV expects BGR for imencode; RealSense color stream is RGB.
    bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return base64.b64encode(buf).decode("utf-8")


class ColorCam(object):
    def __init__(self, serial, width=640, height=480, fps=30):
        self.serial = serial
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        self.profile = self.pipe.start(cfg)
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = 0.0
        self._stop = False
        self._err = None
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop:
            try:
                frames = self.pipe.wait_for_frames(2000)
                color = frames.get_color_frame()
                if not color:
                    continue
                img = np.asanyarray(color.get_data())
                with self.lock:
                    self.frame = img
                    self.stamp = time.time()
            except Exception as e:
                self._err = str(e)
                time.sleep(0.05)

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, None
            return self.frame.copy(), self.stamp

    def close(self):
        self._stop = True
        self._thread.join(timeout=2.0)
        try:
            self.pipe.stop()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ego-sn", default="242622070951")
    ap.add_argument("--head-sn", default="254522070308")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    args = ap.parse_args()

    print("[g1-dual-rgb] devices:")
    for d in rs.context().devices:
        print(
            " ",
            d.get_info(rs.camera_info.serial_number),
            d.get_info(rs.camera_info.name),
            "usb",
            d.get_info(rs.camera_info.usb_type_descriptor),
        )

    ego = ColorCam(args.ego_sn, args.width, args.height, args.fps)
    head = ColorCam(args.head_sn, args.width, args.height, args.fps)
    print("[g1-dual-rgb] started ego=%s head=%s" % (args.ego_sn, args.head_sn))

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 20)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind("tcp://*:%d" % args.port)
    print("[g1-dual-rgb] PUB tcp://*:%d" % args.port)

    stop = {"v": False}

    def _sig(*_a):
        stop["v"] = True

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    interval = 1.0 / float(args.fps)
    t0 = time.monotonic()
    idx = 0
    sent = 0
    dropped = 0
    fps_t = time.monotonic()

    while not stop["v"]:
        target = t0 + (idx + 1) * interval
        ego_img, ego_t = ego.latest()
        head_img, head_t = head.latest()
        if ego_img is not None and head_img is not None:
            payload = {
                "timestamps": {"ego_view": ego_t, "head": head_t},
                "images": {
                    "ego_view": encode_jpeg_b64(ego_img, args.jpeg_quality),
                    "head": encode_jpeg_b64(head_img, args.jpeg_quality),
                },
            }
            try:
                sock.send(msgpack.packb(payload, use_bin_type=True), flags=zmq.NOBLOCK)
                sent += 1
            except zmq.Again:
                dropped += 1
            idx += 1
            if sent % 30 == 0:
                now = time.monotonic()
                print(
                    "[g1-dual-rgb] sent=%d dropped=%d fps=%.1f ego_err=%s head_err=%s"
                    % (
                        sent,
                        dropped,
                        30.0 / max(1e-6, now - fps_t),
                        ego._err,
                        head._err,
                    )
                )
                fps_t = now

        sleep_t = target - time.monotonic()
        if sleep_t > 0:
            time.sleep(sleep_t)
        else:
            idx += 1  # keep cadence if late

    ego.close()
    head.close()
    sock.close()
    ctx.term()
    print("[g1-dual-rgb] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
