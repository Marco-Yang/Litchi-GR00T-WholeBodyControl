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
import json
import os
import signal
import sys
import threading
import time

import cv2
import msgpack
import numpy as np
import pyrealsense2 as rs
import zmq

DEFAULT_WRIST_STATUS_FILE = "/tmp/g1_wrist_status.json"
FRAME_TIMEOUT_MS = 2000


def read_wrist_status(path, max_age_s=3.0):
    """Return (left_err, right_err, fps_str) or None if wrist status unavailable."""
    if not path or not os.path.exists(path):
        return None
    try:
        age = time.time() - os.path.getmtime(path)
        with open(path, "r") as f:
            data = json.load(f)
        if age > max_age_s and not data.get("alive", False):
            return ("stale", "stale", "-")
        if age > max_age_s:
            return ("stale(%.1fs)" % age, "stale(%.1fs)" % age, "-")
        fps = data.get("fps")
        fps_s = "-" if fps is None else ("%.1f" % float(fps))
        return (data.get("left_err"), data.get("right_err"), fps_s)
    except Exception:
        return None


def encode_jpeg_b64(rgb_image, quality=80):
    # OpenCV expects BGR for imencode; RealSense color stream is RGB.
    bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return base64.b64encode(buf).decode("utf-8")


class ColorCam(object):
    def __init__(self, name, serial, width, height, fps, stop):
        self.name = name
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = None
        self._stop = stop
        self.error = "starting"
        self._thread = threading.Thread(target=self._loop, name=name)
        self._thread.start()

    def _read_frames(self, pipe):
        while not self._stop.is_set():
            frames = pipe.wait_for_frames(FRAME_TIMEOUT_MS)
            color = frames.get_color_frame()
            if not color:
                raise RuntimeError("frameset has no color frame")
            # Keep SDK-owned buffers out of the publisher and the next pipeline.
            img = np.asanyarray(color.get_data()).copy()
            with self.lock:
                first_frame = self.frame is None
                self.frame = img
                self.stamp = time.time()
                self.error = None
            if first_frame:
                print("[g1-cam] %s SN=%s streaming" % (self.name, self.serial), flush=True)

    def _loop(self):
        retry_delay = 0.0
        while not self._stop.is_set():
            pipe = None
            started = False
            try:
                # start() can block the Orin's Python runtime for 15s when the
                # selected serial is absent. Wait for USB enumeration ourselves.
                ctx = rs.context()
                if not any(d.get_info(rs.camera_info.serial_number) == self.serial
                           for d in ctx.devices):
                    raise RuntimeError("device not present on USB")
                pipe = rs.pipeline(ctx)
                cfg = rs.config()
                cfg.enable_device(self.serial)
                cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
                pipe.start(cfg)
                started = True
                self._read_frames(pipe)
            except Exception as exc:
                with self.lock:
                    if self.frame is not None:
                        retry_delay = 0.0
                    self.frame = self.stamp = None
                    self.error = str(exc)
                if not self._stop.is_set():
                    print("[g1-cam] %s SN=%s: %s; reopening in %.1fs"
                          % (self.name, self.serial, exc, retry_delay), flush=True)
            finally:
                if started:
                    try:
                        pipe.stop()
                    except RuntimeError as exc:
                        print("[g1-cam] %s stop: %s" % (self.name, exc), flush=True)
                del pipe
            if self._stop.wait(retry_delay):
                break
            retry_delay = min(5.0, max(1.0, 2 * retry_delay))

    def latest(self):
        with self.lock:
            return self.frame, self.stamp

    def join(self):
        self._thread.join()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ego-sn", default="242622070951")
    ap.add_argument("--head-sn", default="254522070308")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument(
        "--wrist-status-file",
        default=os.environ.get("G1_WRIST_STATUS_FILE", DEFAULT_WRIST_STATUS_FILE),
        help="Optional JSON from g1_wrist_abham; when present, status line includes wrist errs",
    )
    ap.add_argument(
        "--show-wrist-status",
        action="store_true",
        default=os.environ.get("G1_SHOW_WRIST_STATUS", "0") in ("1", "true", "TRUE", "yes"),
        help="Always print wrist fields (offline if status file missing)",
    )
    args = ap.parse_args()

    if min(args.fps, args.width, args.height) <= 0:
        ap.error("fps, width and height must be positive")
    if args.ego_sn == args.head_sn:
        ap.error("ego and head must use different serial numbers")

    stop = threading.Event()

    def _sig(*_a):
        stop.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    cameras = {}
    with zmq.Context() as ctx, ctx.socket(zmq.PUB) as sock:
        sock.setsockopt(zmq.SNDHWM, 20)
        sock.setsockopt(zmq.LINGER, 0)
        sock.bind("tcp://*:%d" % args.port)
        print("[g1-dual-rgb] PUB tcp://*:%d" % args.port, flush=True)
        try:
            for name, serial in (("ego_view", args.ego_sn), ("head", args.head_sn)):
                cameras[name] = ColorCam(name, serial, args.width, args.height, args.fps, stop)
            publish(sock, cameras, args, stop)
        finally:
            stop.set()
            for cam in cameras.values():
                cam.join()
    print("[g1-dual-rgb] stopped")
    return 0


def publish(sock, cameras, args, stop):
    interval = 1.0 / float(args.fps)
    sent = 0
    dropped = 0
    previous_sent = 0
    last_status = time.monotonic()
    target = last_status

    while not stop.is_set():
        payload = {"timestamps": {}, "images": {}}
        for name, cam in cameras.items():
            img, stamp = cam.latest()
            if img is not None:
                payload["images"][name] = encode_jpeg_b64(img, args.jpeg_quality)
                payload["timestamps"][name] = stamp
        if payload["images"]:
            try:
                sock.send(msgpack.packb(payload, use_bin_type=True), flags=zmq.NOBLOCK)
                sent += 1
            except zmq.Again:
                dropped += 1

        now = time.monotonic()
        if now - last_status >= 1.0:
            fps = (sent - previous_sent) / (now - last_status)
            line = ("[g1-cam] sent=%d dropped=%d fps=%.1f ego_err=%s head_err=%s"
                    % (sent, dropped, fps, cameras["ego_view"].error, cameras["head"].error))
            wrist = read_wrist_status(args.wrist_status_file)
            if wrist is not None:
                wl, wr, wfps = wrist
                line += " | wrist_fps=%s left_err=%s right_err=%s" % (wfps, wl, wr)
            elif args.show_wrist_status:
                line += " | wrist_fps=- left_err=offline right_err=offline"
            print(line, flush=True)
            previous_sent, last_status = sent, now

        target = max(target + interval, now)
        stop.wait(max(0.0, target - time.monotonic()))


if __name__ == "__main__":
    sys.exit(main())
