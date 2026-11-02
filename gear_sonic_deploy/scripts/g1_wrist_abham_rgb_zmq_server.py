#!/usr/bin/env python3
"""Python 3.8-compatible dual Abham wrist USB-camera ZMQ publisher for G1 Orin.

Abham Image (1c45:6200) is a standard UVC webcam — kernel ``uvcvideo`` + OpenCV.
No vendor SDK. Prefer stable V4L by-id paths (videoN indices change).

Publishes msgpack like composed_camera:
  {"timestamps": {"left_wrist": t, "right_wrist": t},
   "images": {"left_wrist": b64jpg, "right_wrist": b64jpg}}

Default by-id (verified 2026-11-02):
  left_wrist  <- ...Abham_Image_HHW002-video-index0
  right_wrist <- ...Abham_Image_HHW001-video-index0
"""
from __future__ import annotations

import argparse
import base64
import glob
import json
import os
import signal
import sys
import threading
import time

import cv2
import msgpack
import zmq

DEFAULT_STATUS_FILE = "/tmp/g1_wrist_status.json"


def write_status_file(path, payload):
    if not path:
        return
    try:
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        pass


def encode_jpeg_b64(bgr_image, quality=80):
    ok, buf = cv2.imencode(".jpg", bgr_image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return base64.b64encode(buf).decode("utf-8")


def resolve_device(spec):
    """Accept /dev/videoN, by-id path, integer index, or glob with one match."""
    if spec is None:
        return None
    s = str(spec)
    if s.isdigit():
        return int(s)
    if os.path.exists(s):
        return s
    matches = sorted(glob.glob(s))
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise RuntimeError("ambiguous device glob %r -> %s" % (s, matches))
    raise RuntimeError("device not found: %r" % s)


def find_abham_index0():
    """Return sorted unique capture nodes (*-video-index0) for Abham wrists."""
    pats = [
        "/dev/v4l/by-id/*Abham*video-index0",
        "/dev/v4l/by-id/*HHWei*video-index0",
        "/dev/v4l/by-id/*HHW*video-index0",
    ]
    found = []
    seen = set()
    for pat in pats:
        for p in sorted(glob.glob(pat)):
            if p in seen:
                continue
            seen.add(p)
            found.append(p)
    return found


def pick_left_right(devices):
    """Map HHW002→left, HHW001→right when serial is in by-id name; else sorted."""
    left = None
    right = None
    rest = []
    for p in devices:
        name = os.path.basename(p)
        if "HHW002" in name and left is None:
            left = p
        elif "HHW001" in name and right is None:
            right = p
        else:
            rest.append(p)
    for p in rest:
        if left is None:
            left = p
        elif right is None:
            right = p
    return left, right


def wait_abham_pair(timeout_s=12.0, poll_s=0.5):
    """Wait until two Abham index0 nodes exist (hub enum can be flaky)."""
    t0 = time.time()
    last = []
    while time.time() - t0 < timeout_s:
        last = find_abham_index0()
        if len(last) >= 2:
            return last
        time.sleep(poll_s)
    raise RuntimeError(
        "need 2 Abham index0 devices within %.1fs, saw %d: %s"
        % (timeout_s, len(last), last)
    )


class UsbCam(object):
    def __init__(self, device, width=640, height=480, fps=30, name="cam"):
        self.name = name
        self.device = device
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open %s (%r)" % (name, device))
        # Abham prefers MJPG at higher resolutions on USB2.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(15):
            ret, _ = self.cap.read()
            if ret:
                break
            time.sleep(0.05)
        aw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(
            "[%s] opened %r  %dx%d @%.1f"
            % (name, device, aw, ah, self.cap.get(cv2.CAP_PROP_FPS))
        )
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
                ret, fr = self.cap.read()
                if not ret or fr is None:
                    self._err = "read failed"
                    time.sleep(0.02)
                    continue
                with self.lock:
                    self.frame = fr
                    self.stamp = time.time()
                    self._err = None
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
            self.cap.release()
        except Exception:
            pass


def list_abham():
    print("[g1-wrist] Abham / V4L candidates:")
    by_id = "/dev/v4l/by-id"
    if os.path.isdir(by_id):
        for name in sorted(os.listdir(by_id)):
            if "Abham" in name or "HHWei" in name or "HHW" in name:
                print(" ", os.path.join(by_id, name), "->", os.path.realpath(os.path.join(by_id, name)))
    for path in sorted(glob.glob("/dev/video*")):
        # cheap filter via symlink names already printed; still show nodes for debug
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--left-device",
        default="auto",
        help="left_wrist device path / index / glob, or 'auto'",
    )
    ap.add_argument(
        "--right-device",
        default="auto",
        help="right_wrist device path / index / glob, or 'auto'",
    )
    ap.add_argument("--port", type=int, default=5558,
                    help="ZMQ PUB port; default 5558 (avoid Pico pose on 5556)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--wait-secs", type=float, default=12.0,
                    help="seconds to wait for both Abham devices when using auto")
    ap.add_argument(
        "--status-file",
        default=os.environ.get("G1_WRIST_STATUS_FILE", DEFAULT_STATUS_FILE),
        help="JSON status path read by dual RealSense status line (default /tmp/g1_wrist_status.json)",
    )
    args = ap.parse_args()

    list_abham()
    if args.left_device == "auto" or args.right_device == "auto":
        pair = wait_abham_pair(timeout_s=args.wait_secs)
        auto_left, auto_right = pick_left_right(pair)
        left_spec = auto_left if args.left_device == "auto" else args.left_device
        right_spec = auto_right if args.right_device == "auto" else args.right_device
        if left_spec is None or right_spec is None:
            raise RuntimeError("auto discover failed: left=%r right=%r from %s"
                               % (left_spec, right_spec, pair))
    else:
        left_spec, right_spec = args.left_device, args.right_device

    left_dev = resolve_device(left_spec)
    right_dev = resolve_device(right_spec)
    print("[g1-wrist] left=%r right=%r port=%d" % (left_dev, right_dev, args.port))

    left = UsbCam(left_dev, args.width, args.height, args.fps, name="left_wrist")
    right = UsbCam(right_dev, args.width, args.height, args.fps, name="right_wrist")

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 20)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind("tcp://*:%d" % args.port)
    print("[g1-wrist] PUB tcp://*:%d" % args.port)

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
        limg, lt = left.latest()
        rimg, rt = right.latest()
        if limg is not None and rimg is not None:
            payload = {
                "timestamps": {"left_wrist": lt, "right_wrist": rt},
                "images": {
                    "left_wrist": encode_jpeg_b64(limg, args.jpeg_quality),
                    "right_wrist": encode_jpeg_b64(rimg, args.jpeg_quality),
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
                fps = 30.0 / max(1e-6, now - fps_t)
                print(
                    "[g1-wrist] sent=%d dropped=%d fps=%.1f left_err=%s right_err=%s"
                    % (sent, dropped, fps, left._err, right._err)
                )
                write_status_file(
                    args.status_file,
                    {
                        "ts": time.time(),
                        "sent": sent,
                        "dropped": dropped,
                        "fps": fps,
                        "left_err": left._err,
                        "right_err": right._err,
                        "alive": True,
                    },
                )
                fps_t = now
            elif left._err or right._err:
                # Push errors promptly so the dual-server terminal can show them.
                write_status_file(
                    args.status_file,
                    {
                        "ts": time.time(),
                        "sent": sent,
                        "dropped": dropped,
                        "fps": None,
                        "left_err": left._err,
                        "right_err": right._err,
                        "alive": True,
                    },
                )

        sleep_t = target - time.monotonic()
        if sleep_t > 0:
            time.sleep(sleep_t)
        else:
            idx += 1

    write_status_file(
        args.status_file,
        {
            "ts": time.time(),
            "sent": sent,
            "dropped": dropped,
            "fps": None,
            "left_err": "stopped",
            "right_err": "stopped",
            "alive": False,
        },
    )
    left.close()
    right.close()
    sock.close()
    ctx.term()
    print("[g1-wrist] stopped")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print("[g1-wrist] FATAL:", e, file=sys.stderr)
        sys.exit(1)
