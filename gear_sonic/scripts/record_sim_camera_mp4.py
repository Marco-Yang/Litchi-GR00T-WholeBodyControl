#!/usr/bin/env python3
"""Record MuJoCo offscreen camera ZMQ stream to an mp4 until a stop file appears."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--camera", default="ego_view", help="Preferred camera key")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--stop-file", type=Path, required=True)
    ap.add_argument("--timeout-s", type=float, default=600.0)
    args = ap.parse_args()

    from gear_sonic.camera.composed_camera import ComposedCameraClientSensor

    client = ComposedCameraClientSensor(server_ip=args.host, port=args.port)
    writer = None
    n = 0
    t0 = time.time()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    print(f"[cam-rec] recording {args.host}:{args.port} -> {args.out}")

    try:
        while not args.stop_file.exists():
            if time.time() - t0 > args.timeout_s:
                print("[cam-rec] timeout")
                break
            try:
                msg = client.read()
            except Exception:
                time.sleep(0.05)
                continue
            if not msg or "images" not in msg:
                time.sleep(0.02)
                continue
            images = msg["images"]
            if args.camera in images:
                frame = images[args.camera]
            else:
                frame = next(iter(images.values()))
            frame = np.asarray(frame)
            if frame.dtype != np.uint8:
                frame = np.clip(frame, 0, 255).astype(np.uint8)
            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[-1] == 3:
                # MuJoCo/renderer is RGB; OpenCV VideoWriter expects BGR.
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if writer is None:
                h, w = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(str(args.out), fourcc, args.fps, (w, h))
                if not writer.isOpened():
                    raise RuntimeError(f"failed to open VideoWriter for {args.out}")
            writer.write(frame)
            n += 1
            if n % 60 == 0:
                print(f"[cam-rec] frames={n}")
            time.sleep(max(0.0, 1.0 / args.fps - 0.001))
    finally:
        if writer is not None:
            writer.release()
        print(f"[cam-rec] wrote {n} frames -> {args.out}")


if __name__ == "__main__":
    main()
