#!/usr/bin/env python3
"""Burn a category label at the BOTTOM CENTER of each frame, then write H.264.

Telemetry overlays (token dims, SONIC root, grippers, …) sit at the TOP of
MuJoCo panels — keep the category tag at the bottom so they never collide.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

_DEFAULT_FFMPEG = Path("/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/.tools/bin/ffmpeg")


def _draw_bottom_label(frame: np.ndarray, label: str) -> np.ndarray:
    out = frame.copy()
    h, w = out.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = max(0.55, min(1.0, w / 900.0))
    thickness = 2
    (tw, th), baseline = cv2.getTextSize(label, font, scale, thickness)
    pad_x, pad_y = 14, 10
    box_w = tw + 2 * pad_x
    box_h = th + baseline + 2 * pad_y
    x0 = max(0, (w - box_w) // 2)
    y1 = h - 12
    y0 = max(0, y1 - box_h)
    x1 = min(w, x0 + box_w)

    overlay = out.copy()
    cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 0), thickness=-1)
    out = cv2.addWeighted(overlay, 0.55, out, 0.45, 0.0)
    text_x = x0 + pad_x
    text_y = y1 - pad_y - baseline
    cv2.putText(out, label, (text_x, text_y), font, scale, (0, 255, 180), thickness, cv2.LINE_AA)
    return out


def burn(src: Path, label: str, dst: Path, ffmpeg: Path) -> Path:
    cap = cv2.VideoCapture(str(src))
    if not cap.isOpened():
        raise SystemExit(f"cannot open {src}")
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 30.0)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    dst = Path(dst)
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_suffix(".raw.mp4")

    cmd = [
        str(ffmpeg),
        "-y",
        "-loglevel",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "-s",
        f"{w}x{h}",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(tmp),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    assert proc.stdin is not None
    n = 0
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        labeled = _draw_bottom_label(fr, label)
        proc.stdin.write(labeled.tobytes())
        n += 1
    proc.stdin.close()
    cap.release()
    rc = proc.wait()
    if rc != 0:
        raise SystemExit(f"ffmpeg failed rc={rc} writing {tmp}")
    tmp.replace(dst)
    print(f"[burn] {dst} frames={n} label={label!r}", flush=True)
    return dst


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", type=Path, required=True)
    ap.add_argument("--label", type=str, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--ffmpeg", type=Path, default=_DEFAULT_FFMPEG)
    args = ap.parse_args()
    if not args.ffmpeg.is_file():
        print(f"missing ffmpeg: {args.ffmpeg}", file=sys.stderr)
        raise SystemExit(2)
    burn(args.inp, args.label, args.out, args.ffmpeg)


if __name__ == "__main__":
    main()
