#!/usr/bin/env python3
"""Render teleop.smpl_joints (24x3) from a LeRobot episode to an mp4 skeleton video."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
import pyarrow.parquet as pq

# SMPL body-24 kinematic parents (pelvis=0 ... wrists/hands stubs).
_SMPL24_PARENTS = np.array(
    [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21],
    dtype=np.int32,
)
_DEFAULT_FFMPEG = Path("/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/.tools/bin/ffmpeg")


def _project(
    pts: np.ndarray,
    *,
    width: int,
    height: int,
    center_xz: np.ndarray,
    half_extent: float,
) -> np.ndarray:
    """Orthographic full-body framing matching MuJoCo panels.

    Dataset teleop.smpl_joints are pelvis-local with height along +Z (head up, feet -Z).
    Screen: X right, Z up. Fixed global square bbox so the whole body stays in frame.
    """
    # slight 3/4 yaw so depth (Y) contributes, closer to MuJoCo three-quarter look
    yaw = np.deg2rad(25.0)
    c, s = np.cos(yaw), np.sin(yaw)
    x = c * pts[:, 0] + s * pts[:, 1]
    z = pts[:, 2]
    # map center±half_extent → body band, leave room for HUD overlays top/bottom
    u = (x - center_xz[0]) / (2.0 * half_extent) + 0.5
    v = (z - center_xz[1]) / (2.0 * half_extent) + 0.5
    u = np.clip(u, 0.0, 1.0)
    v = np.clip(v, 0.0, 1.0)
    # content band: x 8%–92%, y 14%–90% (top reserved for compose/title HUD)
    px = (u * (width * 0.84) + width * 0.08).astype(np.int32)
    py = ((1.0 - v) * (height * 0.76) + height * 0.14).astype(np.int32)
    return np.stack([px, py], axis=1)


def render(
    ds: Path,
    episode: int,
    out_mp4: Path,
    *,
    fps: float | None,
    ffmpeg: Path,
    width: int = 640,
    height: int = 480,
) -> Path:
    info = json.loads((ds / "meta" / "info.json").read_text())
    if fps is None:
        fps = float(info.get("fps", 50))
    table = pq.read_table(
        ds / "data" / "chunk-000" / f"episode_{episode:06d}.parquet",
        columns=["teleop.smpl_joints", "teleop.smpl_frame_index", "action.motion_token"],
    )
    joints = np.stack(table.column("teleop.smpl_joints").to_pylist()).astype(np.float32)
    if joints.shape[-1] != 72:
        raise ValueError(f"expected smpl_joints 72, got {joints.shape}")
    joints = joints.reshape(-1, 24, 3)
    fi = np.asarray(table.column("teleop.smpl_frame_index").to_pylist()).reshape(-1)
    tok0 = np.stack(table.column("action.motion_token").to_pylist())[:, 0].astype(np.float32)
    t = joints.shape[0]

    # pelvis-local; height is +Z in this dataset (pico local SMPL)
    joints = joints - joints[:, :1, :]

    # Global framing over the whole episode (stable, full-body, MuJoCo-like).
    yaw = np.deg2rad(25.0)
    c, s = np.cos(yaw), np.sin(yaw)
    xs = (c * joints[:, :, 0] + s * joints[:, :, 1]).reshape(-1)
    zs = joints[:, :, 2].reshape(-1)
    center_xz = np.array(
        [0.5 * (float(xs.min()) + float(xs.max())), 0.5 * (float(zs.min()) + float(zs.max()))],
        dtype=np.float64,
    )
    extent = float(max(xs.max() - xs.min(), zs.max() - zs.min(), 1e-3))
    # Extra padding so full body fits with HUD-safe margins
    half_extent = 0.5 * extent * 1.45
    print(
        f"[smpl_skel] global frame center=({center_xz[0]:.3f},{center_xz[1]:.3f}) "
        f"extent={extent:.3f} half={half_extent:.3f} (Z-up orthographic)",
        flush=True,
    )

    out_mp4 = Path(out_mp4)
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    ff = str(ffmpeg) if ffmpeg.is_file() else "ffmpeg"
    proc = subprocess.Popen(
        [
            ff, "-y", "-loglevel", "error",
            "-f", "rawvideo", "-vcodec", "rawvideo",
            "-pix_fmt", "bgr24", "-s", f"{width}x{height}", "-r", str(fps), "-i", "-",
            "-an", "-vcodec", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(out_mp4),
        ],
        stdin=subprocess.PIPE,
    )
    assert proc.stdin is not None

    for i in range(t):
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = (24, 24, 28)
        pts = _project(
            joints[i],
            width=width,
            height=height,
            center_xz=center_xz,
            half_extent=half_extent,
        )
        for j, p in enumerate(_SMPL24_PARENTS):
            if p < 0:
                continue
            cv2.line(img, tuple(pts[p]), tuple(pts[j]), (80, 220, 120), 2, cv2.LINE_AA)
        for j in range(24):
            color = (0, 200, 255) if j == 0 else (220, 220, 220)
            cv2.circle(img, tuple(pts[j]), 3 if j else 5, color, -1, cv2.LINE_AA)
        hud = [
            f"ep{episode:03d} fi={i}/{t - 1}  teleop.smpl_joints (24x3)",
            f"smpl_frame_index={int(fi[i])}  motion_token[0]={tok0[i]:+.4f}",
        ]
        y = 22
        for line in hud:
            cv2.putText(img, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 255, 120), 1, cv2.LINE_AA)
            y += 18
        proc.stdin.write(img.tobytes())
        if i % 50 == 0 or i == t - 1:
            print(f"[smpl_skel] frame {i}/{t - 1}", flush=True)

    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError(f"ffmpeg failed: {out_mp4}")
    print(f"[ok] {out_mp4} frames={t} fps={fps}", flush=True)
    return out_mp4


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--fps", type=float, default=None)
    ap.add_argument("--ffmpeg", type=Path, default=_DEFAULT_FFMPEG)
    args = ap.parse_args()
    render(args.dataset, args.episode, args.out, fps=args.fps, ffmpeg=args.ffmpeg)


if __name__ == "__main__":
    main()
