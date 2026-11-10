#!/usr/bin/env python3
"""Kinematic-only MuJoCo replay of LeRobot episode observation.state (qpos).

Maps joints by name into MJCF (body 29). No physics, no SONIC, no hands remap
(Dex3 names in data ≠ Revo2 in scene — hands stay at model default).

Root: xyz fixed at (0,0,root_z); quat from observation.root_orientation each frame.
Use this to visually check joint order / sign conventions vs your recollection.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
import pyarrow.parquet as pq


_DEFAULT_MJCF = Path(
    "/home/zhenyu/workspace/code/HumanoidWBM/demo/RL_controller/"
    "Litchi-GR00T-WholeBodyControl/gear_sonic_deploy/g1/g1_29dof_with_revo2.xml"
)
_DEFAULT_FFMPEG = Path("/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/.tools/bin/ffmpeg")


def _load_episode(ds: Path, episode: int) -> tuple[np.ndarray, np.ndarray, list[str], float]:
    info = json.loads((ds / "meta" / "info.json").read_text())
    names = list(info["features"]["observation.state"]["names"])
    fps = float(info.get("fps", 50))
    table = pq.read_table(ds / "data" / "chunk-000" / f"episode_{episode:06d}.parquet")
    state = np.stack(table.column("observation.state").to_pylist()).astype(np.float64)
    if "observation.root_orientation" in table.column_names:
        root_quat = np.stack(table.column("observation.root_orientation").to_pylist()).astype(np.float64)
    else:
        root_quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (state.shape[0], 1))
    return state, root_quat, names, fps


def _joint_qpos_map(model: mujoco.MjModel, names: list[str]) -> list[tuple[int, int]]:
    """Return list of (state_idx, qpos_adr) for names present in MJCF."""
    out: list[tuple[int, int]] = []
    missing: list[str] = []
    for i, name in enumerate(names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            missing.append(name)
            continue
        out.append((i, int(model.jnt_qposadr[jid])))
    if missing:
        print(f"[qpos_replay] skip {len(missing)} joints not in MJCF (likely hands): {missing[:6]}...", flush=True)
    print(f"[qpos_replay] mapped {len(out)}/{len(names)} joints by name", flush=True)
    return out


def render(
    ds: Path,
    episode: int,
    out_mp4: Path,
    *,
    mjcf: Path,
    root_z: float,
    fps_override: float | None,
    max_frames: int,
    ffmpeg: Path,
    width: int = 640,
    height: int = 480,
) -> Path:
    state, root_quat, names, fps = _load_episode(ds, episode)
    if fps_override is not None:
        fps = float(fps_override)
    if max_frames > 0:
        state = state[:max_frames]
        root_quat = root_quat[:max_frames]
    t = int(state.shape[0])

    # Resolve relative mesh paths from MJCF directory.
    mjcf = mjcf.resolve()
    cwd = Path.cwd()
    os.chdir(mjcf.parent)
    try:
        model = mujoco.MjModel.from_xml_path(str(mjcf))
    finally:
        os.chdir(cwd)

    data = mujoco.MjData(model)
    mapping = _joint_qpos_map(model, names)

    renderer = mujoco.Renderer(model, height=height, width=width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = 3.2
    cam.elevation = -12.0
    cam.azimuth = 135.0

    out_mp4 = Path(out_mp4)
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    ff = str(ffmpeg) if ffmpeg.is_file() else "ffmpeg"
    proc = subprocess.Popen(
        [
            ff, "-y", "-loglevel", "error",
            "-f", "rawvideo", "-vcodec", "rawvideo",
            "-pix_fmt", "rgb24", "-s", f"{width}x{height}", "-r", str(fps), "-i", "-",
            "-an", "-vcodec", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(out_mp4),
        ],
        stdin=subprocess.PIPE,
    )
    assert proc.stdin is not None

    # elbow indices in state (body naming)
    try:
        i_le = names.index("left_elbow_joint")
        i_re = names.index("right_elbow_joint")
        i_lsp = names.index("left_shoulder_pitch_joint")
        i_rsp = names.index("right_shoulder_pitch_joint")
    except ValueError:
        i_le = i_re = i_lsp = i_rsp = -1

    for i in range(t):
        q = model.qpos0.copy()
        q[0:3] = [0.0, 0.0, float(root_z)]
        q[3:7] = root_quat[i]
        for si, adr in mapping:
            q[adr] = state[i, si]
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        cam.lookat[:] = [0.0, 0.0, float(root_z)]
        renderer.update_scene(data, camera=cam)
        frame = renderer.render()

        # simple HUD via numpy (green text would need PIL; stamp numbers in corner as bars)
        # Use a tiny overlay: encode key angles as colored pixels strip at top.
        # Prefer readable text via OpenCV if available; else raw frame.
        try:
            import cv2

            img = frame.copy()
            le = float(state[i, i_le]) if i_le >= 0 else 0.0
            re = float(state[i, i_re]) if i_re >= 0 else 0.0
            lsp = float(state[i, i_lsp]) if i_lsp >= 0 else 0.0
            rsp = float(state[i, i_rsp]) if i_rsp >= 0 else 0.0
            lines = [
                f"ep{episode:03d} fi={i}/{t - 1}  kinematic observation.state",
                f"L_elbow={le:+.3f}rad ({np.degrees(le):+.1f}deg)  R_elbow={re:+.3f}rad ({np.degrees(re):+.1f}deg)",
                f"L_sh_pitch={lsp:+.3f}  R_sh_pitch={rsp:+.3f}  root_z={root_z:.3f} (xy fixed)",
            ]
            y = 22
            for line in lines:
                cv2.putText(img, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 80), 1, cv2.LINE_AA)
                y += 18
            proc.stdin.write(img.tobytes())
        except Exception:
            proc.stdin.write(frame.tobytes())

        if i % 50 == 0 or i == t - 1:
            print(f"[qpos_replay] frame {i}/{t - 1}", flush=True)

    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError(f"ffmpeg failed writing {out_mp4}")
    print(f"[ok] {out_mp4} frames={t} fps={fps} mjcf={mjcf.name}", flush=True)
    return out_mp4


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--mjcf", type=Path, default=_DEFAULT_MJCF)
    ap.add_argument("--root-z", type=float, default=0.793)
    ap.add_argument("--fps", type=float, default=None)
    ap.add_argument("--max-frames", type=int, default=0, help="0 = all frames")
    ap.add_argument("--ffmpeg", type=Path, default=_DEFAULT_FFMPEG)
    args = ap.parse_args()
    render(
        args.dataset,
        args.episode,
        args.out,
        mjcf=args.mjcf,
        root_z=args.root_z,
        fps_override=args.fps,
        max_frames=args.max_frames,
        ffmpeg=args.ffmpeg,
    )


if __name__ == "__main__":
    main()
