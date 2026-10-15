#!/usr/bin/env python3
# ruff: noqa: T201
"""Batch "SOMA BVH vs retargeted robot" comparison videos -- the ``videos/compare_v15`` layout, for A3 or A3 Ultra.

Left: SOMA skeleton at the robot's scale with a fixed floor line (the retarget scale of that clip), right: the robot
driven by the same retarget the batch run uses (``retarget_bvh``: calibrated IK table, per-clip proportions, floor
lift), L/R labels, "moving: FORWARD/..." in each body's own frame, and a red JUMP tag on frames where a body joint
moves more than 25 deg in one frame.  800x400, 30 fps, the whole clip (at most 900 frames, as compare_v15).

    python gear_sonic/data_process/make_compare_videos.py --robot a3 --names_from videos/compare_v15 \\
        --tar data/bones_seed/soma_proportional.tar.gz --out videos/compare_v15_a3

The tar is read once (it is ~45 GB); the BVHs of the requested names are pulled out in that pass.
"""

from __future__ import annotations

import argparse
import contextlib
import io
from pathlib import Path
import sys
import tarfile
import tempfile

import cv2
import mujoco as mj
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
for p in (PROJECT_ROOT, Path.home() / "gmr_repo"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from gear_sonic.data_process.batch_retarget_bvh_soma_a3ultra import load_soma_bvh_frames, retarget_bvh  # noqa: E402
from gear_sonic.data_process.visualize_retarget import MuJoCoRenderer, motion_label, render_soma_frame  # noqa: E402

MJCF_DIR = PROJECT_ROOT / "gear_sonic/data/assets/robot_description/mjcf"
ROBOTS = {
    "a3": (MJCF_DIR / "a3_t2d5.xml", "A3 (front view)", [3, 4]),  # head joints are locked, not policy joints
    "a3_ultra": (MJCF_DIR / "a3_ultra_t3d0.xml", "A3 Ultra (front view)", [3, 4]),
}
JUMP_DEG = 25.0


def extract_from_tar(tar_path: Path, names: set[str], dest: Path) -> dict[str, Path]:
    """One streaming pass over the archive; stops once every requested BVH is out."""
    found: dict[str, Path] = {}
    with tarfile.open(tar_path, "r|gz") as tf:
        for m in tf:
            stem = Path(m.name).stem
            if m.isfile() and m.name.endswith(".bvh") and stem in names:
                out = dest / f"{stem}.bvh"
                out.write_bytes(tf.extractfile(m).read())
                found[stem] = out
                if len(found) == len(names):
                    break
    return found


def render_clip(bvh: Path, out: Path, robot: str, renderer: MuJoCoRenderer, retargeter, fps: int, size: int, max_frames: int):
    frames, _, fps_src = load_soma_bvh_frames(str(bvh))
    with contextlib.redirect_stdout(io.StringIO()):
        qpos_list, retargeter = retarget_bvh(str(bvh), fps_target=fps, retargeter=retargeter, robot=robot)
    stride = max(1, round(fps_src / fps))
    soma = [frames[i] for i in range(0, len(frames), stride)]
    n = min(len(soma), len(qpos_list), max_frames)
    view = renderer.soma_view(retargeter.human_scale_table["Hips"])

    soma_hips = np.array([f["Hips"][0][:2] for f in soma[:n]])
    soma_left = np.array([np.asarray(f["LeftLeg"][0][:2]) - np.asarray(f["RightLeg"][0][:2]) for f in soma[:n]])
    root = np.array([q[:2] for q in qpos_list[:n]])
    left = []
    for q in qpos_list[:n]:
        renderer.data.qpos[:] = q
        mj.mj_kinematics(renderer.model, renderer.data)
        left.append((renderer.body_pos("left_hip_yaw_Link") - renderer.body_pos("right_hip_yaw_Link"))[:2])
    left = np.array(left)
    joints = np.array([q[7:] for q in qpos_list[:n]])
    names = [mj.mj_id2name(renderer.model, mj.mjtObj.mjOBJ_JOINT, j) for j in range(1, renderer.model.njnt)]  # id 0 = free root
    keep = [j for j in range(joints.shape[1]) if j not in ROBOTS[robot][2]]
    step = np.abs(np.diff(joints[:, keep], axis=0, prepend=joints[:1, keep]))
    worst = np.degrees(step.max(axis=1))
    worst_joint = [names[keep[j]] for j in step.argmax(axis=1)]
    jump_frames = int((worst > JUMP_DEG).sum())

    win = max(1, fps // 3)

    def label(pos, vec, i):
        a, b = max(0, i - win), min(n - 1, i + win)
        return motion_label(pos[b] - pos[a], vec[i])

    out.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), fps, (size * 2, size))
    hold = 0
    for i in range(n):
        lp = render_soma_frame(soma[i], img_size=size, motion_text=label(soma_hips, soma_left, i), view=view)
        rp = renderer.render(qpos_list[i], motion_text=label(root, left, i))
        if worst[i] > JUMP_DEG:
            hold = fps // 4
            tag = f"JUMP {worst[i]:.0f} deg {worst_joint[i]}"
        if hold > 0:
            cv2.putText(rp, tag, (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
            hold -= 1
        both = np.hstack([lp, rp])
        cv2.putText(both, f"{bvh.stem}  {i + 1}/{n}", (10, size - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1, cv2.LINE_AA)
        writer.write(both)
    writer.release()
    return retargeter, n, jump_frames, float(worst.max())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", choices=sorted(ROBOTS), default="a3")
    ap.add_argument("--tar", type=Path, default=PROJECT_ROOT / "data/bones_seed/soma_proportional.tar.gz")
    ap.add_argument("--names", nargs="*", default=[], help="clip names (without .bvh)")
    ap.add_argument("--names_from", type=Path, help="directory of mp4s whose names are the clips")
    ap.add_argument("--bvh_dir", type=Path, help="read the BVHs from here instead of the tar")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--size", type=int, default=400)
    ap.add_argument("--max_frames", type=int, default=900, help="cap per clip (compare_v15 stops at 900)")
    args = ap.parse_args()

    names = list(args.names)
    if args.names_from:
        names += [p.stem for p in sorted(args.names_from.glob("*.mp4"))]
    names = list(dict.fromkeys(names))
    if not names:
        ap.error("no clips: give --names or --names_from")

    with tempfile.TemporaryDirectory(prefix="compare_bvh_") as tmp:
        if args.bvh_dir:
            found = {n: args.bvh_dir / f"{n}.bvh" for n in names if (args.bvh_dir / f"{n}.bvh").exists()}
        else:
            print(f"reading {args.tar.name} once for {len(names)} clips ...", flush=True)
            found = extract_from_tar(args.tar, set(names), Path(tmp))
        missing = [n for n in names if n not in found]
        if missing:
            print(f"no BVH for: {missing}")
        mjcf, title, _ = ROBOTS[args.robot]
        renderer = MuJoCoRenderer(str(mjcf), width=args.size, height=args.size, title=title)
        retargeter = None
        for n in names:
            if n not in found:
                continue
            retargeter, frames, jumps, worst = render_clip(found[n], args.out / f"{n}.mp4", args.robot, renderer, retargeter, args.fps, args.size, args.max_frames)
            print(f"{n}: {frames} frames, {jumps} frames with a >{JUMP_DEG:.0f} deg joint step (max {worst:.0f} deg)", flush=True)
        renderer.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
