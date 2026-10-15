#!/usr/bin/env python3
# ruff: noqa: T201
"""Check that a robot motion library and the SMPL motion data pair up, before training an SMPL encoder on them.

The motion library pairs a robot clip with its SMPL clip by file name (the clip key, e.g. ``walk_forward_amateur_001__A001``)
and asserts equal frame counts after both are brought to the training rate (``target_fps`` 50): a robot clip of ``T``
frames at 30 fps becomes ``len(arange(0, (T - 1) / 30, 1 / 50))`` frames (``interploate_pose``: ``ceil((T - 1) * 5 / 3)``,
so ``(T - 1) * 5 / 3`` exactly when ``T - 1`` is a multiple of 3 -- the end point is excluded), the SMPL file (``bones_seed_smpl`` / ``smpl_filtered``,
``download_from_hf.py --training``) stores its 50 fps data in ``pose_aa`` / ``smpl_joints`` and the 30 fps original in
``original_pose_aa``.  A clip that fails here would stop training with an assertion; a clip without SMPL data simply cannot
sample the SMPL encoder.

The pair must also share one world heading: the encoder's ``smpl_root_ori_b`` is ``quat_inv(robot_anchor_quat_w) *
smpl_root_quat_w``.  A library retargeted without ``--keep_heading`` (every clip turned to face +X at frame 0) sits ~90 deg
off the SMPL data for most clips; ``restore_a3_heading.py`` fixes that.  ``--max_heading_deg`` / ``--max_heading_fraction``
fail the check when more than that share of the pairs is further off than that (G1 library vs SMPL: 0.2 %).

    python gear_sonic/data_process/check_smpl_pairing.py \\
        --robot data/motion_lib_a3_29d/robot_filtered_v15_clean --smpl data/smpl_filtered
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import joblib
import numpy as np

TARGET_FPS = 50.0
PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:  # run as a script: gear_sonic is not importable from the script directory
    sys.path.insert(0, str(PROJECT_ROOT))


def robot_frames_at_target(num_frames: int, fps: float, target_fps: float = TARGET_FPS) -> int:
    """Frames of a robot clip after the motion library interpolates it to the training rate (its ``arange``, end excluded)."""
    return len(np.arange(0, (num_frames - 1) / fps, 1 / target_fps))


def check_pair(key: str, robot_entry: dict, smpl_entry: dict) -> list[str]:
    problems = []
    t = int(robot_entry["root_trans_offset"].shape[0])
    fps = float(robot_entry.get("fps", 30.0))
    want = robot_frames_at_target(t, fps)
    smpl_fps = float(smpl_entry.get("fps", TARGET_FPS))
    if smpl_fps != TARGET_FPS:
        problems.append(f"{key}: SMPL data is at {smpl_fps:g} fps, the library expects {TARGET_FPS:g}")
    for field in ("pose_aa", "smpl_joints"):
        if field in smpl_entry and int(smpl_entry[field].shape[0]) != want:
            problems.append(f"{key}: SMPL {field} has {smpl_entry[field].shape[0]} frames, the robot clip ({t} frames @ {fps:g} fps) needs {want}")
    if "original_pose_aa" in smpl_entry and int(smpl_entry["original_pose_aa"].shape[0]) != t:
        problems.append(f"{key}: SMPL original_pose_aa has {smpl_entry['original_pose_aa'].shape[0]} frames, robot clip has {t}")
    return problems


def heading_offset_deg(robot_entry: dict, smpl_entry: dict) -> float | None:
    """Mean heading of the robot root minus that of the SMPL root over the clip (deg); None if either has no root orientation."""
    if "root_rot" not in robot_entry or "pose_aa" not in smpl_entry:
        return None
    from gear_sonic.data_process.compare_smpl_vs_a3 import yaw_offset

    return float(np.degrees(yaw_offset(smpl_entry, robot_entry)[0]))


def _index(root: Path) -> dict[str, Path]:
    return {p.stem: p for p in sorted(root.rglob("*.pkl")) if p.name != "metadata.pkl"}


def _robot_entry(path: Path) -> dict:
    return next(iter(joblib.load(path).values()))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", type=Path, required=True, help="robot motion library (a3_29 or any) directory")
    ap.add_argument("--smpl", type=Path, required=True, help="SMPL motion directory (data/smpl_filtered)")
    ap.add_argument("--limit", type=int, default=0, help="check only the first N paired clips")
    ap.add_argument("--max_heading_deg", type=float, default=20.0, help="a pair is off in heading beyond this (deg)")
    ap.add_argument("--max_heading_fraction", type=float, default=0.05, help="fail if more than this share of pairs is off in heading")
    args = ap.parse_args()
    robot, smpl = _index(args.robot), _index(args.smpl)
    paired = sorted(set(robot) & set(smpl))
    print(f"robot clips {len(robot)}, SMPL clips {len(smpl)}, paired by name {len(paired)}")
    print(f"  robot clips without SMPL data: {len(set(robot) - set(smpl))} (cannot sample the smpl encoder)")
    print(f"  SMPL clips without a robot clip: {len(set(smpl) - set(robot))}")
    problems: list[str] = []
    offsets: list[float] = []
    for i, key in enumerate(paired):
        if args.limit and i >= args.limit:
            break
        r, s_ = _robot_entry(robot[key]), joblib.load(smpl[key])
        problems += check_pair(key, r, s_)
        off = heading_offset_deg(r, s_)
        if off is not None:
            offsets.append(off)
    print(f"frame / rate problems: {len(problems)}")
    for line in problems[:20]:
        print("  ", line)
    off_share = 0.0
    if offsets:
        off_share = float(np.mean(np.abs(offsets) > args.max_heading_deg))
        print(f"heading: {100 * off_share:.1f}% of {len(offsets)} pairs are more than {args.max_heading_deg:g} deg off "
              f"(median |offset| {np.median(np.abs(offsets)):.1f} deg)")
        if off_share > args.max_heading_fraction:
            print("  the robot library is not in the SMPL data's world heading: see restore_a3_heading.py")
    return 1 if problems or not paired or off_share > args.max_heading_fraction else 0


if __name__ == "__main__":
    raise SystemExit(main())
