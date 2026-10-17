#!/usr/bin/env python3
# ruff: noqa: T201
"""Offline score of the REAL-TIME retargeter's legs: run SMPL mocap clips through it frame by frame (as the Pico stream would) and compare the 12 leg
joints with the offline A3 reference of the same clip (the training library, ``data/motion_lib_a3_29d/<lib>/<shard>/<clip>.pkl``).

    python gear_sonic_deploy/scripts/a3_eval_leg_retarget.py [--foot-rot-weight W] [--ankle-knee-rot-weight W] [--clips a,b,c | --n 12] [--lib robot_filtered_v15_clean]

Run it inside the ``gmr`` conda env (mujoco + mink + GMR) from the repo root: ``PYTHONPATH=. ~/miniconda3/envs/gmr/bin/python gear_sonic_deploy/scripts/a3_eval_leg_retarget.py``.

What is printed, per leg joint over all clips (the SMPL clips are 50 fps, the library 30 fps: library frame k is compared with the real-time result at SMPL frame round(k * 50 / 30); the first ``--settle`` library frames are dropped):
  rmse   rms of (real-time - offline) in rad;   corr  Pearson correlation;   bias  mean difference (a constant offset the policy would also see)
and over all frames: the share of frames whose ankle roll is within 0.05 rad of its limit (the stock table pins a plainly standing person there).
The offline library is the best reference we have for what the legs SHOULD be (it is what the SMPL encoder is trained against) but it is not the
human's ground truth, and mocap SMPL is cleaner than the Pico stream: use it to compare settings, then confirm with a real Pico recording.
"""

from __future__ import annotations

import argparse
import glob
from pathlib import Path
import sys

import joblib
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

LEG_NAMES = ["L_hip_p", "L_hip_r", "L_hip_y", "L_knee", "L_ank_p", "L_ank_r", "R_hip_p", "R_hip_r", "R_hip_y", "R_knee", "R_ank_p", "R_ank_r"]
ANKLE_ROLL_LIMIT = 0.349066
WALKY = ("walk_forward", "walk_ff", "walk_left", "walk_right", "step", "march", "turn", "arc_walk", "stand")


def pick_clips(smpl_dir: Path, lib_dir: Path, n: int, seed: int = 0) -> list[str]:
    """Deterministic sample of clips whose name looks like locomotion / standing and that exist in both the SMPL set and the library."""
    names = sorted(p.stem for p in smpl_dir.glob("*.pkl") if not p.stem.endswith("_M") and any(w in p.stem for w in WALKY))
    rng = np.random.default_rng(seed)
    rng.shuffle(names)
    out = []
    for nm in names:
        if library_path(lib_dir, nm) is not None:
            out.append(nm)
        if len(out) >= n:
            break
    return out


def library_path(lib_dir: Path, name: str) -> Path | None:
    hits = glob.glob(str(lib_dir / "*" / f"{name}.pkl"))
    return Path(hits[0]) if hits else None


def load_clip(path: Path) -> dict:
    d = joblib.load(path)
    return d if "pose_aa" in d and "dof" not in d and "root_rot" not in d else next(iter(d.values()))


def run_clip(retargeter, fk, pose_aa: np.ndarray, fps: float) -> np.ndarray:
    """(T, 29) real-time retarget of one SMPL clip, frame by frame like the Pico stream (warm start, first frames settle the IK)."""
    from gear_sonic.scripts.pico_pose_zmq_replay import frame_from_pose_aa

    out = np.zeros((len(pose_aa), 29))
    retargeter.reset()
    sp, sj, bq = frame_from_pose_aa(pose_aa[0], fk)
    for _ in range(5):
        retargeter.retarget_pico_frame(sp, bq, sj, 0.0, 1)
    for i, row in enumerate(pose_aa):
        sp, sj, bq = frame_from_pose_aa(row, fk)
        out[i] = retargeter.retarget_pico_frame(sp, bq, sj, i / fps, 1 + i).q_mj29
    return out


def score(pairs: list[tuple[np.ndarray, np.ndarray]], settle: int) -> dict:
    """pairs = [(realtime (T,29), offline (T,29))]; returns per-leg-joint rmse / corr / bias and the ankle-roll saturation share."""
    d_all, r_all, o_all = [], [], []
    for rt, off in pairs:
        t = min(len(rt), len(off))
        r, o = rt[settle:t, 17:29], off[settle:t, 17:29]
        r_all.append(r)
        o_all.append(o)
        d_all.append(r - o)
    r, o, d = np.concatenate(r_all), np.concatenate(o_all), np.concatenate(d_all)
    corr = np.array([np.corrcoef(r[:, j], o[:, j])[0, 1] if o[:, j].std() > 1e-6 and r[:, j].std() > 1e-6 else np.nan for j in range(12)])
    sat = np.mean(np.abs(r[:, [5, 11]]) > ANKLE_ROLL_LIMIT - 0.05)
    return dict(rmse=np.sqrt(np.mean(d**2, axis=0)), corr=corr, bias=d.mean(axis=0), ankle_roll_saturation=float(sat), frames=len(r))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clips", default="", help="comma-separated clip names (default: --n locomotion clips found in both sets)")
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--lib", default="robot_filtered_v15_clean")
    ap.add_argument("--smpl-dir", type=Path, default=ROOT / "data/smpl_filtered")
    ap.add_argument("--settle", type=int, default=15, help="frames dropped at the start of every clip")
    ap.add_argument("--foot-rot-weight", type=float, default=None)
    ap.add_argument("--ankle-knee-rot-weight", type=float, default=None)
    ap.add_argument("--max-seconds", type=float, default=12.0, help="use at most this many seconds of every clip")
    a = ap.parse_args()

    from gear_sonic.scripts import pico_pose_zmq_minimal as pm
    from gear_sonic.utils.teleop.a3.retarget import A3RealtimeRetargeter

    lib_dir = ROOT / "data/motion_lib_a3_29d" / a.lib
    names = [c for c in a.clips.split(",") if c] or pick_clips(a.smpl_dir, lib_dir, a.n)
    print(f"{len(names)} clips, foot_rot_weight={a.foot_rot_weight}, ankle_knee_rot_weight={a.ankle_knee_rot_weight}, library {a.lib}")
    ret = A3RealtimeRetargeter(foot_rot_weight=a.foot_rot_weight, ankle_knee_rot_weight=a.ankle_knee_rot_weight)
    fk = pm.HumanJointsFK()
    pairs = []
    for nm in names:
        sm = load_clip(a.smpl_dir / f"{nm}.pkl")
        lp = library_path(lib_dir, nm)
        off = np.asarray(load_clip(lp)["dof"], dtype=np.float64) if lp else None
        if off is None:
            print(f"  {nm}: no library pair, skipped")
            continue
        smpl_fps, lib_fps = float(sm.get("fps", 50.0)), float(load_clip(lp).get("fps", 30.0))
        n_s = int(min(len(sm["pose_aa"]), a.max_seconds * smpl_fps))
        rt50 = run_clip(ret, fk, np.asarray(sm["pose_aa"], dtype=np.float64)[:n_s], smpl_fps)
        n_l = int(min(len(off), a.max_seconds * lib_fps, n_s / smpl_fps * lib_fps))
        idx = np.minimum(np.round(np.arange(n_l) / lib_fps * smpl_fps).astype(int), len(rt50) - 1)
        pairs.append((rt50[idx], off[:n_l]))
    s = score(pairs, a.settle)
    print(f"{s['frames']} frames over {len(pairs)} clips")
    print(f"{'joint':8s} {'rmse':>6s} {'corr':>6s} {'bias':>7s}")
    for j, nm in enumerate(LEG_NAMES):
        print(f"{nm:8s} {s['rmse'][j]:6.3f} {s['corr'][j]:+6.2f} {s['bias'][j]:+7.3f}")
    print(f"mean leg rmse {np.nanmean(s['rmse']):.3f} rad | hip+knee rmse {np.mean(s['rmse'][[0, 1, 3, 6, 7, 9]]):.3f} | ankle rmse {np.mean(s['rmse'][[4, 5, 10, 11]]):.3f} | "
          f"mean corr {np.nanmean(s['corr']):+.2f} | ankle-roll within 0.05 rad of the limit in {s['ankle_roll_saturation'] * 100:.1f}% of frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
