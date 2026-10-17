#!/usr/bin/env python3
# ruff: noqa: T201
"""Audit the data the A3 SMPL encoder is trained on, against the G1 library as the baseline.

The SMPL encoder is trained on pairs (SMPL clip, A3 reference clip of the same name).  What it learns is whatever the reference
library does, so systematic defects of the library become behaviour of the policy.  This tool measures four things on a random
sample of the paired clips (``check_smpl_pairing.py`` checks names / frame counts / heading, this checks what it does not).
The fifth, ``link_asym``, needs only the A3 library:

``root_ori``  the training observation ``smpl_root_ori_b`` is ``quat_inv(robot_anchor_quat_w) * smpl_root_quat_w``: the FULL relative
              rotation of SMPL root and robot pelvis (not only the yaw ``check_smpl_pairing`` reports).  Compared with G1.
``mirror``    every clip ``X`` has a mirrored twin ``X_M``; a mirror-symmetric retarget gives ``joint(X) = sign * counterpart(X_M)``.
              A constant ``bias`` is a left/right calibration error (wrist offsets of the IK table, feet yaw), not noise.
``limits``    how often the reference sits at / beyond the 0.9 soft joint limit (the training penalises it) -- the A3 inward shoulder
              roll limit is only 5 degrees, so "arms at the side" is at the limit.
``link_asym`` the same mirror test in the space the IK works in: orientation of hip / knee / foot / shoulder / elbow / hand links
              (``calibrate_a3_lr_symmetry.py``); an offset of several degrees with a small scatter is a wrong ``rot_offset``.
``wrist``     the six wrist angles are an input of the SMPL encoder.  Train: library wrists (offline IK).  G1 deploy: an analytic
              formula of the SMPL pose (``pico_manager_thread_server.py``).  Fits that formula to the A3 library (gain / offset).

    python gear_sonic/data_process/audit_a3_smpl_training_data.py \\
        --a3 data/motion_lib_a3_29d/robot_filtered_v15_clean --smpl data/smpl_filtered \\
        --g1 data/motion_lib_bones_seed/robot_all --sample 250            # all four checks
    python gear_sonic/data_process/audit_a3_smpl_training_data.py --a3 ... --smpl ... --checks mirror limits
"""

from __future__ import annotations

import argparse
from pathlib import Path
import random
import sys

import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:  # run as a script: gear_sonic is not importable from the script directory
    sys.path.insert(0, str(PROJECT_ROOT))

MJCF = PROJECT_ROOT / "gear_sonic/data/assets/robot_description/mjcf/a3_t2d5.xml"
ARM_NAMES = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw"]
A3_WRIST_COLS = [7, 8, 9, 14, 15, 16]  # policy view: L roll/pitch/yaw, R roll/pitch/yaw
G1_WRIST_COLS = [19, 20, 21, 26, 27, 28]
JOINT_NAMES_A3_ARMS = [f"{side}_{n}_joint" for side in ("left", "right") for n in ARM_NAMES]


def mirror_pairs(kind: str) -> dict[str, list[tuple[int, int]]]:
    """(joint, counterpart) column pairs of a library's ``dof``: A3 policy view or G1 29-DOF view."""
    if kind == "a3":  # waist 0-2, L arm 3-9, R arm 10-16, L leg 17-22, R leg 23-28
        return {
            "waist": [(i, i) for i in range(3)],
            "arm": [(3 + i, 10 + i) for i in range(7)],
            "leg": [(17 + i, 23 + i) for i in range(6)],
        }
    return {  # G1: legs 0-11, waist 12-14, arms 15-21 / 22-28
        "waist": [(12 + i, 12 + i) for i in range(3)],
        "arm": [(15 + i, 22 + i) for i in range(7)],
        "leg": [(i, 6 + i) for i in range(6)],
    }


def mirror_asymmetry(x: np.ndarray, y: np.ndarray, pairs: list[tuple[int, int]]) -> list[dict]:
    """``x`` = dof of the clips, ``y`` = dof of their mirrored twins (same frames).  The mirror sign comes from the data itself."""
    out = []
    for j, jp in pairs:
        s = float(np.sign(np.sum(x[:, j] * y[:, jp])) or 1.0)
        err = x[:, j] - s * y[:, jp]
        out.append({
            "sign": s,
            "bias": float(err.mean()),
            "std": float(err.std()),
            "rmse": float(np.sqrt(np.mean(err**2))),
            "gt_0p3": float(np.mean(np.abs(err) > 0.3)),
            "gt_0p1": float(np.mean(np.abs(err) > 0.1)),
        })
    return out


def limit_occupancy(q: np.ndarray, lo: float, hi: float, near: float = 0.03, soft_factor: float = 0.9) -> dict:
    """Share of frames within ``near`` rad of either limit, and outside the soft range (``soft_factor`` of the range)."""
    cut = (1.0 - soft_factor) / 2.0 * (hi - lo)
    return {
        "near_lo": float(np.mean(q < lo + near)),
        "near_hi": float(np.mean(q > hi - near)),
        "outside_soft": float(np.mean((q < lo + cut) | (q > hi - cut))),
    }


def affine_fit(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    """y ~ gain * x + offset: (gain, offset, residual RMSE)."""
    a = np.stack([x, np.ones(len(x))], axis=1)
    (gain, off), *_ = np.linalg.lstsq(a, y, rcond=None)
    return float(gain), float(off), float(np.sqrt(np.mean((y - (gain * x + off)) ** 2)))


# ---------------------------------------------------------------------------------------------------------- data
def _load(libs, name):
    from gear_sonic.data_process.compare_smpl_vs_a3 import _entry

    return _entry(libs[name])


def _sample(names: list[str], n: int, seed: int) -> list[str]:
    rnd = random.Random(seed)
    return rnd.sample(names, min(n, len(names)))


def _twin_names(lib: dict) -> list[str]:
    return sorted(n for n in lib if n + "_M" in lib and not n.endswith("_M"))


# --------------------------------------------------------------------------------------------------- the checks
def check_root_ori(a3, g1, smpl, n, seed):
    from gear_sonic.data_process.compare_smpl_vs_a3 import TRAIN_FPS, smpl_root_world

    print("\n== root_ori: full rotation quat_inv(robot pelvis) * smpl_root (training smpl_root_ori_b), per frame, in degrees")
    for label, lib in (("A3", a3), ("G1", g1)):
        if lib is None:
            continue
        names = _sample(sorted(set(lib) & set(smpl)), n, seed)
        pit, rol, ang = [], [], []
        for name in names:
            s, r = _load(smpl, name), _load(lib, name)
            t = np.arange(r["root_rot"].shape[0]) / float(r["fps"])
            idx = np.clip(np.round(t * TRAIN_FPS).astype(int), 0, s["pose_aa"].shape[0] - 1)
            rel = Rotation.from_quat(r["root_rot"]).inv() * smpl_root_world(s["pose_aa"])[idx]
            _, p, ro = rel.as_euler("ZYX").T
            pit.append(np.degrees(p))
            rol.append(np.degrees(ro))
            ang.append(np.degrees(rel.magnitude()))
        pit, rol, ang = (np.concatenate(v) for v in (pit, rol, ang))
        print(f"  {label}: pitch mean {pit.mean():+.2f} std {pit.std():.2f} | roll mean {rol.mean():+.2f} std {rol.std():.2f} | "
              f"geodesic median {np.median(ang):.2f} p95 {np.percentile(ang, 95):.1f}   ({len(names)} clips)")


def check_mirror(a3, g1, n, seed):
    print("\n== mirror: |joint(X) - sign * counterpart(X_M)| over the paired twins; bias = constant left/right calibration error")
    res, clip_stats = {}, {}
    for label, kind, lib in (("A3", "a3", a3), ("G1", "g1", g1)):
        if lib is None:
            continue
        names = _sample(_twin_names(lib), n, seed)
        xs, ys = [], []
        for name in names:
            da = np.asarray(_load(lib, name)["dof"], float)
            db = np.asarray(_load(lib, name + "_M")["dof"], float)
            t = min(len(da), len(db))
            xs.append(da[:t])
            ys.append(db[:t])
        x, y = np.concatenate(xs), np.concatenate(ys)
        pairs = mirror_pairs(kind)
        res[label] = {grp: mirror_asymmetry(x, y, pl) for grp, pl in pairs.items()}
        arm = [(j, jp) for j, jp in pairs["arm"][:4]]  # shoulders + elbow: per-clip scatter (a few clips with another IK branch)
        signs = [r["sign"] for r in res[label]["arm"][:4]]
        per_clip = np.array([np.sqrt(np.mean([np.mean((a[:, j] - s * b[:, jp]) ** 2) for (j, jp), s in zip(arm, signs)])) for a, b in zip(xs, ys)])
        clip_stats[label] = (np.median(per_clip), np.percentile(per_clip, 90), np.percentile(per_clip, 99), float(np.mean(per_clip > 0.15)))
    print(f"  {'joint':16s} {'A3 rmse':>8s} {'A3 bias':>8s} {'>0.1':>6s} | {'G1 rmse':>8s} {'G1 bias':>8s} {'>0.1':>6s}")
    for k, nm in enumerate(ARM_NAMES):
        a = res["A3"]["arm"][k]
        g = res.get("G1", {}).get("arm", [None] * 7)[k]
        gs = f"{g['rmse']:8.3f} {g['bias']:+8.3f} {100 * g['gt_0p1']:5.1f}%" if g else ""
        print(f"  {nm:16s} {a['rmse']:8.3f} {a['bias']:+8.3f} {100 * a['gt_0p1']:5.1f}% | {gs}")
    for grp in ("waist", "leg"):
        a = np.mean([d["rmse"] for d in res["A3"][grp]])
        g = np.mean([d["rmse"] for d in res["G1"][grp]]) if "G1" in res else float("nan")
        ab = np.mean([abs(d["bias"]) for d in res["A3"][grp]])
        print(f"  {grp + ' (mean)':16s} {a:8.3f} {ab:8.3f} (|bias|)        | {g:8.3f}")
    for label, (p50, p90, p99, frac) in clip_stats.items():
        print(f"  per clip, shoulders+elbow RMSE {label}: median {p50:.3f} p90 {p90:.3f} p99 {p99:.3f}; clips above 0.15 rad: {100 * frac:.1f}%")


def check_limits(a3, smpl, n, seed):
    import mujoco as mj

    print("\n== limits: share of reference frames near a joint limit / outside the 0.9 soft range (arms)")
    model = mj.MjModel.from_xml_path(str(MJCF))
    names = _sample(sorted(set(a3) & set(smpl)), n, seed)
    d = np.concatenate([np.asarray(_load(a3, name)["dof"], float) for name in names])
    for k, jn in enumerate(JOINT_NAMES_A3_ARMS):
        lo, hi = model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, jn)]
        o = limit_occupancy(d[:, 3 + k], float(lo), float(hi))
        print(f"  {jn:28s} [{lo:+.3f},{hi:+.3f}]  near lower {100 * o['near_lo']:4.1f}%  near upper {100 * o['near_hi']:4.1f}%  "
              f"outside soft {100 * o['outside_soft']:4.1f}%")


def g1_wrist_formula(pose72: np.ndarray) -> np.ndarray:
    """G1's analytic SMPL -> wrist angles (``pico_manager_thread_server.py``, "directly setting the joint position"), A3 joint view.

    ``pose72`` (T, 72) SMPL axis-angle -> (T, 6): left roll/pitch/yaw, right roll/pitch/yaw.  The signs are the ones the G1 deploy uses;
    which of them carries over to the A3 is what the affine fit below says.
    """
    from gear_sonic.trl.utils.rotation_conversion import decompose_rotation_aa

    body = pose72[:, 3:66].reshape(-1, 21, 3)
    axis = np.array([0, 1, 0])
    _, lsw = decompose_rotation_aa(body[:, 17], axis)
    _, rsw = decompose_rotation_aa(body[:, 18], axis)
    le = Rotation.from_quat(lsw[:, [1, 2, 3, 0]]).as_euler("XYZ")
    re_ = Rotation.from_quat(rsw[:, [1, 2, 3, 0]]).as_euler("XYZ")
    lw = Rotation.from_rotvec(body[:, 19]).as_euler("XYZ")
    rw = Rotation.from_rotvec(body[:, 20]).as_euler("XYZ")
    return np.stack([le[:, 0] + lw[:, 0], lw[:, 1], le[:, 2] + lw[:, 2], -(re_[:, 0] + rw[:, 0]), -rw[:, 1], re_[:, 2] + rw[:, 2]], axis=1)


def check_link_asym(a3, n, seed):
    from gear_sonic.data_process.calibrate_a3_lr_symmetry import A3Kinematics, summarize, measure

    print("\n== link_asym: orientation of the mirrored link (IK space): constant offset of the achieved link vs the mirrored other side")
    pairs = [(np.asarray(_load(a3, k)["dof"], float), np.asarray(_load(a3, k + "_M")["dof"], float)) for k in _sample(_twin_names(a3), n, seed)]
    for (side, link), s in summarize(measure(A3Kinematics(), pairs)).items():
        if side == "left":
            print(f"  {link:13s} offset {s['deg']:5.1f} deg  scatter median {s['scatter_median']:.1f} p90 {s['scatter_p90']:.1f}")


def check_wrist(a3, smpl, n, seed):
    print("\n== wrist: G1's analytic SMPL -> wrist formula against the A3 library wrists: library ~ gain * formula + offset")
    names = _sample(sorted(set(a3) & set(smpl)), n, seed)
    g_all, q_all = [], []
    for name in names:
        s, r = _load(smpl, name), _load(a3, name)
        pose = np.asarray(s["pose_aa"], float)[:1500]
        lib = np.asarray(r["dof"], float)
        tt = np.arange(len(pose)) / 50.0
        g_all.append(g1_wrist_formula(pose))
        q_all.append(np.stack([np.interp(tt, np.arange(len(lib)) / float(r["fps"]), lib[:, j]) for j in A3_WRIST_COLS], axis=1))
    g, q = np.concatenate(g_all), np.concatenate(q_all)
    for j, nm in enumerate(["L roll", "L pitch", "L yaw", "R roll", "R pitch", "R yaw"]):
        gain, off, res = affine_fit(g[:, j], q[:, j])
        rm = float(np.sqrt(np.mean((g[:, j] - q[:, j]) ** 2)))
        print(f"  {nm:8s} corr {np.corrcoef(g[:, j], q[:, j])[0, 1]:+.2f}  RMSE as-is {rm:.3f}  | gain {gain:+.2f} offset {off:+.3f} residual RMSE {res:.3f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--a3", required=True, help="A3 motion library (a3_29 format), e.g. data/motion_lib_a3_29d/robot_filtered_v15_clean")
    ap.add_argument("--smpl", required=True, help="SMPL data, e.g. data/smpl_filtered")
    ap.add_argument("--g1", default=None, help="G1 library as the baseline, e.g. data/motion_lib_bones_seed/robot_all")
    ap.add_argument("--sample", type=int, default=250, help="clips per check")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--checks", nargs="+", default=["root_ori", "mirror", "limits", "wrist", "link_asym"], choices=["root_ori", "mirror", "limits", "wrist", "link_asym"])
    args = ap.parse_args()

    from gear_sonic.data_process.compare_smpl_vs_a3 import index

    a3 = index(Path(args.a3))
    smpl = index(Path(args.smpl))
    g1 = index(Path(args.g1)) if args.g1 else None
    print(f"A3 clips {len(a3)}, SMPL clips {len(smpl)}, paired {len(set(a3) & set(smpl))}" + (f", G1 clips {len(g1)}" if g1 else ""))
    if "root_ori" in args.checks:
        check_root_ori(a3, g1 and {k: v for k, v in g1.items() if k in a3}, smpl, args.sample, args.seed)
    if "mirror" in args.checks:
        check_mirror(a3, g1, args.sample, args.seed)
    if "limits" in args.checks:
        check_limits(a3, smpl, args.sample, args.seed)
    if "link_asym" in args.checks:
        check_link_asym(a3, args.sample, args.seed)
    if "wrist" in args.checks:
        check_wrist(a3, smpl, args.sample, args.seed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
