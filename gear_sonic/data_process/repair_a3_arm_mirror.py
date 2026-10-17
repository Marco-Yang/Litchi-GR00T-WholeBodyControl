#!/usr/bin/env python3
# ruff: noqa: T201
"""Repair the arm segments of the A3 library where the two mirror twins disagree because one IK solution went wrong.

Every clip ``X`` has an exact mirror twin ``X_M``, so the arm joints of ``X`` must be the mirror image of the opposite arm of ``X_M``.
In about 2 % of the pairs they are not (shoulder pitch / yaw differ by 0.7-1 rad over stretches of the clip, mostly arms-overhead
motions): in most of those frames one of the two arms follows the human (SMPL) clearly better than the other -- the other IK solution fell
into a local minimum.  The SMPL encoder is trained on the library, so it would see two different joint targets for the same (mirrored)
human arm.  This tool makes the twins consistent without re-running the IK:

  1. frames where ``max |q_X(arm) - sign * q_XM(opposite arm)|`` over shoulder pitch / roll / yaw / elbow exceeds ``--threshold`` mark a segment
     (padded, close segments merged);
  2. in a segment, the arm whose upper-arm / forearm directions are closer to the SMPL human (mean over the segment) is the source; the other
     twin's opposite arm (all 7 joints) is replaced by the source's mirror image;
  3. the replacement fades in and out over ``--ramp`` frames inside the padding, where the two solutions already agree, so no joint jumps.

Only arm columns of one twin change; legs, waist, root and the other twin are untouched, and ``pose_aa`` is rebuilt for the changed joints.
Unchanged clips are hard-linked into the output directory (no extra disk space).

    python gear_sonic/data_process/repair_a3_arm_mirror.py --lib data/motion_lib_a3_29d/robot_filtered_v15_clean \\
        --smpl data/smpl_filtered --output data/motion_lib_a3_29d/robot_filtered_v15_clean_armfix --workers 16
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

import joblib
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

SMPL_ARM = {"left": (16, 18, 20), "right": (17, 19, 21)}  # shoulder, elbow, wrist
ARM_LINKS = ("{s}_shoulder_pitch_Link", "{s}_elbow_Link", "{s}_wrist_roll_Link")
DETECT_JOINTS = 4  # shoulder pitch / roll / yaw, elbow (the first four columns of a side)


# ----------------------------------------------------------------------------------------------------- pure logic
def segments_from_mask(mask: np.ndarray, pad: int, merge_gap: int) -> list[tuple[int, int]]:
    """Runs of True in ``mask`` as half-open ``[start, end)`` intervals, padded by ``pad`` and merged when closer than ``merge_gap``."""
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return []
    runs, start, prev = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - prev > merge_gap:
            runs.append((start, prev))
            start = i
        prev = i
    runs.append((start, prev))
    t = len(mask)
    return [(max(0, a - pad), min(t, b + pad + 1)) for a, b in runs]


def segment_weights(t: int, segs: list[tuple[int, int]], core: list[tuple[int, int]], ramp: int) -> np.ndarray:
    """1 on the core of each segment, linear ramp to 0 across ``ramp`` frames inside the padding, 0 elsewhere."""
    w = np.zeros(t)
    for (a, b), (ca, cb) in zip(segs, core):
        seg = np.zeros(t)
        seg[ca:cb] = 1.0
        for k in range(1, ramp + 1):  # rising edge before the core, falling edge after it
            if ca - k >= a:
                seg[ca - k] = max(seg[ca - k], 1.0 - k / (ramp + 1))
            if cb - 1 + k < b:
                seg[cb - 1 + k] = max(seg[cb - 1 + k], 1.0 - k / (ramp + 1))
        w = np.maximum(w, seg)
    return w


def repair_arm(q_a: np.ndarray, q_b: np.ndarray, ideal_a_from_b: np.ndarray, ideal_b_from_a: np.ndarray, err_a: np.ndarray, err_b: np.ndarray,
               cols_a: list[int], cols_b: list[int], threshold: float, pad: int, merge_gap: int, ramp: int):
    """One mirror-corresponding arm pair: ``q_a[:, cols_a]`` (clip A) against ``q_b[:, cols_b]`` (clip B, the opposite arm of the twin).

    ``ideal_a_from_b`` is clip B's joint trajectory mirrored into A's side ((T, 29) from ``A3Kinematics.mirrored``), and the other way round.
    Returns the repaired copies of ``q_a`` / ``q_b`` and a list of ``(start, end, source, err_a, err_b)`` segments.
    """
    d = np.abs(q_a[:, cols_a[:DETECT_JOINTS]] - ideal_a_from_b[:, cols_a[:DETECT_JOINTS]]).max(axis=1)
    mask = d > threshold
    segs = segments_from_mask(mask, pad, merge_gap)
    out_a, out_b, info = q_a.copy(), q_b.copy(), []
    for a, b in segs:
        core = np.flatnonzero(mask[a:b]) + a
        ca, cb = int(core.min()), int(core.max()) + 1
        ea, eb = float(np.mean(err_a[ca:cb])), float(np.mean(err_b[ca:cb]))
        w = segment_weights(len(q_a), [(a, b)], [(ca, cb)], ramp)[:, None]
        if ea <= eb:  # A follows the human better (ties: A): B's arm becomes the mirror of A's
            out_b[:, cols_b] = (1 - w) * out_b[:, cols_b] + w * ideal_b_from_a[:, cols_b]
            info.append((a, b, "A", ea, eb))
        else:
            out_a[:, cols_a] = (1 - w) * out_a[:, cols_a] + w * ideal_a_from_b[:, cols_a]
            info.append((a, b, "B", ea, eb))
    return out_a, out_b, info


# ------------------------------------------------------------------------------------------------- data dependent
_STATE: dict = {}


def _kin():
    if "kin" not in _STATE:
        from gear_sonic.data_process.calibrate_a3_lr_symmetry import A3Kinematics

        _STATE["kin"] = A3Kinematics()
    return _STATE["kin"]


def _load_robot(path):
    raw = joblib.load(path)
    key = next(iter(raw))
    return raw, raw[key]


def _load_smpl(path):
    d = joblib.load(path)
    return d if "pose_aa" in d else next(iter(d.values()))


def arm_direction_errors(kin, entry: dict, smpl_entry: dict) -> dict[str, np.ndarray]:
    """Per frame, per side: the larger of the upper-arm / forearm direction errors (degrees) against the SMPL human."""
    import mujoco as mj

    from gear_sonic.data_process.compare_smpl_vs_a3 import smpl_world

    q = np.asarray(entry["dof"], float)
    t = len(q)
    idx = np.clip(np.round(np.arange(t) / float(entry["fps"]) * 50).astype(int), 0, len(smpl_entry["smpl_joints"]) - 1)
    human = smpl_world(smpl_entry)[0][idx]
    rot = Rotation.from_quat(np.asarray(entry["root_rot"], float)).as_matrix()
    mdl, dat = kin.model, kin.data
    out = {}

    def angle(u, v):
        u = u / np.linalg.norm(u, axis=-1, keepdims=True)
        v = v / np.linalg.norm(v, axis=-1, keepdims=True)
        return np.degrees(np.arccos(np.clip((u * v).sum(-1), -1, 1)))

    for side in ("left", "right"):
        ids = [mj.mj_name2id(mdl, mj.mjtObj.mjOBJ_BODY, n.format(s=side)) for n in ARM_LINKS]
        qpos = np.zeros(mdl.nq)
        qpos[3] = 1.0
        pos = np.empty((t, 3, 3))
        for k in range(t):
            qpos[kin._qadr] = q[k]
            dat.qpos[:] = qpos
            mj.mj_kinematics(mdl, dat)
            pos[k] = dat.xpos[ids]
        pos = np.einsum("tij,tbj->tbi", rot, pos)
        a, b, c = SMPL_ARM[side]
        out[side] = np.maximum(angle(pos[:, 1] - pos[:, 0], human[:, b] - human[:, a]), angle(pos[:, 2] - pos[:, 1], human[:, c] - human[:, b]))
    return out


def rebuild_pose_aa(entry: dict, cols: list[int]) -> None:
    """Refresh ``pose_aa`` for the changed policy-joint columns (axis * angle, as ``convert_gmr_a3_to_motion_lib`` writes it)."""
    from gear_sonic.data_process.convert_gmr_a3_to_motion_lib import _spec

    spec, _, body = _spec()
    for c in cols:
        j = int(body[c])
        entry["pose_aa"][:, j + 1] = entry["dof"][:, c, None] * spec.axes[j][None, :]


def _limits():
    from gear_sonic.data_process.convert_gmr_a3_to_motion_lib import _spec

    spec, _, body = _spec()
    return spec.limits[body, 0], spec.limits[body, 1]


def process_pair(job):
    """Returns ``(name, status, raw_a, raw_b, infos)``; ``raw_*`` is the repaired pkl content of a changed clip, else None."""
    name, path_a, path_b, smpl_a, smpl_b, threshold, pad, merge_gap, ramp = job
    kin = _kin()
    raw_a, ea = _load_robot(path_a)
    raw_b, eb = _load_robot(path_b)
    qa_full, qb_full = np.asarray(ea["dof"], float), np.asarray(eb["dof"], float)
    t = min(len(qa_full), len(qb_full))  # the twins differ by at most one frame in length; compare the common prefix
    qa, qb = qa_full[:t], qb_full[:t]
    mirror_b, mirror_a = kin.mirrored(qb), kin.mirrored(qa)  # each clip mirrored into the other's left / right
    sides = (("left", "right"), ("right", "left"))  # (A's arm, B's opposite arm)
    if not any((np.abs(qa[:, kin.side_cols(sa)[:DETECT_JOINTS]] - mirror_b[:, kin.side_cols(sa)[:DETECT_JOINTS]]).max(axis=1) > threshold).any() for sa, _ in sides):
        return name, "consistent", None, None, []
    err_a = {k: v[:t] for k, v in arm_direction_errors(kin, ea, _load_smpl(smpl_a)).items()}
    err_b = {k: v[:t] for k, v in arm_direction_errors(kin, eb, _load_smpl(smpl_b)).items()}
    new_a, new_b, infos = qa.copy(), qb.copy(), []
    for sa, sb in sides:
        ca, cb = kin.side_cols(sa), kin.side_cols(sb)
        res_a, res_b, info = repair_arm(qa, qb, mirror_b, mirror_a, err_a[sa], err_b[sb], ca, cb, threshold, pad, merge_gap, ramp)
        new_a[:, ca], new_b[:, cb] = res_a[:, ca], res_b[:, cb]
        infos += [(sa, sb, *x) for x in info]
    new_a, new_b = np.concatenate([new_a, qa_full[t:]]), np.concatenate([new_b, qb_full[t:]])  # the extra last frame of the longer twin stays
    qa, qb = qa_full, qb_full
    lo, hi = _limits()
    out = []
    for raw, entry, q_new, q_old in ((raw_a, ea, new_a, qa), (raw_b, eb, new_b, qb)):
        cols = [c for c in range(29) if np.abs(q_new[:, c] - q_old[:, c]).max() > 1e-9]
        if not cols:
            out.append(None)
            continue
        entry["dof"] = np.clip(q_new, lo, hi).astype(np.float32)
        rebuild_pose_aa(entry, cols)
        out.append(raw)
    return name, "repaired", out[0], out[1], infos


def _write_atomic(raw: dict, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_suffix(dst.suffix + ".tmp")
    joblib.dump(raw, tmp)
    os.replace(tmp, dst)


def main() -> int:
    import json
    from multiprocessing import Pool

    from gear_sonic.data_process.compare_smpl_vs_a3 import index

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lib", type=Path, required=True)
    ap.add_argument("--smpl", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True, help="new library directory (unchanged clips are hard-linked from --lib)")
    ap.add_argument("--threshold", type=float, default=0.3, help="mirror disagreement (rad, max over shoulder pitch/roll/yaw/elbow) that opens a segment")
    ap.add_argument("--ramp", type=int, default=6, help="frames over which a replacement fades in / out")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--limit", type=int, default=0, help="only the first N twin pairs (smoke test)")
    ap.add_argument("--dry_run", action="store_true", help="analyse only, write nothing")
    args = ap.parse_args()

    lib, smpl = index(args.lib), index(args.smpl)
    pad, merge_gap = args.ramp + 2, 2 * (args.ramp + 2)
    pairs = sorted(n for n in lib if n + "_M" in lib and not n.endswith("_M") and n in smpl and n + "_M" in smpl)
    if args.limit:
        pairs = pairs[: args.limit]
    jobs = [(n, lib[n], lib[n + "_M"], smpl[n], smpl[n + "_M"], args.threshold, pad, merge_gap, args.ramp) for n in pairs]
    print(f"{len(lib)} clips, {len(pairs)} twin pairs with SMPL data; threshold {args.threshold} rad, ramp {args.ramp}")
    changed: dict[str, dict] = {}
    report, counts = [], {}
    with Pool(args.workers) as pool:
        for k, (name, status, raw_a, raw_b, infos) in enumerate(pool.imap_unordered(process_pair, jobs, chunksize=16), 1):
            counts[status] = counts.get(status, 0) + 1
            if status == "repaired":
                for clip, raw in ((name, raw_a), (name + "_M", raw_b)):
                    if raw is not None:
                        changed[clip] = raw
                report.append({"pair": name, "segments": [{"arm_of_X": sa, "arm_of_X_M": sb, "start": int(a), "end": int(b), "source": src, "err_deg_X": round(ea, 1), "err_deg_X_M": round(eb, 1)} for sa, sb, a, b, src, ea, eb in infos]})
            if k % 5000 == 0:
                print(f"  [{k}/{len(jobs)}] {counts}", flush=True)
    print("pairs:", counts, "| clips changed:", len(changed))
    if args.dry_run:
        return 0
    n_link = 0
    for name, src in lib.items():
        dst = args.output / src.relative_to(args.lib)
        if name in changed:
            _write_atomic(changed[name], dst)
        else:
            dst.parent.mkdir(parents=True, exist_ok=True)
            if dst.exists():
                dst.unlink()
            os.link(src, dst)
            n_link += 1
    (args.output.parent / (args.output.name + ".arm_repair_report.json")).write_text(json.dumps({"counts": counts, "changed_clips": sorted(changed), "pairs": report}, indent=1))
    print(f"wrote {len(changed)} repaired clips, hard-linked {n_link} unchanged clips -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
