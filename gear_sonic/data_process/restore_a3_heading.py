#!/usr/bin/env python3
# ruff: noqa: T201
"""Turn an A3 / A3 Ultra retarget library back to the capture heading of its clips (the frame of the SMPL data).

``batch_retarget_bvh_soma_a3ultra.py`` solves the IK on every BVH turned to face +X at frame 0 (``_yaw_f0``), so each clip of
the library starts at heading ~0 whatever way the performer faced.  The Bones-SEED SMPL data (and the G1 library) keep the
capture heading, and the SMPL encoder's ``smpl_root_ori_b = quat_inv(robot_anchor_quat_w) * smpl_root_quat_w`` needs both
in one world frame.  The turn is a rigid rotation of the whole scene about Z, so it is undone without redoing the IK:
for every clip, ``root_trans_offset`` (x, y), ``root_rot`` and ``pose_aa[:, 0]`` are rotated by ``+_yaw_f0``; joint angles,
heights and everything else stay as they are.  ``_yaw_f0`` comes from the clip's own BVH (one pass over the archive,
header + first frame only), not from the SMPL data, so the result is checked against the SMPL data rather than fitted to it.

    python gear_sonic/data_process/restore_a3_heading.py \\
        --library data/motion_lib_a3_29d/robot_filtered_v15_clean --tar data/bones_seed/soma_proportional.tar.gz \\
        --output data/motion_lib_a3_29d/robot_filtered_v15_clean_heading
    python gear_sonic/data_process/compare_smpl_vs_a3.py --stats --robot <output>        # offset to the SMPL data
"""

from __future__ import annotations

import argparse
import csv
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
import os
from pathlib import Path
import sys
import tarfile
import time

import joblib
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def rotate_entry(entry: dict, psi: float) -> dict:
    """``a3_29`` / A3 Ultra motion-lib entry with the whole scene turned by ``psi`` about +Z (joint angles untouched)."""
    out = dict(entry)
    rz = Rotation.from_rotvec([0.0, 0.0, psi])
    pos = np.asarray(entry["root_trans_offset"], dtype=np.float64)
    out["root_trans_offset"] = rz.apply(pos).astype(entry["root_trans_offset"].dtype)
    rot = rz * Rotation.from_quat(np.asarray(entry["root_rot"], dtype=np.float64))  # xyzw, world-frame turn: left multiply
    out["root_rot"] = rot.as_quat().astype(entry["root_rot"].dtype)
    pose = np.array(entry["pose_aa"], copy=True)
    pose[:, 0] = rot.as_rotvec().astype(pose.dtype)  # pose_aa[:, 0] is the root rotation
    out["pose_aa"] = pose
    return out


def _head_of_bvh(raw: bytes) -> str:
    """The hierarchy plus the first frame of a BVH (everything ``bvh_heading_f0`` reads), as text."""
    i = raw.index(b"MOTION")
    end = i
    for _ in range(4):  # "MOTION", "Frames:", "Frame Time:", first frame
        end = raw.index(b"\n", end) + 1
    return raw[:end].decode()


def _heading_job(job):
    from gear_sonic.data_process.batch_retarget_bvh_soma_a3ultra import bvh_heading_f0

    name, head = job
    try:
        return name, bvh_heading_f0(text=head), ""
    except Exception as exc:  # noqa: BLE001
        return name, float("nan"), f"{type(exc).__name__}: {exc}"


def capture_headings(tar_path: Path, names: set[str], workers: int) -> dict[str, float]:
    """One pass over the BVH archive -> {clip: capture heading (rad)} for the requested clips."""
    out: dict[str, float] = {}
    errors: list[tuple[str, str]] = []
    t0 = time.time()

    def tally(done):
        for fut in done:
            name, yaw, err = fut.result()
            if err:
                errors.append((name, err))
            else:
                out[name] = yaw
            if (len(out) + len(errors)) % 5000 == 0:
                print(f"  headings: {len(out)} / {len(names)} ({(time.time() - t0) / 60:.1f} min)", flush=True)

    with ProcessPoolExecutor(max_workers=workers) as ex, tarfile.open(tar_path, "r|gz") as tf:
        pending = set()
        seen = 0
        for member in tf:
            stem = Path(member.name).stem
            if not (member.isfile() and member.name.endswith(".bvh") and stem in names):
                continue
            pending.add(ex.submit(_heading_job, (stem, _head_of_bvh(tf.extractfile(member).read()))))
            seen += 1
            if len(pending) >= 8 * workers:
                done, pending = wait(pending, return_when=FIRST_COMPLETED)
                tally(done)
            if seen == len(names):
                break
        tally(wait(pending).done)
    if errors:
        print(f"{len(errors)} BVH could not be read, e.g. {errors[:3]}")
    return out


def _rotate_job(job):
    src, dst, psi = job
    try:
        data = joblib.load(src)
        (key, entry), = data.items()
        Path(dst).parent.mkdir(parents=True, exist_ok=True)
        tmp = f"{dst}.tmp"
        joblib.dump({key: rotate_entry(entry, psi)}, tmp)
        os.replace(tmp, dst)
        return True
    except Exception as exc:  # noqa: BLE001
        print(f"FAILED {src}: {exc}", flush=True)
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--library", type=Path, required=True, help="retarget library to turn (a3_29 or A3 Ultra format)")
    ap.add_argument("--tar", type=Path, default=PROJECT_ROOT / "data/bones_seed/soma_proportional.tar.gz")
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--headings_csv", type=Path, help="cache of the per-clip capture headings (read if it exists, else written)")
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--max_files", type=int, default=0, help="only the first N clips (testing)")
    args = ap.parse_args()

    lib = {p.stem: p for p in sorted(args.library.rglob("*.pkl")) if p.name != "metadata.pkl"}
    if args.max_files:
        lib = dict(list(lib.items())[: args.max_files])
    print(f"{len(lib)} clips in {args.library}", flush=True)

    csv_path = args.headings_csv or args.output.parent / f"{args.output.name}.capture_heading.csv"
    if csv_path.exists():
        with open(csv_path) as f:
            heading = {r["clip"]: float(r["yaw_f0_rad"]) for r in csv.DictReader(f)}
        print(f"capture headings from {csv_path} ({len(heading)})", flush=True)
    else:
        heading = capture_headings(args.tar, set(lib), args.workers)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["clip", "yaw_f0_rad", "yaw_f0_deg"])
            for k in sorted(heading):
                w.writerow([k, f"{heading[k]:.9f}", f"{np.degrees(heading[k]):.4f}"])
        print(f"capture headings -> {csv_path}", flush=True)
    missing = sorted(set(lib) - set(heading))
    if missing:
        print(f"no capture heading for {len(missing)} clips (e.g. {missing[:3]}); nothing written -- a mixed library is worse than either")
        return 1

    jobs = [(str(p), str(args.output / p.relative_to(args.library)), heading[n]) for n, p in lib.items()]
    t0 = time.time()
    ok = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        pending = set()
        for i, job in enumerate(jobs, 1):
            if len(pending) >= 4 * args.workers:
                done, pending = wait(pending, return_when=FIRST_COMPLETED)
                ok += sum(f.result() for f in done)
            pending.add(ex.submit(_rotate_job, job))
            if i % 10000 == 0:
                print(f"  submitted {i} / {len(jobs)} ({(time.time() - t0) / 60:.1f} min)", flush=True)
        ok += sum(f.result() for f in wait(pending).done)
    print(f"turned {ok} / {len(jobs)} clips in {(time.time() - t0) / 60:.1f} min -> {args.output}")
    return 0 if ok == len(jobs) else 1


if __name__ == "__main__":
    raise SystemExit(main())
