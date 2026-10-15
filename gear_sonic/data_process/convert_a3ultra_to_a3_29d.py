#!/usr/bin/env python3
# ruff: noqa: T201
"""Batch-convert the A3 Ultra (T3D0) motion library into the Litchi A3 ``a3_29`` motion-library format.

This is the FORMAT conversion (seconds): the joint angles of the already retargeted Ultra library are re-expressed on the
A3.  The training library itself is made by re-running the retargeting on the A3 with the A3 parameters
(``batch_retarget_bvh_soma_a3ultra.py --robot a3``, ~2 h); the two agree to 0.0011 rad RMSE on average over all 129,116
clips (docs/workflow/a3_integration_plan.md section 17), so this converter stays as a quick path and as a cross-check.

Why: the downloaded 035 policy was trained on the A3 URDF (``a3_t2d5.xml``, 29 policy joints) with domain
randomization tuned for A3 *and* A3 Ultra, and that is the recipe for anything we train for the A3 Ultra as well
("convert and train with the a3 parameters, randomize over A3 and A3 Ultra").  The retargeted Bones-SEED library we
have (``data/motion_lib_a3ultra/robot_filtered_v15_clean``) was written for the 63-DOF A3 Ultra model, so it has to be
brought to the A3 contract before the 035 training path (``train_agent_trl.py --robot a3|a3_ultra``,
``train_a3_035_fromscratch.sh``) can read it.

Source entry (see ``convert_gmr_a3ultra_to_motion_lib.py``)::

    root_trans_offset (T, 3)   pose_aa (T, 64, 3)   dof (T, 31)   root_rot (T, 4) xyzw   smpl_joints (T, 24, 3)   fps

Target entry (Litchi ``convert_a3_pkl_to_motion_lib_29d.py`` / ``convert_a3_csv_to_motion_lib.py --robot a3_29``)::

    root_trans_offset (T, 3)   pose_aa (T, 32, 3)   dof (T, 29)   root_rot (T, 4) xyzw   smpl_joints (T, 24, 3)   fps

with ``pose_aa`` in the body DFS order of ``a3_t2d5.xml`` (head bodies zero, they are not policy joints) and ``dof`` in
the order of its 29 non-head joints.  What the conversion does, all derived from the two MJCFs (nothing hard-coded):

1. **Joint remap by name.**  The 29 body joints have the same names, axes and zero pose in both models
   (``head_pitch_joint`` is ``ultra_head_pitch_joint`` on the Ultra); forward kinematics of the shared bodies agrees to
   <= 1.5 mm.  The two head joints and the 32 finger joints are dropped.
2. **A3 joint limits.**  ``dof`` is clipped to the A3 URDF ranges.  The Ultra's are wider only for the knees (145 deg vs
   143 deg); its hip-roll range is narrower, so nothing else can be out of range.
3. **Ground contact.**  The A3 sole sits 4.54 mm closer to the ankle than the Ultra's (mesh foot vs sphere feet), so the
   root height is lowered by that amount: feet that touched the floor on the Ultra still touch it on the A3.
4. ``root_trans_offset`` x/y, ``root_rot``, ``smpl_joints`` (zero placeholders) and the frame rate are kept.

The output keeps the directory layout and file names (the loader keys a motion by its file name) and each file is a
one-item dict ``{name: entry}`` like the source.

Usage:
    python gear_sonic/data_process/convert_a3ultra_to_a3_29d.py \\
        --input data/motion_lib_a3ultra/robot_filtered_v15_clean \\
        --output data/motion_lib_a3_29d/converted_from_a3ultra_v15_clean --num_workers 16
    # then:  MOTION_FILE=$PWD/data/motion_lib_a3_29d/converted_from_a3ultra_v15_clean bash train_a3_035_fromscratch.sh --preflight
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import multiprocessing as mp
import os
from pathlib import Path
import time

import joblib
import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[2]
MJCF_DIR = REPO_ROOT / "gear_sonic" / "data" / "assets" / "robot_description" / "mjcf"
DEFAULT_A3_MJCF = MJCF_DIR / "a3_t2d5.xml"
DEFAULT_A3U_MJCF = MJCF_DIR / "a3_ultra_t3d0.xml"
DEFAULT_INPUT = REPO_ROOT / "data" / "motion_lib_a3ultra" / "robot_filtered_v15_clean"
DEFAULT_OUTPUT = REPO_ROOT / "data" / "motion_lib_a3_29d" / "converted_from_a3ultra_v15_clean"

HEAD_JOINTS = ("head_yaw_joint", "head_pitch_joint")
JOINT_ALIASES = {"ultra_head_pitch_joint": "head_pitch_joint"}  # A3 Ultra name -> A3 name
SOURCE_NUM_DOF = 31  # the Ultra library stores the 31 actuated joints (no fingers)
SOURCE_NUM_BODIES = 64  # pelvis + 31 actuated + 32 finger bodies


@dataclass(frozen=True)
class RobotSpec:
    """Body / joint order, axes and limits of one MJCF (body DFS order, like Litchi's ``load_a3_mjcf_spec``)."""

    body_names: list[str]
    joint_names: list[str]  # joint i belongs to body i + 1
    axes: np.ndarray  # (num_joints, 3)
    limits: np.ndarray  # (num_joints, 2) radians, (-inf, inf) when the joint is unlimited

    @property
    def joint_index(self) -> dict[str, int]:
        return {JOINT_ALIASES.get(n, n): i for i, n in enumerate(self.joint_names)}


def load_spec(mjcf_path: Path) -> RobotSpec:
    """Bodies / joints in MJCF depth-first order, read from the compiled model (ranges and units are resolved)."""
    import mujoco as mj

    m = mj.MjModel.from_xml_path(str(mjcf_path))
    bodies = [mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, i) for i in range(1, m.nbody)]  # body 0 is the world
    joints, axes, limits, owner = [], [], [], []
    for j in range(m.njnt):
        if m.jnt_type[j] == mj.mjtJoint.mjJNT_FREE:
            continue
        joints.append(mj.mj_id2name(m, mj.mjtObj.mjOBJ_JOINT, j))
        axes.append(m.jnt_axis[j].copy())
        owner.append(int(m.jnt_bodyid[j]) - 1)  # index into `bodies`
        lim = m.jnt_range[j].copy() if m.jnt_limited[j] and m.jnt_range[j][1] > m.jnt_range[j][0] else [-np.inf, np.inf]
        limits.append(lim)
    if len(bodies) != len(joints) + 1 or owner != list(range(1, len(joints) + 1)):
        raise ValueError(f"{mjcf_path}: expected one 1-DOF joint per non-root body in depth-first order")
    return RobotSpec(bodies, joints, np.asarray(axes, dtype=np.float64), np.asarray(limits, dtype=np.float64))


@dataclass(frozen=True)
class Conversion:
    """Everything derived from the two MJCFs that one entry's conversion needs."""

    a3: RobotSpec
    src_dof_to_a3_joint: np.ndarray  # (31,) index into a3.joint_names for every source dof column
    src_body: np.ndarray  # (31,) Ultra pose_aa body index of every source dof column
    src_axes: np.ndarray  # (31, 3) its hinge axis
    keep_src_cols: np.ndarray  # source columns of the 29 non-head joints, in the A3 joint order
    a3_body_joint: np.ndarray  # (29,) a3.joint_names indices of the policy joints (non-head), in order
    a3_head_joint: np.ndarray  # (2,) indices of the head joints
    limits29: np.ndarray  # (29, 2)
    root_z_shift: float  # metres added to root z (A3 sole is closer to the ankle than the Ultra's)

    @property
    def num_out_dof(self) -> int:
        return len(self.a3_body_joint)


def sole_height(mjcf_path: Path) -> float:
    """Ankle-roll origin -> lowest foot contact point at the zero pose, averaged over both feet (metres)."""
    import mujoco as mj

    m = mj.MjModel.from_xml_path(str(mjcf_path))
    d = mj.MjData(m)
    d.qpos[:] = m.qpos0
    mj.mj_forward(m, d)
    heights = []
    for side in ("left", "right"):
        body = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, f"{side}_ankle_roll_Link")
        lowest = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != body or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                continue
            centre, rot = d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3)
            if m.geom_type[g] == mj.mjtGeom.mjGEOM_MESH:
                mesh = m.geom_dataid[g]
                verts = m.mesh_vert[m.mesh_vertadr[mesh] : m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
                lowest.append(float((centre + verts @ rot.T)[:, 2].min()))
            elif m.geom_type[g] == mj.mjtGeom.mjGEOM_SPHERE:
                lowest.append(float(centre[2] - m.geom_size[g][0]))
            else:
                raise ValueError(f"{mjcf_path}: unsupported foot collision geom type {m.geom_type[g]}")
        if not lowest:
            raise ValueError(f"{mjcf_path}: no collision geom on {side}_ankle_roll_Link")
        heights.append(float(d.xpos[body][2]) - min(lowest))
    return float(np.mean(heights))


def build_conversion(a3_mjcf: Path = DEFAULT_A3_MJCF, a3u_mjcf: Path = DEFAULT_A3U_MJCF) -> Conversion:
    a3, a3u = load_spec(a3_mjcf), load_spec(a3u_mjcf)
    a3_index = a3.joint_index
    # the source library stores the actuated Ultra joints (no fingers) in the Ultra DFS order
    src_cols = [i for i, n in enumerate(a3u.joint_names) if JOINT_ALIASES.get(n, n) in a3_index]
    src_names = [a3u.joint_names[i] for i in src_cols]
    if len(src_names) != SOURCE_NUM_DOF:
        raise ValueError(f"expected {SOURCE_NUM_DOF} actuated Ultra joints known to the A3, found {len(src_names)}")
    src_to_a3 = np.asarray([a3_index[JOINT_ALIASES.get(n, n)] for n in src_names])
    head = np.asarray([a3_index[n] for n in HEAD_JOINTS])
    body_joints = np.asarray([i for i in range(len(a3.joint_names)) if i not in set(head)])
    keep = np.asarray([int(np.where(src_to_a3 == j)[0][0]) for j in body_joints])
    # every shared joint must have the same axis in both models, otherwise copying angles would be wrong
    a3u_axis = {JOINT_ALIASES.get(n, n): a3u.axes[i] for i, n in enumerate(a3u.joint_names)}
    for i, n in enumerate(a3.joint_names):
        if not np.allclose(a3.axes[i], a3u_axis[n]):
            raise ValueError(f"joint axis of {n} differs between the A3 and A3 Ultra MJCF")
    return Conversion(
        a3=a3,
        src_dof_to_a3_joint=src_to_a3,
        src_body=np.asarray(src_cols) + 1,
        src_axes=a3u.axes[src_cols],
        keep_src_cols=keep,
        a3_body_joint=body_joints,
        a3_head_joint=head,
        limits29=a3.limits[body_joints],
        root_z_shift=sole_height(a3_mjcf) - sole_height(a3u_mjcf),
    )


def convert_entry(name: str, entry: dict, conv: Conversion) -> tuple[dict, dict]:
    """One Ultra motion -> one ``a3_29`` motion.  Returns ``(entry, stats)``; ``stats['clipped']`` counts per joint."""
    dof_src = np.asarray(entry["dof"], dtype=np.float64)
    pose_src = np.asarray(entry["pose_aa"])
    trans = np.asarray(entry["root_trans_offset"], dtype=np.float64)
    t = trans.shape[0]
    if dof_src.shape != (t, SOURCE_NUM_DOF):
        raise ValueError(f"{name}: dof must be ({t}, {SOURCE_NUM_DOF}), got {dof_src.shape}")
    if pose_src.shape != (t, SOURCE_NUM_BODIES, 3):
        raise ValueError(f"{name}: pose_aa must be ({t}, {SOURCE_NUM_BODIES}, 3), got {pose_src.shape}")
    if not (np.all(np.isfinite(dof_src)) and np.all(np.isfinite(trans)) and np.all(np.isfinite(pose_src))):
        raise ValueError(f"{name}: non-finite values")

    # the compact dof columns must be the actuated Ultra joints in depth-first order: check against pose_aa
    for col in range(SOURCE_NUM_DOF):
        want = dof_src[:, col, None] * conv.src_axes[col][None, :]
        if np.abs(pose_src[:, conv.src_body[col]] - want).max() > 1e-3:
            raise ValueError(f"{name}: dof column {col} does not match pose_aa of body {conv.src_body[col]} (joint order?)")
    dof29 = dof_src[:, conv.keep_src_cols]
    lo, hi = conv.limits29[:, 0], conv.limits29[:, 1]
    clipped = ((dof29 < lo - 1e-6) | (dof29 > hi + 1e-6)).sum(axis=0)
    dof29 = np.clip(dof29, lo, hi)

    pose = np.zeros((t, len(conv.a3.body_names), 3), dtype=np.float64)
    pose[:, 0] = pose_src[:, 0]  # root orientation (axis-angle), unchanged
    for k, j in enumerate(conv.a3_body_joint):
        pose[:, j + 1] = dof29[:, k, None] * conv.a3.axes[j][None, :]  # head bodies stay zero

    trans = trans.copy()
    trans[:, 2] += conv.root_z_shift
    root_rot = np.asarray(entry["root_rot"], dtype=np.float64)
    expected = Rotation.from_rotvec(pose_src[:, 0].astype(np.float64)).as_quat()
    if np.max(np.minimum(np.abs(root_rot - expected).max(axis=1), np.abs(root_rot + expected).max(axis=1))) > 1e-4:
        raise ValueError(f"{name}: root_rot does not match pose_aa[:, 0]")
    smpl = entry.get("smpl_joints")
    out = {
        "root_trans_offset": trans.astype(np.float32),
        "pose_aa": pose.astype(np.float32),
        "dof": dof29.astype(np.float32),
        "root_rot": root_rot.astype(np.float32),
        "smpl_joints": np.asarray(smpl, dtype=np.float32) if smpl is not None else np.zeros((t, 24, 3), np.float32),
        "fps": int(round(float(entry.get("fps", 30.0)))),
    }
    return out, {"frames": t, "clipped": clipped}


def check_a3_29d_entry(name: str, entry: dict, conv: Conversion, source_frames: int | None = None) -> list[str]:
    """Problems of one ``a3_29`` motion (empty list = fine): keys, shapes, dtypes, finiteness, A3 limits, head bodies."""
    problems: list[str] = []
    want = {"root_trans_offset", "pose_aa", "dof", "root_rot", "smpl_joints", "fps"}
    if set(entry) != want:
        return [f"{name}: keys {sorted(entry)} != {sorted(want)}"]
    t = entry["root_trans_offset"].shape[0]
    shapes = {"root_trans_offset": (t, 3), "pose_aa": (t, len(conv.a3.body_names), 3), "dof": (t, conv.num_out_dof),
              "root_rot": (t, 4), "smpl_joints": (t, 24, 3)}
    for key, shape in shapes.items():
        if tuple(entry[key].shape) != shape:
            problems.append(f"{name}: {key} shape {tuple(entry[key].shape)} != {shape}")
        elif entry[key].dtype != np.float32:
            problems.append(f"{name}: {key} dtype {entry[key].dtype} != float32")
        elif not np.all(np.isfinite(entry[key])):
            problems.append(f"{name}: {key} has non-finite values")
    if not isinstance(entry["fps"], int):
        problems.append(f"{name}: fps must be an int, got {type(entry['fps']).__name__}")
    if source_frames is not None and t != source_frames:
        problems.append(f"{name}: {t} frames, the source has {source_frames}")
    if problems:
        return problems
    lo, hi = conv.limits29[:, 0], conv.limits29[:, 1]
    if np.any(entry["dof"] < lo - 1e-5) or np.any(entry["dof"] > hi + 1e-5):
        problems.append(f"{name}: dof outside the A3 joint limits")
    head_bodies = conv.a3_head_joint + 1
    if np.any(entry["pose_aa"][:, head_bodies] != 0):
        problems.append(f"{name}: head bodies of pose_aa are not zero")
    derived = np.einsum("tj,jk->tjk", entry["dof"].astype(np.float64), conv.a3.axes[conv.a3_body_joint])
    if np.abs(entry["pose_aa"][:, conv.a3_body_joint + 1] - derived).max() > 1e-4:
        problems.append(f"{name}: pose_aa does not match dof x joint axis")
    return problems


# --------------------------------------------------------------------------------------------------- batch driver
_CONV: Conversion | None = None


def _init_worker(a3_mjcf: str, a3u_mjcf: str) -> None:
    global _CONV
    _CONV = build_conversion(Path(a3_mjcf), Path(a3u_mjcf))


def _convert_file(job: tuple[str, str, bool]) -> tuple[str, str, int, list[int]]:
    src, dst, overwrite = job
    try:
        if os.path.exists(dst) and not overwrite:
            return src, "skipped", 0, []
        loaded = joblib.load(src)
        if not isinstance(loaded, dict) or len(loaded) < 1:
            return src, "FAIL: not a one-item motion dict", 0, []
        (name, entry), *rest = loaded.items()
        if rest:
            return src, f"FAIL: {len(loaded)} motions in one file", 0, []
        out, stats = convert_entry(name, entry, _CONV)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        tmp = dst + ".tmp"
        joblib.dump({name: out}, tmp)
        os.replace(tmp, dst)
        return src, "ok", int(stats["frames"]), [int(c) for c in stats["clipped"]]
    except Exception as exc:  # noqa: BLE001 -- one bad file must not stop a 130k-file batch
        return src, f"FAIL: {type(exc).__name__}: {exc}", 0, []


def list_inputs(root: Path) -> list[Path]:
    if root.is_file():
        return [root]
    return sorted(p for p in root.rglob("*.pkl") if p.name != "metadata.pkl")


def _check_file(job: tuple[str, str]) -> tuple[str, list[str]]:
    src, dst = job
    try:
        (name, entry), = joblib.load(dst).items()
        frames = None
        if src:
            frames = int(np.asarray(list(joblib.load(src).values())[0]["root_trans_offset"]).shape[0])
        return dst, check_a3_29d_entry(name, entry, _CONV, frames)
    except Exception as exc:  # noqa: BLE001
        return dst, [f"{dst}: {type(exc).__name__}: {exc}"]


def check(args: argparse.Namespace) -> int:
    """``--check``: every file of ``--output`` has the a3_29 format and the frame count of its source in ``--input``."""
    conv = build_conversion(args.a3_mjcf, args.a3u_mjcf)
    out_files = list_inputs(args.output)
    src_base = args.input if args.input.is_dir() else args.input.parent
    jobs = []
    for p in out_files:
        src = src_base / p.relative_to(args.output)
        jobs.append((str(src) if src.exists() else "", str(p)))
    missing_src = sum(1 for src, _ in jobs if not src)
    src_files = list_inputs(args.input) if args.input.exists() else []
    print(f"checking {len(out_files)} converted files in {args.output} against {len(src_files)} sources "
          f"({missing_src} without a source file)")
    bad: list[str] = []
    with mp.Pool(args.num_workers, initializer=_init_worker, initargs=(str(args.a3_mjcf), str(args.a3u_mjcf))) as pool:
        for _, problems in pool.imap_unordered(_check_file, jobs, chunksize=32):
            bad.extend(problems)
    not_converted = len(src_files) - (len(out_files) - missing_src)
    print(f"problems: {len(bad)}; sources without an output: {not_converted}")
    for line in bad[:20]:
        print("  ", line)
    return 1 if bad or not_converted else 0


def run(args: argparse.Namespace) -> int:
    if args.check:
        return check(args)
    conv = build_conversion(args.a3_mjcf, args.a3u_mjcf)
    src_root = args.input
    files = list_inputs(src_root)
    if args.limit:
        files = files[: args.limit]
    base = src_root if src_root.is_dir() else src_root.parent
    print(f"input  {src_root}  ({len(files)} pkl files)")
    print(f"output {args.output}")
    print(f"A3 spec: {len(conv.a3.joint_names)} joints / {len(conv.a3.body_names)} bodies from {args.a3_mjcf.name}; "
          f"policy joints {conv.num_out_dof}; root z shift {conv.root_z_shift * 1000:+.2f} mm")
    if not files:
        print("nothing to convert")
        return 1
    jobs = [(str(p), str(args.output / p.relative_to(base)), args.overwrite) for p in files]
    t0 = time.time()
    counts = {"ok": 0, "skipped": 0, "failed": 0}
    frames = 0
    clipped = np.zeros(conv.num_out_dof, dtype=np.int64)
    failures: list[tuple[str, str]] = []
    with mp.Pool(args.num_workers, initializer=_init_worker, initargs=(str(args.a3_mjcf), str(args.a3u_mjcf))) as pool:
        for i, (src, status, n, clip) in enumerate(pool.imap_unordered(_convert_file, jobs, chunksize=16), 1):
            if status == "ok":
                counts["ok"] += 1
                frames += n
                clipped += np.asarray(clip, dtype=np.int64)
            elif status == "skipped":
                counts["skipped"] += 1
            else:
                counts["failed"] += 1
                failures.append((src, status))
            if i % 5000 == 0 or i == len(jobs):
                rate = i / max(time.time() - t0, 1e-9)
                print(f"  {i}/{len(jobs)}  ok={counts['ok']} skipped={counts['skipped']} failed={counts['failed']}  {rate:.0f} files/s", flush=True)
    names = [conv.a3.joint_names[j] for j in conv.a3_body_joint]
    report = {
        "input": str(src_root), "output": str(args.output), "files": len(files), **counts, "frames_converted": frames,
        "root_z_shift_m": conv.root_z_shift,
        "clipped_frames_per_joint": {n: int(c) for n, c in zip(names, clipped) if c},
        "seconds": round(time.time() - t0, 1),
        "failures": failures[:50],
    }
    print(json.dumps({k: v for k, v in report.items() if k != "failures"}, indent=2))
    for src, status in failures[:20]:
        print(f"  {status}  <- {src}")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2))
    return 1 if counts["failed"] else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Ultra motion-lib directory (searched recursively) or one .pkl")
    ap.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    ap.add_argument("--a3-mjcf", type=Path, default=DEFAULT_A3_MJCF)
    ap.add_argument("--a3u-mjcf", type=Path, default=DEFAULT_A3U_MJCF)
    ap.add_argument("--num_workers", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--limit", type=int, default=0, help="convert only the first N files (smoke test)")
    ap.add_argument("--overwrite", action="store_true", help="rewrite files that already exist in --output")
    ap.add_argument("--report", type=Path, default=None, help="write a JSON report here")
    ap.add_argument("--check", action="store_true",
                    help="do not convert: validate every file of --output (format, shapes, A3 limits, frame count vs --input)")
    return run(ap.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
