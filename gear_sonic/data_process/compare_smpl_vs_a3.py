#!/usr/bin/env python3
# ruff: noqa: T201
"""Side-by-side check of the SMPL training data against the A3 retarget of the same clip.

The SMPL encoder is trained on pairs (SMPL clip, A3 reference clip) of the same name.  This renders both for a sample
of clips, the way ``videos/compare_v15`` showed SOMA against the robot: left the SMPL joints exactly as the training
pipeline sees them (``smpl_joints``, Z-up world frame, root fixed), middle the same joints turned about Z by the clip's
measured heading offset so they face the way the robot does, right the A3 (``a3_t2d5.xml``) driven by the library
``dof`` / ``root_*``.  All panels use the same camera and the SMPL panels are scaled to the robot's leg length.

Besides the picture it measures what the picture can hide: the training observation ``smpl_root_ori_b`` is
``quat_inv(robot_anchor_quat_w) * smpl_root_quat_w`` -- a world-frame difference, so the SMPL data and the robot
reference must share one world heading.  ``yaw offset`` below is that difference (robot - SMPL) over the clip: ~0 for a
consistent pair (the G1 library), anything else is read by the encoder as a constant rotation of the person.

    python gear_sonic/data_process/compare_smpl_vs_a3.py --clips walk_forward_amateur_001__A001 --out videos/x
    python gear_sonic/data_process/compare_smpl_vs_a3.py --like videos/compare_v15 --sample 12 --out videos/x
    python gear_sonic/data_process/compare_smpl_vs_a3.py --stats --every 20            # heading offset over the library
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import joblib
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:  # run as a script: gear_sonic is not importable from the script directory
    sys.path.insert(0, str(PROJECT_ROOT))
MJCF = PROJECT_ROOT / "gear_sonic/data/assets/robot_description/mjcf/a3_t2d5.xml"
TRAIN_FPS = 50.0

# SMPL 24-joint tree (pelvis, L/R hip, spine1, L/R knee, spine2, L/R ankle, spine3, L/R foot, neck, L/R collar, head,
# L/R shoulder, L/R elbow, L/R wrist, L/R hand)
PARENTS = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21]
LEFT = {1, 4, 7, 10, 13, 16, 18, 20, 22}
RIGHT = {2, 5, 8, 11, 14, 17, 19, 21, 23}
COL_SPINE, COL_LEFT, COL_RIGHT = (255, 200, 100), (100, 200, 255), (100, 255, 150)  # BGR, as visualize_retarget.py

_BASE = Rotation.from_quat([0.5, 0.5, 0.5, 0.5], scalar_first=True)  # rotations.remove_smpl_base_rot
_YTOZ = Rotation.from_rotvec([np.pi / 2, 0.0, 0.0])  # smpl_root_ytoz_up (the recipes set smpl_y_up: true)


# ----------------------------------------------------------------------------------------------------------- data
def _entry(path: Path) -> dict:
    d = joblib.load(path)
    return d if "pose_aa" in d and "root_rot" not in d and "dof" not in d else next(iter(d.values()))


def index(root: Path) -> dict[str, Path]:
    return {p.stem: p for p in sorted(root.rglob("*.pkl")) if p.name != "metadata.pkl"}


def smpl_root_world(pose_aa: np.ndarray) -> Rotation:
    """SMPL root orientation in the Z-up world frame, exactly as ``TrackingCommand.smpl_root_quat_w``."""
    return (_YTOZ * Rotation.from_rotvec(pose_aa[:, :3])) * _BASE.inv()


def heading(rot: Rotation) -> np.ndarray:
    f = rot.apply([1.0, 0.0, 0.0])
    return np.arctan2(f[:, 1], f[:, 0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_offset(smpl: dict, robot: dict) -> tuple[float, float]:
    """(mean, std) in radians of heading(robot root) - heading(SMPL root), the SMPL resampled to the robot's frames."""
    ys = np.unwrap(heading(smpl_root_world(smpl["pose_aa"])))
    fps = float(robot["fps"])
    t = np.arange(robot["root_rot"].shape[0]) / fps
    ys_at = np.interp(t, np.arange(len(ys)) / TRAIN_FPS, ys)
    d = wrap(heading(Rotation.from_quat(robot["root_rot"])) - ys_at)
    mean = float(np.angle(np.mean(np.exp(1j * d))))
    return mean, float(np.std(wrap(d - mean)))


def smpl_world(smpl: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(joints relative to the pelvis, Z-up world axes; pelvis height above the floor; pelvis xy in the world)."""
    rel = smpl["smpl_joints"].astype(np.float64)
    transl = smpl["transl"].astype(np.float64)  # Y-up (AMASS): (x, height, z) -> world (x, -z, height)
    h = transl[:, 1].copy()
    lowest = h + rel[..., 2].min(axis=1)
    h = h - np.percentile(lowest, 2)  # the floor is where the feet are most of the time
    return rel, h, np.stack([transl[:, 0], -transl[:, 2]], axis=1)


# --------------------------------------------------------------------------------------------------------- render
def _stick(img, pts, ppm, floor_row, size):
    import cv2

    cv2.line(img, (0, int(floor_row)), (size, int(floor_row)), (70, 70, 70), 1)
    px = np.stack([size / 2 + pts[:, 1] * ppm, floor_row - pts[:, 2] * ppm], axis=1).astype(int)
    for j, p in enumerate(PARENTS):
        if p < 0:
            continue
        col = COL_LEFT if j in LEFT else COL_RIGHT if j in RIGHT else COL_SPINE
        cv2.line(img, tuple(px[p]), tuple(px[j]), col, 3, cv2.LINE_AA)
    for q in px:
        cv2.circle(img, tuple(q), 4, (255, 255, 255), -1, cv2.LINE_AA)
    for j, tag, col in ((22, "L", COL_LEFT), (10, "L", COL_LEFT), (23, "R", COL_RIGHT), (11, "R", COL_RIGHT)):
        cv2.putText(img, tag, (int(px[j][0]) + 8, int(px[j][1]) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)


def _text(img, title, bottom=""):
    import cv2

    cv2.putText(img, title, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
    if bottom:
        cv2.putText(img, bottom, (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)


def _robot_qpos(robot: dict, spec_cols: np.ndarray) -> np.ndarray:
    t = robot["dof"].shape[0]
    q = np.zeros((t, 38))
    q[:, :3] = robot["root_trans_offset"]
    q[:, 3:7] = robot["root_rot"][:, [3, 0, 1, 2]]  # xyzw -> wxyz
    q[:, 7 + spec_cols] = robot["dof"]
    return q


def render_clip(name: str, smpl: dict, robot: dict, out: Path, size: int, max_frames: int, start: int) -> dict:
    import cv2
    import mujoco as mj

    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))
    from gear_sonic.data_process.convert_gmr_a3_to_motion_lib import _spec
    from gear_sonic.data_process.visualize_retarget import MuJoCoRenderer, motion_label

    _, _, body_cols = _spec()
    qpos = _robot_qpos(robot, body_cols)
    fps = float(robot["fps"])
    rel, h, xy = smpl_world(smpl)
    mean_off, std_off = yaw_offset(smpl, robot)
    aligned = Rotation.from_rotvec([0, 0, mean_off]).apply(rel.reshape(-1, 3)).reshape(rel.shape)

    rend = MuJoCoRenderer(str(MJCF), width=size, height=size, title="A3 retarget (front view)")
    b = lambda n: mj.mj_name2id(rend.model, mj.mjtObj.mjOBJ_BODY, n)  # noqa: E731
    mj.mj_kinematics(rend.model, rend.data)
    leg_a3 = sum(
        np.linalg.norm(rend.data.xpos[b(a)] - rend.data.xpos[b(c)])
        for a, c in (("left_hip_pitch_Link", "left_knee_Link"), ("left_knee_Link", "left_ankle_roll_Link"))
    )
    leg_smpl = np.linalg.norm(rel[0, 1] - rel[0, 4]) + np.linalg.norm(rel[0, 4] - rel[0, 7])
    scale = float(leg_a3 / leg_smpl)  # the human panels are drawn at robot size, as in compare_v15
    ppm, floor_row = rend.soma_view(1.0)

    n_robot = qpos.shape[0]
    idx = np.arange(start, min(n_robot, start + max_frames))
    si = np.minimum(np.round(idx / fps * TRAIN_FPS).astype(int), rel.shape[0] - 1)  # the library's own 30 -> 50 fps map

    out.parent.mkdir(parents=True, exist_ok=True)
    wr = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), fps, (size * 3, size))
    win = max(1, int(fps) // 3)
    lefts_r = []
    for q in qpos[idx]:
        rend.data.qpos[:] = q
        mj.mj_kinematics(rend.model, rend.data)
        lefts_r.append((rend.data.xpos[b("left_hip_yaw_Link")] - rend.data.xpos[b("right_hip_yaw_Link")])[:2])
    lefts_r = np.array(lefts_r)
    lefts_s = (rel[si, 1] - rel[si, 2])[:, :2]
    head_s = np.degrees(heading(smpl_root_world(smpl["pose_aa"])))[si]
    head_r = np.degrees(heading(Rotation.from_quat(robot["root_rot"])))[idx]

    def lab(pos, left, k):
        a, c = max(0, k - win), min(len(idx) - 1, k + win)
        return motion_label(pos[c] - pos[a], left[k])

    pos_s, pos_r = xy[si], qpos[idx, :2]
    for k, i in enumerate(idx):
        panels = []
        for j, (joints, title) in enumerate(((rel, "SMPL data (training frame)"), (aligned, "SMPL turned by offset"))):
            img = np.zeros((size, size, 3), dtype=np.uint8)
            pts = joints[si[k]] * scale
            pts = pts.copy()
            pts[:, 2] += h[si[k]] * scale
            _stick(img, pts, ppm, floor_row, size)
            _text(img, title, lab(pos_s, lefts_s, k) if j == 0 else f"turned {np.degrees(mean_off):+.0f} deg")
            panels.append(img)
        right = rend.render(qpos[i], motion_text=lab(pos_r, lefts_r, k))
        cv2.putText(right, f"heading {head_r[k]:+.0f}", (10, size - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
        cv2.putText(panels[0], f"heading {head_s[k]:+.0f}", (10, size - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
        row = np.hstack([panels[0], panels[1], right])
        cv2.putText(row, f"{name}  t={i / fps:5.1f}s", (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1, cv2.LINE_AA)
        cv2.putText(row, f"yaw offset robot-SMPL {np.degrees(mean_off):+.0f} deg (std {np.degrees(std_off):.0f})",
                    (size + 10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1, cv2.LINE_AA)
        wr.write(row)
    wr.release()
    rend.close()
    return {"clip": name, "frames_robot": n_robot, "frames_smpl": int(rel.shape[0]), "yaw_offset_deg": float(np.degrees(mean_off)),
            "yaw_offset_std_deg": float(np.degrees(std_off)), "scale": scale, "video": str(out)}


# ------------------------------------------------------------------------------------------------------- stats
def _stat_one(args):
    from gear_sonic.data_process.check_smpl_pairing import robot_frames_at_target

    name, sp, rp = args
    try:
        s, r = _entry(sp), _entry(rp)
        m, sd = yaw_offset(s, r)
        want = robot_frames_at_target(r["root_rot"].shape[0], float(r["fps"]), TRAIN_FPS)
        return name, np.degrees(m), np.degrees(sd), int(s["pose_aa"].shape[0]) - want
    except Exception as exc:  # noqa: BLE001
        return name, float("nan"), float("nan"), f"{type(exc).__name__}: {exc}"


def stats(smpl_idx, robot_idx, every: int, workers: int, out_csv: Path | None):
    from multiprocessing import Pool

    names = sorted(set(smpl_idx) & set(robot_idx))[::every]
    jobs = [(n, smpl_idx[n], robot_idx[n]) for n in names]
    with Pool(workers) as pool:
        rows = pool.map(_stat_one, jobs, chunksize=16)
    ok = [r for r in rows if isinstance(r[3], int)]
    off = np.array([r[1] for r in ok])
    print(f"clips measured {len(ok)} / {len(rows)}  (frame-count mismatches: {sum(1 for r in ok if r[3] != 0)})")
    c = np.angle(np.mean(np.exp(1j * np.radians(off))))
    print(f"yaw offset robot - SMPL (deg): circular mean {np.degrees(c):+.1f}; |offset| p5/p50/p95 = "
          f"{np.percentile(np.abs(wrap(np.radians(off))) * 180 / np.pi, [5, 50, 95]).round(1).tolist()}; "
          f"within 10 deg: {(np.abs(off) < 10).mean() * 100:.1f}% ; within 20 deg: {(np.abs(off) < 20).mean() * 100:.1f}%")
    print(f"per-clip heading drift (std of the offset inside a clip) p50/p95 = {np.percentile([r[2] for r in ok], [50, 95]).round(1).tolist()} deg")
    hist, edges = np.histogram(off, bins=np.arange(-180, 181, 30))
    for h_, e in zip(hist, edges[:-1]):
        print(f"  [{e:+4.0f},{e + 30:+4.0f}) {h_:6d} {'#' * int(60 * h_ / max(1, hist.max()))}")
    if out_csv:
        with open(out_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["clip", "yaw_offset_deg", "yaw_offset_std_deg", "smpl_frames_minus_expected"])
            w.writerows(rows)
        print("csv:", out_csv)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--smpl", type=Path, default=PROJECT_ROOT / "data/smpl_filtered")
    ap.add_argument("--robot", type=Path, default=PROJECT_ROOT / "data/motion_lib_a3_29d/robot_filtered_v15_clean")
    ap.add_argument("--out", type=Path, default=PROJECT_ROOT / "videos/compare_smpl_a3")
    ap.add_argument("--clips", nargs="*", default=[], help="clip names")
    ap.add_argument("--like", type=Path, help="directory of mp4s whose names are the clips (e.g. videos/compare_v15)")
    ap.add_argument("--sample", type=int, default=0, help="add N random paired clips")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--max_frames", type=int, default=300)
    ap.add_argument("--start", type=int, default=0, help="first robot frame")
    ap.add_argument("--size", type=int, default=400)
    ap.add_argument("--stats", action="store_true", help="heading offset over the whole library instead of videos")
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--workers", type=int, default=12)
    args = ap.parse_args()

    smpl_idx, robot_idx = index(args.smpl), index(args.robot)
    paired = sorted(set(smpl_idx) & set(robot_idx))
    print(f"SMPL clips {len(smpl_idx)}, robot clips {len(robot_idx)}, paired {len(paired)}")
    if args.stats:
        stats(smpl_idx, robot_idx, args.every, args.workers, args.out if args.out.suffix == ".csv" else None)
        return 0
    names = list(args.clips)
    if args.like:
        names += [p.stem for p in sorted(args.like.glob("*.mp4"))]
    missing = [n for n in names if n not in paired]
    if missing:
        print(f"not in both libraries ({len(missing)}): {missing}")
    names = [n for n in names if n in paired]
    if args.sample:
        rng = np.random.default_rng(args.seed)
        pool = [n for n in paired if n not in names]
        names += [pool[i] for i in rng.choice(len(pool), min(args.sample, len(pool)), replace=False)]
    rows = []
    for n in names:
        row = render_clip(n, _entry(smpl_idx[n]), _entry(robot_idx[n]), args.out / f"{n}.mp4", args.size, args.max_frames, args.start)
        print(f"{n}: yaw offset {row['yaw_offset_deg']:+7.1f} deg (std {row['yaw_offset_std_deg']:.1f}), frames robot {row['frames_robot']} / SMPL {row['frames_smpl']}, scale {row['scale']:.2f}")
        rows.append(row)
    if rows:
        with open(args.out / "summary.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
