#!/usr/bin/env python3
# ruff: noqa: T201
"""Remove the constant left/right asymmetry of the A3 retarget library by correcting the IK table's ``rot_offset``.

Why.  ``calibrate_soma_to_a3ultra.py`` derives every ``rot_offset`` from the FIRST frame of ONE clip and assumes the performer
stands symmetrically there.  The performer does not: the left and right legs / wrists of that frame differ, and the difference is
baked into the table as a constant rotation of the target link.  Every library clip then carries it (measured on 120 mirror
twins: the two feet are yawed by 11.8 degrees against each other, the two hands by 19.7 degrees; G1's library has none of it).
The SMPL encoder is trained to track this library, so it would learn the bias.

How.  Each clip ``X`` has an exact mirror twin ``X_M`` (the G1 library confirms: its mirror error is 0.01 rad).  For a link of the
right side, the *symmetric ideal* of ``X_M`` is the left chain of ``X`` mirrored (joint signs from the MJCF axes); the achieved
orientation relative to that ideal, in the link frame, is ``Delta`` -- a constant rotation plus a small scatter when the error
is an offset error (the scatter is reported: 0.7-2.4 degrees against offsets of 11-20).  Only the DIFFERENCE between left and
right is observable, so each side is corrected by half::

    rot_offset_side  <-  rot_offset_side * Delta_side^(-1/2)        (right-multiplied: the IK target is q_human * rot_offset)

and the result is exactly symmetric (derivation in docs/workflow/a3_integration_plan.md 19.6).  A common-mode error (both sides
off the same mirrored way) is not observable from the library and is left alone.  Links whose offset is below ``--min_deg``
(shoulder yaw, elbow: 2-3 degrees, inside the scatter) are not touched.

    python gear_sonic/data_process/calibrate_a3_lr_symmetry.py --lib data/motion_lib_a3_29d/robot_filtered_v15_clean \\
        --json gear_sonic/data_process/gmr_ik_configs/bvh_nokov_to_a3.json --out_json /tmp/bvh_nokov_to_a3_sym.json
    ... --check          # only measure (use it on a re-retargeted subset: the offsets should be ~0)
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import random
import sys

import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

MJCF = PROJECT_ROOT / "gear_sonic/data/assets/robot_description/mjcf/a3_t2d5.xml"
LINKS = {
    "hip_yaw": "hip_yaw_Link",
    "knee": "knee_Link",
    "ankle_roll": "ankle_roll_Link",
    "shoulder_yaw": "shoulder_yaw_Link",
    "elbow": "elbow_Link",
    "wrist_yaw": "wrist_yaw_Link",
}
MIRROR = np.diag([1.0, -1.0, 1.0])  # left <-> right: y -> -y in the pelvis frame (x forward, y left, z up)


def mirror_name(name: str) -> str:
    return name.replace("left_", "\0").replace("right_", "left_").replace("\0", "right_")


class A3Kinematics:
    """FK of the 29 policy joints of the A3 (head fixed at 0, root at the origin with identity orientation)."""

    def __init__(self, mjcf: Path = MJCF):
        import mujoco as mj

        self._mj = mj
        self.model = mj.MjModel.from_xml_path(str(mjcf))
        self.data = mj.MjData(self.model)
        jname = lambda j: mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, j)  # noqa: E731
        self.names = [jname(j) for j in range(1, self.model.njnt) if "head" not in jname(j)]
        if len(self.names) != 29:
            raise ValueError(f"expected 29 policy joints, got {len(self.names)}")
        self.col = {n: i for i, n in enumerate(self.names)}
        self._qadr = np.array([self.model.jnt_qposadr[self._jid(n)] for n in self.names])
        axis = lambda n: self.model.jnt_axis[self._jid(n)]  # noqa: E731
        # pseudovector rule: a rotation of theta about axis a mirrors to -theta about (M a)
        self.sign = np.ones(29)
        for n, i in self.col.items():  # waist joints mirror onto themselves (yaw, roll: -1; pitch: +1)
            self.sign[i] = -(MIRROR @ axis(n)) @ axis(mirror_name(n))
        self.mirror_cols = np.array([self.col.get(mirror_name(n), self.col[n]) for n in self.names])

    def _jid(self, name):
        return self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_JOINT, name)

    def side_cols(self, side: str) -> list[int]:
        return [i for n, i in self.col.items() if n.startswith(side + "_")]

    def mirrored(self, q: np.ndarray) -> np.ndarray:
        """The mirror image of a (T, 29) joint trajectory: left <-> right with the axis-derived signs."""
        return q[:, self.mirror_cols] * self.sign

    def link_rotations(self, q: np.ndarray, bodies: list[str]) -> dict[str, np.ndarray]:
        mj = self._mj
        ids = {b: mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, b) for b in bodies}
        out = {b: np.empty((len(q), 3, 3)) for b in bodies}
        qpos = np.zeros(self.model.nq)
        qpos[3] = 1.0
        for t in range(len(q)):
            qpos[self._qadr] = q[t]
            self.data.qpos[:] = qpos
            mj.mj_kinematics(self.model, self.data)
            for b, i in ids.items():
                out[b][t] = self.data.xmat[i].reshape(3, 3)
        return out


def relative_to_symmetric_ideal(kin: A3Kinematics, q_side: np.ndarray, q_other: np.ndarray, side: str) -> dict[str, np.ndarray]:
    """Per link of ``side``: ``ideal^T actual`` (link frame) per frame.

    ``q_side`` is the clip the ``side`` chain is taken from (actual); ``q_other`` is the twin, whose opposite chain mirrored gives the
    ideal.  Everything else (waist) is taken from ``q_side`` so that only the arm / leg joints differ.
    """
    ideal = q_side.copy()
    cols = kin.side_cols(side)
    ideal[:, cols] = kin.mirrored(q_other)[:, cols]
    bodies = [f"{side}_{v}" for v in LINKS.values()]
    actual_r = kin.link_rotations(q_side, bodies)
    ideal_r = kin.link_rotations(ideal, bodies)
    return {k: np.einsum("tji,tjk->tik", ideal_r[f"{side}_{v}"], actual_r[f"{side}_{v}"]) for k, v in LINKS.items()}


def measure(kin: A3Kinematics, pairs: list[tuple[np.ndarray, np.ndarray]], max_frames: int = 60) -> dict[tuple[str, str], Rotation]:
    """Mean (chordal) relative rotation per (side, link) over the twin pairs ``(q_X, q_X_M)``; scatter via ``rotations``."""
    acc: dict[tuple[str, str], list[np.ndarray]] = {(s, k): [] for s in ("left", "right") for k in LINKS}
    for qa, qb in pairs:
        t = min(len(qa), len(qb))
        step = max(1, t // max_frames)
        qa, qb = qa[:t:step], qb[:t:step]
        for side, q_side, q_other in (("right", qb, qa), ("left", qa, qb)):
            for k, rel in relative_to_symmetric_ideal(kin, q_side, q_other, side).items():
                acc[(side, k)].append(rel)
    return {key: Rotation.from_matrix(np.concatenate(v)) for key, v in acc.items()}


def summarize(rots: dict[tuple[str, str], Rotation]) -> dict[tuple[str, str], dict]:
    out = {}
    for key, r in rots.items():
        mean = r.mean()
        dev = np.degrees((mean.inv() * r).magnitude())
        out[key] = {
            "mean": mean,
            "deg": float(np.degrees(mean.magnitude())),
            "scatter_median": float(np.median(dev)),
            "scatter_p90": float(np.percentile(dev, 90)),
        }
    return out


def half_inverse(delta: Rotation) -> Rotation:
    """``delta^(-1/2)``: the rotation about the same axis by minus half the angle."""
    return Rotation.from_rotvec(-0.5 * delta.as_rotvec())


def corrections(summary: dict[tuple[str, str], dict], min_deg: float) -> dict[tuple[str, str], Rotation]:
    return {key: half_inverse(s["mean"]) for key, s in summary.items() if s["deg"] >= min_deg}


def apply_to_table(cfg: dict, corr: dict[tuple[str, str], Rotation]) -> list[str]:
    """Right-multiply the rot_offset (wxyz, last element of each row) of the corrected links in both IK tables."""
    done = []
    for (side, link), e in corr.items():
        key = f"{side}_{LINKS[link]}"
        for table in ("ik_match_table1", "ik_match_table2"):
            row = cfg[table][key]
            new = Rotation.from_quat(row[4], scalar_first=True) * e
            row[4] = new.as_quat(scalar_first=True).tolist()
        done.append(key)
    return done


def load_pairs(lib_dir: Path, n: int, seed: int) -> list[tuple[np.ndarray, np.ndarray]]:
    from gear_sonic.data_process.compare_smpl_vs_a3 import _entry, index

    lib = index(lib_dir)
    cand = sorted(k for k in lib if k + "_M" in lib and not k.endswith("_M"))
    sel = random.Random(seed).sample(cand, min(n, len(cand)))
    return [(np.asarray(_entry(lib[k])["dof"], float), np.asarray(_entry(lib[k + "_M"])["dof"], float)) for k in sel]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lib", required=True, help="A3 library (a3_29 pkl directory) with the clips and their _M twins")
    ap.add_argument("--json", default=str(PROJECT_ROOT / "gear_sonic/data_process/gmr_ik_configs/bvh_nokov_to_a3.json"))
    ap.add_argument("--out_json", action="append", default=[], help="write the corrected table here (repeatable)")
    ap.add_argument("--pairs", type=int, default=150)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--min_deg", type=float, default=5.0, help="only correct links whose mean offset is at least this many degrees")
    ap.add_argument("--check", action="store_true", help="measure only")
    args = ap.parse_args()

    kin = A3Kinematics()
    pairs = load_pairs(Path(args.lib), args.pairs, args.seed)
    summ = summarize(measure(kin, pairs))
    print(f"{len(pairs)} mirror-twin pairs from {args.lib}")
    print("left/right asymmetry of the achieved link orientation (actual relative to the mirrored other side, link frame):")
    for (side, link), s in summ.items():
        rv = np.degrees(s["mean"].as_rotvec())
        print(f"  {side:5s} {link:13s} offset {s['deg']:5.1f} deg  rotvec(deg) {np.round(rv, 1)}  scatter median {s['scatter_median']:.1f} p90 {s['scatter_p90']:.1f}")
    if args.check:
        return 0
    corr = corrections(summ, args.min_deg)
    cfg = json.loads(Path(args.json).read_text())
    done = apply_to_table(cfg, corr)
    print(f"corrected {len(done)} IK rows: {', '.join(done)}")
    for out in args.out_json:
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        Path(out).write_text(json.dumps(cfg, indent=4))
        print("wrote", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
