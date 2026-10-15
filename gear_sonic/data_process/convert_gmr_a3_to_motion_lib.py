#!/usr/bin/env python3
"""GMR retarget output for the A3 (T2D5, Litchi a3 parameters) -> Litchi ``a3_29`` motion-lib entry.

The counterpart of ``convert_gmr_a3ultra_to_motion_lib.py`` for the A3 robot of the 035 contract
(``gear_sonic/data/assets/robot_description/mjcf/a3_t2d5.xml``: 31 actuated joints, 32 bodies).

GMR returns one ``qpos`` per frame (nq = 38):
  qpos[0:3]  = root position (xyz)
  qpos[3:7]  = root quaternion (wxyz)
  qpos[7:38] = 31 joint angles in MJCF depth-first order
               (waist 3, head 2, left arm 7, right arm 7, left leg 6, right leg 6)

Output entry (identical layout to ``convert_a3_pkl_to_motion_lib_29d.py`` / ``convert_a3_csv_to_motion_lib.py --robot a3_29``)::

    root_trans_offset (T, 3)   pose_aa (T, 32, 3)   dof (T, 29)   root_rot (T, 4) xyzw   smpl_joints (T, 24, 3) zeros   fps int

The head joints are not policy joints: the two head columns are dropped and the head bodies of ``pose_aa`` stay zero.
``dof`` is clipped to the A3 joint limits (GMR's IK already respects them, this only removes rounding overshoot).
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[2]
A3_MJCF = REPO_ROOT / "gear_sonic" / "data" / "assets" / "robot_description" / "mjcf" / "a3_t2d5.xml"
NUM_QPOS = 38


@lru_cache(maxsize=1)
def _spec():
    from gear_sonic.data_process.convert_a3ultra_to_a3_29d import HEAD_JOINTS, load_spec

    spec = load_spec(A3_MJCF)
    head = [spec.joint_names.index(n) for n in HEAD_JOINTS]
    body = [i for i in range(len(spec.joint_names)) if i not in head]
    if len(spec.joint_names) != NUM_QPOS - 7 or len(body) != 29:
        raise ValueError(f"unexpected A3 MJCF layout: {len(spec.joint_names)} joints")
    return spec, np.asarray(head), np.asarray(body)


def convert_sequence(qpos_list: list, fps: float) -> dict:
    """List of (38,) GMR qpos arrays -> one ``a3_29`` motion-lib entry."""
    spec, head, body = _spec()
    q = np.stack(qpos_list, axis=0).astype(np.float64)
    if q.ndim != 2 or q.shape[1] != NUM_QPOS:
        raise ValueError(f"expected (T, {NUM_QPOS}) qpos for the A3, got {q.shape}")
    t = q.shape[0]
    quat_xyzw = q[:, [4, 5, 6, 3]]  # GMR gives wxyz, motion libs store xyzw (scipy order)
    dof31 = q[:, 7:]
    lo, hi = spec.limits[body, 0], spec.limits[body, 1]
    dof29 = np.clip(dof31[:, body], lo, hi)
    pose = np.zeros((t, len(spec.body_names), 3))
    pose[:, 0] = Rotation.from_quat(quat_xyzw).as_rotvec()
    for k, j in enumerate(body):
        pose[:, j + 1] = dof29[:, k, None] * spec.axes[j][None, :]
    return {
        "root_trans_offset": q[:, 0:3].astype(np.float32),
        "pose_aa": pose.astype(np.float32),
        "dof": dof29.astype(np.float32),
        "root_rot": quat_xyzw.astype(np.float32),
        "smpl_joints": np.zeros((t, 24, 3), dtype=np.float32),
        "fps": int(round(float(fps))),
    }
