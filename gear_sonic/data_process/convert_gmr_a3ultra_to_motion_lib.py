#!/usr/bin/env python3
"""Convert A3 Ultra GMR retarget output (qpos pkl) to SONIC motion_lib format.

GMR retarget outputs a list of qpos arrays (shape (70,) each):
  qpos[0:3]  = root position (xyz)
  qpos[3:7]  = root quaternion (wxyz)
  qpos[7:70] = 63 DOF in MuJoCo qpos declaration order (waist → head → left arm →
               left fingers → right arm → right fingers → left leg → right leg)

SONIC training format (per motion):
  root_trans_offset  (T, 3)    root xyz
  pose_aa            (T, 64, 3) axis-angle for all 64 MJCF bodies (DFS order, fingers=0)
  dof                (T, 31)   31 actuated DOF values (compact MuJoCo order, no fingers)
  root_rot           (T, 4)    root quaternion xyzw (scipy convention)
  smpl_joints        (T, 24, 3) zeros placeholder
  fps                float

Body-ordering reference:  gear_sonic/data_process/a3ultra_fk_order.py
MJCF: gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml

Usage:
    python gear_sonic/data_process/convert_gmr_a3ultra_to_motion_lib.py \\
        --input /path/to/a3ultra_retarget.pkl \\
        --output data/motion_lib_a3ultra/robot_filtered/motion_001.pkl \\
        --fps 30
"""

import argparse
import os
import pickle
import sys
from pathlib import Path

import joblib
import numpy as np
from scipy.spatial.transform import Rotation

# ---------------------------------------------------------------------------
# A3 Ultra MJCF DFS body index → (MuJoCo qpos DOF index, joint axis)
# Body 0 = pelvis_link (root) → pose_aa[0] is root orientation, not a DOF.
# Bodies 1-63 are non-root; actuated ones are listed here; fingers stay at 0.
# ---------------------------------------------------------------------------
# fmt: off
_ACTUATED_MAP = [
    # (dfs_body_idx, qpos_dof_idx, axis_xyz)
    ( 1,  0, [0, 0, 1]),  # waist_yaw_joint
    ( 2,  1, [1, 0, 0]),  # waist_roll_joint
    ( 3,  2, [0, 1, 0]),  # waist_pitch_joint
    ( 4,  3, [0, 0, 1]),  # head_yaw_joint
    ( 5,  4, [0, 1, 0]),  # ultra_head_pitch_joint
    ( 6,  5, [0, 1, 0]),  # left_shoulder_pitch_joint
    ( 7,  6, [1, 0, 0]),  # left_shoulder_roll_joint
    ( 8,  7, [0, 0, 1]),  # left_shoulder_yaw_joint
    ( 9,  8, [0, 1, 0]),  # left_elbow_joint
    (10,  9, [1, 0, 0]),  # left_wrist_roll_joint
    (11, 10, [0, 1, 0]),  # left_wrist_pitch_joint
    (12, 11, [0, 0, 1]),  # left_wrist_yaw_joint
    # 13-28: left fingers → zeros
    (29, 28, [0, 1, 0]),  # right_shoulder_pitch_joint
    (30, 29, [1, 0, 0]),  # right_shoulder_roll_joint
    (31, 30, [0, 0, 1]),  # right_shoulder_yaw_joint
    (32, 31, [0, 1, 0]),  # right_elbow_joint
    (33, 32, [1, 0, 0]),  # right_wrist_roll_joint
    (34, 33, [0, 1, 0]),  # right_wrist_pitch_joint
    (35, 34, [0, 0, 1]),  # right_wrist_yaw_joint
    # 36-51: right fingers → zeros
    (52, 51, [0, 1, 0]),  # left_hip_pitch_joint
    (53, 52, [1, 0, 0]),  # left_hip_roll_joint
    (54, 53, [0, 0, 1]),  # left_hip_yaw_joint
    (55, 54, [0, 1, 0]),  # left_knee_joint
    (56, 55, [0, 1, 0]),  # left_ankle_pitch_joint
    (57, 56, [1, 0, 0]),  # left_ankle_roll_joint
    (58, 57, [0, 1, 0]),  # right_hip_pitch_joint
    (59, 58, [1, 0, 0]),  # right_hip_roll_joint
    (60, 59, [0, 0, 1]),  # right_hip_yaw_joint
    (61, 60, [0, 1, 0]),  # right_knee_joint
    (62, 61, [0, 1, 0]),  # right_ankle_pitch_joint
    (63, 62, [1, 0, 0]),  # right_ankle_roll_joint
]
# fmt: on

# 31 actuated DOF indices in qpos[7:] order (fingers at 12-27, 35-50 are skipped).
# These are stored in `dof` field in the same compact order (0-30).
_ACTUATED_DOF_QPOS_INDICES = [row[1] for row in _ACTUATED_MAP]

NUM_BODIES = 64   # pelvis_link + 63 joint bodies (DFS)
NUM_ACT_DOF = 31  # actuated joints (no fingers)


def convert_sequence(qpos_list: list, fps: float) -> dict:
    """Convert a list of qpos arrays to SONIC motion_lib format.

    Args:
        qpos_list: list of (70,) numpy arrays, one per frame.
        fps: frame rate of the input data.

    Returns:
        Motion lib entry dict.
    """
    qpos_arr = np.stack(qpos_list, axis=0).astype(np.float32)  # (T, 70)
    T = qpos_arr.shape[0]

    root_pos = qpos_arr[:, 0:3]            # (T, 3) xyz
    root_quat_wxyz = qpos_arr[:, 3:7]     # (T, 4) wxyz
    dof_all = qpos_arr[:, 7:]             # (T, 63) MuJoCo qpos DOF order

    # Root quaternion: wxyz → xyzw (scipy convention)
    root_quat_xyzw = root_quat_wxyz[:, [1, 2, 3, 0]]  # (T, 4) xyzw

    # Root orientation as axis-angle
    root_aa = Rotation.from_quat(root_quat_xyzw).as_rotvec().astype(np.float32)  # (T, 3)

    # pose_aa: (T, 64, 3) — body 0 = root, fingers at indices 13-28 / 36-51 stay 0
    pose_aa = np.zeros((T, NUM_BODIES, 3), dtype=np.float32)
    pose_aa[:, 0, :] = root_aa

    for dfs_idx, dof_idx, axis in _ACTUATED_MAP:
        theta = dof_all[:, dof_idx]  # (T,)
        ax = np.array(axis, dtype=np.float32)
        pose_aa[:, dfs_idx, :] = theta[:, None] * ax[None, :]

    # Compact actuated DOF array (31 values in order matching _ACTUATED_MAP)
    dof = dof_all[:, _ACTUATED_DOF_QPOS_INDICES]  # (T, 31)

    return {
        "root_trans_offset": root_pos,
        "pose_aa": pose_aa,
        "dof": dof,
        "root_rot": root_quat_xyzw,
        "smpl_joints": np.zeros((T, 24, 3), dtype=np.float32),
        "fps": float(fps),
    }


def convert_pkl(input_path: str, output_path: str, fps: float, motion_key: str = None):
    """Convert one GMR retarget pkl to SONIC motion_lib format.

    The GMR pkl can be either:
      - list of (70,) arrays (output of validate_a3ultra_retarget.py)
      - dict {motion_key: list_of_qpos} (batch retarget output)
    """
    with open(input_path, "rb") as f:
        data = pickle.load(f)

    if isinstance(data, list):
        # Single sequence: list of qpos arrays
        seq_name = motion_key or Path(input_path).stem
        sequences = {seq_name: data}
    elif isinstance(data, dict):
        sequences = data
    else:
        raise ValueError(f"Unexpected pkl type: {type(data)}")

    out_data = {}
    for seq_name, qpos_list in sequences.items():
        entry = convert_sequence(qpos_list, fps)
        out_data[seq_name] = entry
        T = len(qpos_list)
        print(f"  {seq_name}: {T} frames, pose_aa{entry['pose_aa'].shape}, dof{entry['dof'].shape}")

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    joblib.dump(out_data, output_path)
    print(f"Saved {len(out_data)} sequence(s) → {output_path}")


def main():
    ap = argparse.ArgumentParser(description="Convert A3 Ultra GMR qpos pkl to SONIC training format")
    ap.add_argument("--input", required=True, help="Input GMR retarget pkl file")
    ap.add_argument("--output", required=True, help="Output SONIC motion_lib pkl file")
    ap.add_argument("--fps", type=float, default=30.0, help="Frame rate (default 30)")
    ap.add_argument("--key", default=None, help="Motion key name (for single-sequence input)")
    args = ap.parse_args()

    print(f"Converting {args.input} → {args.output} @ {args.fps} fps")
    convert_pkl(args.input, args.output, args.fps, motion_key=args.key)


if __name__ == "__main__":
    main()
