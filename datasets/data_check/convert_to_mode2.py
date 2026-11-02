"""Convert LeRobot walking episodes into SONIC encode-mode-2 teleop-stream bundles.

Mirrors the **real Pico → ZMQ v3 → smpl encoder** path (not recorded robot state):

  smpl_motions/  world-frame smpl_joints + pose_aa(72)  → encoder joints / root ori
  robot_motions/ standing carrier + wrist-only DOFs from smpl_pose parse + Dex3 hands
                 → wrist obs (IL 23–28) + hand_dof; NOT a tracking reference of observation.state

Produces:
  <out>/robot_motions/<key>.pkl
  <out>/smpl_motions/<key>.pkl
  <out>/mode2_meta.json
"""

from __future__ import annotations

import argparse
import json
import pickle
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from .constants import (
    DEFAULT_FPS,
    G1_BODY29_DOF_AXES,
    G1_ISAACLAB_TO_MUJOCO_DOF,
    G1_STANDING_ROOT_Z,
    ISAACLAB_WRIST_DOF_INDICES,
)
from .lerobot_io import EpisodeMotion, add_episode_range_args, iter_episodes, resolve_episode_indices

_REPO = Path(__file__).resolve().parents[2]
_VENDOR = Path(__file__).resolve().parent / "_vendor"


def _ensure_paths() -> None:
    for p in (str(_VENDOR), str(_REPO)):
        if p not in sys.path:
            sys.path.insert(0, p)


def quat_wxyz_to_aa(quat_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(quat_wxyz, dtype=np.float64)
    rot = R.from_quat(q[:, [1, 2, 3, 0]])
    return rot.as_rotvec().astype(np.float32)


def quat_wxyz_to_xyzw(quat_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(quat_wxyz, dtype=np.float32)
    return q[:, [1, 2, 3, 0]]


def _decompose_rotation_aa(rotation_aa: np.ndarray, axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Match ``gear_sonic.trl.utils.rotation_conversion.decompose_rotation_aa`` (wxyz quat)."""
    aa = np.asarray(rotation_aa, dtype=np.float64)
    v2 = np.asarray(axis, dtype=np.float64)
    angle = np.linalg.norm(aa, axis=1)[:, None]
    safe = np.maximum(angle, 1e-8)
    w = np.cos(angle / 2.0)
    v = np.sin(angle / 2.0) * aa / safe
    q = np.concatenate([w, v], axis=1)
    v_twist = np.dot(v, v2)[:, None] * v2
    q_twist = np.concatenate([w, v_twist], axis=1)
    q_twist = q_twist / np.maximum(np.linalg.norm(q_twist, axis=1, keepdims=True), 1e-8)
    q_twist_inv = q_twist * np.array([1.0, -1.0, -1.0, -1.0])
    # q_swing = q_twist_inv ⊗ q  (wxyz)
    w1, x1, y1, z1 = q_twist_inv.T
    w2, x2, y2, z2 = q.T
    q_swing = np.stack(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        axis=1,
    )
    return q_twist.astype(np.float32), q_swing.astype(np.float32)


def compute_wrist_joint_pos_il(smpl_pose: np.ndarray) -> np.ndarray:
    """Pico/manager wrist parse → IsaacLab 29-D ``joint_pos`` (only wrists nonzero).

    Same formulas as ``pico_manager_thread_server.PoseStreamer.run_once`` /
    ``pico_pose_zmq_minimal._compute_wrist_joint_pos``.
    """
    t = smpl_pose.shape[0]
    joint_pos = np.zeros((t, 29), dtype=np.float32)
    body_pose = np.asarray(smpl_pose, dtype=np.float64).reshape(t, 21, 3)

    smpl_l_elbow = body_pose[:, 17]
    smpl_l_wrist = body_pose[:, 19]
    smpl_r_elbow = body_pose[:, 18]
    smpl_r_wrist = body_pose[:, 20]

    _, g1_l_elbow_q_swing = _decompose_rotation_aa(smpl_l_elbow, np.array([0.0, 1.0, 0.0]))
    _, g1_r_elbow_q_swing = _decompose_rotation_aa(smpl_r_elbow, np.array([0.0, 1.0, 0.0]))

    l_elbow_swing_euler = R.from_quat(g1_l_elbow_q_swing[:, [1, 2, 3, 0]]).as_euler(
        "XYZ", degrees=False
    )
    r_elbow_swing_euler = R.from_quat(g1_r_elbow_q_swing[:, [1, 2, 3, 0]]).as_euler(
        "XYZ", degrees=False
    )
    l_wrist_euler = R.from_rotvec(smpl_l_wrist).as_euler("XYZ", degrees=False)
    r_wrist_euler = R.from_rotvec(smpl_r_wrist).as_euler("XYZ", degrees=False)

    g1_l_wrist_roll = l_elbow_swing_euler[:, 0] + l_wrist_euler[:, 0]
    g1_l_wrist_pitch = -l_wrist_euler[:, 1]
    g1_l_wrist_yaw = l_elbow_swing_euler[:, 2] + l_wrist_euler[:, 2]
    g1_r_wrist_roll = -(r_elbow_swing_euler[:, 0] + r_wrist_euler[:, 0])
    g1_r_wrist_pitch = -r_wrist_euler[:, 1]
    g1_r_wrist_yaw = r_elbow_swing_euler[:, 2] + r_wrist_euler[:, 2]

    # IsaacLab indices (same as deploy ZMQ joint_pos)
    joint_pos[:, 23] = g1_l_wrist_roll
    joint_pos[:, 25] = -g1_l_wrist_pitch
    joint_pos[:, 27] = g1_l_wrist_yaw
    joint_pos[:, 24] = g1_r_wrist_roll
    joint_pos[:, 26] = g1_r_wrist_pitch
    joint_pos[:, 28] = g1_r_wrist_yaw
    return joint_pos.astype(np.float32)


def isaaclab_dof_to_mujoco(joint_pos_il: np.ndarray) -> np.ndarray:
    """Reorder IsaacLab 29-D → MuJoCo/motion_lib body29."""
    mapping = np.asarray(G1_ISAACLAB_TO_MUJOCO_DOF, dtype=np.int64)
    return np.asarray(joint_pos_il, dtype=np.float32)[..., mapping]


def build_smpl_pose_aa72(ep: EpisodeMotion) -> np.ndarray:
    t = ep.length
    out = np.zeros((t, 72), dtype=np.float32)
    out[:, 0:3] = quat_wxyz_to_aa(ep.body_quat_w)
    out[:, 3:66] = ep.smpl_pose.reshape(t, 63)
    return out


def build_standing_pose_aa(body29_mj: np.ndarray, root_quat_wxyz: np.ndarray | None = None) -> np.ndarray:
    """(T,30,3) pose_aa: identity/standing root + hinge rotvecs (wrists only nonzero)."""
    t = body29_mj.shape[0]
    axes = np.asarray(G1_BODY29_DOF_AXES, dtype=np.float32)
    pose_aa = np.zeros((t, 30, 3), dtype=np.float32)
    if root_quat_wxyz is not None:
        pose_aa[:, 0, :] = quat_wxyz_to_aa(root_quat_wxyz)
    pose_aa[:, 1:30, :] = body29_mj[:, :, None] * axes[None, :, :]
    return pose_aa


def standing_root_trans(t: int, z: float = G1_STANDING_ROOT_Z) -> np.ndarray:
    out = np.zeros((t, 3), dtype=np.float32)
    out[:, 2] = float(z)
    return out


def snap_root_to_ground(pose_aa: np.ndarray, root_trans: np.ndarray) -> np.ndarray:
    """Put mesh lowest vertex on z=0 using frame-0 FK (constant Δz)."""
    _ensure_paths()
    out = np.asarray(root_trans, dtype=np.float32).copy()
    try:
        import torch
        from omegaconf import OmegaConf
        from gear_sonic.utils.motion_lib.torch_humanoid_batch import Humanoid_Batch

        cfg = OmegaConf.create(
            {
                "asset": {
                    "assetRoot": "gear_sonic/data/assets/robot_description/mjcf/",
                    "assetFileName": "g1_29dof_rev_1_0.xml",
                    "urdfFileName": "",
                },
                "extend_config": [],
            }
        )
        hb = Humanoid_Batch(cfg, device=torch.device("cpu"))
        pose = torch.tensor(pose_aa[None, :1], dtype=torch.float32)
        trans = torch.tensor(out[None, :1], dtype=torch.float32)
        mesh_obj = hb.mesh_fk(pose, trans)
        height_diff = float(np.asarray(mesh_obj.vertices)[..., 2].min())
        out[:, 2] = out[:, 2] - height_diff
        print(f"[convert] snap_root_to_ground Δz={-height_diff:.4f} → z0={out[0, 2]:.4f}")
    except Exception as exc:  # noqa: BLE001
        print(f"[convert] snap_root_to_ground skipped ({exc}); z stays {out[0, 2]:.3f}")
        out[:, 2] = np.clip(out[:, 2], 0.55, 1.05)
    return out.astype(np.float32)


def episode_to_teleop_entries(ep: EpisodeMotion) -> tuple[dict, dict]:
    """Build (robot_carrier, smpl_stream) matching real mode-2 teleop inputs."""
    t = ep.length
    joint_pos_il = compute_wrist_joint_pos_il(ep.smpl_pose)
    body29_mj = isaaclab_dof_to_mujoco(joint_pos_il)
    hands14 = np.concatenate(
        [ep.left_hand.astype(np.float32), ep.right_hand.astype(np.float32)], axis=-1
    )
    dof43 = np.concatenate([body29_mj, hands14], axis=-1).astype(np.float32)

    # Standing carrier: identity root yaw; policy closes the loop from here.
    root_quat = np.zeros((t, 4), dtype=np.float32)
    root_quat[:, 0] = 1.0  # wxyz identity
    pose_aa = build_standing_pose_aa(body29_mj, root_quat_wxyz=root_quat)
    root_trans = snap_root_to_ground(pose_aa, standing_root_trans(t))

    # Keep Pico local joints + body_quat as-is (C++ deploy path). Isaac must set
    # motion_lib_cfg.smpl_teleop_preprocessed=true so it does not Y→Z / base_rot /
    # re-localize again.
    smpl_joints_local = ep.smpl_joints.astype(np.float32)

    robot_entry = {
        "root_trans_offset": root_trans,
        "pose_aa": pose_aa,
        "dof": dof43,
        "root_rot": quat_wxyz_to_xyzw(root_quat),
        "fps": float(ep.fps or DEFAULT_FPS),
        "hand_action_left": ep.left_hand.astype(np.float32),
        "hand_action_right": ep.right_hand.astype(np.float32),
        "joint_pos_il": joint_pos_il,  # debug / parity with ZMQ
        "encode_mode": 2,
        "teleop_stream": True,
        "source_episode": int(ep.episode_index),
        "tasks": list(ep.tasks),
    }
    smpl_entry = {
        "pose_aa": build_smpl_pose_aa72(ep),
        "smpl_joints": smpl_joints_local,
        "transl": root_trans.copy(),
        "fps": float(ep.fps or DEFAULT_FPS),
        "source_episode": int(ep.episode_index),
        "teleop_stream": True,
    }
    return robot_entry, smpl_entry


def convert_dataset(
    dataset_path: str | Path,
    output_dir: str | Path,
    episode_indices: list[int] | None = None,
) -> dict:
    out = Path(output_dir).expanduser().resolve()
    robot_dir = out / "robot_motions"
    smpl_dir = out / "smpl_motions"
    robot_dir.mkdir(parents=True, exist_ok=True)
    smpl_dir.mkdir(parents=True, exist_ok=True)

    episodes = iter_episodes(dataset_path, episode_indices)
    meta = {
        "dataset": str(Path(dataset_path).resolve()),
        "episodes": [],
        "encode_mode": 2,
        "teleop_stream": True,
        "notes": (
            "robot_motions = standing carrier + Pico wrist parse + Dex3 hands; "
            "smpl_motions.smpl_joints = Pico local (teleop.smpl_joints); "
            "requires Isaac motion_lib_cfg.smpl_teleop_preprocessed=true; "
            "does NOT replay observation.state as body reference."
        ),
    }

    for ep in episodes:
        key = ep.key
        robot_entry, smpl_entry = episode_to_teleop_entries(ep)
        with open(robot_dir / f"{key}.pkl", "wb") as f:
            pickle.dump({key: robot_entry}, f, protocol=pickle.HIGHEST_PROTOCOL)
        with open(smpl_dir / f"{key}.pkl", "wb") as f:
            pickle.dump(smpl_entry, f, protocol=pickle.HIGHEST_PROTOCOL)

        wrists = robot_entry["joint_pos_il"][:, list(ISAACLAB_WRIST_DOF_INDICES)]
        meta["episodes"].append(
            {
                "key": key,
                "episode_index": ep.episode_index,
                "length": ep.length,
                "fps": ep.fps,
                "tasks": ep.tasks,
                "wrist_abs_mean": float(np.abs(wrists).mean()),
                "smpl_pelvis_local0": smpl_entry["smpl_joints"][0, 0].tolist(),
                "robot_pkl": str(robot_dir / f"{key}.pkl"),
                "smpl_pkl": str(smpl_dir / f"{key}.pkl"),
            }
        )
        print(
            f"[convert] {key} T={ep.length} "
            f"wrist|mean|={float(np.abs(wrists).mean()):.3f} "
            f"smpl_pelvis_local0={smpl_entry['smpl_joints'][0, 0]}"
        )

    meta_path = out / "mode2_meta.json"
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"[convert] wrote {len(episodes)} teleop-stream clips -> {out}")
    return meta


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--output", "--out", type=Path, required=True, dest="output")
    add_episode_range_args(p)
    args = p.parse_args(argv)
    indices = resolve_episode_indices(
        args.dataset, episodes=args.episodes, start=args.start, end=args.end
    )
    convert_dataset(args.dataset, args.output, indices)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
