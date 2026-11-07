"""Convert LeRobot walking episodes into SONIC encode-mode-0 (g1 / qpos) bundles.

Mirrors the **joint-based encoder** path (ZMQ protocol v1 → encode_mode=0):

  robot_motions/  body29 qpos from ``observation.state`` + Revo2 teleop hands
                  → g1 encoder reference (NOT SMPL)

Hands match mode-2: ``teleop.left/right_hand_joints`` (Revo2, padded 7D).
Body does **not** use SMPL / standing-carrier wrists — full recorded qpos.

Produces:
  <out>/robot_motions/<key>.pkl
  <out>/mode0_meta.json
"""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path

import numpy as np

from .constants import BODY_DOF_SLICES, DEFAULT_FPS
from .convert_to_mode2 import (
    build_standing_pose_aa,
    quat_wxyz_to_xyzw,
    snap_root_to_ground,
    standing_root_trans,
)
from .lerobot_io import EpisodeMotion, add_episode_range_args, iter_episodes, resolve_episode_indices


def state_to_body29_mujoco(state: np.ndarray) -> np.ndarray:
    """Extract 29-D body DOF (MuJoCo order) from interleaved 43-D ``observation.state``."""
    parts = [np.asarray(state, dtype=np.float32)[:, s] for s in BODY_DOF_SLICES]
    return np.concatenate(parts, axis=-1).astype(np.float32)


def episode_to_mode0_entry(ep: EpisodeMotion) -> dict:
    """Build robot motion clip for g1 encoder (encode mode 0)."""
    t = ep.length
    body29 = state_to_body29_mujoco(ep.state)
    hands14 = np.concatenate(
        [ep.left_hand.astype(np.float32), ep.right_hand.astype(np.float32)], axis=-1
    )
    dof43 = np.concatenate([body29, hands14], axis=-1).astype(np.float32)

    root_quat_wxyz = ep.root_quat_wxyz.astype(np.float32)
    pose_aa = build_standing_pose_aa(body29, root_quat_wxyz=root_quat_wxyz)
    root_trans = snap_root_to_ground(pose_aa, standing_root_trans(t))

    return {
        "root_trans_offset": root_trans,
        "pose_aa": pose_aa,
        "dof": dof43,
        "root_rot": quat_wxyz_to_xyzw(root_quat_wxyz),
        "fps": float(ep.fps or DEFAULT_FPS),
        "hand_action_left": ep.left_hand.astype(np.float32),
        "hand_action_right": ep.right_hand.astype(np.float32),
        "encode_mode": 0,
        "teleop_stream": False,
        "source_episode": int(ep.episode_index),
        "tasks": list(ep.tasks),
        "hand_source": "teleop.revo2",
        "body_source": "observation.state",
    }


def convert_dataset(
    dataset_path: str | Path,
    output_dir: str | Path,
    episode_indices: list[int] | None = None,
) -> dict:
    out = Path(output_dir).expanduser().resolve()
    robot_dir = out / "robot_motions"
    robot_dir.mkdir(parents=True, exist_ok=True)

    episodes = iter_episodes(dataset_path, episode_indices)
    meta = {
        "dataset": str(Path(dataset_path).resolve()),
        "episodes": [],
        "encode_mode": 0,
        "use_encoder": "g1",
        "notes": (
            "robot_motions = observation.state body29 (MuJoCo) + teleop Revo2 hands; "
            "Isaac: ++use_encoder=g1, smpl_motion_file=dummy; "
            "does NOT use SMPL / mode-2 wrist parse."
        ),
    }

    for ep in episodes:
        key = ep.key
        entry = episode_to_mode0_entry(ep)
        with open(robot_dir / f"{key}.pkl", "wb") as f:
            pickle.dump({key: entry}, f, protocol=pickle.HIGHEST_PROTOCOL)

        body = entry["dof"][:, :29]
        hands = entry["dof"][:, 29:]
        meta["episodes"].append(
            {
                "key": key,
                "episode_index": ep.episode_index,
                "length": ep.length,
                "fps": ep.fps,
                "tasks": ep.tasks,
                "body_abs_mean": float(np.abs(body).mean()),
                "hand_abs_mean": float(np.abs(hands).mean()),
                "root_z0": float(entry["root_trans_offset"][0, 2]),
                "robot_pkl": str(robot_dir / f"{key}.pkl"),
            }
        )
        print(
            f"[convert-mode0] {key} T={ep.length} "
            f"body|mean|={float(np.abs(body).mean()):.3f} "
            f"hand|mean|={float(np.abs(hands).mean()):.3f} "
            f"z0={float(entry['root_trans_offset'][0, 2]):.3f}"
        )

    meta_path = out / "mode0_meta.json"
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    print(f"[convert-mode0] wrote {len(episodes)} qpos clips -> {out}")
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
