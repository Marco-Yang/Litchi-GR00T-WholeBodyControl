#!/usr/bin/env python3
"""Validate a collected LeRobot dataset for GR00T N1.7 + UNITREE_G1_SONIC fine-tune.

Checks the fields used by Isaac-GR00T ``unitree_g1_sonic`` modality config:
  video:  ego_view
  state:  left/right_leg, waist, left/right_arm, left/right_hand, projected_gravity
  action: motion_token (64), left_hand_joints (7), right_hand_joints (7)  # Dex3
  language: annotation.human.task_description (-> task_index)

Usage:
  source .venv_data_collection/bin/activate
  python gear_sonic/scripts/validate_g1_sonic_vla_dataset.py \\
      --dataset-path outputs/2026-10-30-13-13-04
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
import tyro


REQUIRED_STATE = [
    "left_leg",
    "right_leg",
    "waist",
    "left_arm",
    "right_arm",
    "left_hand",
    "right_hand",
    "projected_gravity",
]
REQUIRED_ACTION = ["motion_token", "left_hand_joints", "right_hand_joints"]


@dataclass
class Config:
    dataset_path: str
    """Path to a LeRobot dataset directory (contains meta/, data/, videos/)."""

    hand_nonzero_warn_frac: float = 0.01
    """Warn if teleop hand-joint nonzero frame fraction is below this (Dex3 trigger unused)."""


def _load_json(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def main(config: Config) -> int:
    root = Path(config.dataset_path)
    info = _load_json(root / "meta" / "info.json")
    modality = _load_json(root / "meta" / "modality.json")
    ok = True

    print(f"dataset: {root}")
    print(
        f"episodes={info.get('total_episodes')} frames={info.get('total_frames')} "
        f"fps={info.get('fps')} videos={info.get('total_videos')}"
    )

    # --- modality keys ---
    state_keys = set(modality.get("state", {}))
    action_keys = set(modality.get("action", {}))
    video_keys = set(modality.get("video", {}))
    ann_keys = set(modality.get("annotation", {}))

    for k in REQUIRED_STATE:
        if k not in state_keys:
            print(f"[FAIL] modality.state missing '{k}'")
            ok = False
    for k in REQUIRED_ACTION:
        if k not in action_keys:
            print(f"[FAIL] modality.action missing '{k}'")
            ok = False
    if "ego_view" not in video_keys:
        print("[FAIL] modality.video missing 'ego_view' (required by unitree_g1_sonic)")
        ok = False
    else:
        print("[OK] video.ego_view present")
    if "head" in video_keys:
        print("[INFO] video.head present (extra; GR00T unitree_g1_sonic ignores it)")

    if "human.task_description" not in ann_keys:
        print("[FAIL] modality.annotation missing 'human.task_description'")
        ok = False
    else:
        print("[OK] annotation.human.task_description present")

    # --- on-disk videos ---
    ego_videos = list((root / "videos").rglob("observation.images.ego_view/*.mp4"))
    if not ego_videos:
        # LeRobot v2.1 pattern: videos/chunk-*/observation.images.ego_view/
        ego_videos = list((root / "videos").rglob("**/observation.images.ego_view/*.mp4"))
    if not ego_videos:
        print("[FAIL] no ego_view mp4 under videos/")
        ok = False
    else:
        print(f"[OK] ego_view videos: {len(ego_videos)}")

    # --- parquet content ---
    parquets = sorted((root / "data").rglob("*.parquet"))
    if not parquets:
        print("[FAIL] no parquet under data/")
        return 1

    token_nz_frames = 0
    left_nz_frames = 0
    right_nz_frames = 0
    state_hand_nz_frames = 0
    total_frames = 0

    for pq_path in parquets:
        table = pq.read_table(pq_path)
        n = table.num_rows
        total_frames += n
        cols = set(table.column_names)

        need_cols = [
            "observation.state",
            "action.motion_token",
            "teleop.left_hand_joints",
            "teleop.right_hand_joints",
        ]
        for c in need_cols:
            if c not in cols:
                print(f"[FAIL] {pq_path.name} missing column '{c}'")
                ok = False

        if "action.motion_token" in cols:
            tok = np.stack(table.column("action.motion_token").to_pylist())
            token_nz_frames += int(np.sum(np.any(np.abs(tok) > 1e-8, axis=1)))
            if tok.shape[1] != 64:
                print(f"[FAIL] motion_token dim={tok.shape[1]} (want 64) in {pq_path.name}")
                ok = False

        if "teleop.left_hand_joints" in cols:
            lh = np.stack(table.column("teleop.left_hand_joints").to_pylist())
            left_nz_frames += int(np.sum(np.any(np.abs(lh) > 1e-6, axis=1)))
            if lh.shape[1] != 7:
                print(f"[FAIL] left_hand_joints dim={lh.shape[1]} (Dex3 want 7)")
                ok = False
        if "teleop.right_hand_joints" in cols:
            rh = np.stack(table.column("teleop.right_hand_joints").to_pylist())
            right_nz_frames += int(np.sum(np.any(np.abs(rh) > 1e-6, axis=1)))

        if "observation.state" in cols:
            state = np.stack(table.column("observation.state").to_pylist())
            # left_hand 22:29, right_hand 36:43 in standard 43-D G1+Dex3 layout
            if state.shape[1] >= 43:
                hand = np.concatenate([state[:, 22:29], state[:, 36:43]], axis=1)
                state_hand_nz_frames += int(np.sum(np.any(np.abs(hand) > 1e-6, axis=1)))

    if total_frames == 0:
        print("[FAIL] zero frames")
        return 1

    token_frac = token_nz_frames / total_frames
    left_frac = left_nz_frames / total_frames
    right_frac = right_nz_frames / total_frames
    state_hand_frac = state_hand_nz_frames / total_frames

    print(f"[check] action.motion_token nonzero frames: {token_frac:.1%}")
    if token_frac < 0.5:
        print("[FAIL] motion_token mostly zeros — SONIC token_state was not recorded")
        ok = False
    else:
        print("[OK] motion_token looks populated (SONIC latent action labels)")

    print(f"[check] observation.state Dex3 hand nonzero frames: {state_hand_frac:.1%}")
    print(f"[check] teleop.left_hand_joints nonzero frames:  {left_frac:.1%}")
    print(f"[check] teleop.right_hand_joints nonzero frames: {right_frac:.1%}")
    print(
        "        (teleop.*_hand_joints = Dex3 7D from Pico Trigger/Grip, "
        "or exporter fallback to deploy hand cmd/state)"
    )
    if left_frac < config.hand_nonzero_warn_frac and right_frac < config.hand_nonzero_warn_frac:
        print(
            "[WARN] teleop hand joints almost always zero. "
            "Squeeze Trigger or Grip while recording (Left Grip+A also closes left Dex3). "
            "Restart pico_manager + exporter if still on old code."
        )
    elif left_frac < config.hand_nonzero_warn_frac or right_frac < config.hand_nonzero_warn_frac:
        which = "left" if left_frac < config.hand_nonzero_warn_frac else "right"
        print(f"[WARN] {which} teleop hand joints nearly zero — check that side's TRIGGER / Dex3 DDS.")

    if ok:
        print("\nRESULT: PASS — schema matches UNITREE_G1_SONIC. Run process_dataset.py before fine-tune.")
        return 0
    print("\nRESULT: FAIL — fix issues above before GR00T fine-tune.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main(tyro.cli(Config)))
