"""GPU joint / SMPL jump analysis for walking LeRobot episodes.

Detects frame-to-frame discontinuities that can destabilize balance when replayed
through SONIC encode-mode-2.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from .constants import (
    ANKLE_STATE_INDICES,
    BALANCE_STATE_INDICES,
    KNEE_STATE_INDICES,
    STATE_JOINT_NAMES_43,
    THRESHOLDS,
)
from .lerobot_io import EpisodeMotion, add_episode_range_args, iter_episodes, resolve_episode_indices


def _device(prefer_gpu: bool = True) -> torch.device:
    if prefer_gpu and torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def _diff1(x: torch.Tensor) -> torch.Tensor:
    return x[1:] - x[:-1]


def _count_spikes(abs_d: torch.Tensor, thr: float) -> tuple[int, float, list[int]]:
    """abs_d: (T-1, D) -> count frames where any dim exceeds thr."""
    mask = (abs_d > thr).any(dim=-1)
    idxs = torch.nonzero(mask, as_tuple=False).flatten().tolist()
    peak = float(abs_d.max().item()) if abs_d.numel() else 0.0
    return int(mask.sum().item()), peak, [int(i) for i in idxs]


def analyze_episode(ep: EpisodeMotion, device: torch.device, thr: dict) -> dict:
    state = torch.as_tensor(ep.state, device=device, dtype=torch.float32)
    wbc = torch.as_tensor(ep.action_wbc, device=device, dtype=torch.float32)
    smpl_j = torch.as_tensor(ep.smpl_joints, device=device, dtype=torch.float32)
    smpl_p = torch.as_tensor(ep.smpl_pose, device=device, dtype=torch.float32)
    lh = torch.as_tensor(ep.left_hand, device=device, dtype=torch.float32)
    rh = torch.as_tensor(ep.right_hand, device=device, dtype=torch.float32)
    grav = torch.as_tensor(ep.projected_gravity, device=device, dtype=torch.float32)

    d_state = _diff1(state).abs()
    dd_state = _diff1(_diff1(state)).abs()
    d_wbc = _diff1(wbc).abs()
    d_smpl_j = _diff1(smpl_j.reshape(ep.length, -1)).abs()
    d_smpl_p = _diff1(smpl_p.reshape(ep.length, -1)).abs()
    d_hand = _diff1(torch.cat([lh, rh], dim=-1)).abs()
    d_grav = _diff1(grav).abs()

    bal_idx = torch.tensor(BALANCE_STATE_INDICES, device=device, dtype=torch.long)
    ankle_idx = torch.tensor(ANKLE_STATE_INDICES, device=device, dtype=torch.long)
    knee_idx = torch.tensor(KNEE_STATE_INDICES, device=device, dtype=torch.long)

    n_state, peak_state, idx_state = _count_spikes(d_state, thr["state_dq_rad"])
    n_dd, peak_dd, idx_dd = _count_spikes(dd_state, thr["state_ddq_rad"])
    n_bal, peak_bal, idx_bal = _count_spikes(d_state[:, bal_idx], thr["balance_dq_rad"])
    n_ankle, peak_ankle, idx_ankle = _count_spikes(d_state[:, ankle_idx], thr["balance_dq_rad"])
    n_knee, peak_knee, idx_knee = _count_spikes(d_state[:, knee_idx], thr["balance_dq_rad"])
    n_smpl_j, peak_sj, idx_sj = _count_spikes(d_smpl_j, thr["smpl_joint_m"])
    n_smpl_p, peak_sp, idx_sp = _count_spikes(d_smpl_p, thr["smpl_pose_rad"])
    n_hand, peak_hand, idx_hand = _count_spikes(d_hand, thr["hand_dq_rad"])
    n_grav, peak_grav, idx_grav = _count_spikes(d_grav, thr["gravity_tilt"])
    n_wbc, peak_wbc, idx_wbc = _count_spikes(d_wbc, thr["state_dq_rad"])

    # Per-joint worst offenders in state
    per_joint_peak = d_state.max(dim=0).values.detach().cpu().numpy()
    top = np.argsort(-per_joint_peak)[:8]
    top_joints = [
        {"name": STATE_JOINT_NAMES_43[int(i)], "index": int(i), "peak_dq": float(per_joint_peak[i])}
        for i in top
    ]

    flags = []
    if n_bal > 0:
        flags.append("balance_joint_jump")
    if n_ankle > 0:
        flags.append("ankle_jump")
    if n_knee > 0:
        flags.append("knee_jump")
    if n_smpl_j > 0 or n_smpl_p > 0:
        flags.append("smpl_jump")
    if n_grav > 0:
        flags.append("gravity_tilt_jump")
    if n_dd > 0:
        flags.append("state_accel_spike")

    return {
        "episode_index": ep.episode_index,
        "key": ep.key,
        "length": ep.length,
        "fps": ep.fps,
        "flags": flags,
        "unstable": bool(flags),
        "counts": {
            "state_jump_frames": n_state,
            "state_accel_frames": n_dd,
            "balance_jump_frames": n_bal,
            "ankle_jump_frames": n_ankle,
            "knee_jump_frames": n_knee,
            "smpl_joint_jump_frames": n_smpl_j,
            "smpl_pose_jump_frames": n_smpl_p,
            "hand_jump_frames": n_hand,
            "gravity_jump_frames": n_grav,
            "wbc_jump_frames": n_wbc,
        },
        "peaks": {
            "state_dq": peak_state,
            "state_ddq": peak_dd,
            "balance_dq": peak_bal,
            "ankle_dq": peak_ankle,
            "knee_dq": peak_knee,
            "smpl_joint_m": peak_sj,
            "smpl_pose_rad": peak_sp,
            "hand_dq": peak_hand,
            "gravity": peak_grav,
            "wbc_dq": peak_wbc,
        },
        "first_event_frames": {
            "balance": idx_bal[:5],
            "ankle": idx_ankle[:5],
            "smpl_joints": idx_sj[:5],
            "smpl_pose": idx_sp[:5],
            "gravity": idx_grav[:5],
            "state_accel": idx_dd[:5],
        },
        "top_state_joints": top_joints,
    }


def analyze_dataset(
    dataset_path: str | Path,
    output_json: str | Path,
    episode_indices: list[int] | None = None,
    prefer_gpu: bool = True,
    thresholds: dict | None = None,
) -> dict:
    device = _device(prefer_gpu)
    thr = dict(THRESHOLDS)
    if thresholds:
        thr.update(thresholds)
    print(f"[analyze] device={device} dataset={dataset_path}")

    episodes = iter_episodes(dataset_path, episode_indices)
    rows = [analyze_episode(ep, device, thr) for ep in episodes]
    unstable = [r for r in rows if r["unstable"]]
    report = {
        "dataset": str(Path(dataset_path).resolve()),
        "device": str(device),
        "thresholds": thr,
        "num_episodes": len(rows),
        "num_unstable": len(unstable),
        "unstable_episode_indices": [r["episode_index"] for r in unstable],
        "episodes": rows,
    }
    out = Path(output_json)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    # also write a short CSV for quick glance
    csv_path = out.with_suffix(".csv")
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write(
            "episode_index,length,unstable,flags,balance_jumps,ankle_jumps,"
            "smpl_jumps,peak_balance_dq,peak_ankle_dq,peak_smpl_joint_m\n"
        )
        for r in rows:
            f.write(
                f"{r['episode_index']},{r['length']},{int(r['unstable'])},"
                f"\"{'|'.join(r['flags'])}\","
                f"{r['counts']['balance_jump_frames']},"
                f"{r['counts']['ankle_jump_frames']},"
                f"{r['counts']['smpl_joint_jump_frames']},"
                f"{r['peaks']['balance_dq']:.4f},"
                f"{r['peaks']['ankle_dq']:.4f},"
                f"{r['peaks']['smpl_joint_m']:.4f}\n"
            )
    print(
        f"[analyze] {len(unstable)}/{len(rows)} unstable -> {out} ; csv={csv_path}"
    )
    return report


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True, help="JSON report path")
    add_episode_range_args(p)
    p.add_argument("--cpu", action="store_true")
    args = p.parse_args(argv)
    indices = resolve_episode_indices(
        args.dataset, episodes=args.episodes, start=args.start, end=args.end
    )
    analyze_dataset(args.dataset, args.output, indices, prefer_gpu=not args.cpu)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
