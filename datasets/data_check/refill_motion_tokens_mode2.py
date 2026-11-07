"""Refill ``action.motion_token`` via SONIC v1.1 encoder **mode 2** (SMPL).

Aligns with the **Isaac mode-2 screening path** (``convert_to_mode2`` +
``++use_encoder=smpl`` + ``smpl_teleop_preprocessed=true``), NOT the recorded
teleop ``token_state`` (those can be wrong / stale).

Encoder inputs (same as deploy mode-2 / Isaac smpl encoder):

  encoder_mode_4 = 2
  + smpl_joints_10frame_step1                 (teleop.smpl_joints, already local)
  + smpl_anchor_orientation_heading_10frame   (body_quat vs *open-loop* heading)
  + motion_joint_positions_wrists_10frame     (Pico wrist parse from smpl_pose,
                                               same as ``convert_to_mode2``)

Open-loop heading: robot yaw = heading(body_quat[t]) with apply_delta=I.
This matches Isaac when tracking is good (see ``isaac_mode2_walking_155752``),
and deliberately ignores ``observation.root_orientation`` from the bad recording.

Finger / Revo2 hands stay in ``teleop.*_hand_joints``; they are not mode-2
encoder obs (wrists are the hand-side signal).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from .constants import ISAACLAB_WRIST_DOF_INDICES
from .convert_to_mode2 import compute_wrist_joint_pos_il
from .lerobot_io import (
    add_episode_range_args,
    dataset_root,
    list_episodes,
    load_info,
    resolve_episode_indices,
)


def _ensure_ort_cuda_libs() -> None:
    """Prepend pip nvidia cu13/cudnn libs so ORT CUDA EP can load."""
    try:
        import nvidia  # type: ignore
    except ImportError:
        return
    root = Path(nvidia.__file__).resolve().parent
    extras = [
        root / "cu13" / "lib",
        root / "cudnn" / "lib",
        root / "cublas" / "lib",
        root / "cuda_runtime" / "lib",
        root / "cufft" / "lib",
        root / "curand" / "lib",
        root / "cusolver" / "lib",
        root / "cusparse" / "lib",
        root / "nvjitlink" / "lib",
    ]
    cur = os.environ.get("LD_LIBRARY_PATH", "")
    parts = [str(p) for p in extras if p.is_dir()]
    if not parts:
        return
    os.environ["LD_LIBRARY_PATH"] = ":".join(parts + ([cur] if cur else []))

_THIS = Path(__file__).resolve().parent
_REPO = _THIS.parents[1]
DEFAULT_ENCODER = (
    _REPO / "gear_sonic_deploy" / "policy" / "sonic_v1_1" / "model_encoder.onnx"
)
_DEFAULT_ENCODER = DEFAULT_ENCODER  # back-compat alias

# Observation layout must match sonic_v1_1/observation_config.yaml encoder_observations
# concatenated in order (unused modes left at 0). Total = 1751.
_ENCODER_OBS_DIMS: list[tuple[str, int]] = [
    ("encoder_mode_4", 4),
    ("motion_joint_positions_10frame_step5", 290),
    ("motion_joint_velocities_10frame_step5", 290),
    ("motion_anchor_orientation_heading_10frame_step5", 60),
    ("motion_anchor_orientation_heading", 6),
    ("motion_joint_positions_lowerbody_10frame_step5", 120),
    ("motion_joint_velocities_lowerbody_10frame_step5", 120),
    ("vr_3point_local_target", 9),
    ("vr_3point_local_orn_target", 12),
    ("smpl_joints_10frame_step1", 720),
    ("smpl_anchor_orientation_heading_10frame_step1", 60),
    ("motion_joint_positions_wrists_10frame_step1", 60),
]
assert sum(d for _, d in _ENCODER_OBS_DIMS) == 1751

_OFFSETS = {}
_off = 0
for _name, _dim in _ENCODER_OBS_DIMS:
    _OFFSETS[_name] = (_off, _dim)
    _off += _dim

ENCODE_MODE = 2
NUM_FUTURE = 10
STEP = 1
TOKEN_DIM = 64


# ---------------------------------------------------------------------------
# Quaternion helpers (wxyz), matching gear_sonic_deploy math_utils.hpp
# ---------------------------------------------------------------------------


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.maximum(n, 1e-12)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product, wxyz, broadcastable on leading dims."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    w1, x1, y1, z1 = np.moveaxis(a, -1, 0)
    w2, x2, y2, z2 = np.moveaxis(b, -1, 0)
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)
    return np.stack([w, x, y, z], axis=-1)


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    out = q.copy()
    out[..., 1:] *= -1.0
    return out


def _quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    q = _quat_normalize(q)
    v = np.asarray(v, dtype=np.float64)
    zeros = np.zeros(q.shape[:-1] + (1,), dtype=np.float64)
    vq = np.concatenate([zeros, v], axis=-1)
    return _quat_mul(_quat_mul(q, vq), _quat_conjugate(q))[..., 1:]


def _calc_heading(q: np.ndarray) -> np.ndarray:
    ref = np.zeros(q.shape[:-1] + (3,), dtype=np.float64)
    ref[..., 0] = 1.0
    rot = _quat_rotate(q, ref)
    return np.arctan2(rot[..., 1], rot[..., 0])


def _calc_heading_quat(q: np.ndarray) -> np.ndarray:
    heading = _calc_heading(q)
    half = 0.5 * heading
    out = np.zeros(q.shape[:-1] + (4,), dtype=np.float64)
    out[..., 0] = np.cos(half)
    out[..., 3] = np.sin(half)
    return out


def _euler_z_to_quat(angle: np.ndarray | float) -> np.ndarray:
    angle = np.asarray(angle, dtype=np.float64)
    half = 0.5 * angle
    out = np.zeros(np.shape(angle) + (4,), dtype=np.float64)
    out[..., 0] = np.cos(half)
    out[..., 3] = np.sin(half)
    return out


def _quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    q = _quat_normalize(q)
    w, x, y, z = np.moveaxis(q, -1, 0)
    r00 = 1 - 2 * (y * y + z * z)
    r01 = 2 * (x * y - w * z)
    r02 = 2 * (x * z + w * y)
    r10 = 2 * (x * y + w * z)
    r11 = 1 - 2 * (x * x + z * z)
    r12 = 2 * (y * z - w * x)
    r20 = 2 * (x * z - w * y)
    r21 = 2 * (y * z + w * x)
    r22 = 1 - 2 * (x * x + y * y)
    return np.stack(
        [
            np.stack([r00, r01, r02], axis=-1),
            np.stack([r10, r11, r12], axis=-1),
            np.stack([r20, r21, r22], axis=-1),
        ],
        axis=-2,
    )


def _rotmat_first_two_cols(mat: np.ndarray) -> np.ndarray:
    """(…,3,3) → (…,6) row-major first two columns — matches C++ gather."""
    return np.stack(
        [
            mat[..., 0, 0],
            mat[..., 0, 1],
            mat[..., 1, 0],
            mat[..., 1, 1],
            mat[..., 2, 0],
            mat[..., 2, 1],
        ],
        axis=-1,
    )


def _calc_heading_quat_inv(q: np.ndarray) -> np.ndarray:
    """Matches C++ ``calc_heading_quat_inv_d`` (yaw-only with negated angle)."""
    return _euler_z_to_quat(-_calc_heading(q))


def _apply_delta_heading(
    init_base_quat: np.ndarray,
    init_ref_quat: np.ndarray,
    delta_heading: float,
) -> np.ndarray:
    init_heading = _calc_heading_quat(init_base_quat)
    data_heading_inv = _calc_heading_quat_inv(init_ref_quat)
    apply = _quat_mul(init_heading, data_heading_inv)
    if abs(float(delta_heading)) > 1e-12:
        apply = _quat_mul(_euler_z_to_quat(delta_heading), apply)
    return apply.astype(np.float64)


# ---------------------------------------------------------------------------
# Observation builders
# ---------------------------------------------------------------------------


def _future_indices(t: int, length: int, num_frames: int = NUM_FUTURE, step: int = STEP) -> np.ndarray:
    idxs = t + np.arange(num_frames) * step
    return np.minimum(idxs, length - 1)


def wrists_il_from_teleop(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """(T,3)+(T,3) → (T,6) in IsaacLab wrist gather order {23,24,25,26,27,28}."""
    left = np.asarray(left, dtype=np.float32)
    right = np.asarray(right, dtype=np.float32)
    # L_roll, R_roll, L_pitch, R_pitch, L_yaw, R_yaw
    return np.stack(
        [left[:, 0], right[:, 0], left[:, 1], right[:, 1], left[:, 2], right[:, 2]],
        axis=-1,
    )


def build_all_encoder_obs_mode2(
    smpl_joints: np.ndarray,
    wrists6: np.ndarray,
    body_quat_w: np.ndarray,
) -> np.ndarray:
    """Vectorized (T, 1751) mode-2 obs — Isaac open-loop heading, apply_delta=I."""
    smpl_joints = np.asarray(smpl_joints, dtype=np.float32)
    wrists6 = np.asarray(wrists6, dtype=np.float32)
    body_quat_w = np.asarray(body_quat_w, dtype=np.float64)
    t_len = smpl_joints.shape[0]
    obs = np.zeros((t_len, 1751), dtype=np.float32)
    obs[:, 0] = float(ENCODE_MODE)

    # Future index table: (T, 10)
    idxs = np.minimum(
        np.arange(t_len)[:, None] + np.arange(NUM_FUTURE)[None, :] * STEP,
        t_len - 1,
    )

    smpl_off, _ = _OFFSETS["smpl_joints_10frame_step1"]
    smpl_flat = smpl_joints.reshape(t_len, 72)
    for fi in range(NUM_FUTURE):
        obs[:, smpl_off + fi * 72 : smpl_off + (fi + 1) * 72] = smpl_flat[idxs[:, fi]]

    wrist_off, _ = _OFFSETS["motion_joint_positions_wrists_10frame_step1"]
    for fi in range(NUM_FUTURE):
        obs[:, wrist_off + fi * 6 : wrist_off + (fi + 1) * 6] = wrists6[idxs[:, fi]]

    # Anchor ori heading: left = heading(body[t]); ref = body[t+f]
    ori_off, _ = _OFFSETS["smpl_anchor_orientation_heading_10frame_step1"]
    left = _calc_heading_quat(body_quat_w)  # (T, 4)
    left_inv = _quat_conjugate(left)
    for fi in range(NUM_FUTURE):
        ref = body_quat_w[idxs[:, fi]]
        rel = _quat_mul(left_inv, ref)
        obs[:, ori_off + fi * 6 : ori_off + (fi + 1) * 6] = _rotmat_first_two_cols(
            _quat_to_rotmat(rel)
        ).astype(np.float32)
    return obs


def build_encoder_obs_mode2(
    smpl_joints: np.ndarray,
    wrists6: np.ndarray,
    body_quat_w: np.ndarray,
    base_quat_w: np.ndarray,
    apply_delta: np.ndarray,
    t: int,
) -> np.ndarray:
    """Single-frame wrapper (tests / debug). Prefer ``build_all_encoder_obs_mode2``."""
    del base_quat_w, apply_delta  # open-loop uses body heading only
    return build_all_encoder_obs_mode2(smpl_joints, wrists6, body_quat_w)[t]


# ---------------------------------------------------------------------------
# ONNX encoder
# ---------------------------------------------------------------------------


def _load_encoder_session(encoder_path: Path, *, device: str = "auto"):
    _ensure_ort_cuda_libs()
    try:
        import onnxruntime as ort
    except ImportError as exc:  # pragma: no cover
        raise ImportError(
            "onnxruntime is required for mode-2 token refill "
            "(conda activate sikan && pip install onnxruntime-gpu)"
        ) from exc
    path = Path(encoder_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"SONIC v1.1 encoder ONNX not found: {path}")

    want = (device or "auto").lower()
    if want == "cuda":
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
    elif want == "cpu":
        providers = ["CPUExecutionProvider"]
    else:
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]

    so = ort.SessionOptions()
    so.intra_op_num_threads = 1
    so.inter_op_num_threads = 1
    sess = ort.InferenceSession(str(path), sess_options=so, providers=providers)
    active = sess.get_providers()
    if want == "cuda" and "CUDAExecutionProvider" not in active:
        raise RuntimeError(
            f"Requested CUDA but ORT fell back to {active}. "
            "Check LD_LIBRARY_PATH / cuDNN (call _ensure_ort_cuda_libs)."
        )
    inp = sess.get_inputs()[0].name
    out = sess.get_outputs()[0].name
    return sess, inp, out, active


def encode_episode_tokens_isaac_mode2(
    sess,
    inp_name: str,
    out_name: str,
    smpl_joints: np.ndarray,
    wrists6: np.ndarray,
    body_quat_w: np.ndarray,
) -> np.ndarray:
    """Return (T, 64) tokens with Isaac mode-2 open-loop heading (vectorized obs)."""
    obs_all = build_all_encoder_obs_mode2(smpl_joints, wrists6, body_quat_w)
    t_len = obs_all.shape[0]
    tokens = np.zeros((t_len, TOKEN_DIM), dtype=np.float64)
    # ONNX graph is fixed batch=1; keep a tight GPU-friendly loop.
    for t in range(t_len):
        out = sess.run([out_name], {inp_name: obs_all[t : t + 1]})[0]
        tokens[t] = np.asarray(out, dtype=np.float64).reshape(TOKEN_DIM)
    return tokens


# Back-compat alias
encode_episode_tokens = encode_episode_tokens_isaac_mode2


def _stack_col(table: pq.Table, name: str, dtype=np.float32) -> np.ndarray:
    return np.asarray(table.column(name).to_pylist(), dtype=dtype)


def _episode_parquet_path(root: Path, info: dict, episode_index: int) -> Path:
    chunks_size = int(info.get("chunks_size", 1000))
    episode_chunk = episode_index // chunks_size
    pattern = info.get(
        "data_path",
        "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
    )
    return root / pattern.format(episode_chunk=episode_chunk, episode_index=episode_index)


def _replace_motion_token_column(table: pa.Table, tokens: np.ndarray) -> pa.Table:
    name = "action.motion_token"
    idx = table.schema.get_field_index(name)
    if idx < 0:
        raise KeyError(f"missing column {name}")
    field = table.schema.field(idx)
    values = [tokens[i].astype(np.float64) for i in range(tokens.shape[0])]
    return table.set_column(idx, field, pa.array(values, type=field.type))


def refill_episode(
    parquet_path: Path,
    sess,
    inp_name: str,
    out_name: str,
    *,
    backup_dir: Path | None = None,
    dry_run: bool = False,
) -> dict:
    table = pq.read_table(parquet_path)
    t = table.num_rows
    cols = set(table.column_names)
    if "teleop.smpl_joints" not in cols:
        raise KeyError(f"{parquet_path}: missing teleop.smpl_joints")
    if "action.motion_token" not in cols:
        raise KeyError(f"{parquet_path}: missing action.motion_token")

    smpl = _stack_col(table, "teleop.smpl_joints").reshape(t, -1)
    if smpl.shape[1] != 72:
        raise ValueError(f"smpl_joints dim {smpl.shape} in {parquet_path}")
    smpl = smpl.reshape(t, 24, 3)

    if "teleop.smpl_pose" not in cols:
        raise KeyError(f"{parquet_path}: missing teleop.smpl_pose (needed for mode2 wrists)")
    smpl_pose = _stack_col(table, "teleop.smpl_pose").reshape(t, -1)
    if smpl_pose.shape[1] != 63:
        raise ValueError(f"smpl_pose dim {smpl_pose.shape} in {parquet_path}")

    # Same wrist parse as convert_to_mode2 / Isaac mode-2 robot_motions.
    joint_pos_il = compute_wrist_joint_pos_il(smpl_pose)
    wrists6 = joint_pos_il[:, list(ISAACLAB_WRIST_DOF_INDICES)].astype(np.float32)

    body_quat = _stack_col(table, "teleop.body_quat_w", np.float64).reshape(t, 4)

    old = _stack_col(table, "action.motion_token", np.float64).reshape(t, -1)
    # Clear old (bad) tokens in memory first; write path replaces the column.
    tokens = encode_episode_tokens_isaac_mode2(
        sess,
        inp_name,
        out_name,
        smpl,
        wrists6,
        body_quat,
    )

    # Diagnostic only — old tokens are known-bad; low cosine is expected/OK.
    old_n = old / np.maximum(np.linalg.norm(old, axis=-1, keepdims=True), 1e-8)
    new_n = tokens / np.maximum(np.linalg.norm(tokens, axis=-1, keepdims=True), 1e-8)
    cos = float((old_n * new_n).sum(axis=-1).mean())
    l2 = float(np.linalg.norm(old - tokens, axis=-1).mean())

    stats = {
        "parquet": str(parquet_path),
        "frames": t,
        "encode_path": "isaac_mode2_openloop",
        "sonic": "v1.1",
        "encode_mode": ENCODE_MODE,
        "token_abs_mean": float(np.abs(tokens).mean()),
        "old_token_abs_mean": float(np.abs(old).mean()),
        "mean_cosine_vs_old": cos,
        "mean_l2_vs_old": l2,
        "wrist_abs_mean": float(np.abs(wrists6).mean()),
        "dry_run": bool(dry_run),
    }

    if dry_run:
        return stats

    if backup_dir is not None:
        backup_dir.mkdir(parents=True, exist_ok=True)
        np.save(backup_dir / f"{parquet_path.stem}_motion_token_old.npy", old)

    new_table = _replace_motion_token_column(table, tokens)
    tmp = parquet_path.with_suffix(".parquet.tmp")
    pq.write_table(new_table, tmp)
    tmp.replace(parquet_path)
    return stats


def _refill_one_episode_job(payload: dict) -> dict:
    """Worker entry: one episode, own ORT session (safe for CUDA multi-process)."""
    _ensure_ort_cuda_libs()
    enc = Path(payload["encoder"])
    sess, inp_name, out_name, providers = _load_encoder_session(
        enc, device=payload.get("device", "auto")
    )
    bak = Path(payload["backup_dir"]) if payload.get("backup_dir") else None
    stats = refill_episode(
        Path(payload["parquet"]),
        sess,
        inp_name,
        out_name,
        backup_dir=bak,
        dry_run=bool(payload.get("dry_run", False)),
    )
    stats["episode_index"] = int(payload["episode_index"])
    stats["providers"] = list(providers)
    return stats


def refill_dataset(
    dataset_path: str | Path,
    *,
    episode_indices: list[int] | None = None,
    encoder_path: str | Path | None = None,
    backup_dir: str | Path | None = None,
    dry_run: bool = False,
    device: str = "auto",
    workers: int = 1,
) -> dict:
    root = dataset_root(dataset_path)
    info = load_info(root)
    eps = list_episodes(root)
    indices = [int(e["episode_index"]) for e in eps]
    if episode_indices is not None:
        want = set(int(i) for i in episode_indices)
        indices = [i for i in indices if i in want]

    enc = Path(encoder_path) if encoder_path else DEFAULT_ENCODER
    bak_root = Path(backup_dir).expanduser().resolve() if backup_dir else None
    workers = max(1, int(workers))

    jobs = []
    for ei in indices:
        pq_path = _episode_parquet_path(root, info, ei)
        if not pq_path.is_file():
            print(f"[refill-tokens] missing {pq_path}, skip")
            continue
        jobs.append(
            {
                "episode_index": ei,
                "parquet": str(pq_path),
                "encoder": str(enc),
                "backup_dir": str(bak_root / f"episode_{ei:06d}") if bak_root else None,
                "dry_run": bool(dry_run),
                "device": device,
            }
        )

    results: list[dict] = []
    if not jobs:
        return {
            "dataset": str(root),
            "encoder": str(enc.resolve()),
            "encode_mode": ENCODE_MODE,
            "sonic": "v1.1",
            "num_episodes": 0,
            "dry_run": bool(dry_run),
            "device": device,
            "workers": workers,
            "episodes": [],
        }

    print(
        f"[refill-tokens] {root.name}: {len(jobs)} episodes, "
        f"workers={workers}, device={device}, encoder={enc.name}"
    )

    if workers == 1:
        # Single process — reuse one session.
        sess, inp_name, out_name, providers = _load_encoder_session(enc, device=device)
        print(f"[refill-tokens] ORT providers={providers}", flush=True)
        for job in jobs:
            stats = refill_episode(
                Path(job["parquet"]),
                sess,
                inp_name,
                out_name,
                backup_dir=Path(job["backup_dir"]) if job["backup_dir"] else None,
                dry_run=dry_run,
            )
            stats["episode_index"] = job["episode_index"]
            stats["providers"] = list(providers)
            results.append(stats)
            print(
                f"  ep{stats['episode_index']:03d} T={stats['frames']} "
                f"|token|={stats['token_abs_mean']:.4f} "
                f"l2_vs_old={stats['mean_l2_vs_old']:.3f}"
            )
    else:
        # Parallel episodes; each worker owns a CUDA/CPU session.
        # spawn avoids forking an already-initialized CUDA context.
        import multiprocessing as mp

        ctx = mp.get_context("spawn")
        with ProcessPoolExecutor(max_workers=workers, mp_context=ctx) as pool:
            futs = {pool.submit(_refill_one_episode_job, job): job for job in jobs}
            for fut in as_completed(futs):
                stats = fut.result()
                results.append(stats)
                print(
                    f"  ep{stats['episode_index']:03d} T={stats['frames']} "
                    f"|token|={stats['token_abs_mean']:.4f} "
                    f"l2_vs_old={stats['mean_l2_vs_old']:.3f} "
                    f"prov={stats.get('providers')}",
                    flush=True,
                )
        results.sort(key=lambda s: int(s["episode_index"]))

    summary = {
        "dataset": str(root),
        "encoder": str(enc.resolve()),
        "encode_mode": ENCODE_MODE,
        "sonic": "v1.1",
        "encode_path": "isaac_mode2_openloop",
        "num_episodes": len(results),
        "dry_run": bool(dry_run),
        "device": device,
        "workers": workers,
        "episodes": results,
    }
    return summary


def list_lerobot_sessions(root: str | Path) -> list[Path]:
    root = Path(root).expanduser().resolve()
    if (root / "meta" / "info.json").is_file():
        return [root]
    return sorted(
        p for p in root.iterdir() if p.is_dir() and (p / "meta" / "info.json").is_file()
    )


def refill_sessions(
    roots: list[str | Path],
    *,
    encoder_path: str | Path | None = None,
    backup_root: str | Path | None = None,
    dry_run: bool = False,
    device: str = "auto",
    workers: int = 1,
) -> dict:
    """Refill one or many LeRobot dataset roots (session dirs or a parent folder)."""
    sessions: list[Path] = []
    for r in roots:
        sessions.extend(list_lerobot_sessions(r))
    # de-dupe preserve order
    seen = set()
    uniq = []
    for s in sessions:
        if s not in seen:
            seen.add(s)
            uniq.append(s)

    all_summaries = []
    for s in uniq:
        bak = None
        if backup_root is not None:
            bak = Path(backup_root).expanduser().resolve() / s.name
        summary = refill_dataset(
            s,
            encoder_path=encoder_path,
            backup_dir=bak,
            dry_run=dry_run,
            device=device,
            workers=workers,
        )
        out = s / "meta" / "motion_token_mode2_refill.json"
        if not dry_run:
            out.parent.mkdir(parents=True, exist_ok=True)
            with open(out, "w", encoding="utf-8") as f:
                json.dump(summary, f, ensure_ascii=False, indent=2)
            print(f"[refill-tokens] summary → {out}")
        all_summaries.append(summary)
    return {
        "num_sessions": len(all_summaries),
        "sessions": all_summaries,
        "device": device,
        "workers": workers,
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--dataset",
        type=Path,
        action="append",
        default=None,
        help="LeRobot dataset session dir (repeatable). Or a parent of many sessions.",
    )
    p.add_argument(
        "--encoder",
        type=Path,
        default=DEFAULT_ENCODER,
        help="SONIC v1.1 model_encoder.onnx (mode-2 capable)",
    )
    p.add_argument(
        "--backup-dir",
        type=Path,
        default=None,
        help="Optional dir to save old motion_token .npy before overwrite",
    )
    p.add_argument("--dry-run", action="store_true", help="Encode + report, do not write")
    p.add_argument(
        "--device",
        type=str,
        default="auto",
        choices=["auto", "cuda", "cpu"],
        help="ORT execution provider (auto tries CUDA then CPU)",
    )
    p.add_argument(
        "--workers",
        type=int,
        default=1,
        help="Parallel episode workers (each loads its own ORT session; use 4–8 on GPU)",
    )
    p.add_argument(
        "--summary",
        type=Path,
        default=None,
        help="Write JSON summary path (single-dataset mode only)",
    )
    add_episode_range_args(p)
    args = p.parse_args(argv)

    if not args.dataset:
        p.error("at least one --dataset is required")

    # Multi-root / parent-folder batch path (ignore --episodes when >1 session).
    sessions = []
    for d in args.dataset:
        sessions.extend(list_lerobot_sessions(d))
    sessions = list(dict.fromkeys(sessions))

    if len(sessions) > 1 or (
        len(args.dataset) == 1 and list_lerobot_sessions(args.dataset[0]) != [Path(args.dataset[0]).resolve()]
        and (not (Path(args.dataset[0]).resolve() / "meta" / "info.json").is_file())
    ):
        batch = refill_sessions(
            args.dataset,
            encoder_path=args.encoder,
            backup_root=args.backup_dir,
            dry_run=args.dry_run,
            device=args.device,
            workers=args.workers,
        )
        print(
            f"[refill-tokens] batch done: {batch['num_sessions']} sessions, "
            f"workers={batch['workers']} device={batch['device']}"
        )
        return 0

    ds = sessions[0]
    indices = resolve_episode_indices(
        ds, episodes=args.episodes, start=args.start, end=args.end
    )
    summary = refill_dataset(
        ds,
        episode_indices=indices,
        encoder_path=args.encoder,
        backup_dir=args.backup_dir,
        dry_run=args.dry_run,
        device=args.device,
        workers=args.workers,
    )
    out = args.summary
    if out is None and not args.dry_run:
        out = dataset_root(ds) / "meta" / "motion_token_mode2_refill.json"
    if out is not None:
        out = Path(out)
        out.parent.mkdir(parents=True, exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print(f"[refill-tokens] summary → {out}")
    return 0


if __name__ == "__main__":
    # Allow ``python datasets/data_check/refill_motion_tokens_mode2.py``
    if __package__ is None or __package__ == "":
        sys.path.insert(0, str(_REPO))
        from datasets.data_check.refill_motion_tokens_mode2 import main as _main

        raise SystemExit(_main())
    raise SystemExit(main())
