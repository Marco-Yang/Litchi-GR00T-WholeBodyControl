"""Rewrite ``action.motion_token`` with reproducible SONIC v1.1 mode-2 tokens.

Matches ``datasets.data_check.refill_motion_tokens_mode2`` / Isaac mode-2 screening:

  - encoder inputs: SMPL joints + Pico wrist parse + **open-loop** heading
  - ONNX: ``gear_sonic_deploy/policy/sonic_v1_1/model_encoder.onnx``

Live deploy tokens (closed-loop heading) remain usable for control; this module
is for **disk supervision** only.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_ENCODER = (
    _REPO_ROOT / "gear_sonic_deploy" / "policy" / "sonic_v1_1" / "model_encoder.onnx"
)
# IsaacLab wrist DOF order used by mode-2 encoder obs (L/R roll,pitch,yaw)
_ISAACLAB_WRIST_DOF_INDICES = (23, 24, 25, 26, 27, 28)


def _import_refill():
    """Import refill helpers without relying on the ``datasets`` package name.

    Exporter processes already bind ``datasets`` to HuggingFace (via the local
    shim / site-packages). ``from datasets.data_check ...`` then fails with
    ``No module named 'datasets.data_check'``. Load the repo files by path.
    """
    import importlib.util
    import sys
    import types

    dc_dir = _REPO_ROOT / "datasets" / "data_check"
    # Ensure sibling imports inside refill/convert (``from .constants`` etc.) work.
    pkg_name = "_sonic_data_check_rt"
    if pkg_name not in sys.modules:
        pkg = types.ModuleType(pkg_name)
        pkg.__path__ = [str(dc_dir)]  # type: ignore[attr-defined]
        pkg.__file__ = str(dc_dir / "__init__.py")
        sys.modules[pkg_name] = pkg

    def _load(mod_leaf: str):
        full = f"{pkg_name}.{mod_leaf}"
        if full in sys.modules:
            return sys.modules[full]
        path = dc_dir / f"{mod_leaf}.py"
        spec = importlib.util.spec_from_file_location(full, path)
        if spec is None or spec.loader is None:
            raise ImportError(f"cannot load {path}")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[full] = mod
        spec.loader.exec_module(mod)
        return mod

    # Preload modules that convert/refill import relatively.
    for leaf in (
        "constants",
        "lerobot_io",
        "convert_to_mode2",
        "refill_motion_tokens_mode2",
    ):
        _load(leaf)

    convert = sys.modules[f"{pkg_name}.convert_to_mode2"]
    refill = sys.modules[f"{pkg_name}.refill_motion_tokens_mode2"]
    return (
        convert.compute_wrist_joint_pos_il,
        refill._load_encoder_session,
        refill.encode_episode_tokens_isaac_mode2,
    )


def wrists6_from_smpl_pose(smpl_pose: np.ndarray) -> np.ndarray:
    """(T, 6) wrists in IsaacLab encoder order — same as §8 refill."""
    compute_wrist_joint_pos_il, _, _ = _import_refill()
    joint_pos_il = compute_wrist_joint_pos_il(smpl_pose)
    return np.asarray(joint_pos_il[:, _ISAACLAB_WRIST_DOF_INDICES], dtype=np.float32)


def encode_supervised_mode2_tokens(
    smpl_joints: np.ndarray,
    smpl_pose: np.ndarray,
    body_quat_w: np.ndarray,
    *,
    encoder_path: str | Path | None = None,
    device: str = "cpu",
) -> tuple[np.ndarray, dict[str, Any]]:
    """Encode (T, 64) supervised tokens. Returns (tokens, meta)."""
    compute_wrist_joint_pos_il, load_sess, encode_fn = _import_refill()
    del compute_wrist_joint_pos_il  # wrists via helper below

    smpl_joints = np.asarray(smpl_joints, dtype=np.float32)
    if smpl_joints.ndim == 2 and smpl_joints.shape[-1] == 72:
        pass
    elif smpl_joints.ndim == 3 and smpl_joints.shape[-1] == 3:
        smpl_joints = smpl_joints.reshape(smpl_joints.shape[0], -1).astype(np.float32)
    else:
        raise ValueError(f"bad smpl_joints shape {smpl_joints.shape}")

    smpl_pose = np.asarray(smpl_pose, dtype=np.float32)
    if smpl_pose.ndim == 2 and smpl_pose.shape[-1] == 63:
        pass
    elif smpl_pose.ndim == 3:
        smpl_pose = smpl_pose.reshape(smpl_pose.shape[0], -1).astype(np.float32)
    else:
        raise ValueError(f"bad smpl_pose shape {smpl_pose.shape}")

    body_quat_w = np.asarray(body_quat_w, dtype=np.float64)
    if body_quat_w.ndim != 2 or body_quat_w.shape[-1] != 4:
        raise ValueError(f"bad body_quat_w shape {body_quat_w.shape}")

    t = smpl_joints.shape[0]
    if smpl_pose.shape[0] != t or body_quat_w.shape[0] != t:
        raise ValueError(
            f"length mismatch joints={t} pose={smpl_pose.shape[0]} quat={body_quat_w.shape[0]}"
        )

    wrists6 = wrists6_from_smpl_pose(smpl_pose)
    enc = Path(encoder_path) if encoder_path else _DEFAULT_ENCODER
    sess, inp_name, out_name, providers = load_sess(enc, device=device)
    tokens = encode_fn(sess, inp_name, out_name, smpl_joints, wrists6, body_quat_w)
    meta = {
        "encode_path": "isaac_mode2_openloop",
        "sonic": "v1.1",
        "encode_mode": 2,
        "encoder": str(enc.resolve()),
        "ort_providers": list(providers),
        "num_frames": int(t),
        "token_abs_mean": float(np.abs(tokens).mean()),
    }
    return tokens.astype(np.float64), meta


def rewrite_episode_buffer_motion_tokens(
    episode_buffer: dict,
    *,
    encoder_path: str | Path | None = None,
    device: str = "cpu",
    live_backup_dir: Path | None = None,
    episode_index: int | None = None,
) -> dict[str, Any]:
    """In-place replace ``action.motion_token`` list in an unsaved episode buffer.

    Expects LeRobot-style buffer lists (pre-``np.stack``). Saves live tokens to
    ``live_backup_dir/episode_XXXXXX_motion_token_live.npy`` when provided.
    """
    size = int(episode_buffer.get("size", 0))
    if size <= 0:
        return {"skipped": True, "reason": "empty_buffer"}

    required = ("teleop.smpl_joints", "teleop.smpl_pose", "teleop.body_quat_w", "action.motion_token")
    for key in required:
        if key not in episode_buffer:
            return {"skipped": True, "reason": f"missing_{key}"}
        if len(episode_buffer[key]) != size:
            return {
                "skipped": True,
                "reason": f"len_mismatch_{key}",
                "size": size,
                "len": len(episode_buffer[key]),
            }

    live = np.stack(
        [np.asarray(x, dtype=np.float64).reshape(-1) for x in episode_buffer["action.motion_token"]],
        axis=0,
    )
    smpl_joints = np.stack(
        [np.asarray(x, dtype=np.float32).reshape(-1) for x in episode_buffer["teleop.smpl_joints"]],
        axis=0,
    )
    smpl_pose = np.stack(
        [np.asarray(x, dtype=np.float32).reshape(-1) for x in episode_buffer["teleop.smpl_pose"]],
        axis=0,
    )
    body_quat_w = np.stack(
        [np.asarray(x, dtype=np.float64).reshape(-1) for x in episode_buffer["teleop.body_quat_w"]],
        axis=0,
    )

    # If SMPL stream was gated to zeros for most frames, skip rewrite (keep live).
    smpl_abs = float(np.abs(smpl_joints).mean())
    if smpl_abs < 1e-6:
        return {
            "skipped": True,
            "reason": "smpl_all_zero",
            "live_token_abs_mean": float(np.abs(live).mean()),
        }

    tokens, meta = encode_supervised_mode2_tokens(
        smpl_joints,
        smpl_pose,
        body_quat_w,
        encoder_path=encoder_path,
        device=device,
    )

    if live_backup_dir is not None:
        live_backup_dir = Path(live_backup_dir)
        live_backup_dir.mkdir(parents=True, exist_ok=True)
        ei = int(episode_index if episode_index is not None else episode_buffer.get("episode_index", 0))
        live_path = live_backup_dir / f"episode_{ei:06d}_motion_token_live.npy"
        np.save(live_path, live.astype(np.float64))
        meta["live_token_backup"] = str(live_path)
        meta["live_token_abs_mean"] = float(np.abs(live).mean())
        meta["l2_vs_live_mean"] = float(np.linalg.norm(live - tokens, axis=-1).mean())

    episode_buffer["action.motion_token"] = [tokens[i].astype(np.float64) for i in range(size)]
    meta["rewrote"] = True
    meta["episode_index"] = int(
        episode_index if episode_index is not None else episode_buffer.get("episode_index", -1)
    )

    # Append per-episode record under dataset meta when possible
    return meta


def append_rewrite_record(dataset_root: Path, record: dict) -> None:
    """Append one JSONL line to ``meta/motion_token_supervised_rewrite.jsonl``."""
    path = Path(dataset_root) / "meta" / "motion_token_supervised_rewrite.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")
