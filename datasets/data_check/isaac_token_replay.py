"""Isaac replay of **stored** ``action.motion_token`` (not mode-2 re-encode).

Loads parquet FSQ tokens (64 = 2×32), bypasses the SMPL encoder, and decodes
them through the loaded SONIC policy (``forward_with_external_tokens``) into
29-DoF body actions — same decode path as deploy Protocol-v4 token injection.
Mode-2 carrier pkls are only used for clip timing / ref motion.

Example:

  python -m datasets.data_check isaac-token-replay \\
    --dataset datasets/pick_toy --seed 42 --num 5 \\
    --work-dir datasets/data_check/runs/pick_toy_token_direct_sample5 \\
    --save-videos --headless --exit-after-metrics
"""

from __future__ import annotations

import argparse
import json
import os
import pickle
import random
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

_THIS = Path(__file__).resolve().parent
_REPO = _THIS.parents[1]
for _p in (_THIS / "_vendor", _REPO):
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from datasets.data_check.convert_to_mode2 import convert_dataset  # noqa: E402
from datasets.data_check.isaac_batch_validate import (  # noqa: E402
    build_eval_command,
    _repo_root,
)


def _list_episodes(pick_root: Path) -> list[tuple[Path, int, int]]:
    out = []
    for s in sorted(p for p in pick_root.iterdir() if (p / "meta" / "info.json").is_file()):
        for line in open(s / "meta" / "episodes.jsonl", encoding="utf-8"):
            if not line.strip():
                continue
            e = json.loads(line)
            out.append((s, int(e["episode_index"]), int(e.get("length", 0))))
    return out


def _load_tokens_and_hands(sess: Path, episode_index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    info = json.load(open(sess / "meta" / "info.json", encoding="utf-8"))
    chunks = int(info.get("chunks_size", 1000))
    chunk = episode_index // chunks
    pattern = info.get(
        "data_path",
        "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
    )
    pq_path = sess / pattern.format(episode_chunk=chunk, episode_index=episode_index)
    table = pq.read_table(pq_path)
    t = table.num_rows
    tok = np.asarray(table.column("action.motion_token").to_pylist(), dtype=np.float32).reshape(t, -1)
    if tok.shape[1] != 64:
        raise ValueError(f"bad token dim {tok.shape} in {pq_path}")
    left = np.asarray(table.column("teleop.left_hand_joints").to_pylist(), dtype=np.float32).reshape(t, 7)
    right = np.asarray(table.column("teleop.right_hand_joints").to_pylist(), dtype=np.float32).reshape(t, 7)
    return tok, left, right


def prepare_bundle(
    samples: list[tuple[Path, int, int]],
    work: Path,
) -> dict:
    """Build mode2 carrier motions + token .npy sidecars with unique keys."""
    mode2 = work / "mode2"
    tok_dir = work / "tokens"
    robot_out = mode2 / "robot_motions"
    smpl_out = mode2 / "smpl_motions"
    for d in (robot_out, smpl_out, tok_dir):
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True, exist_ok=True)

    meta = {"encode_path": "direct_latent_token", "sonic": "v1.1", "episodes": []}
    for sess, ei, length in samples:
        tmp = work / f"_tmp_{sess.name}_{ei:06d}"
        if tmp.exists():
            shutil.rmtree(tmp)
        convert_dataset(sess, tmp, [ei])
        old_key = f"episode_{ei:06d}"
        new_key = f"{sess.name}__episode_{ei:06d}"

        with open(tmp / "robot_motions" / f"{old_key}.pkl", "rb") as f:
            robot = pickle.load(f)
        entry = robot[old_key]
        # Ensure teleop hands from parquet are on the carrier (Revo2 7+7).
        tok, left, right = _load_tokens_and_hands(sess, ei)
        entry["hand_action_left"] = left
        entry["hand_action_right"] = right
        # Keep dof hands in sync if present
        if "dof" in entry and entry["dof"].shape[-1] >= 43:
            dof = np.asarray(entry["dof"], dtype=np.float32).copy()
            n = min(dof.shape[0], left.shape[0])
            dof[:n, 29:36] = left[:n]
            dof[:n, 36:43] = right[:n]
            entry["dof"] = dof
        entry["source_session"] = sess.name
        entry["source_episode"] = int(ei)
        entry["token_source"] = "action.motion_token"
        with open(robot_out / f"{new_key}.pkl", "wb") as f:
            pickle.dump({new_key: entry}, f, protocol=pickle.HIGHEST_PROTOCOL)

        with open(tmp / "smpl_motions" / f"{old_key}.pkl", "rb") as f:
            smpl = pickle.load(f)
        smpl_entry = smpl[old_key] if isinstance(smpl, dict) and old_key in smpl else smpl
        with open(smpl_out / f"{new_key}.pkl", "wb") as f:
            pickle.dump(smpl_entry, f, protocol=pickle.HIGHEST_PROTOCOL)

        np.save(tok_dir / f"{new_key}.npy", tok)
        shutil.rmtree(tmp)
        meta["episodes"].append(
            {
                "key": new_key,
                "session": sess.name,
                "episode_index": ei,
                "length": int(tok.shape[0]),
                "token_npy": str(tok_dir / f"{new_key}.npy"),
                "token_abs_mean": float(np.abs(tok).mean()),
            }
        )
        print(
            f"[token-replay] prepared {new_key} T={tok.shape[0]} "
            f"|token|={float(np.abs(tok).mean()):.4f}"
        )

    with open(mode2 / "mode2_meta.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    with open(work / "sample_manifest.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    return meta


def _write_patch_launcher(work: Path, token_dir: Path) -> Path:
    """Write a small runner that patches ImEval to decode stored tokens via policy."""
    path = work / "_run_token_eval.py"
    path.write_text(
        f'''#!/usr/bin/env python
"""Auto-generated: patch im_eval to decode stored motion_token via policy decoder.

Stored tokens are SONIC v1.1 FSQ codes (T, 64) = (T, 2, 32). We bypass the
encoder with ``forward_with_external_tokens`` and step the env with decoded
29-DoF body actions (same path as deploy Protocol-v4 token injection).
"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
import torch

REPO = Path({str(_REPO)!r})
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

TOKEN_DIR = Path({str(token_dir)!r})
_TOKEN_CACHE: dict[str, torch.Tensor] = {{}}


def _load_tokens(device):
    global _TOKEN_CACHE
    if _TOKEN_CACHE:
        return _TOKEN_CACHE
    for p in sorted(TOKEN_DIR.glob("*.npy")):
        arr = np.load(p).astype(np.float32)
        _TOKEN_CACHE[p.stem] = torch.from_numpy(arr)
    print(f"[token-replay] loaded {{len(_TOKEN_CACHE)}} token clips from {{TOKEN_DIR}}", flush=True)
    return _TOKEN_CACHE


def _resolve_tok(cache, key):
    stem = Path(str(key)).stem
    if stem in cache:
        return cache[stem]
    for k, v in cache.items():
        if stem.endswith(k) or k.endswith(stem) or stem == k:
            return v
    return None


def _gather_tokens(self, device):
    cache = _load_tokens(device)
    inner = getattr(self.env, "env", self.env)
    motion_cmd = inner.command_manager.get_term("motion")
    motion_lib = motion_cmd.motion_lib
    keys = list(getattr(motion_lib, "_motion_data_keys", []))
    motion_ids = motion_cmd.motion_ids
    t = (motion_cmd.motion_start_time_steps + motion_cmd.time_steps).long()
    num_envs = self.env.num_envs
    flat = torch.zeros(num_envs, 64, device=device, dtype=torch.float32)
    for i in range(num_envs):
        mid = int(motion_ids[i].item())
        key = keys[mid] if mid < len(keys) else None
        tok = _resolve_tok(cache, key)
        if tok is None:
            raise KeyError(
                f"no token clip for motion_id={{mid}} key={{key!r}} "
                f"available={{list(cache.keys())}}"
            )
        ti = int(t[i].item())
        ti = max(0, min(ti, tok.shape[0] - 1))
        flat[i] = tok[ti].to(device)
    # (B, 64) → (B, 2, 32) FSQ layout expected by forward_with_external_tokens
    return flat.view(num_envs, 2, 32)


def _patch_im_eval():
    import gear_sonic.trl.callbacks.im_eval_callback as im_mod

    def _pre_eval_env_step(self, actor_state):
        device = self.env.device
        dones = actor_state.get(
            "dones", torch.zeros(self.env.num_envs, device=device)
        )
        obs_dict = actor_state["obs"]
        external = _gather_tokens(self, device)

        policy = self.model.policy
        # Keep proprio history buffer in sync (same as act_inference).
        policy._update_obs_buffer(obs_dict, None, dones)
        obs_last = {{k: v[:, -1:] for k, v in policy.obs_dict_buffer.items()}}
        with torch.no_grad():
            actions = policy.actor_module.forward_with_external_tokens(
                input_data=obs_last, external_tokens=external
            )
        if actions.dim() == 3 and actions.shape[1] == 1:
            actions = actions.squeeze(1)
        actor_state["actions"] = actions
        return actor_state

    im_mod.ImEvalCallback._pre_eval_env_step = _pre_eval_env_step
    print(
        "[token-replay] patched ImEvalCallback → decode stored motion_token "
        "via forward_with_external_tokens",
        flush=True,
    )


if __name__ == "__main__":
    argv = sys.argv[1:]
    sys.argv = ["eval_agent_trl.py"] + argv
    _patch_im_eval()
    import runpy
    runpy.run_path(str(REPO / "gear_sonic" / "eval_agent_trl.py"), run_name="__main__")
''',
        encoding="utf-8",
    )
    return path


def run_isaac_token_replay(
    *,
    mode2_dir: Path,
    token_dir: Path,
    output_dir: Path,
    checkpoint: Path,
    num_envs: int,
    headless: bool,
    env_spacing: float,
    save_videos: bool,
    keep_gui_alive: bool,
    python_exe: str,
) -> int:
    robot_dir = mode2_dir / "robot_motions"
    smpl_dir = mode2_dir / "smpl_motions"
    n_motions = len(list(robot_dir.glob("*.pkl")))
    if num_envs <= 0:
        num_envs = n_motions
    output_dir.mkdir(parents=True, exist_ok=True)
    video_dir = output_dir / "videos" if save_videos else None

    # Reuse mode2 command builder, then swap python entry to patched launcher.
    cmd = build_eval_command(
        python_exe=python_exe,
        checkpoint=checkpoint,
        robot_motion_dir=robot_dir,
        smpl_motion_dir=smpl_dir,
        output_dir=output_dir,
        num_envs=num_envs,
        headless=headless,
        env_spacing=env_spacing,
        max_steps_hint=None,
        keep_gui_alive=keep_gui_alive,
        fall_filter=False,
        save_videos=save_videos,
        video_dir=video_dir,
        encode_mode=2,  # still need smpl motion lib for clip timing; encoder bypassed
    )
    # Tokens are decoded inside the patched ImEval callback via the loaded
    # sonic policy (forward_with_external_tokens). No ATM / student_direct_latent
    # flags — those require a separate action_transform_module.

    launcher = _write_patch_launcher(mode2_dir.parent, token_dir)
    # Replace eval_agent_trl.py path with our patched launcher
    for i, part in enumerate(cmd):
        if part.endswith("eval_agent_trl.py"):
            cmd[i] = str(launcher)
            break

    print("=" * 72)
    print(" Isaac token replay (direct_latent / stored motion_token)")
    print("=" * 72)
    print(f"  motions:  {n_motions}")
    print(f"  tokens:   {token_dir}")
    print(f"  videos:   {save_videos}")
    print(f"  output:   {output_dir}")
    print("=" * 72)
    print(" \\\n  ".join(cmd))

    env = os.environ.copy()
    env.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    env.setdefault("CUDA_VISIBLE_DEVICES", env.get("CUDA_VISIBLE_DEVICES", "0"))
    vendor = Path(__file__).resolve().parent / "_vendor"
    py_parts = []
    if vendor.is_dir():
        py_parts.append(str(vendor))
    if env.get("PYTHONPATH"):
        py_parts.append(env["PYTHONPATH"])
    if py_parts:
        env["PYTHONPATH"] = os.pathsep.join(py_parts)

    return int(subprocess.run(cmd, cwd=str(_repo_root()), env=env, check=False).returncode)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", type=Path, default=_REPO / "datasets" / "pick_toy")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--num", type=int, default=5, help="How many episodes to sample")
    p.add_argument(
        "--work-dir",
        type=Path,
        default=_THIS / "runs" / "pick_toy_token_direct_sample5",
    )
    p.add_argument("--checkpoint", type=Path, default=_REPO / "sonic_v1_1" / "last.pt")
    p.add_argument("--env-spacing", type=float, default=6.0)
    p.add_argument("--headless", action="store_true")
    p.add_argument("--exit-after-metrics", action="store_true")
    p.add_argument("--save-videos", action="store_true")
    p.add_argument("--python", type=str, default="")
    p.add_argument("--skip-prepare", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args(argv)

    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)

    if not args.skip_prepare:
        all_eps = _list_episodes(args.dataset.resolve())
        if len(all_eps) < args.num:
            raise RuntimeError(f"only {len(all_eps)} episodes available")
        random.seed(args.seed)
        samples = sorted(random.sample(all_eps, args.num), key=lambda x: (x[0].name, x[1]))
        prepare_bundle(samples, work)
    else:
        print(f"[token-replay] reuse existing bundle under {work}")

    if args.dry_run:
        return 0

    py = args.python or sys.executable
    rc = run_isaac_token_replay(
        mode2_dir=work / "mode2",
        token_dir=work / "tokens",
        output_dir=work / "isaac_eval",
        checkpoint=args.checkpoint.resolve(),
        num_envs=0,
        headless=bool(args.headless),
        env_spacing=float(args.env_spacing),
        save_videos=bool(args.save_videos),
        keep_gui_alive=not bool(args.exit_after_metrics),
        python_exe=py,
    )

    # Rename videos using manifest order (env i ↔ sorted motion keys)
    if args.save_videos and rc == 0:
        manifest = json.load(open(work / "sample_manifest.json", encoding="utf-8"))
        keys = [e["key"] for e in manifest["episodes"]]
        # motion lib usually sorts by filename
        keys_sorted = sorted(keys)
        vdir = work / "isaac_eval" / "videos"
        out_dir = work / "videos"
        out_dir.mkdir(parents=True, exist_ok=True)
        for i, key in enumerate(keys_sorted):
            src = vdir / f"{i:06d}.mp4"
            dst = out_dir / f"{key}.mp4"
            if src.is_file():
                shutil.copy2(src, dst)
                print(f"[token-replay] video {src.name} → {dst}")
            else:
                print(f"[token-replay] missing {src}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
