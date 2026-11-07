"""Batch-validate walking episodes in Isaac Sim with SONIC encoders.

- Spawns multiple G1 envs (each env = one episode / motion clip)
- ``encode_mode=2`` → ``use_encoder=smpl`` (SMPL / mode-2)
- ``encode_mode=0`` → ``use_encoder=g1`` (qpos / mode-0)
- **GUI on by default** so you can visually inspect tracking / balance
- Writes eval metrics under ``--output``

Requires the Isaac Lab conda env (e.g. ``sikan``) with ``gear_sonic`` importable
from this repo, plus ``sonic_v1_1/last.pt``.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


def _repo_root() -> Path:
    # datasets/data_check -> repo root
    return Path(__file__).resolve().parents[2]


def build_eval_command(
    *,
    python_exe: str,
    checkpoint: Path,
    robot_motion_dir: Path,
    smpl_motion_dir: Path | None,
    output_dir: Path,
    num_envs: int,
    headless: bool,
    env_spacing: float,
    max_steps_hint: int | None,
    keep_gui_alive: bool,
    fall_filter: bool = True,
    save_videos: bool = False,
    video_dir: Path | None = None,
    encode_mode: int = 2,
) -> list[str]:
    """Build ``gear_sonic/eval_agent_trl.py`` argv for multi-G1 eval."""
    if int(encode_mode) not in (0, 2):
        raise ValueError(f"encode_mode must be 0 or 2, got {encode_mode}")
    use_encoder = "g1" if int(encode_mode) == 0 else "smpl"
    metrics_dir = output_dir / "metrics"
    metrics_dir.mkdir(parents=True, exist_ok=True)
    if save_videos:
        video_dir = Path(video_dir or (output_dir / "videos"))
        video_dir.mkdir(parents=True, exist_ok=True)
    # GUI dynamics check: keep stepping after metrics so the window does not
    # immediately os._exit. Headless metrics-only stays on the one-shot path.
    run_eval_loop = bool(keep_gui_alive and not headless)
    # Pull camera back for a spaced multi-G1 grid (approx sqrt(N) * spacing).
    grid = max(1.0, float(num_envs) ** 0.5)
    eye_xy = max(6.0, 0.55 * grid * float(env_spacing))
    eye_z = max(4.0, 0.35 * grid * float(env_spacing))
    if fall_filter:
        term_overrides = [
            "+manager_env/terminations=tracking/data_check_teleop",
            "+data_check_teleop_filter=true",
        ]
    else:
        # GUI pose inspection: only time_out so robots keep playing the stream.
        term_overrides = [
            "+manager_env/terminations=tracking/data_check_gui",
            "+data_check_loose_terminations=true",
        ]
    if int(encode_mode) == 0:
        # Joint-based g1 encoder; no SMPL stream required.
        smpl_file_override = "dummy"
        smpl_y_up = "false"
        smpl_teleop = "false"
    else:
        if smpl_motion_dir is None:
            raise ValueError("smpl_motion_dir required for encode_mode=2")
        smpl_file_override = str(smpl_motion_dir)
        # Pico stream already Y→Z + base_rot + local joints — match C++ deploy.
        smpl_y_up = "false"
        smpl_teleop = "true"
    cmd = [
        python_exe,
        "-u",
        str(_repo_root() / "gear_sonic" / "eval_agent_trl.py"),
        f"+checkpoint={checkpoint}",
        f"+headless={str(bool(headless)).lower()}",
        "++eval_callbacks=im_eval",
        f"++run_eval_loop={str(run_eval_loop).lower()}",
        f"++use_encoder={use_encoder}",
        f"++num_envs={int(num_envs)}",
        f"++env_spacing={float(env_spacing)}",
        f"++manager_env.config.env_spacing={float(env_spacing)}",
        # Dynamics check: flat ground only (sonic_v1_1 ckpt defaults to trimesh)
        "++manager_env.config.terrain_type=plane",
        f"++manager_env.config.viewer_eye=[{eye_xy:.3f},0.0,{eye_z:.3f}]",
        "++manager_env.config.viewer_lookat=[0.0,0.0,0.8]",
        f"++eval_output_dir={metrics_dir}",
        # Unitree G1 + Dex3 (SONIC v1.1 / walking teleop)
        "++manager_env.config.robot.type=g1_model_12_dex",
        *term_overrides,
        # Snap feet to ground (enum: 0=no, 1=full, 2=ankle).
        "++manager_env.commands.motion.motion_lib_cfg.fix_height=1",
        f"++manager_env.commands.motion.motion_lib_cfg.smpl_y_up={smpl_y_up}",
        "++manager_env.commands.motion.motion_lib_cfg.smpl_teleop_preprocessed="
        f"{smpl_teleop}",
        "++manager_env.commands.motion.motion_lib_cfg.motion_file="
        f"{robot_motion_dir}",
        "++manager_env.commands.motion.motion_lib_cfg.smpl_motion_file="
        f"{smpl_file_override}",
        "++manager_env.commands.motion.motion_lib_cfg.multi_thread=true",
        # Dex/Revo2 hands: 14 = 7+7 so dof[:,29:] is consumed as hand_dof_pos
        "++manager_env.commands.motion.motion_lib_cfg.hand_dof_count=14",
        # GUI: disable periodic cuda empty_cache (was freq=10 → 1–2s collective stutter).
        # Headless metrics can afford occasional cleanup.
        (
            "++callbacks.im_eval.empty_cache_freq=0"
            if not headless
            else "++callbacks.im_eval.empty_cache_freq=50"
        ),
        # Keep GUI responsive; video dump optional via save_videos=
        f"++algo.config.eval.save_videos={str(bool(save_videos)).lower()}",
        "+eval_debug_plot=false",
        "+show_episode_labels=true",
        # MuJoCo-like viewport: free camera (no robot lock) + hide IsaacLab docks
        "+simple_viewer=true",
    ]
    if save_videos:
        # Per-env tiled eval_camera + RenderEnvsRecorder → <video_dir>/000000.mp4 …
        assert video_dir is not None
        cmd.extend(
            [
                "manager_env/recorders=render",
                "++manager_env.config.render_results=true",
                f"++manager_env.config.save_rendering_dir={video_dir}",
                # Slightly smaller than 1080p to keep headless encode light.
                "++manager_env.config.render_width=1280",
                "++manager_env.config.render_height=720",
                "++manager_env.config.render_frame_skip=2",
                "++manager_env.config.eval_camera_offset=[3.5,3.5,1.6]",
            ]
        )
    if run_eval_loop:
        # im_eval otherwise os._exit(0) when eval_frequency==1
        cmd.append("++callbacks.im_eval.exit_after_eval=false")
        # If the eval loop ever returns, still keep the Kit window open.
        cmd.append("+keep_gui_process=true")

    if max_steps_hint is not None:
        cmd.append(f"++callbacks.im_eval.max_render_steps={int(max_steps_hint)}")
        if run_eval_loop:
            cmd.append(f"++max_render_steps={int(max_steps_hint)}")
    return cmd


def run_isaac_batch(
    *,
    mode2_dir: Path,
    checkpoint: Path,
    output_dir: Path,
    num_envs: int,
    headless: bool,
    env_spacing: float,
    dry_run: bool,
    python_exe: str | None = None,
    keep_gui_alive: bool = True,
    max_steps_hint: int | None = None,
    fall_filter: bool | None = None,
    save_videos: bool = False,
    encode_mode: int = 2,
) -> int:
    motion_dir = mode2_dir.resolve()
    robot_dir = motion_dir / "robot_motions"
    smpl_dir = motion_dir / "smpl_motions"
    encode_mode = int(encode_mode)
    if not robot_dir.is_dir():
        raise FileNotFoundError(
            f"Missing robot_motions/ under {motion_dir}. "
            "Run convert_to_mode0.py or convert_to_mode2.py first."
        )
    if encode_mode == 2 and not smpl_dir.is_dir():
        raise FileNotFoundError(
            f"Missing smpl_motions/ under {motion_dir}. "
            "Run convert_to_mode2.py first."
        )
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)

    n_motions = len(list(robot_dir.glob("*.pkl")))
    # num_envs<=0 means "spawn one G1 per motion in this dir" (full range).
    if int(num_envs) <= 0:
        num_envs = n_motions
    else:
        num_envs = max(1, min(int(num_envs), n_motions))

    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    video_dir = output_dir / "videos" if save_videos else None

    # Match the GUI 0-19 data_check setup: time_out only.
    # Teleop standing-carrier + anchor_pos/ori fall filter false-positives at ~step 9.
    # Pass fall_filter=True explicitly only when debugging that detector.
    if fall_filter is None:
        fall_filter = False
    py = python_exe or sys.executable
    use_encoder = "g1" if encode_mode == 0 else "smpl"
    cmd = build_eval_command(
        python_exe=py,
        checkpoint=checkpoint,
        robot_motion_dir=robot_dir,
        smpl_motion_dir=smpl_dir if encode_mode == 2 else None,
        output_dir=output_dir,
        num_envs=num_envs,
        headless=headless,
        env_spacing=env_spacing,
        max_steps_hint=max_steps_hint,
        keep_gui_alive=keep_gui_alive,
        fall_filter=fall_filter,
        save_videos=save_videos,
        video_dir=video_dir,
        encode_mode=encode_mode,
    )

    launch = {
        "cwd": str(_repo_root()),
        "num_envs": num_envs,
        "n_motions": n_motions,
        "headless": headless,
        "env_spacing": env_spacing,
        "terrain_type": "plane",
        "show_episode_labels": True,
        "simple_viewer": True,
        "keep_gui_alive": bool(keep_gui_alive and not headless),
        "encode_mode": encode_mode,
        "use_encoder": use_encoder,
        "checkpoint": str(checkpoint),
        "motion_dir": str(motion_dir),
        "command": cmd,
    }
    with open(output_dir / "launch_command.json", "w", encoding="utf-8") as f:
        json.dump(launch, f, ensure_ascii=False, indent=2)

    label = (
        "SONIC encode-mode-0 / g1 (qpos)"
        if encode_mode == 0
        else "SONIC encode-mode-2 / SMPL"
    )
    print("=" * 72)
    print(f" Isaac Sim batch validate ({label})")
    print("=" * 72)
    print(f"  motions:   {n_motions}")
    print(f"  num_envs:  {num_envs}  (1 G1 per episode; show full range at once)")
    print(f"  encode:    mode {encode_mode} → use_encoder={use_encoder}")
    print(f"  headless:  {headless}  (False => GUI for visual inspection)")
    print(f"  spacing:   {env_spacing} m  (env grid; robots must not overlap)")
    print("  terrain:   plane  (no trimesh background)")
    print("  labels:    episode id above each robot head")
    print("  viewer:    simple (free camera + minimal UI, MuJoCo-like)")
    print(
        f"  gui_loop:  {bool(keep_gui_alive and not headless)}  "
        "(True => keep window open after metrics)"
    )
    print(f"  ckpt:      {checkpoint}")
    print(f"  output:    {output_dir}")
    print("=" * 72)
    print("Command:")
    print(" \\\n  ".join(cmd))

    if dry_run:
        print("[dry-run] not launching Isaac")
        return 0

    # Hint NVIDIA EULA vars if unset (Isaac Lab will otherwise prompt/hang)
    env = os.environ.copy()
    env.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    # Prefer GPU 0 unless user already set CUDA_VISIBLE_DEVICES
    env.setdefault("CUDA_VISIBLE_DEVICES", env.get("CUDA_VISIBLE_DEVICES", "0"))
    # Vendored deps (open3d/trl/joblib/...). Do NOT prepend repo root here:
    # eval_agent_trl.py already inserts it, and an early repo-root entry makes
    # local datasets/ shadow HuggingFace datasets (breaks trl import).
    vendor = Path(__file__).resolve().parent / "_vendor"
    py_path_parts: list[str] = []
    if vendor.is_dir():
        py_path_parts.append(str(vendor))
    existing = env.get("PYTHONPATH", "")
    if existing:
        py_path_parts.append(existing)
    if py_path_parts:
        env["PYTHONPATH"] = os.pathsep.join(py_path_parts)

    proc = subprocess.run(cmd, cwd=str(_repo_root()), env=env, check=False)
    return int(proc.returncode)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--mode2-dir",
        type=Path,
        required=True,
        help=(
            "Motion bundle dir: mode2 has robot_motions/+smpl_motions/; "
            "mode0 needs robot_motions/ only"
        ),
    )
    p.add_argument(
        "--encode-mode",
        type=int,
        default=2,
        choices=(0, 2),
        help="0=g1/qpos encoder, 2=smpl encoder (default)",
    )
    p.add_argument(
        "--checkpoint",
        type=Path,
        default=None,
        help="SONIC Isaac checkpoint (.pt). Default: <repo>/sonic_v1_1/last.pt",
    )
    p.add_argument("--output", type=Path, required=True, help="Eval output directory")
    p.add_argument(
        "--num-envs",
        type=int,
        default=0,
        help="Parallel G1 count; 0 = one env per motion in --mode2-dir (full range)",
    )
    p.add_argument(
        "--env-spacing",
        type=float,
        default=6.0,
        help="Meters between G1 env origins (default 6; avoid overlap in GUI)",
    )
    p.add_argument(
        "--headless",
        action="store_true",
        help="Disable GUI (default is GUI ON for visual inspection)",
    )
    p.add_argument(
        "--gui",
        action="store_true",
        help="Force GUI on (default). Useful to override scripts that pass --headless",
    )
    p.add_argument(
        "--exit-after-metrics",
        action="store_true",
        help="Close Isaac immediately after im_eval metrics (default: keep GUI loop)",
    )
    p.add_argument("--dry-run", action="store_true")
    p.add_argument(
        "--save-videos",
        action="store_true",
        help="Write per-env overview mp4s under <output>/videos/",
    )
    p.add_argument(
        "--python",
        type=str,
        default="",
        help="Python with Isaac Lab (default: current interpreter)",
    )
    args = p.parse_args(argv)

    repo = _repo_root()
    ckpt = args.checkpoint or (repo / "sonic_v1_1" / "last.pt")
    headless = bool(args.headless) and not bool(args.gui)
    return run_isaac_batch(
        mode2_dir=args.mode2_dir,
        checkpoint=ckpt,
        output_dir=args.output,
        num_envs=args.num_envs,
        headless=headless,
        env_spacing=args.env_spacing,
        dry_run=args.dry_run,
        python_exe=args.python or None,
        keep_gui_alive=not bool(args.exit_after_metrics),
        save_videos=bool(args.save_videos),
        encode_mode=int(args.encode_mode),
    )


if __name__ == "__main__":
    raise SystemExit(main())
