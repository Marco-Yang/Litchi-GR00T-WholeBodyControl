"""One-shot pipeline: convert → stability screen → mode2 token refill → Isaac GUI.

Steps:
  1. Convert LeRobot episodes → mode-2 teleop bundles
  2. GPU joint-stability analysis; drop unstable (optional)
  3. Refill ``action.motion_token`` with SONIC **v1.1 encoder mode 2**
     using the same inputs as Isaac mode-2 screening
     (``convert_to_mode2``: SMPL joints + Pico wrist parse, open-loop heading;
     matches ``isaac_mode2_walking_*``, not the recorded teleop token_state)
  4. Isaac GUI batch replay for visual screening

Example (GUI, all episodes in range, stay open):

  conda activate sikan
  cd /path/to/Litchi-GR00T-WholeBodyControl
  python -m datasets.data_check pipeline \\
    --dataset datasets/walking/2026-11-01-15-43-46 \\
    --work-dir datasets/data_check/runs/walking_0_15 \\
    --start 0 --end 15 \\
    --env-spacing 6

Token refill alone (no Isaac):

  python -m datasets.data_check refill-tokens \\
    --dataset datasets/pick_toy/2026-11-07-14-13-29
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import sys

# Allow ``python datasets/data_check/run_pipeline.py`` as well as ``-m``.
_THIS = Path(__file__).resolve().parent
_REPO = _THIS.parents[1]
_VENDOR = _THIS / "_vendor"
for _p in (_VENDOR, _REPO):
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from datasets.data_check.analyze_joint_stability import analyze_dataset  # noqa: E402
from datasets.data_check.convert_to_mode2 import convert_dataset  # noqa: E402
from datasets.data_check.isaac_batch_validate import run_isaac_batch  # noqa: E402
from datasets.data_check.lerobot_io import (  # noqa: E402
    add_episode_range_args,
    resolve_episode_indices,
)
from datasets.data_check.refill_motion_tokens_mode2 import (  # noqa: E402
    DEFAULT_ENCODER,
    refill_dataset,
)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", type=Path, default=_REPO / "datasets" / "walking" / "2026-11-01-15-43-46")
    p.add_argument(
        "--work-dir",
        type=Path,
        default=_THIS / "runs" / "walking_check",
        help="Working directory for mode2 cache + reports + isaac outputs",
    )
    add_episode_range_args(p)
    p.add_argument(
        "--num-envs",
        type=int,
        default=0,
        help="Parallel G1 count; 0 = spawn all episodes in the selected range",
    )
    p.add_argument(
        "--env-spacing",
        type=float,
        default=6.0,
        help="Meters between G1 env origins (default 6; avoid overlap)",
    )
    p.add_argument("--checkpoint", type=Path, default=None)
    p.add_argument(
        "--headless",
        action="store_true",
        help="Disable Isaac GUI (default: GUI ON for visual inspection)",
    )
    p.add_argument(
        "--exit-after-metrics",
        action="store_true",
        help="Close Isaac right after metrics (default: keep GUI alive)",
    )
    p.add_argument("--skip-convert", action="store_true")
    p.add_argument("--skip-analyze", action="store_true")
    p.add_argument("--skip-isaac", action="store_true")
    p.add_argument(
        "--skip-token-refill",
        action="store_true",
        help="Skip SONIC v1.1 mode-2 motion_token rewrite on the LeRobot dataset",
    )
    p.add_argument(
        "--encoder",
        type=Path,
        default=DEFAULT_ENCODER,
        help="SONIC v1.1 model_encoder.onnx used for mode-2 token refill",
    )
    p.add_argument(
        "--token-device",
        type=str,
        default="auto",
        choices=["auto", "cuda", "cpu"],
        help="ORT device for motion_token refill (auto prefers CUDA)",
    )
    p.add_argument(
        "--token-workers",
        type=int,
        default=4,
        help="Parallel episode workers for motion_token refill (default 4)",
    )
    p.add_argument(
        "--keep-unstable",
        action="store_true",
        help="Keep episodes that fail the joint-stability screen (default: drop them)",
    )
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--cpu-analyze", action="store_true")
    p.add_argument("--python", type=str, default="")
    args = p.parse_args(argv)

    work = args.work_dir.resolve()
    mode2_dir = work / "mode2"
    report_json = work / "joint_stability.json"
    isaac_out = work / "isaac_eval"
    work.mkdir(parents=True, exist_ok=True)

    indices = resolve_episode_indices(
        args.dataset, episodes=args.episodes, start=args.start, end=args.end
    )
    nothing_left = False
    if indices is not None:
        print(f"[pipeline] episodes {indices[0]}..{indices[-1]} (n={len(indices)})")

    summary = {
        "dataset": str(args.dataset.resolve()),
        "work_dir": str(work),
        "episode_indices": indices,
        "start": args.start,
        "end": args.end,
    }

    if not args.skip_convert:
        meta = convert_dataset(args.dataset, mode2_dir, indices)
        summary["num_converted"] = len(meta["episodes"])
    else:
        summary["num_converted"] = "skipped"

    if not args.skip_analyze:
        report = analyze_dataset(
            args.dataset,
            report_json,
            indices,
            prefer_gpu=not args.cpu_analyze,
        )
        summary["num_unstable"] = report["num_unstable"]
        summary["unstable_episode_indices"] = report["unstable_episode_indices"]
        print(
            f"[pipeline] unstable episodes: {report['unstable_episode_indices']}"
        )
        if report["unstable_episode_indices"] and not args.keep_unstable and not args.dry_run:
            from datasets.data_check.drop_unstable import drop_episodes

            details = {
                int(row["episode_index"]): {"flags": row.get("flags", [])}
                for row in report["episodes"]
                if row.get("unstable")
            }
            dropped = drop_episodes(
                args.dataset,
                report["unstable_episode_indices"],
                details=details,
            )
            summary["dropped_episode_indices"] = dropped["dropped"]
            if indices is not None:
                indices = [
                    dropped["old_to_new"][i]
                    for i in indices
                    if i not in set(dropped["dropped"])
                ]
            if mode2_dir.exists():
                shutil.rmtree(mode2_dir)
            if indices is not None and not indices:
                summary["num_converted"] = 0
                nothing_left = True
                print("[pipeline] every selected episode was unstable; nothing left to replay")
            else:
                meta = convert_dataset(args.dataset, mode2_dir, indices)
                summary["num_converted"] = len(meta["episodes"])
        else:
            summary["dropped_episode_indices"] = []
    else:
        summary["num_unstable"] = "skipped"
        summary["dropped_episode_indices"] = []

    # Rewrite action.motion_token with SONIC v1.1 mode-2 (SMPL + wrist qpos)
    # after unstable drop so remaining parquet clips get fresh latents.
    if not args.skip_token_refill and not nothing_left and not args.dry_run:
        token_backup = work / "motion_token_backup"
        token_summary = refill_dataset(
            args.dataset,
            episode_indices=indices,
            encoder_path=args.encoder,
            backup_dir=token_backup,
            dry_run=False,
            device=args.token_device,
            workers=max(1, int(args.token_workers)),
        )
        n_tok = max(len(token_summary["episodes"]), 1)
        mean_abs = sum(e["token_abs_mean"] for e in token_summary["episodes"]) / n_tok
        summary["token_refill"] = {
            "encode_mode": 2,
            "sonic": "v1.1",
            "encode_path": "isaac_mode2_openloop",
            "encoder": str(Path(args.encoder).resolve()),
            "num_episodes": token_summary["num_episodes"],
            "backup_dir": str(token_backup),
            "mean_token_abs": float(mean_abs),
        }
        summary_path = Path(args.dataset).resolve() / "meta" / "motion_token_mode2_refill.json"
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(token_summary, f, ensure_ascii=False, indent=2)
        print(
            f"[pipeline] refilled motion_token (v1.1 mode2 isaac-openloop) for "
            f"{token_summary['num_episodes']} episodes; "
            f"|token|mean={mean_abs:.4f}"
        )
    elif args.skip_token_refill:
        summary["token_refill"] = "skipped"
    elif args.dry_run:
        summary["token_refill"] = "skipped_dry_run"
    else:
        summary["token_refill"] = "skipped_empty"

    # GUI is the default; --headless turns it off.
    headless = bool(args.headless)

    rc = 0
    if not args.skip_isaac and not nothing_left:
        ckpt = args.checkpoint or (_REPO / "sonic_v1_1" / "last.pt")
        rc = run_isaac_batch(
            mode2_dir=mode2_dir,
            checkpoint=ckpt,
            output_dir=isaac_out,
            num_envs=args.num_envs,
            headless=headless,
            env_spacing=args.env_spacing,
            dry_run=args.dry_run,
            python_exe=args.python or None,
            keep_gui_alive=not bool(args.exit_after_metrics),
        )
        summary["isaac_returncode"] = rc
    else:
        summary["isaac_returncode"] = "skipped"

    with open(work / "pipeline_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return int(rc)


if __name__ == "__main__":
    raise SystemExit(main())
