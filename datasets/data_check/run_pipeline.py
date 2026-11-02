"""One-shot pipeline: convert → GPU jump analysis → Isaac GUI batch validate.

Example (GUI, all episodes in range, stay open):

  conda activate sikan
  cd /path/to/Litchi-GR00T-WholeBodyControl
  python -m datasets.data_check pipeline \\
    --dataset datasets/walking/2026-11-01-15-43-46 \\
    --work-dir datasets/data_check/runs/walking_0_15 \\
    --start 0 --end 15 \\
    --env-spacing 6
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
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
    else:
        summary["num_unstable"] = "skipped"

    # GUI is the default; --headless turns it off.
    headless = bool(args.headless)

    rc = 0
    if not args.skip_isaac:
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
