"""Screen Isaac im_eval metrics and drop low-quality episodes.

Reads ``metrics_eval.json`` produced by ``isaac_batch_validate`` /
``im_eval`` (per-episode lists under ``eval/all_metrics_dict``), flags
failures against ``ISAAC_QUALITY_THRESHOLDS``, and optionally removes them
from the LeRobot dataset via ``drop_episodes``.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

from .constants import ISAAC_QUALITY_THRESHOLDS
from .drop_unstable import drop_episodes

_EPISODE_KEY_RE = re.compile(r"(?:^|[^0-9])(?:episode[_-]?)?(\d{1,6})$", re.IGNORECASE)


def _episode_index_from_motion_key(key: str) -> int | None:
    text = str(key).strip()
    if not text:
        return None
    # Preferred: episode_000017 / episode-17
    m = re.search(r"episode[_-]?(\d+)", text, flags=re.IGNORECASE)
    if m:
        return int(m.group(1))
    # Fallback: trailing digits
    m = _EPISODE_KEY_RE.search(text)
    if m:
        return int(m.group(1))
    return None


def _as_bool(v) -> bool:
    if isinstance(v, (bool, int, float)):
        return bool(v)
    if isinstance(v, str):
        return v.strip().lower() in {"1", "true", "yes", "y"}
    return bool(v)


def load_metrics_eval(metrics_path: str | Path) -> dict:
    path = Path(metrics_path).expanduser().resolve()
    if path.is_dir():
        cand = path / "metrics_eval.json"
        if not cand.is_file():
            cand = path / "metrics" / "metrics_eval.json"
        path = cand
    if not path.is_file():
        raise FileNotFoundError(f"metrics_eval.json not found: {metrics_path}")
    return json.loads(path.read_text(encoding="utf-8"))


def screen_metrics(
    metrics_eval: dict,
    *,
    thresholds: dict | None = None,
) -> dict:
    """Return per-episode fail list + keep indices from ``metrics_eval``."""
    thr = dict(ISAAC_QUALITY_THRESHOLDS)
    if thresholds:
        thr.update(thresholds)

    amd = metrics_eval.get("eval/all_metrics_dict")
    if not isinstance(amd, dict):
        raise ValueError("metrics_eval missing eval/all_metrics_dict")

    motion_keys = list(amd.get("motion_keys") or [])
    n = len(motion_keys)
    if n == 0:
        raise ValueError("eval/all_metrics_dict.motion_keys is empty")

    def _col(name: str, default=None):
        vals = amd.get(name)
        if vals is None:
            return [default] * n
        if len(vals) != n:
            raise ValueError(f"{name} length {len(vals)} != motion_keys {n}")
        return list(vals)

    terminated = [_as_bool(v) for v in _col("terminated", False)]
    progress = [float(v) if v is not None else 0.0 for v in _col("progress", 0.0)]
    mpjpe_l = [float(v) for v in _col("mpjpe_l", float("nan"))]
    mpjpe_pa = [float(v) for v in _col("mpjpe_pa", float("nan"))]
    joint_mse_l = [float(v) for v in _col("joint_mse_l", float("nan"))]
    accel_dist = [float(v) for v in _col("accel_dist", float("nan"))]
    vel_dist = [float(v) for v in _col("vel_dist", float("nan"))]

    # Also honor aggregate failed_idxes / failed_keys from im_eval.
    failed_idxes = {int(i) for i in (metrics_eval.get("failed_idxes") or [])}
    failed_keys = {str(k) for k in (metrics_eval.get("failed_keys") or [])}

    episodes: list[dict] = []
    bad_indices: list[int] = []
    for i, key in enumerate(motion_keys):
        ep_i = _episode_index_from_motion_key(key)
        reasons: list[str] = []
        if ep_i is None:
            reasons.append("unmapped_motion_key")
        if thr.get("drop_if_terminated", True) and terminated[i]:
            reasons.append("terminated")
        if progress[i] < float(thr["min_progress"]):
            reasons.append(f"progress<{thr['min_progress']}")
        if mpjpe_l[i] > float(thr["max_mpjpe_l"]):
            reasons.append(f"mpjpe_l>{thr['max_mpjpe_l']}")
        if mpjpe_pa[i] > float(thr["max_mpjpe_pa"]):
            reasons.append(f"mpjpe_pa>{thr['max_mpjpe_pa']}")
        if joint_mse_l[i] > float(thr["max_joint_mse_l"]):
            reasons.append(f"joint_mse_l>{thr['max_joint_mse_l']}")
        if accel_dist[i] > float(thr["max_accel_dist"]):
            reasons.append(f"accel_dist>{thr['max_accel_dist']}")
        if vel_dist[i] > float(thr["max_vel_dist"]):
            reasons.append(f"vel_dist>{thr['max_vel_dist']}")
        if i in failed_idxes or str(key) in failed_keys:
            reasons.append("im_eval_failed")

        row = {
            "motion_key": key,
            "episode_index": ep_i,
            "failed": bool(reasons),
            "reasons": reasons,
            "terminated": terminated[i],
            "progress": progress[i],
            "mpjpe_l": mpjpe_l[i],
            "mpjpe_pa": mpjpe_pa[i],
            "joint_mse_l": joint_mse_l[i],
            "accel_dist": accel_dist[i],
            "vel_dist": vel_dist[i],
        }
        episodes.append(row)
        if reasons and ep_i is not None:
            bad_indices.append(int(ep_i))

    bad_indices = sorted(set(bad_indices))
    keep_indices = sorted(
        {
            int(r["episode_index"])
            for r in episodes
            if r["episode_index"] is not None and not r["failed"]
        }
    )
    return {
        "thresholds": thr,
        "num_motions": n,
        "num_failed": len(bad_indices),
        "failed_episode_indices": bad_indices,
        "keep_episode_indices": keep_indices,
        "episodes": episodes,
    }


def write_screen_report(report: dict, work_dir: str | Path) -> dict[str, Path]:
    """Write isaac_quality.json + failed_validation.txt under ``work_dir``."""
    work = Path(work_dir).expanduser().resolve()
    work.mkdir(parents=True, exist_ok=True)
    json_path = work / "isaac_quality.json"
    txt_path = work / "failed_validation.txt"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    lines = [
        f"isaac_quality_screen failed={report['num_failed']}/{report['num_motions']}",
        f"thresholds={json.dumps(report['thresholds'], ensure_ascii=False)}",
        f"failed_episode_indices={report['failed_episode_indices']}",
        "",
    ]
    for row in report["episodes"]:
        if not row["failed"]:
            continue
        lines.append(
            f"ep{row['episode_index']} {row['motion_key']} "
            f"reasons={','.join(row['reasons'])} "
            f"progress={row['progress']:.3f} "
            f"mpjpe_l={row['mpjpe_l']:.1f} "
            f"joint_mse_l={row['joint_mse_l']:.4f} "
            f"terminated={row['terminated']}"
        )
    if report["num_failed"] == 0:
        lines.append("(none)")
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"json": json_path, "txt": txt_path}


def screen_and_drop(
    *,
    dataset: str | Path,
    metrics_path: str | Path,
    work_dir: str | Path,
    thresholds: dict | None = None,
    keep_failed: bool = False,
    dry_run: bool = False,
) -> dict:
    """Screen Isaac metrics; drop failing episodes from ``dataset`` by default."""
    metrics = load_metrics_eval(metrics_path)
    report = screen_metrics(metrics, thresholds=thresholds)
    paths = write_screen_report(report, work_dir)
    summary = {
        "dataset": str(Path(dataset).resolve()),
        "metrics": str(Path(metrics_path).resolve()),
        "num_failed": report["num_failed"],
        "failed_episode_indices": report["failed_episode_indices"],
        "report_json": str(paths["json"]),
        "report_txt": str(paths["txt"]),
        "dropped": [],
    }
    print(
        f"[isaac-screen] {report['num_failed']}/{report['num_motions']} failed "
        f"-> {paths['txt']}"
    )
    if report["failed_episode_indices"]:
        print(f"[isaac-screen] failed episodes: {report['failed_episode_indices']}")
    if keep_failed or dry_run or not report["failed_episode_indices"]:
        summary["drop"] = "skipped" if (keep_failed or dry_run) else "none_failed"
        return {**summary, "report": report}

    details = {
        int(row["episode_index"]): {
            "reasons": row["reasons"],
            "mpjpe_l": row["mpjpe_l"],
            "joint_mse_l": row["joint_mse_l"],
            "progress": row["progress"],
            "terminated": row["terminated"],
            "motion_key": row["motion_key"],
        }
        for row in report["episodes"]
        if row["failed"] and row["episode_index"] is not None
    }
    dropped = drop_episodes(
        dataset,
        report["failed_episode_indices"],
        reason="isaac_quality",
        details=details,
    )
    summary["dropped"] = dropped["dropped"]
    summary["num_episodes_kept"] = dropped["num_episodes"]
    summary["drop"] = "applied"
    summary["old_to_new"] = dropped["old_to_new"]
    return {**summary, "report": report}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument(
        "--metrics",
        type=Path,
        required=True,
        help="metrics_eval.json or its parent dir (…/isaac_eval or …/metrics)",
    )
    p.add_argument(
        "--work-dir",
        type=Path,
        required=True,
        help="Write isaac_quality.json + failed_validation.txt here",
    )
    p.add_argument(
        "--keep-failed",
        action="store_true",
        help="Only report failures; do not drop episodes",
    )
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args(argv)
    summary = screen_and_drop(
        dataset=args.dataset,
        metrics_path=args.metrics,
        work_dir=args.work_dir,
        keep_failed=bool(args.keep_failed),
        dry_run=bool(args.dry_run),
    )
    print(json.dumps({k: v for k, v in summary.items() if k != "report"}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
