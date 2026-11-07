"""Drop episodes from a cleaned LeRobot dataset and renumber the rest.

Used after joint-stability screening. Unstable episodes are removed from the
training set; remaining episodes stay contiguous at 0..N-1.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

import pyarrow as pa
import pyarrow.parquet as pq

from datasets.data_check.lerobot_io import dataset_root, list_episodes, load_info


def _video_keys(info: dict) -> list[str]:
    keys = info.get("video_keys") or []
    if keys:
        return list(keys)
    return [
        key
        for key, spec in info.get("features", {}).items()
        if spec.get("dtype") in ("video", "image")
    ]


def _episode_paths(root: Path, info: dict, episode_index: int) -> tuple[Path, list[Path]]:
    chunks_size = int(info.get("chunks_size", 1000))
    episode_chunk = episode_index // chunks_size
    data_pattern = info.get(
        "data_path",
        "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
    )
    video_pattern = info.get(
        "video_path",
        "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
    )
    parquet_path = root / data_pattern.format(
        episode_chunk=episode_chunk, episode_index=episode_index
    )
    videos = []
    for key in _video_keys(info):
        videos.append(
            root
            / video_pattern.format(
                video_key=key,
                episode_index=episode_index,
                episode_chunk=episode_chunk,
            )
        )
    return parquet_path, videos


def _replace_column(table: pa.Table, name: str, values: list) -> pa.Table:
    idx = table.schema.get_field_index(name)
    if idx < 0:
        return table
    field = table.schema.field(idx)
    return table.set_column(idx, field, pa.array(values, type=field.type))


def drop_episodes(
    dataset_path: str | Path,
    episode_indices: list[int],
    *,
    reason: str = "unstable",
    details: dict | None = None,
) -> dict:
    """Remove ``episode_indices`` and renumber the surviving episodes.

    Returns a summary including ``old_to_new``, the map from kept original
    indices to their new indices.
    """
    root = dataset_root(dataset_path)
    info = load_info(root)
    episodes = list_episodes(root)
    known = {int(ep["episode_index"]) for ep in episodes}
    drop = sorted({int(i) for i in episode_indices})
    missing = [i for i in drop if i not in known]
    if missing:
        raise FileNotFoundError(f"Episode indices not in {root}: {missing}")
    if not drop:
        return {
            "dataset": str(root),
            "dropped": [],
            "num_episodes": len(episodes),
            "old_to_new": {int(ep["episode_index"]): int(ep["episode_index"]) for ep in episodes},
        }

    drop_set = set(drop)
    kept = [ep for ep in episodes if int(ep["episode_index"]) not in drop_set]
    old_to_new = {int(ep["episode_index"]): new_i for new_i, ep in enumerate(kept)}

    staging = root / "_drop_staging"
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)

    total_frames = 0
    new_episodes = []
    for new_i, ep in enumerate(kept):
        old_i = int(ep["episode_index"])
        src_parquet, src_videos = _episode_paths(root, info, old_i)
        if not src_parquet.is_file():
            raise FileNotFoundError(src_parquet)
        table = pq.read_table(src_parquet)
        n = table.num_rows
        table = _replace_column(table, "episode_index", [new_i] * n)
        table = _replace_column(table, "index", list(range(total_frames, total_frames + n)))
        table = _replace_column(table, "frame_index", list(range(n)))
        dst_parquet, dst_videos = _episode_paths(staging, info, new_i)
        dst_parquet.parent.mkdir(parents=True, exist_ok=True)
        pq.write_table(table, dst_parquet)

        for src_video, dst_video in zip(src_videos, dst_videos):
            if src_video.is_file():
                dst_video.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src_video, dst_video)

        new_meta = dict(ep)
        new_meta["episode_index"] = new_i
        new_meta["length"] = n
        new_episodes.append(new_meta)
        total_frames += n

    # Replace episode files only after the staging copy is complete.
    for ep in episodes:
        old_i = int(ep["episode_index"])
        parquet_path, videos = _episode_paths(root, info, old_i)
        if parquet_path.is_file():
            parquet_path.unlink()
        for video in videos:
            if video.is_file():
                video.unlink()

    for path in staging.rglob("*"):
        if not path.is_file():
            continue
        dest = root / path.relative_to(staging)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(path), str(dest))
    shutil.rmtree(staging)

    info["total_episodes"] = len(new_episodes)
    info["total_frames"] = total_frames
    info["total_videos"] = len(new_episodes) * len(_video_keys(info))
    info["splits"] = {"train": f"0:{len(new_episodes)}"}
    with open(root / "meta" / "info.json", "w", encoding="utf-8") as f:
        json.dump(info, f, indent=4)
    with open(root / "meta" / "episodes.jsonl", "w", encoding="utf-8") as f:
        for ep in new_episodes:
            f.write(json.dumps(ep, ensure_ascii=False) + "\n")

    by_index = {int(ep["episode_index"]): ep for ep in episodes}
    record_path = root / "meta" / "dropped_unstable.json"
    previous = []
    if record_path.is_file():
        previous = json.loads(record_path.read_text(encoding="utf-8"))
    entry = {
        "reason": reason,
        "dropped_indices": drop,
        "episodes": [],
    }
    for old_i in drop:
        item = {
            "episode_index": old_i,
            "length": int(by_index[old_i].get("length", 0)),
            "tasks": by_index[old_i].get("tasks", []),
        }
        if details and old_i in details:
            item.update(details[old_i])
        entry["episodes"].append(item)
    previous.append(entry)
    record_path.write_text(json.dumps(previous, ensure_ascii=False, indent=2), encoding="utf-8")

    print(
        f"[drop] removed {len(drop)} episode(s) {drop} from {root.name}; "
        f"{len(new_episodes)} kept, {total_frames} frames"
    )
    return {
        "dataset": str(root),
        "dropped": drop,
        "num_episodes": len(new_episodes),
        "total_frames": total_frames,
        "old_to_new": old_to_new,
    }


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--episodes", type=str, required=True, help="Comma-separated indices to drop")
    args = p.parse_args(argv)
    indices = [int(part) for part in args.episodes.split(",") if part.strip()]
    drop_episodes(args.dataset, indices)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
