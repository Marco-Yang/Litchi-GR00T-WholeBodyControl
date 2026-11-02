"""Load LeRobot walking episodes for mode-2 checks."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq

from .constants import DEFAULT_FPS


@dataclass
class EpisodeMotion:
    episode_index: int
    key: str
    fps: float
    length: int
    # robot
    state: np.ndarray  # (T, 43)
    action_wbc: np.ndarray  # (T, 43)
    root_quat_wxyz: np.ndarray  # (T, 4)
    projected_gravity: np.ndarray  # (T, 3)
    # mode-2 / teleop
    smpl_joints: np.ndarray  # (T, 24, 3)
    smpl_pose: np.ndarray  # (T, 21, 3)
    body_quat_w: np.ndarray  # (T, 4) wxyz
    left_hand: np.ndarray  # (T, 7)
    right_hand: np.ndarray  # (T, 7)
    left_wrist: np.ndarray  # (T, 3)
    right_wrist: np.ndarray  # (T, 3)
    stream_mode: np.ndarray  # (T,)
    tasks: list[str]


def _stack(table: pq.Table, name: str) -> np.ndarray:
    return np.asarray(table.column(name).to_pylist(), dtype=np.float32)


def _stack_int(table: pq.Table, name: str) -> np.ndarray:
    return np.asarray(table.column(name).to_pylist(), dtype=np.int32).reshape(-1)


def dataset_root(path: str | Path) -> Path:
    root = Path(path).expanduser().resolve()
    if not (root / "meta" / "info.json").is_file():
        raise FileNotFoundError(f"Not a LeRobot dataset (missing meta/info.json): {root}")
    return root


def list_episodes(dataset_path: str | Path) -> list[dict]:
    root = dataset_root(dataset_path)
    eps = []
    with open(root / "meta" / "episodes.jsonl", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                eps.append(json.loads(line))
    return eps


def load_info(dataset_path: str | Path) -> dict:
    root = dataset_root(dataset_path)
    with open(root / "meta" / "info.json", encoding="utf-8") as f:
        return json.load(f)


def load_episode(dataset_path: str | Path, episode_index: int) -> EpisodeMotion:
    root = dataset_root(dataset_path)
    info = load_info(root)
    fps = float(info.get("fps", DEFAULT_FPS))
    pq_path = root / f"data/chunk-000/episode_{episode_index:06d}.parquet"
    if not pq_path.is_file():
        raise FileNotFoundError(pq_path)
    table = pq.read_table(pq_path)
    t = table.num_rows

    smpl_joints = _stack(table, "teleop.smpl_joints").reshape(t, -1)
    if smpl_joints.shape[1] == 72:
        smpl_joints = smpl_joints.reshape(t, 24, 3)
    else:
        raise ValueError(f"unexpected smpl_joints dim {smpl_joints.shape} in {pq_path}")

    smpl_pose = _stack(table, "teleop.smpl_pose").reshape(t, -1)
    if smpl_pose.shape[1] == 63:
        smpl_pose = smpl_pose.reshape(t, 21, 3)
    else:
        raise ValueError(f"unexpected smpl_pose dim {smpl_pose.shape} in {pq_path}")

    tasks = []
    for ep in list_episodes(root):
        if int(ep["episode_index"]) == int(episode_index):
            tasks = list(ep.get("tasks", []))
            break

    return EpisodeMotion(
        episode_index=int(episode_index),
        key=f"episode_{episode_index:06d}",
        fps=fps,
        length=t,
        state=_stack(table, "observation.state"),
        action_wbc=_stack(table, "action.wbc"),
        root_quat_wxyz=_stack(table, "observation.root_orientation"),
        projected_gravity=_stack(table, "observation.projected_gravity"),
        smpl_joints=smpl_joints,
        smpl_pose=smpl_pose,
        body_quat_w=_stack(table, "teleop.body_quat_w"),
        left_hand=_stack(table, "teleop.left_hand_joints"),
        right_hand=_stack(table, "teleop.right_hand_joints"),
        left_wrist=_stack(table, "teleop.left_wrist_joints"),
        right_wrist=_stack(table, "teleop.right_wrist_joints"),
        stream_mode=_stack_int(table, "teleop.stream_mode")
        if "teleop.stream_mode" in table.column_names
        else np.ones(t, dtype=np.int32),
        tasks=tasks,
    )


def iter_episodes(
    dataset_path: str | Path,
    episode_indices: list[int] | None = None,
) -> list[EpisodeMotion]:
    eps_meta = list_episodes(dataset_path)
    indices = [int(e["episode_index"]) for e in eps_meta]
    if episode_indices is not None:
        want = set(int(i) for i in episode_indices)
        indices = [i for i in indices if i in want]
    return [load_episode(dataset_path, i) for i in indices]


def _parse_episodes_token(token: str) -> list[int]:
    """Parse ``12`` or ``3-10`` into episode indices."""
    token = token.strip()
    if not token:
        return []
    if "-" in token:
        left, right = token.split("-", 1)
        lo, hi = int(left), int(right)
        if lo > hi:
            lo, hi = hi, lo
        return list(range(lo, hi + 1))
    return [int(token)]


def resolve_episode_indices(
    dataset_path: str | Path,
    *,
    episodes: str = "",
    start: int | None = None,
    end: int | None = None,
) -> list[int] | None:
    """Resolve CLI episode selection.

    - ``--start/--end``: inclusive contiguous range (either side optional)
    - ``--episodes``: comma list, also accepts ``7-12`` tokens
    - both: intersection
    - neither: ``None`` (= all)
    """
    available = [int(e["episode_index"]) for e in list_episodes(dataset_path)]
    if not available:
        return []

    selected: list[int] | None = None

    if episodes and episodes.strip():
        selected = []
        for part in episodes.split(","):
            selected.extend(_parse_episodes_token(part))

    if start is not None or end is not None:
        lo = int(start) if start is not None else min(available)
        hi = int(end) if end is not None else max(available)
        if lo > hi:
            lo, hi = hi, lo
        ranged = [i for i in available if lo <= i <= hi]
        if selected is None:
            selected = ranged
        else:
            want = set(ranged)
            selected = [i for i in selected if i in want]

    if selected is None:
        return None

    avail_set = set(available)
    out = sorted({i for i in selected if i in avail_set})
    missing = sorted({i for i in selected if i not in avail_set})
    if missing:
        print(f"[episodes] skip missing indices: {missing}")
    return out


def add_episode_range_args(parser) -> None:
    """Attach ``--episodes`` / ``--start`` / ``--end`` to an argparse parser."""
    parser.add_argument(
        "--episodes",
        type=str,
        default="",
        help="Episode ids: comma list and/or ranges, e.g. 0,3,7-12",
    )
    parser.add_argument(
        "--start",
        type=int,
        default=None,
        help="First episode index (inclusive). Pair with --end.",
    )
    parser.add_argument(
        "--end",
        type=int,
        default=None,
        help="Last episode index (inclusive). Pair with --start.",
    )
