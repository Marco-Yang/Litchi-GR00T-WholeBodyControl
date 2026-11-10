#!/usr/bin/env python3
"""Replay dataset action.motion_token as SONIC latent ZMQ v4 (no Phi_0 / no VLA)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
import zmq

WBC = Path("/home/zhenyu/workspace/code/HumanoidWBM/demo/RL_controller/Litchi-GR00T-WholeBodyControl")
sys.path.insert(0, str(WBC))

from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (  # noqa: E402
    build_command_message,
    pack_pose_message,
)


def pack_v4(
    token: np.ndarray,
    frame_index: int,
    lh: np.ndarray,
    rh: np.ndarray,
    body_quat_w: np.ndarray | None = None,
) -> bytes:
    pose = {
        "token_state": np.asarray(token, dtype=np.float32).reshape(1, -1),
        "frame_index": np.asarray([int(frame_index)], dtype=np.int64),
        "left_hand_joints": np.asarray(lh, dtype=np.float32).reshape(1, 7),
        "right_hand_joints": np.asarray(rh, dtype=np.float32).reshape(1, 7),
    }
    # body_quat_w aligns deploy heading/anchor with v3 (cmd_z stays 0 via deploy streamed motion)
    if body_quat_w is not None:
        pose["body_quat_w"] = np.asarray(body_quat_w, dtype=np.float32).reshape(1, 4)
    return pack_pose_message(pose, topic="pose", version=4)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--zmq-host", default="127.0.0.1")
    ap.add_argument("--zmq-port", type=int, default=5556)
    ap.add_argument("--fps", type=float, default=50.0)
    ap.add_argument("--arm-flag", type=Path, required=True)
    ap.add_argument("--stream-flag", type=Path, required=True)
    ap.add_argument("--go-flag", type=Path, required=True)
    ap.add_argument("--ready-flag", type=Path, required=True)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument(
        "--tokens-npz",
        type=Path,
        default=None,
        help="Optional [T,64] tokens (or dump with tokens_post_go). Overrides parquet action.motion_token.",
    )
    ap.add_argument("--token-start", type=int, default=0, help="Slice start into --tokens-npz sequence")
    ap.add_argument("--token-stride", type=int, default=1, help="Stride into --tokens-npz sequence")
    ap.add_argument(
        "--token-n",
        type=int,
        default=0,
        help="Max frames from tokens-npz after start/stride (0=match parquet length)",
    )
    args = ap.parse_args()

    ep = f"{int(args.episode):06d}"
    parquet = args.dataset / "data/chunk-000" / f"episode_{ep}.parquet"
    cols = ["action.motion_token", "teleop.left_hand_joints", "teleop.right_hand_joints"]
    # Prefer teleop.body_quat_w (matches v3 heading); fall back to observation root quat.
    schema_names = set(pq.read_schema(parquet).names)
    quat_col = None
    for c in ("teleop.body_quat_w", "observation.init_base_quat", "observation.root_orientation"):
        if c in schema_names:
            quat_col = c
            break
    if quat_col is not None:
        cols.append(quat_col)
    table = pq.read_table(parquet, columns=cols)
    lh = np.stack(table.column("teleop.left_hand_joints").to_pylist()).astype(np.float32)
    rh = np.stack(table.column("teleop.right_hand_joints").to_pylist()).astype(np.float32)
    if quat_col is not None:
        bq = np.stack(table.column(quat_col).to_pylist()).astype(np.float32)
        if bq.ndim == 1:
            bq = np.repeat(bq.reshape(1, -1), lh.shape[0], axis=0)
        elif bq.ndim > 2:
            bq = bq.reshape(bq.shape[0], -1)[:, :4]
    else:
        bq = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (lh.shape[0], 1))
    disk_n = int(lh.shape[0])
    if args.tokens_npz is not None:
        raw = np.load(args.tokens_npz)
        if "tokens" in raw.files:
            seq = np.asarray(raw["tokens"], dtype=np.float32)
        elif "tokens_post_go" in raw.files:
            seq = np.asarray(raw["tokens_post_go"], dtype=np.float32)
        elif "arr_0" in raw.files:
            seq = np.asarray(raw["arr_0"], dtype=np.float32)
        else:
            # single-array npz / npy-like
            keys = [k for k in raw.files if np.asarray(raw[k]).ndim == 2]
            if not keys:
                raise RuntimeError(f"{args.tokens_npz}: no 2D token array found in {raw.files}")
            seq = np.asarray(raw[keys[0]], dtype=np.float32)
        if seq.ndim != 2 or seq.shape[1] < 64:
            raise RuntimeError(f"bad tokens shape {seq.shape} in {args.tokens_npz}")
        seq = seq[:, :64]
        start = max(0, int(args.token_start))
        stride = max(1, int(args.token_stride))
        n_want = int(args.token_n) if int(args.token_n) > 0 else disk_n
        idx = start + np.arange(n_want, dtype=np.int64) * stride
        if int(idx[-1]) >= seq.shape[0]:
            raise RuntimeError(
                f"tokens-npz too short: need index {int(idx[-1])} but len={seq.shape[0]} "
                f"(start={start} stride={stride} n={n_want})"
            )
        tokens = seq[idx]
        src = f"npz:{args.tokens_npz.name}[{start}::{stride}][:{n_want}]"
    else:
        tokens = np.stack(table.column("action.motion_token").to_pylist()).astype(np.float32)
        src = "parquet:action.motion_token"
    n = tokens.shape[0]
    if n > disk_n:
        tokens = tokens[:disk_n]
        n = disk_n
    elif n < disk_n:
        # pad hands already sized to disk; truncate hands to token length
        lh = lh[:n]
        rh = rh[:n]
        bq = bq[:n]
    print(
        f"[gt_replay] ep={args.episode} frames={n} src={src} quat={quat_col or 'identity'} "
        f"token0={tokens[0, :4]}",
        flush=True,
    )

    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    pub.bind(f"tcp://{args.zmq_host}:{args.zmq_port}")
    print(f"[gt_replay] bound tcp://{args.zmq_host}:{args.zmq_port}", flush=True)
    args.ready_flag.parent.mkdir(parents=True, exist_ok=True)
    args.ready_flag.touch()
    time.sleep(0.5)

    print(f"[gt_replay] wait arm {args.arm_flag}", flush=True)
    while not args.arm_flag.exists():
        time.sleep(0.05)
    pub.send(build_command_message(start=True, stop=False, planner=True))
    print("[gt_replay] sent start (planner)", flush=True)
    time.sleep(0.3)
    while not args.stream_flag.exists():
        time.sleep(0.05)
    pub.send(build_command_message(start=True, stop=False, planner=False))
    print("[gt_replay] sent start (streamed)", flush=True)

    zero = np.zeros(7, dtype=np.float32)
    # Hold first token until go (safe standing-ish / frame0)
    print(f"[gt_replay] wait go {args.go_flag}", flush=True)
    fi = 0
    while not args.go_flag.exists():
        pub.send(pack_v4(tokens[0], fi, zero, zero, bq[0]))
        fi += 1
        time.sleep(1.0 / args.fps)
    print("[gt_replay] go — replaying GT motion tokens", flush=True)

    period = 1.0 / float(args.fps)
    t_next = time.monotonic()
    idx = 0
    while True:
        pub.send(pack_v4(tokens[idx], fi, lh[idx], rh[idx], bq[idx]))
        if fi % 50 == 0:
            print(f"[gt_replay] fi={fi} idx={idx} t0={tokens[idx, 0]:+.4f}", flush=True)
        fi += 1
        idx += 1
        if idx >= n:
            if not args.loop:
                print("[gt_replay] end of episode", flush=True)
                break
            idx = 0
        t_next += period
        dt = t_next - time.monotonic()
        if dt > 0:
            time.sleep(dt)
        else:
            t_next = time.monotonic()


if __name__ == "__main__":
    main()
