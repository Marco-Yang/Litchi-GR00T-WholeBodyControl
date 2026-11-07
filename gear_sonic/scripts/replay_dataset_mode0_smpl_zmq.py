#!/usr/bin/env python3
"""Stream one LeRobot episode as SONIC encode-mode-0 ZMQ pose (protocol v1).

Joint reference comes from **SMPL wrist parse** (same as mode-2 ``joint_pos`` /
``compute_wrist_joint_pos_il``) + Revo2 ``teleop.*_hand_joints``.

Unlike mode-2, this does **not** send ``smpl_joints`` / ``smpl_pose`` — only
protocol v1 → deploy ``encode_mode=0`` (g1 joint encoder).

Unlike ``replay_dataset_mode0_zmq.py``, body joints are **not** from
``observation.state``.

Commands:
  y  -> A+B+X+Y  (OFF -> PLANNER stand, or stop)
  x  -> A+X      (PLANNER <-> POSE follow)

Example:
  python gear_sonic/scripts/replay_dataset_mode0_smpl_zmq.py \\
    --dataset datasets/walking/2026-11-05-15-57-52 \\
    --episode 1 \\
    --command-fifo /tmp/mode0_smpl_cmd.fifo \\
    --wait-for /tmp/mode0_smpl_go
"""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
import os
import queue
import select
import sys
import threading
import time
from pathlib import Path

import numpy as np
import zmq

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
_ds = sys.modules.get("datasets")
if _ds is not None:
    _f = (getattr(_ds, "__file__", None) or "").replace("\\", "/")
    if "site-packages" in _f or not (_f and "Litchi-GR00T-WholeBodyControl/datasets" in _f):
        for _k in list(sys.modules):
            if _k == "datasets" or _k.startswith("datasets."):
                del sys.modules[_k]

from datasets.data_check.convert_to_mode2 import compute_wrist_joint_pos_il
from datasets.data_check.lerobot_io import load_episode
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
    build_command_message,
    build_planner_message,
    pack_pose_message,
)


def _idle_planner_bytes() -> bytes:
    return build_planner_message(0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], -1.0, -1.0)


def finite_diff_vel(pos: np.ndarray, fps: float) -> np.ndarray:
    vel = np.zeros_like(pos)
    if pos.shape[0] < 2:
        return vel
    dt = 1.0 / max(float(fps), 1e-6)
    vel[1:] = (pos[1:] - pos[:-1]) / dt
    vel[0] = vel[1]
    return vel.astype(np.float32)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--episode", type=int, required=True)
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--target-fps", type=float, default=50.0)
    ap.add_argument("--num-frames-to-send", type=int, default=10)
    ap.add_argument(
        "--stand-seconds",
        type=float,
        default=2.0,
        help="Hold frame 0 this long after --wait-for (POSE settle) before walking",
    )
    ap.add_argument("--command-fifo", default="")
    ap.add_argument(
        "--wait-for",
        default="",
        help="Keep planner IDLE until this path exists, then settle+play",
    )
    ap.add_argument("--topic", default="pose")
    args = ap.parse_args()

    ep = load_episode(args.dataset, args.episode)
    t = ep.length
    # Same wrist IL parse as mode-2 / real Pico teleop (IsaacLab 29-D).
    joint_pos = compute_wrist_joint_pos_il(ep.smpl_pose).astype(np.float32)
    joint_vel = finite_diff_vel(joint_pos, ep.fps or args.target_fps)
    left_hand = np.asarray(ep.left_hand, dtype=np.float32)
    right_hand = np.asarray(ep.right_hand, dtype=np.float32)
    body_quat_w = np.asarray(ep.body_quat_w, dtype=np.float32)

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 4)
    sock.bind(f"tcp://*:{args.port}")
    wrists = joint_pos[:, [23, 24, 25, 26, 27, 28]]
    print(
        f"[mode0-smpl-zmq] episode={ep.episode_index} T={t} fps={ep.fps} "
        f"pub tcp://*:{args.port} topic={args.topic} protocol=v1 "
        f"wrist|mean|={float(np.abs(wrists).mean()):.3f} "
        f"hand|mean|={float(np.abs(np.concatenate([left_hand, right_hand], -1)).mean()):.3f}"
    )
    time.sleep(0.5)

    sock.send(build_command_message(start=False, stop=False, planner=True))
    cmd_q: queue.SimpleQueue = queue.SimpleQueue()
    state = {"mode": "OFF"}

    def handle(key: str) -> None:
        mode = state["mode"]
        if key == "y":
            if mode == "OFF":
                state["mode"] = "PLANNER"
                cmd_q.put(
                    (
                        build_command_message(start=True, stop=False, planner=True),
                        "A+B+X+Y: start (PLANNER stand)",
                    )
                )
            else:
                state["mode"] = "OFF"
                cmd_q.put(
                    (
                        build_command_message(start=False, stop=True, planner=True),
                        "A+B+X+Y: stop (OFF)",
                    )
                )
        elif key == "x" and mode != "OFF":
            if mode == "PLANNER":
                state["mode"] = "POSE"
                cmd_q.put(
                    (
                        build_command_message(start=True, stop=False, planner=False),
                        "A+X: POSE (mode-0 / g1 from SMPL wrists)",
                    )
                )
            else:
                state["mode"] = "PLANNER"
                cmd_q.put(
                    (
                        build_command_message(start=True, stop=False, planner=True),
                        "A+X: back to PLANNER",
                    )
                )

    def key_thread() -> None:
        if not args.command_fifo:
            return
        if not os.path.exists(args.command_fifo):
            os.mkfifo(args.command_fifo)
        fifo_fd = os.open(args.command_fifo, os.O_RDWR | os.O_NONBLOCK)
        try:
            while True:
                ready, _, _ = select.select([fifo_fd], [], [], 0.2)
                if not ready:
                    continue
                try:
                    data = os.read(fifo_fd, 64)
                except BlockingIOError:
                    continue
                for ch in data.decode(errors="ignore"):
                    handle(ch.lower())
        finally:
            os.close(fifo_fd)

    if args.command_fifo:
        threading.Thread(target=key_thread, daemon=True).start()
        print(f"[mode0-smpl-zmq] command fifo: {args.command_fifo}")

    buf: dict[str, deque] = defaultdict(lambda: deque(maxlen=args.num_frames_to_send))
    dt = 1.0 / args.target_fps
    step = 0
    next_t = time.perf_counter()
    settle_steps = int(args.stand_seconds * args.target_fps)
    planner_every = max(1, int(args.target_fps // 10))

    def drain_cmds() -> None:
        while not cmd_q.empty():
            msg_bytes, label = cmd_q.get()
            sock.send(msg_bytes)
            print(f"[mode0-smpl-zmq] {label}")

    def publish_idx(idx: int) -> None:
        nonlocal step
        idx = int(np.clip(idx, 0, t - 1))
        buf["joint_pos"].append(joint_pos[idx])
        buf["joint_vel"].append(joint_vel[idx])
        buf["body_quat_w"].append(body_quat_w[idx])
        buf["left_hand_joints"].append(left_hand[idx])
        buf["right_hand_joints"].append(right_hand[idx])
        buf["frame_index"].append(np.int64(step))
        if len(buf["frame_index"]) >= args.num_frames_to_send:
            msg = {
                "joint_pos": np.stack(buf["joint_pos"]).astype(np.float32),
                "joint_vel": np.stack(buf["joint_vel"]).astype(np.float32),
                "body_quat_w": np.stack(buf["body_quat_w"]).astype(np.float32),
                "left_hand_joints": np.stack(buf["left_hand_joints"]).astype(np.float32),
                "right_hand_joints": np.stack(buf["right_hand_joints"]).astype(np.float32),
                "frame_index": np.asarray(buf["frame_index"], dtype=np.int64),
                "timestamp_realtime": np.array([time.time()], dtype=np.float64),
                "timestamp_monotonic": np.array([time.monotonic()], dtype=np.float64),
            }
            sock.send(pack_pose_message(msg, topic=args.topic, version=1))
        if state["mode"] == "PLANNER" and (step % planner_every == 0):
            sock.send(_idle_planner_bytes())
        step += 1

    def rate_sleep() -> None:
        nonlocal next_t
        next_t += dt
        sleep = next_t - time.perf_counter()
        if sleep > 0:
            time.sleep(sleep)
        else:
            next_t = time.perf_counter()

    def planner_keepalive() -> None:
        nonlocal step
        if state["mode"] == "PLANNER" and (step % planner_every == 0):
            sock.send(_idle_planner_bytes())
        step += 1

    try:
        if args.wait_for:
            print(
                f"[mode0-smpl-zmq] waiting for {args.wait_for} "
                "(planner keepalive only; no pose yet) ..."
            )
            while not Path(args.wait_for).exists():
                drain_cmds()
                planner_keepalive()
                rate_sleep()
            print("[mode0-smpl-zmq] go received")

        if state["mode"] == "PLANNER":
            sock.send(build_command_message(start=True, stop=False, planner=True))
            print("[mode0-smpl-zmq] re-assert start (PLANNER)")
            for _ in range(int(0.5 * args.target_fps)):
                drain_cmds()
                planner_keepalive()
                rate_sleep()

        print(f"[mode0-smpl-zmq] POSE settle {args.stand_seconds:.1f}s at frame-0")
        for _ in range(settle_steps):
            drain_cmds()
            publish_idx(0)
            rate_sleep()

        print("[mode0-smpl-zmq] streaming SMPL-wrists+hands (mode-0 / g1)")
        for k in range(t):
            drain_cmds()
            publish_idx(k)
            rate_sleep()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close(0)
        ctx.term()
        print(f"[mode0-smpl-zmq] done episode={ep.episode_index} frames_sent={step}")
        print("done")


if __name__ == "__main__":
    main()
