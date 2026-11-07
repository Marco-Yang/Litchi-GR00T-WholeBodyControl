#!/usr/bin/env python3
"""Stream one LeRobot episode as SONIC encode-mode-2 ZMQ pose (protocol v3).

Uses recorded ``teleop.smpl_*`` + hand qpos (not ``action.wbc``), with Pico wrist
parse for ``joint_pos`` — the same inputs as real teleop / ``convert_to_mode2``.

Commands (same as ``pico_pose_zmq_replay.py``):
  y  -> A+B+X+Y  (OFF -> PLANNER stand, or stop)
  x  -> A+X      (PLANNER <-> POSE follow)

While waiting for ``--wait-for`` / PLANNER settle, keeps publishing frame-0 pose at
50 Hz and IDLE planner messages so deploy can stand before walking starts.

Example:
  python gear_sonic/scripts/replay_dataset_mode2_zmq.py \\
    --dataset datasets/walking/2026-11-05-15-57-52 \\
    --episode 3 \\
    --command-fifo /tmp/mode2_cmd.fifo \\
    --wait-for /tmp/mode2_go
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
# Always put repo first. A later PYTHONPATH/cwd entry is not enough: HuggingFace's
# site-packages ``datasets`` shadows local ``datasets.data_check``.
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
from gear_sonic.scripts.pico_pose_zmq_minimal import pack_pose_message
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import (
    build_command_message,
    build_planner_message,
)


def _idle_planner_bytes() -> bytes:
    return build_planner_message(0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], -1.0, -1.0)


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
        help="Keep publishing frame-0 + planner IDLE until this path exists, then settle+play",
    )
    ap.add_argument("--topic", default="pose")
    args = ap.parse_args()

    ep = load_episode(args.dataset, args.episode)
    t = ep.length
    joint_pos = compute_wrist_joint_pos_il(ep.smpl_pose)  # (T, 29)
    joint_vel = np.zeros_like(joint_pos)
    left_hand = np.asarray(ep.left_hand, dtype=np.float32)
    right_hand = np.asarray(ep.right_hand, dtype=np.float32)
    smpl_pose = np.asarray(ep.smpl_pose, dtype=np.float32)
    smpl_joints = np.asarray(ep.smpl_joints, dtype=np.float32)
    body_quat_w = np.asarray(ep.body_quat_w, dtype=np.float32)

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 4)
    sock.bind(f"tcp://*:{args.port}")
    print(
        f"[mode2-zmq] episode={ep.episode_index} T={t} fps={ep.fps} "
        f"pub tcp://*:{args.port} topic={args.topic}"
    )
    time.sleep(0.5)  # ZMQ slow-joiner

    sock.send(build_command_message(start=False, stop=False, planner=False))
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
                        "A+X: POSE (mode-2 follow)",
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
        print(f"[mode2-zmq] command fifo: {args.command_fifo}")

    buf: dict[str, deque] = defaultdict(lambda: deque(maxlen=args.num_frames_to_send))
    dt = 1.0 / args.target_fps
    step = 0
    next_t = time.perf_counter()
    settle_steps = int(args.stand_seconds * args.target_fps)
    planner_every = max(1, int(args.target_fps // 10))  # ~10 Hz planner keepalive

    def drain_cmds() -> None:
        while not cmd_q.empty():
            msg_bytes, label = cmd_q.get()
            sock.send(msg_bytes)
            print(f"[mode2-zmq] {label}")

    def publish_idx(idx: int) -> None:
        nonlocal step
        idx = int(np.clip(idx, 0, t - 1))
        buf["smpl_pose"].append(smpl_pose[idx])
        buf["smpl_joints"].append(smpl_joints[idx])
        buf["body_quat_w"].append(body_quat_w[idx])
        buf["joint_pos"].append(joint_pos[idx])
        buf["joint_vel"].append(joint_vel[idx])
        buf["left_hand_joints"].append(left_hand[idx])
        buf["right_hand_joints"].append(right_hand[idx])
        buf["frame_index"].append(np.int64(step))
        if len(buf["frame_index"]) >= args.num_frames_to_send:
            msg = {
                "smpl_pose": np.stack(buf["smpl_pose"]).astype(np.float32),
                "smpl_joints": np.stack(buf["smpl_joints"]).astype(np.float32),
                "body_quat_w": np.stack(buf["body_quat_w"]).astype(np.float32),
                "joint_pos": np.stack(buf["joint_pos"]).astype(np.float32),
                "joint_vel": np.stack(buf["joint_vel"]).astype(np.float32),
                "left_hand_joints": np.stack(buf["left_hand_joints"]).astype(np.float32),
                "right_hand_joints": np.stack(buf["right_hand_joints"]).astype(np.float32),
                "frame_index": np.asarray(buf["frame_index"], dtype=np.int64),
                "timestamp_realtime": np.array([time.time()], dtype=np.float64),
                "timestamp_monotonic": np.array([time.monotonic()], dtype=np.float64),
            }
            sock.send(pack_pose_message(msg, topic=args.topic))
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

    try:
        # Phase A: hold frame 0 until --wait-for exists (PLANNER stand / band settle).
        if args.wait_for:
            print(f"[mode2-zmq] holding frame-0 until {args.wait_for} ...")
            while not Path(args.wait_for).exists():
                drain_cmds()
                publish_idx(0)
                rate_sleep()
            print("[mode2-zmq] go received")

        # Phase B: POSE settle — keep frame 0 so SMPL window is near-static.
        print(f"[mode2-zmq] POSE settle {args.stand_seconds:.1f}s at frame-0")
        for _ in range(settle_steps):
            drain_cmds()
            publish_idx(0)
            rate_sleep()

        # Phase C: stream walking.
        print("[mode2-zmq] streaming SMPL+hands (mode-2)")
        for k in range(t):
            drain_cmds()
            publish_idx(k)
            rate_sleep()
    except KeyboardInterrupt:
        pass
    finally:
        sock.close(0)
        ctx.term()
        print(f"[mode2-zmq] done episode={ep.episode_index} frames_sent={step}")
        print("done")


if __name__ == "__main__":
    main()
