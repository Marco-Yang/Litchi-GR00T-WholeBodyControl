#!/usr/bin/env python3
# ruff: noqa: T201
"""Replay a recording made by ``a3_pose_recorder.py`` on tcp://*:5556 with the original timing: a stand-in for the Pico manager (and the human).

    python3 gear_sonic_deploy/scripts/a3_pose_replay_raw.py pico_session1.pkl [--speed 1.0] [--start 0] [--end 0] [--loop] [--no-commands] [--timing frame|arrival]

``--timing frame`` (default) sends every ``pose`` window at the time of its frame index (frame_index / 50 Hz), ``arrival`` at the time the recorder received
it.  The two differ when the recorder stalled: ``pico_leg1_slim.pkl`` has a ~4.8 s silence every 30 s (25.6, 55.6, 85.6, 115.6 s) with the frame indexes
continuous across it (the frames were buffered, not lost), and replaying the ARRIVAL times makes the deploy see a dead stream, drop to the standing pose
and never follow again: the first replay of that recording moved the robot for ~20 s only.

Use it instead of ``pico_manager_thread_server.py`` (they bind the same port: never both).  ``command`` messages (A+B+X+Y / A+X) are replayed at their original
times unless ``--no-commands``; then press S / T / P in the deploy terminal yourself (or use ``pico_pose_zmq_replay.py``'s fifo for the commands).  ``--start`` /
``--end`` are seconds from the first message of the recording.

``--wait-for PATH`` holds the replay until that file exists (so you can bring the robot to PD stand and into follow first: the recording starts with the
person standing still); ``--command-fifo F`` reads ``y`` (A+B+X+Y: start / stop) and ``x`` (A+X: follow <-> stand) from the named pipe F and sends the same
``command`` messages the manager would (``echo y > F``).  Together they make a replay scriptable without a human or a TTY:
    mkfifo F; replay ... --no-commands --command-fifo F --wait-for T &   then   echo y > F; <press 9 in the sim window>; echo x > F; touch T
"""

from __future__ import annotations

import argparse
from pathlib import Path
import pickle
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # `gear_sonic` is importable whatever the python and the current directory


HZ = 50.0


def pose_last_frame_index(raw: bytes, topic: bytes = b"pose", header_size: int = 1280) -> int | None:
    """Frame index of the newest frame in a ``pose`` message (header JSON, then the fields back to back); None if it is not a pose message / has no frame_index."""
    import json

    import numpy as np

    if not raw.startswith(topic):
        return None
    header = json.loads(raw[len(topic) : len(topic) + header_size].rstrip(b"\x00").decode("utf-8"))
    dtypes = {"f32": "<f4", "f64": "<f8", "i32": "<i4", "i64": "<i8", "bool": "?"}
    offset = len(topic) + header_size
    for field in header["fields"]:
        dt = np.dtype(dtypes[field["dtype"]])
        n = int(np.prod(field["shape"])) if field["shape"] else 1
        if field["name"] == "frame_index":
            return int(np.frombuffer(raw, dtype=dt, count=n, offset=offset)[-1])
        offset += n * dt.itemsize
    return None


def schedule(messages, timing: str = "frame") -> list[tuple[float, bytes]]:
    """[(time in ns from the first message, message)] sorted by time.  ``arrival``: the recorded arrival times.  ``frame``: pose windows at their frame-index
    time (the first pose window at its arrival time), everything else (commands) at its arrival time."""
    t0 = messages[0][0]
    out = [(float(t - t0), m) for t, m in messages]
    if timing == "frame":
        first = None
        for k, (t, m) in enumerate(out):
            fi = pose_last_frame_index(m) if m.startswith(b"pose") else None
            if fi is None:
                continue
            if first is None:
                first = (t, fi)
            out[k] = (first[0] + (fi - first[1]) / HZ * 1e9, m)
        out.sort(key=lambda x: x[0])
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=0.0)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--no-commands", action="store_true")
    ap.add_argument("--timing", choices=("frame", "arrival"), default="frame", help="send pose windows at their frame-index time (default) or their recorded arrival time")
    ap.add_argument("--wait-for", default="", help="start the replay only when this file exists")
    ap.add_argument("--command-fifo", default="", help="named pipe to read y / x from (created if missing)")
    a = ap.parse_args()
    import zmq

    with open(a.recording, "rb") as f:
        rec = pickle.load(f)
    msgs = rec["messages"]
    sel = [(t, m) for t, m in schedule(msgs, a.timing) if t / 1e9 >= a.start and (a.end <= 0 or t / 1e9 <= a.end)]
    if a.no_commands:
        sel = [(t, m) for t, m in sel if not m.startswith(b"command")]
    if not sel:
        print("no messages in the selected range")
        return 1
    base = sel[0][0]
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 100)
    sock.setsockopt(zmq.LINGER, 200)
    sock.bind(f"tcp://*:{a.port}")
    print(f"a3_pose_replay_raw: {len(sel)} messages, {(sel[-1][0] - base) / 1e9:.1f} s at speed {a.speed:g} on tcp://*:{a.port}", flush=True)
    time.sleep(0.7)  # let subscribers connect
    import os
    import queue
    import threading

    cmd_q: queue.SimpleQueue = queue.SimpleQueue()
    if a.command_fifo:
        from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_command_message

        if not os.path.exists(a.command_fifo):
            os.mkfifo(a.command_fifo)
        state = {"mode": "OFF"}

        def handle(ch: str) -> None:
            mode = state["mode"]
            if ch == "y":
                state["mode"] = "PLANNER" if mode == "OFF" else "OFF"
                cmd_q.put((build_command_message(start=mode == "OFF", stop=mode != "OFF", planner=True), "A+B+X+Y " + ("start (stand)" if mode == "OFF" else "stop (disable)")))
            elif ch == "x" and mode != "OFF":
                state["mode"] = "POSE" if mode == "PLANNER" else "PLANNER"
                cmd_q.put((build_command_message(start=True, stop=False, planner=state["mode"] == "PLANNER"), "A+X -> " + ("follow" if state["mode"] == "POSE" else "stand")))

        def reader() -> None:
            fd = os.open(a.command_fifo, os.O_RDWR | os.O_NONBLOCK)  # read-write: never sees EOF when a writer closes
            import select

            while True:
                if select.select([fd], [], [], 0.2)[0]:
                    try:
                        for ch in os.read(fd, 64).decode(errors="ignore"):
                            handle(ch.lower())
                    except BlockingIOError:
                        pass

        threading.Thread(target=reader, daemon=True).start()
        sock.send(build_command_message(start=False, stop=False, planner=False))  # the manager's first message
        print("a3_pose_replay_raw: commands via the fifo: y = A+B+X+Y, x = A+X", flush=True)

    def flush_commands() -> None:
        while not cmd_q.empty():
            raw, label = cmd_q.get()
            sock.send(raw)
            print(f"[command] {label}", flush=True)

    if a.wait_for:
        print(f"a3_pose_replay_raw: waiting for {a.wait_for} ...", flush=True)
        while not os.path.exists(a.wait_for):
            flush_commands()
            time.sleep(0.05)
    while True:
        start = time.time()
        for k, (t, m) in enumerate(sel):
            wait = start + (t - base) / 1e9 / a.speed - time.time()
            if wait > 0:
                time.sleep(wait)
            flush_commands()
            sock.send(m)
            if m.startswith(b"command"):
                print(f"t={(t - base) / 1e9:6.1f}s  [command] replayed", flush=True)
        print("a3_pose_replay_raw: done", flush=True)
        if not a.loop:
            break
    time.sleep(0.3)
    return 0


if __name__ == "__main__":
    sys.exit(main())
