#!/usr/bin/env python3
# ruff: noqa: T201
"""Read-only recorder of the Pico manager's ZMQ stream (topics ``pose`` and ``command`` on tcp://localhost:5556) into a pickle.

    python3 gear_sonic_deploy/scripts/a3_pose_recorder.py --out pico_session1.pkl [--seconds 600] [--host localhost] [--port 5556]

It only SUBSCRIBES (the manager's PUB socket has any number of subscribers), so it can run next to the deploy / retarget chain without disturbing them.  Every
raw message is stored with its arrival time (wall clock, ns): lossless, so the same session can be replayed later into the simulator
(``a3_pose_replay_raw.py``) or re-scored with other retarget settings (``a3_eval_pose_recording.py``) without the human.  The file is saved every 30 s and on
Ctrl-C / when ``--seconds`` runs out.  A live line every 2 s shows the rate of ``pose`` windows (the manager sends ~50 per second) and the ``command`` messages
(A+B+X+Y / A+X) as they arrive, so you can see that the headset and the trackers are being recorded.

Only the topics ``pose``, ``command`` and ``planner`` are kept (``--all`` keeps everything).  The manager also publishes a ``manager_status`` stream at tens of
thousands of messages per second (1.3 KB each): recording it made a 139 s session 7.8 GB instead of 82 MB (first real session, 2026-10-17).
"""

from __future__ import annotations

import argparse
import pickle
import sys
import time

KEEP = (b"pose", b"command", b"planner")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--seconds", type=float, default=600.0)
    ap.add_argument("--all", action="store_true", help="keep every topic (default: only pose / command / planner)")
    a = ap.parse_args()
    import zmq

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVHWM, 10000)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.connect(f"tcp://{a.host}:{a.port}")
    msgs: list[tuple[int, bytes]] = []
    t0 = time.time()
    last_print, last_save, n_pose_win, n_cmd = t0, t0, 0, 0
    print(f"a3_pose_recorder: recording tcp://{a.host}:{a.port} (all topics) to {a.out} for {a.seconds:.0f} s; Ctrl-C saves and stops", flush=True)

    def save() -> int:
        tmp = a.out + ".tmp"
        with open(tmp, "wb") as f:
            pickle.dump({"version": 1, "host": a.host, "port": a.port, "messages": msgs}, f, protocol=4)
        import os

        os.replace(tmp, a.out)
        return len(msgs)

    try:
        while time.time() - t0 < a.seconds:
            if not sock.poll(200):
                continue
            m = sock.recv()
            now = time.time_ns()
            if not a.all and not m.startswith(KEEP):
                continue
            msgs.append((now, m))
            if m.startswith(b"command"):
                n_cmd += 1
                print(f"t={time.time() - t0:6.1f}s  [command] message #{n_cmd} ({len(m)} bytes)", flush=True)
            elif m.startswith(b"pose"):
                n_pose_win += 1
            t = time.time()
            if t - last_print >= 2.0:
                print(f"t={t - t0:6.1f}s  pose windows {n_pose_win} ({n_pose_win / max(t - t0, 1e-9):.0f}/s)  command messages {n_cmd}  total {len(msgs)}", flush=True)
                last_print = t
            if t - last_save >= 30.0:
                save()
                last_save = t
    except KeyboardInterrupt:
        pass
    n = save()
    print(f"saved {n} messages ({n_pose_win} pose windows, {n_cmd} command messages) to {a.out}" if n else "nothing received: is pico_manager_thread_server.py running?")
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
