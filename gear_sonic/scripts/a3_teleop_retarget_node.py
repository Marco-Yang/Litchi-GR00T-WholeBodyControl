#!/usr/bin/env python3
"""Real-time Pico -> A3 retarget node (the "upstream retargeter" for the 035 policy).

    Pico / pico_pose_zmq_minimal.py / pico_manager_thread_server.py / pico_pose_zmq_replay.py
        --(ZMQ PUB "pose", SMPL)-->  this node  --(ZMQ PUB "a3_ref", 29-DoF frames)-->
            * MuJoCo:      sim2sim_a3_mujoco.py --teleop-zmq tcp://localhost:5560
            * real robot:  a3_ta_bridge (ZMQ -> ROS2 /ta/whole_body_command) -> a3_deploy_onnx_ref

Two modes, matching ``deploy.sh --encoder``:

* ``--mode a3-ref`` (default, ``--encoder a3-fast``): the stream above, A3 joint-space references for the A3-fast
  encoder (and the g1 encoder) -- consumed through the ``a3_ta_bridge``.
* ``--mode smpl-pose`` (``--encoder smpl``): the SMPL-encoder path.  The deploy's SMPL source consumes the Pico SMPL
  stream itself (no retargeting of the body), but its six wrist inputs must be A3 wrist angles, so this mode re-publishes
  the Pico ``pose`` message with ``joint_pos`` replaced by the A3 joint angles from the same GMR IK, on ``--out-port``
  (default 5561) -- point the deploy's ``smpl_zmq`` at it.

Run inside the ``gmr`` conda env (mujoco + mink + GMR + pyzmq)::

    PYTHONPATH=. python gear_sonic/scripts/a3_teleop_retarget_node.py --pico-port 5556 --out-port 5560
    PYTHONPATH=. python gear_sonic/scripts/a3_teleop_retarget_node.py --mode smpl-pose --pico-port 5556 --out-port 5561
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import zmq

from gear_sonic.utils.teleop.a3.frames import pack_frame
from gear_sonic.utils.teleop.a3.retarget import (
    DEFAULT_HUMAN_HEIGHT_M,
    A3RealtimeRetargeter,
    A3RefStream,
    A3SmplPoseAugmenter,
    unpack_pose_message,
)


def run_smpl_pose(args, retargeter, sub, pub) -> None:
    """Re-publish every Pico ``pose`` message with ``joint_pos`` = A3 joint angles (IsaacLab order)."""
    from gear_sonic.scripts.pico_pose_zmq_minimal import pack_pose_message

    aug = A3SmplPoseAugmenter(retargeter=retargeter, fps=args.fps)
    topic = args.pico_topic
    sent = 0
    catch_ups = 0
    solve_ms: list[float] = []
    last_report = time.time()
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    try:
        while True:
            if not poller.poll(100):
                continue
            msg = unpack_pose_message(sub.recv(), topic=topic.encode())
            out = aug.augment(msg)
            if aug.catch_ups != catch_ups:
                catch_ups = aug.catch_ups
                print("[a3-retarget] smpl-pose: new Pico session (G1 catch-up rule: the sender re-entered POSE mode, "
                      "restarted its frame counter, or frames were lost)", flush=True)
            pub.send(pack_pose_message(out, topic=topic))
            sent += 1
            solve_ms.append(retargeter.last_solve_ms)
            now = time.time()
            if now - last_report >= args.report_interval and solve_ms:
                print(
                    f"[a3-retarget] smpl-pose sent={sent} solve_ms mean={np.mean(solve_ms):.2f} "
                    f"max={np.max(solve_ms):.2f}",
                    flush=True,
                )
                solve_ms.clear()
                last_report = now
    except KeyboardInterrupt:
        pass


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pico-host", default="localhost")
    ap.add_argument("--pico-port", type=int, default=5556)
    ap.add_argument("--pico-topic", default="pose")
    ap.add_argument("--mode", choices=["a3-ref", "smpl-pose"], default="a3-ref",
                    help="a3-ref: A3 joint references for the A3-fast encoder; smpl-pose: Pico SMPL stream with A3 wrist "
                         "angles for the SMPL encoder")
    ap.add_argument("--out-port", type=int, default=None,
                    help="ZMQ PUB port (default 5560 for a3-ref, 5561 for smpl-pose)")
    ap.add_argument("--fps", type=float, default=50.0, help="stream rate (used for velocity finite differences)")
    ap.add_argument("--human-height", type=float, default=DEFAULT_HUMAN_HEIGHT_M)
    ap.add_argument("--max-iter", type=int, default=None, help="GMR inner IK iterations per table (default: GMR's)")
    ap.add_argument("--q-lowpass", type=float, default=0.0, help="EMA weight on previous q (0 = off)")
    ap.add_argument("--dq-lowpass", type=float, default=0.5)
    ap.add_argument("--pelvis-rot-weight", type=float, default=50.0,
                    help="IK pelvis orientation cost (GMR stock 10/5 tilts the pelvis ~40 deg); <=0 keeps GMR stock")
    ap.add_argument("--report-interval", type=float, default=5.0)
    ap.add_argument("--track-head", action="store_true",
                    help="also send the IK's head joint angles (default: off -- the head stays at its default pose)")
    args = ap.parse_args()
    if args.out_port is None:
        args.out_port = 5561 if args.mode == "smpl-pose" else 5560

    retargeter = A3RealtimeRetargeter(
        human_height=args.human_height,
        max_iter=args.max_iter,
        q_lowpass=args.q_lowpass,
        dq_lowpass=args.dq_lowpass,
        pelvis_rot_weight=args.pelvis_rot_weight if args.pelvis_rot_weight > 0 else None,
        track_head=args.track_head,
    )
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt_string(zmq.SUBSCRIBE, args.pico_topic)
    sub.setsockopt(zmq.RCVHWM, 8)
    sub.connect(f"tcp://{args.pico_host}:{args.pico_port}")
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 8)
    pub.bind(f"tcp://*:{args.out_port}")
    out_name = "pose (A3 wrist angles)" if args.mode == "smpl-pose" else "a3_ref"
    print(
        f"[a3-retarget] mode={args.mode}  pose <- tcp://{args.pico_host}:{args.pico_port}   {out_name} -> tcp://*:{args.out_port}",
        flush=True,
    )
    if args.mode == "smpl-pose":
        run_smpl_pose(args, retargeter, sub, pub)
        sub.close(0)
        pub.close(0)
        ctx.term()
        return

    stream = A3RefStream(retargeter, fps=args.fps)
    sent = 0
    solve_ms: list[float] = []
    last_report = time.time()
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    try:
        while True:
            if not poller.poll(100):
                continue
            msg = unpack_pose_message(sub.recv(), topic=args.pico_topic.encode())
            for frame in stream.process(msg, time.time_ns()):
                pub.send(pack_frame(frame))
                sent += 1
            for event in stream.events:
                print(f"[a3-retarget] {event}", flush=True)
            stream.events.clear()
            solve_ms.extend(stream.solve_ms)
            stream.solve_ms.clear()
            now = time.time()
            if now - last_report >= args.report_interval and solve_ms:
                print(
                    f"[a3-retarget] sent={sent} solve_ms mean={np.mean(solve_ms):.2f} "
                    f"p95={np.percentile(solve_ms, 95):.2f} max={np.max(solve_ms):.2f}",
                    flush=True,
                )
                solve_ms.clear()
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        sub.close(0)
        pub.close(0)
        ctx.term()


if __name__ == "__main__":
    main()
