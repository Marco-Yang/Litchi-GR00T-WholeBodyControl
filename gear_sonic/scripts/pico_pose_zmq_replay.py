#!/usr/bin/env python3
"""Pico emulator: replay an SMPL clip as the ``pose`` ZMQ stream.

Publishes exactly the message the headset senders emit (``pico_manager_thread_server.py``
and Litchi's ``pico_pose_zmq_minimal.py``): ``smpl_pose``, ``smpl_joints``,
``body_quat_w``, ``frame_index`` (+ timestamps) in a sliding window of
``--num_frames_to_send`` frames at ``--target_fps``. It reuses the minimal sender's
SMPL FK / quaternion helpers, so the numbers go through the same conventions as a
real headset. Use it to test the whole teleop chain (retarget node -> MuJoCo / robot)
without a Pico.

    python gear_sonic/scripts/pico_pose_zmq_replay.py \
        --clip sample_data/smpl_filtered/walk_forward_amateur_001__A001.pkl --loop
"""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
import time

import joblib
import numpy as np
from scipy.spatial.transform import Rotation as R
import zmq

from gear_sonic.scripts import pico_pose_zmq_minimal as pm


def frame_from_pose_aa(pose_aa_72: np.ndarray, fk: pm.HumanJointsFK):
    """SMPL (Y-up native) pose_aa row -> (smpl_pose[21,3], smpl_joints_local[24,3], body_quat_w[4])."""
    body = np.asarray(pose_aa_72[3:66], dtype=np.float64).reshape(21, 3)
    root_q = R.from_rotvec(np.asarray(pose_aa_72[:3], dtype=np.float64)).as_quat()[[3, 0, 1, 2]]
    q_z = pm._quat_multiply_wxyz(pm.Y_TO_Z_UP_QUAT_WXYZ.reshape(1, 4), root_q.reshape(1, 4))[0]
    joints = fk.compute(body, R.from_quat(q_z[[1, 2, 3, 0]]).as_rotvec())
    body_quat_w = pm._quat_normalize_wxyz(
        pm._quat_multiply_wxyz(q_z.reshape(1, 4), pm.SMPL_BASE_ROT_CONJ_WXYZ.reshape(1, 4))[0]
    )
    joints_local = pm._quat_apply_wxyz(pm._quat_conjugate_wxyz(body_quat_w), joints)
    return body.astype(np.float32), joints_local.astype(np.float32), body_quat_w.astype(np.float32)


def synthetic_clip(kind: str, fps: float, seconds: float = 20.0) -> tuple[np.ndarray, float]:
    """Hand-made SMPL pose_aa rows: ``stand`` (arms down), ``wave`` (right arm raise/lower + elbow bend)."""
    n = int(seconds * fps)
    pose = np.zeros((n, 72))
    pose[:, 3 * 16 + 2] = -1.1  # left shoulder: arm down along the body
    pose[:, 3 * 17 + 2] = 1.1
    if kind == "wave":
        t = np.arange(n) / fps
        pose[:, 3 * 17 + 2] = 1.1 - 0.9 * (0.5 - 0.5 * np.cos(2 * np.pi * 0.25 * t))  # right arm lift
        pose[:, 3 * 19 + 1] = -0.8 * (0.5 - 0.5 * np.cos(2 * np.pi * 0.5 * t))  # right elbow
    elif kind != "stand":
        raise SystemExit(f"unknown synthetic clip '{kind}' (stand|wave)")
    return pose, fps


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clip", required=True, help="SMPL pkl with pose_aa (T,72) at --clip-fps (SONIC smpl_filtered), or synthetic:stand / synthetic:wave")
    ap.add_argument("--clip-fps", type=float, default=0.0, help="override clip fps (default: pkl 'fps')")
    ap.add_argument("--port", type=int, default=5556)
    ap.add_argument("--target_fps", type=int, default=50)
    ap.add_argument("--num_frames_to_send", type=int, default=10)
    ap.add_argument("--start", type=float, default=0.0, help="start offset in the clip (s)")
    ap.add_argument("--duration", type=float, default=0.0, help="play only this many seconds (0 = whole clip)")
    ap.add_argument("--stand-seconds", type=float, default=2.0, help="hold the first frame this long before playing")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--topic", default="pose")
    args = ap.parse_args()

    if args.clip.startswith("synthetic:"):
        pose_aa, clip_fps = synthetic_clip(args.clip.split(":", 1)[1], args.clip_fps or 30.0)
    else:
        data = joblib.load(args.clip)
        data = data if "pose_aa" in data else next(iter(data.values()))
        pose_aa = np.asarray(data["pose_aa"], dtype=np.float64)
        clip_fps = args.clip_fps or float(data.get("fps", 30.0))
    i0 = int(args.start * clip_fps)
    i1 = len(pose_aa) if args.duration <= 0 else min(len(pose_aa), i0 + int(args.duration * clip_fps))
    fk = pm.HumanJointsFK()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 4)
    sock.bind(f"tcp://*:{args.port}")
    print(f"[replay] publishing '{args.topic}' on tcp://*:{args.port}  clip={args.clip} frames={i0}..{i1} @{clip_fps} fps")
    time.sleep(0.5)  # let subscribers connect

    buf: dict[str, deque] = defaultdict(lambda: deque(maxlen=args.num_frames_to_send))
    dt = 1.0 / args.target_fps
    step = 0
    next_t = time.perf_counter()
    stand_steps = int(args.stand_seconds * args.target_fps)
    try:
        while True:
            n_play = int((i1 - i0) / clip_fps * args.target_fps)
            for k in range(stand_steps + n_play):
                t_clip = 0.0 if k < stand_steps else (k - stand_steps) / args.target_fps
                idx = min(i1 - 1, i0 + int(round(t_clip * clip_fps)))
                smpl_pose, smpl_joints, body_quat_w = frame_from_pose_aa(pose_aa[idx], fk)
                buf["smpl_pose"].append(smpl_pose)
                buf["smpl_joints"].append(smpl_joints)
                buf["body_quat_w"].append(body_quat_w)
                buf["frame_index"].append(np.int64(step))
                if len(buf["frame_index"]) >= args.num_frames_to_send:
                    msg = {
                        "smpl_pose": np.stack(buf["smpl_pose"]).astype(np.float32),
                        "smpl_joints": np.stack(buf["smpl_joints"]).astype(np.float32),
                        "body_quat_w": np.stack(buf["body_quat_w"]).astype(np.float32),
                        "frame_index": np.asarray(buf["frame_index"], dtype=np.int64),
                        "timestamp_realtime": np.array([time.time()], dtype=np.float64),
                        "timestamp_monotonic": np.array([time.monotonic()], dtype=np.float64),
                    }
                    sock.send(pm.pack_pose_message(msg, topic=args.topic))
                step += 1
                next_t += dt
                sleep = next_t - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_t = time.perf_counter()
            if not args.loop:
                break
            stand_steps = 0
    except KeyboardInterrupt:
        pass
    finally:
        sock.close(0)
        ctx.term()
        print(f"[replay] sent {step} frames")


if __name__ == "__main__":
    main()
