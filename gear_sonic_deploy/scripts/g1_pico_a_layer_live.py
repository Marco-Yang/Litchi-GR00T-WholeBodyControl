#!/usr/bin/env python3
"""Live A-layer (SMPL wrist raise°) from pico_manager pose :5556 only.

No deploy / no robot. Use to compare tracking at real-teleop stance vs sim stance.

  L  mark LEFT side-raise sample window start
  S  stop window and print hold-ish peak/mean
  R  mark RIGHT window
  Q  quit

Usage:
  source .venv_teleop/bin/activate
  python gear_sonic_deploy/scripts/g1_pico_a_layer_live.py
"""

from __future__ import annotations

import json
import select
import sys
import termios
import threading
import time
import tty
from collections import deque

import numpy as np
import zmq

SMPL_L_SHOULDER, SMPL_R_SHOULDER = 16, 17
SMPL_L_WRIST, SMPL_R_WRIST = 20, 21


def unpack_pose_message(packed_data: bytes, topic: str = "pose") -> dict:
    header_size = 1280
    topic_bytes = topic.encode("utf-8")
    if not packed_data.startswith(topic_bytes):
        raise ValueError(f"expected topic '{topic}'")
    offset = len(topic_bytes)
    header_bytes = packed_data[offset : offset + header_size]
    null_idx = header_bytes.find(b"\x00")
    if null_idx > 0:
        header_bytes = header_bytes[:null_idx]
    header = json.loads(header_bytes.decode("utf-8"))
    result: dict = {}
    cur = offset + header_size
    dtype_map = {
        "f32": np.float32,
        "f64": np.float64,
        "i32": np.int32,
        "i64": np.int64,
        "bool": np.bool_,
        "u8": np.uint8,
        "i8": np.int8,
    }
    for field_info in header.get("fields", []):
        dtype = dtype_map[field_info["dtype"]]
        shape = tuple(field_info["shape"])
        n_bytes = int(np.prod(shape)) * np.dtype(dtype).itemsize
        chunk = packed_data[cur : cur + n_bytes]
        result[field_info["name"]] = np.frombuffer(chunk, dtype=dtype).reshape(shape).copy()
        cur += n_bytes
    return result


def raise_deg(smpl: np.ndarray, side: str) -> float:
    j = np.asarray(smpl)
    if j.ndim == 3:
        j = j[-1]
    j = j.reshape(-1, 3)
    if side == "left":
        sh, wr = j[SMPL_L_SHOULDER], j[SMPL_L_WRIST]
        lateral = float(wr[1] - sh[1])
    else:
        sh, wr = j[SMPL_R_SHOULDER], j[SMPL_R_WRIST]
        lateral = float(sh[1] - wr[1])
    height = float(wr[2] - sh[2])
    return float(np.degrees(np.arctan2(max(lateral, 0.0), max(-height, 1e-6))))


class Keys:
    def __init__(self) -> None:
        self._q: deque[str] = deque()
        self._stop = threading.Event()
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        tty.setcbreak(self._fd)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r:
                ch = sys.stdin.read(1)
                if ch:
                    self._q.append(ch)

    def pop(self) -> str | None:
        return self._q.popleft() if self._q else None

    def close(self) -> None:
        self._stop.set()
        try:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        except Exception:
            pass


def main() -> int:
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"pose")
    sock.setsockopt(zmq.RCVTIMEO, 200)
    sock.connect("tcp://127.0.0.1:5556")
    print("[a-live] SUB pose tcp://127.0.0.1:5556")
    print("  Need pico_manager in POSE (A+X after calibrate).")
    print("  Keys: L start left | R start right | S stop+summary | Q quit")
    keys = Keys()
    keys.start()
    side: str | None = None
    buf: list[float] = []
    last_l = last_r = float("nan")
    try:
        while True:
            ch = keys.pop()
            if ch in ("q", "Q"):
                break
            if ch in ("l", "L"):
                side, buf = "left", []
                print("\n[rec] LEFT raise→hold→lower, then press S")
            if ch in ("r", "R"):
                side, buf = "right", []
                print("\n[rec] RIGHT raise→hold→lower, then press S")
            if ch in ("s", "S") and side is not None:
                arr = np.asarray(buf, dtype=float)
                if arr.size:
                    # top quartile ≈ hold
                    thr = float(np.quantile(arr, 0.7))
                    hold = arr[arr >= thr]
                    print(
                        f"\n=== {side.upper()} done n={arr.size} "
                        f"peak={arr.max():.1f}° mean={arr.mean():.1f}° "
                        f"hold≈{hold.mean():.1f}° (n_hold={hold.size}) ==="
                    )
                else:
                    print(f"\n=== {side.upper()} empty ===")
                side, buf = None, []

            try:
                msg = sock.recv()
            except zmq.Again:
                continue
            try:
                pose = unpack_pose_message(msg, "pose")
            except Exception as exc:
                print(f"\n[warn] unpack: {exc}")
                continue
            if "smpl_joints" not in pose:
                continue
            last_l = raise_deg(pose["smpl_joints"], "left")
            last_r = raise_deg(pose["smpl_joints"], "right")
            if side is not None:
                buf.append(last_l if side == "left" else last_r)
            print(
                f"\r  A_smpl  L={last_l:5.1f}°  R={last_r:5.1f}°  "
                f"{'REC '+side.upper() if side else '     '}   ",
                end="",
                flush=True,
            )
    finally:
        keys.close()
        sock.close()
        ctx.term()
        print("\n[a-live] bye")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
