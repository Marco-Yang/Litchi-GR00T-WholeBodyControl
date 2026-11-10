"""Decode ZMQ ``pose`` topic messages (same wire format as ``pack_pose_message``)."""

from __future__ import annotations

import json
import struct
from typing import Any

import numpy as np

from gear_sonic.utils.teleop.zmq.zmq_planner_sender import HEADER_SIZE

# PICO semantic 5 keypoints in SMPL-24 skeleton (same as demo pipeline).
PICO5_SMPL_INDICES = (15, 20, 21, 7, 8)
PICO5_NAMES = ("head", "left_wrist", "right_wrist", "left_ankle", "right_ankle")


def unpack_pose_message(raw: bytes, topic: str = "pose") -> dict[str, np.ndarray]:
    """Return field name -> numpy array for one pose message."""
    prefix = topic.encode("utf-8")
    if not raw.startswith(prefix):
        raise ValueError(f"unexpected topic prefix: {raw[:8]!r}")
    header_raw = raw[len(prefix) : len(prefix) + HEADER_SIZE].split(b"\x00", 1)[0]
    header = json.loads(header_raw)
    payload = raw[len(prefix) + HEADER_SIZE :]
    out: dict[str, np.ndarray] = {}
    offset = 0
    for field in header["fields"]:
        name = field["name"]
        dtype = field["dtype"]
        shape = field["shape"]
        count = int(np.prod(shape))
        if dtype == "f32":
            size = 4 * count
            vals = struct.unpack("<" + "f" * count, payload[offset : offset + size])
            arr = np.array(vals, dtype=np.float32).reshape(shape)
        elif dtype == "f64":
            size = 8 * count
            vals = struct.unpack("<" + "d" * count, payload[offset : offset + size])
            arr = np.array(vals, dtype=np.float64).reshape(shape)
        elif dtype == "i64":
            size = 8 * count
            vals = struct.unpack("<" + "q" * count, payload[offset : offset + size])
            arr = np.array(vals, dtype=np.int64).reshape(shape)
        elif dtype == "i32":
            size = 4 * count
            vals = struct.unpack("<" + "i" * count, payload[offset : offset + size])
            arr = np.array(vals, dtype=np.int32).reshape(shape)
        else:
            raise ValueError(f"unsupported dtype {dtype} for field {name}")
        offset += size
        out[name] = arr
    return out


def extract_pico5_world_points(msg: dict[str, np.ndarray]) -> np.ndarray | None:
    """Return (5, 3) world positions — prefer :func:`zmq_pose_frames.extract_pico5_world_points`."""
    from gear_sonic.utils.zmq_pose_frames import extract_pico5_world_points as _extract

    return _extract(msg)


def summarize_pose_command(msg: dict[str, np.ndarray] | None) -> dict[str, Any]:
    if msg is None:
        return {"ok": False}
    summary: dict[str, Any] = {"ok": True, "fields": sorted(msg.keys())}
    if "frame_index" in msg:
        summary["frame_index"] = int(np.asarray(msg["frame_index"]).reshape(-1)[0])
    p5 = extract_pico5_world_points(msg)
    if p5 is not None:
        summary["pico5_xy"] = p5[:, :2].tolist()
        summary["pico5_z"] = p5[:, 2].tolist()
    if "root_trans" in msg:
        rt = np.asarray(msg["root_trans"], dtype=np.float64).reshape(-1)[:3]
        summary["root_trans"] = rt.tolist()
    return summary
