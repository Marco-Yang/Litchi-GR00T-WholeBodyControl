"""Publish training-format boundary vision tokens from the Orin camera stream.

Runs inside the C2 shared-VLM process so observe clients can SUB tokens without
loading a second vision tower.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
import threading
import time
from typing import Any

import cv2
import msgpack
import msgpack_numpy as mnp
import numpy as np
import torch
import zmq

from gear_sonic.utils.inference.boundary_vision import (
    BOUNDARY_VISION_TOPIC,
    DEFAULT_CAMERAS,
    VisionEncoder,
    resize_rgb,
)


def _decode_camera_images(packed: bytes) -> dict[str, np.ndarray]:
    data = msgpack.unpackb(packed, object_hook=mnp.decode)
    out: dict[str, np.ndarray] = {}
    for key, value in data.get("images", {}).items():
        if isinstance(value, (bytes, bytearray)):
            bgr = cv2.imdecode(np.frombuffer(value, dtype=np.uint8), cv2.IMREAD_COLOR)
        elif isinstance(value, str):
            bgr = cv2.imdecode(
                np.frombuffer(base64.b64decode(value), dtype=np.uint8), cv2.IMREAD_COLOR
            )
        elif isinstance(value, np.ndarray):
            out[key] = value
            continue
        else:
            continue
        if bgr is not None:
            out[key] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return out


@dataclass
class BoundaryVisionPublisherConfig:
    camera_host: str = "192.168.123.164"
    camera_port: int = 5555
    pub_bind: str = "tcp://*:5591"
    vision_checkpoint: str = ""
    vision_base: str = "Qwen/Qwen3-VL-2B-Instruct"
    device: str = "cuda:0"
    hz: float = 5.0
    image_wh: tuple[int, int] = (384, 288)
    grid: tuple[int, int] = (3, 4)
    cameras: tuple[str, ...] = DEFAULT_CAMERAS


class BoundaryVisionPublisher(threading.Thread):
    """SUB camera → VisionEncoder → PUB ``boundary_vision`` packets."""

    def __init__(self, config: BoundaryVisionPublisherConfig):
        super().__init__(daemon=True, name="boundary-vision-pub")
        self.config = config
        self.stop_event = threading.Event()
        self._missing_warned = False
        self.encoder = VisionEncoder(
            base=config.vision_base,
            vla_checkpoint=config.vision_checkpoint or None,
            image_wh=config.image_wh,
            grid=config.grid,
            device=config.device,
        )
        self.ctx = zmq.Context.instance()
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub.setsockopt(zmq.CONFLATE, 1)
        self.sub.setsockopt(zmq.RCVHWM, 3)
        self.sub.connect(f"tcp://{config.camera_host}:{config.camera_port}")
        self.pub = self.ctx.socket(zmq.PUB)
        self.pub.setsockopt(zmq.SNDHWM, 3)
        self.pub.bind(config.pub_bind)
        self.period = 1.0 / max(float(config.hz), 1e-3)

    def run(self) -> None:
        print(
            f"[boundary-vision] publishing on {self.config.pub_bind} "
            f"from camera {self.config.camera_host}:{self.config.camera_port} "
            f"@ {self.config.hz:.1f} Hz  ckpt={self.encoder.source}",
            flush=True,
        )
        last = 0.0
        while not self.stop_event.is_set():
            if not self.sub.poll(100):
                continue
            packed = self.sub.recv()
            now = time.monotonic()
            if now - last < self.period:
                continue
            try:
                imgs = _decode_camera_images(packed)
            except Exception as exc:  # noqa: BLE001
                print(f"[boundary-vision] camera decode failed: {exc}", flush=True)
                continue
            if not all(c in imgs for c in self.config.cameras):
                if not self._missing_warned:
                    print(
                        f"[boundary-vision] missing cams "
                        f"{set(self.config.cameras) - set(imgs)}; have {list(imgs)}",
                        flush=True,
                    )
                    self._missing_warned = True
                continue
            ordered = [resize_rgb(imgs[c], self.config.image_wh) for c in self.config.cameras]
            try:
                tokens = self.encoder.encode(ordered)  # [C, G, D]
            except Exception as exc:  # noqa: BLE001
                print(f"[boundary-vision] encode failed: {exc}", flush=True)
                continue
            # Stamp AFTER encode. Pre-encode t_mono makes observe see age≈encode_ms
            # (~2–3s under VLA GPU load) and always trip the 1s stale gate.
            sent_mono = time.monotonic()
            payload: dict[str, Any] = {
                "t_mono": sent_mono,
                "wall_time": time.time(),
                "encode_s": float(sent_mono - now),
                "cameras": list(self.config.cameras),
                "tokens": tokens.detach().float().cpu().numpy().astype(np.float16),
                "image_wh": list(self.config.image_wh),
                "grid": list(self.config.grid),
                "vision_dim": int(self.encoder.dim),
                "n_tokens": int(self.encoder.n_tokens),
                "vision_checkpoint": self.encoder.source,
            }
            self.pub.send(BOUNDARY_VISION_TOPIC + msgpack.packb(payload, default=mnp.encode))
            last = sent_mono

    def close(self) -> None:
        self.stop_event.set()
        self.join(timeout=2.0)
        self.sub.close(0)
        self.pub.close(0)
