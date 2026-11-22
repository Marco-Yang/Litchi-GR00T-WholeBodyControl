"""Record ego/head camera video + skill-switch timestamps for agent training."""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
from typing import Any

import cv2
import numpy as np


def _utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def _to_bgr_u8(img: np.ndarray) -> np.ndarray:
    arr = np.asarray(img)
    if arr.ndim == 2:
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    if arr.ndim != 3:
        raise ValueError(f"Expected HxW or HxWxC image, got shape={arr.shape}")
    if arr.shape[-1] == 3:
        # Camera client stores RGB; OpenCV writer expects BGR.
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if arr.shape[-1] == 4:
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    raise ValueError(f"Unsupported channel count: {arr.shape}")


def _compose_panel(images: dict[str, np.ndarray], keys: list[str]) -> np.ndarray:
    tiles: list[np.ndarray] = []
    for key in keys:
        if key not in images or images[key] is None:
            continue
        tiles.append(_to_bgr_u8(images[key]))
    if not tiles:
        raise ValueError("No camera images available to record")
    if len(tiles) == 1:
        return tiles[0]
    # Match heights, then side-by-side.
    h = min(t.shape[0] for t in tiles)
    resized = []
    for tile in tiles:
        if tile.shape[0] == h:
            resized.append(tile)
            continue
        scale = h / float(tile.shape[0])
        w = max(1, int(round(tile.shape[1] * scale)))
        resized.append(cv2.resize(tile, (w, h), interpolation=cv2.INTER_AREA))
    return np.concatenate(resized, axis=1)


@dataclass
class AgentSkillVideoRecorder:
    """Session recorder: MP4 + JSON events aligned to wall-clock / frame index."""

    output_dir: Path
    fps: float = 10.0
    camera_keys: tuple[str, ...] = ("ego_view", "head")
    fourcc: str = "mp4v"

    _active: bool = field(default=False, init=False, repr=False)
    _session_dir: Path | None = field(default=None, init=False, repr=False)
    _video_path: Path | None = field(default=None, init=False, repr=False)
    _events_path: Path | None = field(default=None, init=False, repr=False)
    _writer: cv2.VideoWriter | None = field(default=None, init=False, repr=False)
    _frame_size: tuple[int, int] | None = field(default=None, init=False, repr=False)
    _frame_index: int = field(default=0, init=False, repr=False)
    _t0: float | None = field(default=None, init=False, repr=False)
    _last_write_mono: float | None = field(default=None, init=False, repr=False)
    _events: list[dict[str, Any]] = field(default_factory=list, init=False, repr=False)
    _meta: dict[str, Any] = field(default_factory=dict, init=False, repr=False)
    _active_skill_id: str | None = field(default=None, init=False, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    @property
    def active(self) -> bool:
        return self._active

    def start(
        self,
        *,
        skill_id: str | None,
        prompt: str | None,
        mono_time: float,
    ) -> Path:
        with self._lock:
            if self._active:
                return self._session_dir  # type: ignore[return-value]
            self.output_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._session_dir = self.output_dir / f"session_{stamp}"
            self._session_dir.mkdir(parents=True, exist_ok=False)
            self._video_path = self._session_dir / "cameras.mp4"
            self._events_path = self._session_dir / "skill_events.json"
            self._t0 = float(mono_time)
            self._last_write_mono = None
            self._frame_index = 0
            self._events = []
            self._active_skill_id = None if skill_id is None else str(skill_id)
            self._meta = {
                "session_id": self._session_dir.name,
                "started_at_utc": _utc_iso(),
                "fps": float(self.fps),
                "camera_keys": list(self.camera_keys),
                "video_file": self._video_path.name,
                "events_file": self._events_path.name,
            }
            self._active = True
            self._append_event(
                "start",
                mono_time=mono_time,
                skill_id=self._active_skill_id,
                prompt=prompt,
            )
            print(
                f"[agent-video] recording started → {self._session_dir}",
                flush=True,
            )
            return self._session_dir

    def mark_skill_switch(
        self,
        *,
        from_skill_id: str | None,
        to_skill_id: str,
        prompt: str | None,
        mono_time: float,
    ) -> None:
        with self._lock:
            if not self._active:
                return
            self._active_skill_id = str(to_skill_id)
            self._append_event(
                "skill_switch",
                mono_time=mono_time,
                skill_id=str(to_skill_id),
                prompt=prompt,
                extra={
                    "from_skill_id": None if from_skill_id is None else str(from_skill_id),
                    "to_skill_id": str(to_skill_id),
                },
            )
            print(
                f"[agent-video] stamp skill_switch "
                f"{from_skill_id}→{to_skill_id} t={self._events[-1]['t_sec']:.3f}s "
                f"frame={self._events[-1]['frame_index']}",
                flush=True,
            )

    def maybe_write_camera_msg(self, camera_msg: dict[str, Any] | None, *, mono_time: float) -> None:
        if camera_msg is None:
            return
        with self._lock:
            if not self._active:
                return
            interval = 1.0 / max(float(self.fps), 1e-3)
            if self._last_write_mono is not None and (mono_time - self._last_write_mono) < interval:
                return
            images = camera_msg.get("images") or {}
            try:
                panel = _compose_panel(images, list(self.camera_keys))
            except ValueError:
                return
            self._ensure_writer(panel)
            assert self._writer is not None
            self._writer.write(panel)
            self._last_write_mono = float(mono_time)
            self._frame_index += 1

    def stop(self, *, mono_time: float | None = None) -> Path | None:
        with self._lock:
            if not self._active:
                return None
            import time as _time

            now = float(mono_time if mono_time is not None else _time.monotonic())
            self._append_event(
                "stop",
                mono_time=now,
                skill_id=self._active_skill_id,
                prompt=None,
            )
            if self._writer is not None:
                self._writer.release()
                self._writer = None
            self._meta["ended_at_utc"] = _utc_iso()
            self._meta["num_frames"] = int(self._frame_index)
            self._meta["duration_sec"] = (
                None if self._t0 is None else round(max(0.0, now - self._t0), 6)
            )
            self._meta["events"] = self._events
            assert self._events_path is not None
            self._events_path.write_text(
                json.dumps(self._meta, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            session = self._session_dir
            print(
                f"[agent-video] saved video={self._video_path} events={self._events_path} "
                f"frames={self._frame_index}",
                flush=True,
            )
            self._active = False
            return session

    def _ensure_writer(self, panel_bgr: np.ndarray) -> None:
        if self._writer is not None:
            return
        h, w = panel_bgr.shape[:2]
        self._frame_size = (w, h)
        assert self._video_path is not None
        fourcc = cv2.VideoWriter_fourcc(*self.fourcc)
        writer = cv2.VideoWriter(
            str(self._video_path),
            fourcc,
            float(self.fps),
            (w, h),
        )
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open VideoWriter at {self._video_path}")
        self._writer = writer
        self._meta["frame_width"] = w
        self._meta["frame_height"] = h

    def _append_event(
        self,
        event_type: str,
        *,
        mono_time: float,
        skill_id: str | None,
        prompt: str | None,
        extra: dict[str, Any] | None = None,
    ) -> None:
        t0 = self._t0 if self._t0 is not None else float(mono_time)
        event: dict[str, Any] = {
            "type": event_type,
            "t_sec": round(max(0.0, float(mono_time) - t0), 6),
            "frame_index": int(self._frame_index),
            "skill_id": skill_id,
            "prompt": prompt,
            "wall_time_utc": _utc_iso(),
        }
        if extra:
            event.update(extra)
        self._events.append(event)
