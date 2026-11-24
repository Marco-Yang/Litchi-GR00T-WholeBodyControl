"""Record ego/head camera video + aligned robot proprio + skill-switch timestamps.

Robot proprio comes from the same ``g1_debug`` ZMQ stream used by VLA inference
(C++ deploy on the workstation), not a separate SSH poll to Orin — so video
frames and state share one monotonic clock / ``frame_index``.

Field set mirrors ``run_data_exporter._add_data_frame_sonic`` /
``_add_cpp_state_features`` (pico 真机数采落盘的 g1_debug 侧)，不含 Pico/SMPL
teleop（agent 推理时无 pose topic）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
from typing import Any

import cv2
import numpy as np


# Keys persisted in robot_state.npz (1:1 with video frames). Scalar→length-1 vectors.
PROPRIO_VECTOR_KEYS: tuple[str, ...] = (
    # Full configuration / EEF (derived like exporter)
    "qpos",
    "eef_state",
    "action_wbc",
    # Body / hand joints & velocities (g1_debug)
    "body_q",
    "body_dq",
    "left_hand_q",
    "right_hand_q",
    "left_hand_dq",
    "right_hand_dq",
    # Base / torso
    "base_quat",
    "base_ang_vel",
    "body_torso_quat",
    "body_torso_ang_vel",
    "projected_gravity",
    "init_base_quat",
    "cpp_rotation_offset",
    "delta_heading",
    # Deploy actions / latent
    "last_action",
    "last_left_hand_action",
    "last_right_hand_action",
    "token_state",
    # State-logger meta
    "state_index",
    "ros_timestamp",
)


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


def _as_f32_1d(x: Any) -> np.ndarray:
    if x is None:
        return np.zeros((0,), dtype=np.float32)
    arr = np.asarray(x, dtype=np.float32).reshape(-1)
    return arr


@dataclass
class AgentSkillVideoRecorder:
    """Session recorder: MP4 + robot_state.npz + JSON events on one timeline."""

    output_dir: Path
    fps: float = 10.0
    camera_keys: tuple[str, ...] = ("ego_view", "head")
    fourcc: str = "mp4v"

    _active: bool = field(default=False, init=False, repr=False)
    _session_dir: Path | None = field(default=None, init=False, repr=False)
    _video_path: Path | None = field(default=None, init=False, repr=False)
    _events_path: Path | None = field(default=None, init=False, repr=False)
    _qpos_path: Path | None = field(default=None, init=False, repr=False)
    _writer: cv2.VideoWriter | None = field(default=None, init=False, repr=False)
    _frame_size: tuple[int, int] | None = field(default=None, init=False, repr=False)
    _frame_index: int = field(default=0, init=False, repr=False)
    _t0: float | None = field(default=None, init=False, repr=False)
    _last_write_mono: float | None = field(default=None, init=False, repr=False)
    _events: list[dict[str, Any]] = field(default_factory=list, init=False, repr=False)
    _meta: dict[str, Any] = field(default_factory=dict, init=False, repr=False)
    _active_skill_id: str | None = field(default=None, init=False, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False, repr=False)

    # Per-frame buffers (1:1 with written video frames).
    _t_sec: list[float] = field(default_factory=list, init=False, repr=False)
    _frame_indices: list[int] = field(default_factory=list, init=False, repr=False)
    _skill_ids: list[str] = field(default_factory=list, init=False, repr=False)
    _wall_times: list[str] = field(default_factory=list, init=False, repr=False)
    _cam_ts_rows: list[float] = field(default_factory=list, init=False, repr=False)
    _proprio_rows: dict[str, list[np.ndarray]] = field(
        default_factory=dict, init=False, repr=False
    )

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
            self._qpos_path = self._session_dir / "robot_state.npz"
            self._t0 = float(mono_time)
            self._last_write_mono = None
            self._frame_index = 0
            self._events = []
            self._active_skill_id = None if skill_id is None else str(skill_id)
            self._t_sec.clear()
            self._frame_indices.clear()
            self._skill_ids.clear()
            self._wall_times.clear()
            self._cam_ts_rows.clear()
            self._proprio_rows = {k: [] for k in PROPRIO_VECTOR_KEYS}
            self._meta = {
                "session_id": self._session_dir.name,
                "started_at_utc": _utc_iso(),
                "fps": float(self.fps),
                "camera_keys": list(self.camera_keys),
                "video_file": self._video_path.name,
                "events_file": self._events_path.name,
                "robot_state_file": self._qpos_path.name,
                "proprio_source": "zmq_g1_debug",
                "proprio_keys": list(PROPRIO_VECTOR_KEYS),
                "note": (
                    "robot_state.npz rows share t_sec/frame_index with cameras.mp4 "
                    "and skill_events.json (monotonic clock from session start). "
                    "Fields mirror pico data-exporter g1_debug side "
                    "(qpos/eef/gravity/token_state/last_action/…); no Pico SMPL. "
                    "Source is C++ deploy g1_debug ZMQ, not SSH to Orin."
                ),
            }
            self._active = True
            self._append_event(
                "start",
                mono_time=mono_time,
                skill_id=self._active_skill_id,
                prompt=prompt,
            )
            print(
                f"[agent-video] recording started → {self._session_dir} "
                f"(video + aligned robot_state)",
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

    def maybe_write_sample(
        self,
        camera_msg: dict[str, Any] | None,
        *,
        mono_time: float,
        proprio: dict[str, Any] | None = None,
        # Back-compat kwargs (merged into proprio if provided).
        qpos: np.ndarray | None = None,
        body_q: np.ndarray | None = None,
        left_hand_q: np.ndarray | None = None,
        right_hand_q: np.ndarray | None = None,
        base_quat: np.ndarray | None = None,
    ) -> None:
        """Write one video frame + matching proprio row (same frame_index / t_sec)."""
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

            t0 = self._t0 if self._t0 is not None else float(mono_time)
            t_sec = round(max(0.0, float(mono_time) - t0), 6)
            frame_i = int(self._frame_index)
            wall = _utc_iso()

            bundle: dict[str, Any] = {}
            if proprio:
                bundle.update(proprio)
            if qpos is not None:
                bundle.setdefault("qpos", qpos)
            if body_q is not None:
                bundle.setdefault("body_q", body_q)
            if left_hand_q is not None:
                bundle.setdefault("left_hand_q", left_hand_q)
            if right_hand_q is not None:
                bundle.setdefault("right_hand_q", right_hand_q)
            if base_quat is not None:
                bundle.setdefault("base_quat", base_quat)

            cam_ts = camera_msg.get("timestamps") or {}
            ego_ts = cam_ts.get("ego_view")
            try:
                cam_t = float(ego_ts) if ego_ts is not None else float("nan")
            except (TypeError, ValueError):
                cam_t = float("nan")

            self._t_sec.append(t_sec)
            self._frame_indices.append(frame_i)
            self._skill_ids.append(
                "" if self._active_skill_id is None else str(self._active_skill_id)
            )
            self._wall_times.append(wall)
            self._cam_ts_rows.append(cam_t)
            for key in PROPRIO_VECTOR_KEYS:
                self._proprio_rows[key].append(_as_f32_1d(bundle.get(key)).copy())

            self._last_write_mono = float(mono_time)
            self._frame_index += 1

    # Back-compat alias used by older call sites.
    def maybe_write_camera_msg(
        self, camera_msg: dict[str, Any] | None, *, mono_time: float
    ) -> None:
        self.maybe_write_sample(camera_msg, mono_time=mono_time, proprio=None)

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

            n = len(self._frame_indices)
            qpos_path = self._save_robot_state_npz(n)

            self._meta["ended_at_utc"] = _utc_iso()
            self._meta["num_frames"] = int(self._frame_index)
            self._meta["num_qpos_samples"] = int(n)
            self._meta["duration_sec"] = (
                None if self._t0 is None else round(max(0.0, now - self._t0), 6)
            )
            self._meta["events"] = self._events
            if qpos_path is not None:
                self._meta["robot_state_file"] = qpos_path.name
            assert self._events_path is not None
            self._events_path.write_text(
                json.dumps(self._meta, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            session = self._session_dir
            print(
                f"[agent-video] saved video={self._video_path} "
                f"robot_state={qpos_path} events={self._events_path} "
                f"frames={self._frame_index}",
                flush=True,
            )
            self._active = False
            return session

    def _save_robot_state_npz(self, n: int) -> Path | None:
        assert self._qpos_path is not None

        def _stack_or_empty(rows: list[np.ndarray]) -> np.ndarray:
            if n <= 0 or not rows:
                return np.zeros((max(n, 0), 0), dtype=np.float32)
            widths = {int(r.shape[0]) for r in rows}
            if len(widths) == 1:
                return np.stack(rows, axis=0).astype(np.float32, copy=False)
            w = max(widths)
            out = np.zeros((len(rows), w), dtype=np.float32)
            for i, r in enumerate(rows):
                out[i, : r.shape[0]] = r
            return out

        payload: dict[str, Any] = {
            "t_sec": np.asarray(self._t_sec, dtype=np.float64),
            "frame_index": np.asarray(self._frame_indices, dtype=np.int32),
            "skill_id": np.asarray(self._skill_ids, dtype=str),
            "wall_time_utc": np.asarray(self._wall_times, dtype=str),
            "camera_ego_timestamp": np.asarray(self._cam_ts_rows, dtype=np.float64),
        }
        for key in PROPRIO_VECTOR_KEYS:
            payload[key] = _stack_or_empty(self._proprio_rows.get(key, []))

        np.savez_compressed(self._qpos_path, **payload)
        return self._qpos_path

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
