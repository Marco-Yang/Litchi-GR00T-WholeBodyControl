"""Python port of ``a3_deploy::A3TeleopReferenceBuffer`` (a3_teleop_reference.cpp).

Turns a stream of joint-space teleop frames into the 640-float tokenizer prefix the
035 ``g1`` / ``a3_fast`` ONNX encoders consume::

    [ q_ref_il[10,29] | dq_ref_il[10,29] | rel_root_ori_6d[10,6] ]  (10*29*2 + 10*6 = 640)

The window starts ``delay_ns`` in the past and has 10 slots spaced
``future_frame_skip / policy_hz`` seconds apart, so with the defaults (delay 0.94 s,
skip 5, 50 Hz) the last slot lands on "now". ``a3_fast`` uses a shorter window
(skip 1, delay 0.22 s). Frames are re-stamped by the caller on arrival, matching the
AimRT path.

Keep this file numerically identical to the C++ implementation; the unit tests
(`tests/test_a3_teleop_reference.py`) pin the layout and the sampling rules.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import enum
import math
import threading

import numpy as np

from gear_sonic.utils.teleop.a3.frames import (
    A3_IL_TO_MJ29,
    DEFAULT_Q_MJ29,
    NUM_POLICY_DOFS,
    A3TeleopFrame,
)

NUM_FUTURE_FRAMES = 10
TOKENIZER_FLOATS = NUM_FUTURE_FRAMES * NUM_POLICY_DOFS * 2 + NUM_FUTURE_FRAMES * 6  # 640
_COMMAND_DOF_FLOATS = NUM_FUTURE_FRAMES * NUM_POLICY_DOFS  # 290
_COMMAND_MULTI_FUTURE_FLOATS = 2 * _COMMAND_DOF_FLOATS  # 580


class TokenizerStatus(enum.Enum):
    NO_DATA = 0
    BUFFERING = 1
    RUNNING = 2


@dataclass
class A3TeleopReferenceOptions:
    delay_ns: int = 900_000_000
    policy_hz: float = 50.0
    future_frame_skip: int = 5
    max_frames: int = 512


# ---------------------------------------------------------------------------
# quaternion helpers (wxyz)
# ---------------------------------------------------------------------------
def _normalize(q: np.ndarray) -> np.ndarray:
    n2 = float(np.dot(q, q))
    if n2 <= np.finfo(np.float64).eps:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / math.sqrt(n2)


def _conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def _yaw(q: np.ndarray) -> float:
    w, x, y, z = _normalize(np.asarray(q, dtype=np.float64))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_pi(x: float) -> float:
    while x > math.pi:
        x -= 2.0 * math.pi
    while x < -math.pi:
        x += 2.0 * math.pi
    return x


def yaw_offset_rad(robot_quat_wxyz, ref_quat_wxyz) -> float:
    return _wrap_pi(_yaw(np.asarray(robot_quat_wxyz)) - _yaw(np.asarray(ref_quat_wxyz)))


def apply_yaw_offset(offset_rad: float, ref_quat_wxyz) -> np.ndarray:
    half = 0.5 * offset_rad
    qz = np.array([math.cos(half), 0.0, 0.0, math.sin(half)])
    return _mul(qz, _normalize(np.asarray(ref_quat_wxyz, dtype=np.float64)))


def _slerp(qa: np.ndarray, qb: np.ndarray, alpha: float) -> np.ndarray:
    a = _normalize(qa)
    b = _normalize(qb)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        dot = -dot
        b = -b
    if dot > 0.9995:
        return _normalize((1.0 - alpha) * a + alpha * b)
    dot = min(1.0, max(-1.0, dot))
    theta0 = math.acos(dot)
    theta = theta0 * alpha
    s0 = math.cos(theta) - dot * math.sin(theta) / math.sin(theta0)
    s1 = math.sin(theta) / math.sin(theta0)
    return _normalize(s0 * a + s1 * b)


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = _normalize(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]
    )


def _write_ori_6d(rel_quat: np.ndarray, frame: int, out: np.ndarray) -> None:
    m = _quat_to_matrix(rel_quat)
    o = _COMMAND_MULTI_FUTURE_FLOATS + frame * 6
    out[o : o + 6] = [m[0, 0], m[0, 1], m[1, 0], m[1, 1], m[2, 0], m[2, 1]]


def build_default_stand_tokenizer_slice() -> np.ndarray:
    """Standing tokenizer prefix used before any teleop data arrives."""
    out = np.zeros(TOKENIZER_FLOATS, dtype=np.float32)
    q_il = DEFAULT_Q_MJ29[A3_IL_TO_MJ29]
    for k in range(NUM_FUTURE_FRAMES):
        out[k * NUM_POLICY_DOFS : (k + 1) * NUM_POLICY_DOFS] = q_il
        _write_ori_6d(np.array([1.0, 0.0, 0.0, 0.0]), k, out)
    return out


class A3TeleopReferenceBuffer:
    def __init__(self, options: A3TeleopReferenceOptions | None = None) -> None:
        self._opt = replace(options) if options else A3TeleopReferenceOptions()
        if self._opt.max_frames <= 0:
            self._opt.max_frames = 1
        self._frames: list[A3TeleopFrame] = []
        self._running_started = False
        self._lock = threading.Lock()

    # -- state ---------------------------------------------------------
    @property
    def delay_ns(self) -> int:
        return self._opt.delay_ns

    def reset(self) -> None:
        with self._lock:
            self._frames.clear()
            self._running_started = False

    def reset_to_latest_frame(self) -> bool:
        with self._lock:
            if not self._frames:
                return False
            self._frames = [self._frames[-1]]
            self._running_started = False
            return True

    def frame_count(self) -> int:
        with self._lock:
            return len(self._frames)

    def has_any_frame(self) -> bool:
        with self._lock:
            return bool(self._frames)

    def latest_stamp_ns(self) -> int:
        with self._lock:
            return self._frames[-1].stamp_ns if self._frames else 0

    def has_ready_window(self, now_ns: int) -> bool:
        with self._lock:
            if len(self._frames) < 2:
                return False
            base = now_ns - self._opt.delay_ns
            return self._frames[0].stamp_ns <= base <= self._frames[-1].stamp_ns

    # -- input ---------------------------------------------------------
    def push_frame(self, frame: A3TeleopFrame) -> None:
        if frame.stamp_ns <= 0 or not frame.is_finite():
            return
        norm = A3TeleopFrame(
            stamp_ns=int(frame.stamp_ns),
            q_mj29=np.asarray(frame.q_mj29, dtype=np.float64).copy(),
            dq_mj29=np.asarray(frame.dq_mj29, dtype=np.float64).copy(),
            pelvis_quat_wxyz=_normalize(np.asarray(frame.pelvis_quat_wxyz, dtype=np.float64)),
            head_q=np.asarray(frame.head_q, dtype=np.float64).copy() if frame.has_head else np.zeros(2),
            head_dq=np.asarray(frame.head_dq, dtype=np.float64).copy() if frame.has_head else np.zeros(2),
            has_head=bool(frame.has_head),
        )
        with self._lock:
            frames = self._frames
            if frames and norm.stamp_ns < frames[-1].stamp_ns:
                idx = next((i for i, f in enumerate(frames) if f.stamp_ns >= norm.stamp_ns), len(frames))
                if idx < len(frames) and frames[idx].stamp_ns == norm.stamp_ns:
                    frames[idx] = norm
                else:
                    frames.insert(idx, norm)
            elif frames and norm.stamp_ns == frames[-1].stamp_ns:
                frames[-1] = norm
            else:
                frames.append(norm)
            while len(frames) > self._opt.max_frames:
                frames.pop(0)

    # -- sampling ------------------------------------------------------
    def _sample_locked(self, stamp_ns: int) -> A3TeleopFrame | None:
        frames = self._frames
        if not frames:
            return None
        if stamp_ns <= frames[0].stamp_ns:
            return frames[0]
        if stamp_ns >= frames[-1].stamp_ns:
            return frames[-1]
        hi_idx = next(i for i, f in enumerate(frames) if f.stamp_ns >= stamp_ns)
        hi = frames[hi_idx]
        if hi.stamp_ns == stamp_ns:
            return hi
        lo = frames[hi_idx - 1]
        denom = float(hi.stamp_ns - lo.stamp_ns)
        alpha = float(stamp_ns - lo.stamp_ns) / denom if denom > 0.0 else 0.0
        has_head = lo.has_head and hi.has_head
        return A3TeleopFrame(
            stamp_ns=stamp_ns,
            q_mj29=(1.0 - alpha) * lo.q_mj29 + alpha * hi.q_mj29,
            dq_mj29=(1.0 - alpha) * lo.dq_mj29 + alpha * hi.dq_mj29,
            pelvis_quat_wxyz=_slerp(lo.pelvis_quat_wxyz, hi.pelvis_quat_wxyz, alpha),
            head_q=(1.0 - alpha) * lo.head_q + alpha * hi.head_q if has_head else np.zeros(2),
            head_dq=(1.0 - alpha) * lo.head_dq + alpha * hi.head_dq if has_head else np.zeros(2),
            has_head=has_head,
        )

    def sample_delayed_frame(self, now_ns: int) -> A3TeleopFrame | None:
        with self._lock:
            return self._sample_locked(now_ns - self._opt.delay_ns)

    def compute_latest_yaw_offset_rad(self, robot_quat_wxyz) -> float:
        with self._lock:
            if not self._frames:
                return 0.0
            return yaw_offset_rad(robot_quat_wxyz, self._frames[-1].pelvis_quat_wxyz)

    def _future_step_ns(self) -> int:
        hz = self._opt.policy_hz if self._opt.policy_hz > 0.0 else 50.0
        skip = self._opt.future_frame_skip if self._opt.future_frame_skip > 0 else 5
        return int(round(1.0e9 * skip / hz))

    def build_tokenizer_slice(
        self,
        now_ns: int,
        robot_root_quat_wxyz,
        reference_yaw_offset_rad: float = 0.0,
    ) -> tuple[np.ndarray | None, TokenizerStatus]:
        """Return (640-float slice, status); slice is None unless status == RUNNING."""
        with self._lock:
            frames = self._frames
            if not frames:
                return None, TokenizerStatus.NO_DATA
            if len(frames) < 2:
                return None, TokenizerStatus.BUFFERING
            base_ns = now_ns - self._opt.delay_ns
            if base_ns < frames[0].stamp_ns:
                return None, TokenizerStatus.BUFFERING
            if base_ns > frames[-1].stamp_ns and not self._running_started:
                return None, TokenizerStatus.BUFFERING

            inv_robot = _conj(_normalize(np.asarray(robot_root_quat_wxyz, dtype=np.float64)))
            step_ns = self._future_step_ns()
            out = np.zeros(TOKENIZER_FLOATS, dtype=np.float32)
            for k in range(NUM_FUTURE_FRAMES):
                sample = self._sample_locked(base_ns + k * step_ns)
                if sample is None:
                    return None, TokenizerStatus.BUFFERING
                out[k * NUM_POLICY_DOFS : (k + 1) * NUM_POLICY_DOFS] = sample.q_mj29[A3_IL_TO_MJ29]
                out[_COMMAND_DOF_FLOATS + k * NUM_POLICY_DOFS : _COMMAND_DOF_FLOATS + (k + 1) * NUM_POLICY_DOFS] = (
                    sample.dq_mj29[A3_IL_TO_MJ29]
                )
                aligned = apply_yaw_offset(reference_yaw_offset_rad, sample.pelvis_quat_wxyz)
                _write_ori_6d(_mul(inv_robot, aligned), k, out)
            self._running_started = True
            return out, TokenizerStatus.RUNNING
