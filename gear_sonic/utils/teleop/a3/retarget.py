"""Real-time Pico (SMPL stream) -> A3 29-DoF joint-space reference.

This is the "upstream real-time retargeting module" the Litchi README says is missing:
it takes the SMPL stream that ``pico_manager_thread_server.py`` already publishes
(topic ``pose``) and produces ``A3TeleopFrame`` joint targets that the 035
``g1`` / ``a3_fast`` encoders consume (same payload as ``/ta/whole_body_command``).

Pipeline per frame::

    pico "pose" message (smpl_pose[21,3], smpl_joints[24,3] local, body_quat_w[4])
      -> undo the SONIC Pico conventions        (``pico_frame_to_human_data``)
      -> GMR smplx->agibot_a3_ultra whole-body IK (warm-started frame to frame)
      -> pick the 29 body joints (A3 and A3 Ultra share kinematics, worklog #37)
      -> finite-difference velocities + optional low-pass
      -> ``A3TeleopFrame``

Conventions reconstructed from ``pico_manager_thread_server.process_smpl_joints``:

* ``body_quat_w = remove_smpl_base_rot(ytoz(q_native))`` with
  ``ytoz(q) = X90 * q`` and ``remove_smpl_base_rot(q) = q * conj(BASE)``.
  So the Z-up SMPL(-X) root orientation GMR expects is ``body_quat_w * BASE``.
* ``smpl_joints`` are ``inv(body_quat_w)`` applied to the Z-up joint positions,
  so world positions are ``body_quat_w`` applied to them.
* Child joint orientations chain through ``smpl_pose`` exactly like
  ``general_motion_retargeting.utils.smpl.get_smplx_data``.

GMR is imported lazily (needs the ``gmr`` conda env: mujoco + mink + smplx).
"""

from __future__ import annotations

import contextlib
import io
import os
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

from gear_sonic.utils.teleop.a3.frames import (
    A3_IL_TO_MJ29,
    HEAD_JOINT_NAMES,
    MJ29_JOINT_NAMES,
    NUM_POLICY_DOFS,
    A3TeleopFrame,
)

# remove_smpl_base_rot() constant: 120 deg about [1,1,1] (wxyz).
_BASE_ROT = R.from_quat([0.5, 0.5, 0.5, 0.5], scalar_first=True)

# First 22 SMPL joints share order and parents with the SMPL-X body (GMR's JOINT_NAMES[:22]).
SMPL_PARENTS = (-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21)
SMPL_BODY_JOINT_NAMES = (
    "pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee", "spine2",
    "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot", "neck",
    "left_collar", "right_collar", "head", "left_shoulder", "right_shoulder",
    "left_elbow", "right_elbow", "left_wrist", "right_wrist",
)  # fmt: skip
NUM_BODY_JOINTS = len(SMPL_BODY_JOINT_NAMES)

# GMR scale reference ("actual_human_height" -> ratio = h / 1.8 on every body of the IK scale table).  This is NOT
# the operator's height: the Pico SMPL skeleton is always the neutral template (~1.66 m), but the A3 is a much
# bigger robot (pelvis 1.07 m standing vs 0.9 m).  At the literal 1.66 a standing person came out with 84 deg
# knees (pelvis 0.76 m) and the 035 policy crouched ~20 cm in MuJoCo.  2.18 makes the template body land on the
# policy's own default stand (knees ~12 deg vs the 035 default 14 deg); see docs/workflow/a3_integration_plan.md 12.5.
DEFAULT_HUMAN_HEIGHT_M = 2.18
DEFAULT_PELVIS_HEIGHT_M = 0.9

# Lower-body joints in the mj29 order (left leg 17..22, right leg 23..28).
LOWER_BODY_SLICE = slice(17, 29)
# Leg angles of the deploy's "policy stand" reference: the mean of teleop_motions/BMD_0319_a3_filtered_20260319_164305__stand_Skeleton0.csv
# (rad, left hip pitch/roll/yaw, knee, ankle pitch/roll, then right).  ``lower_body="stand"`` sends exactly these for the legs, so that
# following the operator and the policy stand (second A+X) use the same lower-body reference and the A+X switch has no leg jump.
# Why: the realtime GMR table (smplx_to_a3_ultra.json) gives the feet rotation weight 0, so a plainly standing person comes out with
# the ankle rolls at the joint limit (+0.348 / -0.347 rad) where the stand reference has +0.007 / -0.054 (2026-10-17, a3_ref_monitor.py).
STAND_LOWER_BODY_Q = np.array(
    [-0.0311, -0.0066, 0.0557, 0.1446, -0.0925, 0.0066, -0.0331, 0.0420, -0.1751, 0.1192, -0.0760, -0.0543],
    dtype=np.float64,
)
LOWER_BODY_MODES = ("track", "stand")
# SMPL bodies whose position is the IK target of the robot foot (smplx_to_a3_ultra.json, weight 100).
_FOOT_BODIES = ("left_foot", "right_foot")


def scale_foot_lift(human_data: dict, scale: float) -> dict:
    """Multiply each foot's height above the lower foot by ``scale``.

    ``1`` leaves the targets unchanged.  Only the vertical part above the lower foot moves, so a two-footed stance
    stays put and the foot's horizontal target does not.  The pelvis and the knees are not moved: the foot position
    weight (100) is what sets the reference lift the policy is asked to track.
    """
    if scale == 1.0:
        return human_data
    if scale <= 0.0:
        raise ValueError(f"foot_lift_scale must be positive, got {scale}")
    if any(name not in human_data for name in _FOOT_BODIES):
        return human_data
    ground = min(float(human_data[name][0][2]) for name in _FOOT_BODIES)
    out = {}
    for name, (pos, quat) in human_data.items():
        pos = np.array(pos, dtype=np.float64, copy=True)
        if name in _FOOT_BODIES:
            pos[2] = ground + scale * (float(pos[2]) - ground)
        out[name] = (pos, quat)
    return out


def pico_frame_to_human_data(
    smpl_pose: np.ndarray,
    body_quat_w: np.ndarray,
    smpl_joints_local: np.ndarray,
    pelvis_height: float = DEFAULT_PELVIS_HEIGHT_M,
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Rebuild GMR's ``{joint_name: (pos, quat_wxyz)}`` from one Pico SMPL frame.

    Args:
        smpl_pose: (21, 3) axis-angle local rotations of SMPL joints 1..21.
        body_quat_w: (4,) wxyz, root orientation with the SMPL base rotation removed
            (identity == standing, facing +x, Z up).
        smpl_joints_local: (24, 3) joint positions expressed in the ``body_quat_w`` frame.
        pelvis_height: constant world z for the pelvis target (the floating base is free
            in the IK, this only keeps targets away from the origin).
    """
    q_removed = R.from_quat(np.asarray(body_quat_w, dtype=np.float64), scalar_first=True)
    q_root = q_removed * _BASE_ROT  # Z-up SMPL-X style root orientation
    local = np.asarray(smpl_joints_local, dtype=np.float64)[:NUM_BODY_JOINTS]
    pos = q_removed.apply(local)
    pos = pos - pos[0] + np.array([0.0, 0.0, pelvis_height])

    rots: list[R] = []
    pose = np.asarray(smpl_pose, dtype=np.float64).reshape(-1, 3)
    for j in range(NUM_BODY_JOINTS):
        if j == 0:
            rots.append(q_root)
        else:
            rots.append(rots[SMPL_PARENTS[j]] * R.from_rotvec(pose[j - 1]))
    return {
        SMPL_BODY_JOINT_NAMES[j]: (pos[j].copy(), rots[j].as_quat(scalar_first=True))
        for j in range(NUM_BODY_JOINTS)
    }


class A3RealtimeRetargeter:
    """Stateful frame-by-frame SMPL -> A3 retargeter (GMR IK, warm-started)."""

    def __init__(
        self,
        human_height: float = DEFAULT_HUMAN_HEIGHT_M,
        max_iter: int | None = None,
        q_lowpass: float = 0.0,
        dq_lowpass: float = 0.5,
        verbose: bool = False,
        pelvis_rot_weight: float | None = 50.0,
        track_head: bool = False,
        lower_body: str = "track",
        foot_rot_weight: float | None = None,
        ankle_knee_rot_weight: float | None = None,
        foot_lift_scale: float = 1.0,
    ) -> None:
        if lower_body not in LOWER_BODY_MODES:
            raise ValueError(f"lower_body must be one of {LOWER_BODY_MODES}, got {lower_body!r}")
        if foot_lift_scale <= 0.0:
            raise ValueError(f"foot_lift_scale must be positive, got {foot_lift_scale}")
        self.lower_body = lower_body
        self.foot_lift_scale = float(foot_lift_scale)
        import mujoco as mj
        from general_motion_retargeting import GeneralMotionRetargeting as GMR

        sink = io.StringIO()
        with contextlib.redirect_stdout(sink if not verbose else io.StringIO()):
            self.gmr = GMR(
                src_human="smplx",
                tgt_robot="agibot_a3_ultra",
                actual_human_height=human_height,
                verbose=False,
            )
        if max_iter is not None:
            self.gmr.max_iter = int(max_iter)
        if pelvis_rot_weight is not None:
            # GMR's stock weights (10 / 5) let the free base trade pelvis orientation for foot/hip
            # targets: a plain standing human comes out with a ~35-40 deg pelvis tilt that the
            # waist/hips then compensate. The 035 policy tracks the pelvis anchor orientation, so
            # the reference must be upright like the Litchi mocap data (root rotation ~1 deg).
            for tasks in (self.gmr.tasks1, self.gmr.tasks2):
                for task in tasks:
                    if task.frame_name == "pelvis_link":
                        task.cost[3:] = float(pelvis_rot_weight)

        # Foot rotation: the stock smplx->A3 Ultra table gives the ankle-roll links position weight 100 and rotation weight 0, so nothing
        # constrains the foot orientation; the ankle pitch links instead follow the human KNEE orientation (weight 10 / 5).  A standing person
        # then comes out with the ankle roll at its limit (+0.348 / -0.347 rad).  ``foot_rot_weight`` gives the ankle-roll links a rotation
        # weight (they follow the human foot orientation), ``ankle_knee_rot_weight`` overrides the ankle-pitch-vs-knee rotation weight
        # (0 switches it off).  None keeps the stock value.
        for tasks in (self.gmr.tasks1, self.gmr.tasks2):
            for task in tasks:
                if foot_rot_weight is not None and task.frame_name in ("left_ankle_roll_Link", "right_ankle_roll_Link"):
                    task.cost[3:] = float(foot_rot_weight)
                if ankle_knee_rot_weight is not None and task.frame_name in ("left_ankle_pitch_Link", "right_ankle_pitch_Link"):
                    task.cost[3:] = float(ankle_knee_rot_weight)

        model = self.gmr.model
        self._qpos_adr = np.array(
            [model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)] for n in MJ29_JOINT_NAMES],
            dtype=np.int64,
        )
        head_ids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n) for n in HEAD_JOINT_NAMES]
        # The 035 policy (29 body joints) does not control the head: the deploy code holds both head joints at their
        # default pose (q = 0, Kp 40 / Kd 2) exactly as for the A3, and only follows a head command that arrives in
        # the frame.  The IK has no head task, so those joint values would be arbitrary solver leftovers -- send the
        # head only when explicitly asked to (``track_head``); by default frames carry ``has_head = False``.
        self._head_adr = (
            np.array([model.jnt_qposadr[i] for i in head_ids], dtype=np.int64)
            if (track_head and min(head_ids) >= 0)
            else None
        )
        self._q_alpha = float(q_lowpass)  # 0 -> no smoothing, else EMA weight on the previous value
        self._dq_alpha = float(dq_lowpass)
        self._prev_q: np.ndarray | None = None
        self._prev_dq = np.zeros(NUM_POLICY_DOFS)
        self._prev_t: float | None = None
        self.last_solve_ms = 0.0

    def reset(self) -> None:
        self._prev_q = None
        self._prev_dq = np.zeros(NUM_POLICY_DOFS)
        self._prev_t = None

    def retarget_human_data(self, human_data, t_seconds: float, stamp_ns: int) -> A3TeleopFrame:
        t0 = time.perf_counter()
        qpos = self.gmr.retarget(scale_foot_lift(human_data, self.foot_lift_scale))
        self.last_solve_ms = (time.perf_counter() - t0) * 1e3

        q = np.asarray(qpos[self._qpos_adr], dtype=np.float64)
        if self.lower_body == "stand":
            q = q.copy()
            q[LOWER_BODY_SLICE] = STAND_LOWER_BODY_Q
        if self._q_alpha > 0.0 and self._prev_q is not None:
            q = self._q_alpha * self._prev_q + (1.0 - self._q_alpha) * q
        if self._prev_q is not None and self._prev_t is not None and t_seconds > self._prev_t:
            dq_raw = (q - self._prev_q) / (t_seconds - self._prev_t)
            dq = self._dq_alpha * self._prev_dq + (1.0 - self._dq_alpha) * dq_raw
        else:
            dq = np.zeros(NUM_POLICY_DOFS)
        self._prev_q, self._prev_dq, self._prev_t = q.copy(), dq.copy(), t_seconds

        frame = A3TeleopFrame(
            stamp_ns=int(stamp_ns),
            q_mj29=q,
            dq_mj29=dq,
            pelvis_quat_wxyz=np.asarray(qpos[3:7], dtype=np.float64).copy(),
        )
        if self._head_adr is not None:
            frame.head_q = np.asarray(qpos[self._head_adr], dtype=np.float64).copy()
            frame.has_head = True
        return frame

    def retarget_pico_frame(
        self,
        smpl_pose: np.ndarray,
        body_quat_w: np.ndarray,
        smpl_joints_local: np.ndarray,
        t_seconds: float,
        stamp_ns: int | None = None,
    ) -> A3TeleopFrame:
        human = pico_frame_to_human_data(smpl_pose, body_quat_w, smpl_joints_local)
        return self.retarget_human_data(
            human, t_seconds, time.time_ns() if stamp_ns is None else stamp_ns
        )


def pose_window_catch_up(prev_window: tuple[int, int] | None, window_indices) -> str | None:
    """Why G1's deploy would "catch up" on this Pico window (drop what it buffered, restart from the incoming window).

    Port of ``StreamedMotionMerger::CalculateSlidingWindow`` (gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/
    input_interface/streamed_motion_merger.hpp), the receiver of the official sender.  ``prev_window`` is the
    ``(first, last)`` ``frame_index`` of the previously accepted message, ``window_indices`` the new message's indices
    (time-ordered, like the sender's sliding window).  Returns ``None`` for a normal slide, else the reason:

    * ``"restart"`` -- ``start <= previous start``: the sender restarted its frame counter.  ``PoseStreamer.
      on_mode_exit()`` sets ``step = 0`` whenever the operator leaves POSE mode, so every re-entry (A+X) starts again
      from frame 0 -- the "exit and re-enter POSE mode" case.  (An old or repeated window lands here too, as in G1.)
    * ``"stale"``   -- ``end <= previous end``: nothing newer than what was already merged.
    * ``"gap"``     -- ``start > previous end + step``: frames were lost between the two windows.

    G1 additionally catches up when its playback cursor lags more than ``MAX_GAP_FRAMES``; that depends on the playback
    position of the receiver and has no counterpart here.  The first packet (``prev_window is None``) is not a
    catch-up: a new session starts anyway.
    """
    if prev_window is None:
        return None
    idx = np.asarray(window_indices).reshape(-1)
    start, end = int(idx[0]), int(idx[-1])
    step = abs(int(idx[1]) - int(idx[0])) if len(idx) >= 2 else 1  # G1's CalculateFrameStep
    step = step if step > 0 else 1
    if start <= prev_window[0]:
        return "restart"
    if end <= prev_window[1]:
        return "stale"
    if start > prev_window[1] + step:
        return "gap"
    return None


class A3RefStream:
    """Per-message logic of ``a3_teleop_retarget_node.py --mode a3-ref``: Pico window -> new A3 joint-reference frames.

    * overlapping windows are de-duplicated by ``frame_index`` (each frame is solved once),
    * the session restarts exactly when G1's receiver catches up (:func:`pose_window_catch_up`) -- in particular when
      the sender restarted its frame counter after the operator left and re-entered POSE mode; without this every frame
      of the new session would be skipped until its counter passed the old maximum (tens of seconds of frozen arms),
    * the first frame of a session lets the warm-started IK settle, frames are back-dated inside the window.

    ``events`` collects human-readable notes (catch-ups, dropped / non-finite frames) for the node to print.
    """

    def __init__(self, retargeter: A3RealtimeRetargeter, fps: float = 50.0):
        self.retargeter = retargeter
        self.fps = float(fps)
        self.last_index = -1
        self.catch_ups = 0
        self.window: tuple[int, int] | None = None
        self.events: list[str] = []
        self.solve_ms: list[float] = []

    def process(self, msg: dict[str, np.ndarray], arrival_ns: int) -> list[A3TeleopFrame]:
        idx = np.asarray(msg["frame_index"]).reshape(-1)
        n = len(idx)
        step_ns = int(1e9 / self.fps)
        reason = pose_window_catch_up(self.window, idx)
        if reason:
            self.events.append(
                f"new Pico session ({reason}: window [{int(idx[0])}..{int(idx[-1])}] after "
                f"[{self.window[0]}..{self.window[1]}]) -- same catch-up rule as G1's receiver"
            )
            self.catch_ups += 1
            self.last_index = -1
        self.window = (int(idx[0]), int(idx[-1]))
        out: list[A3TeleopFrame] = []
        for k in range(n):
            fi = int(idx[k])
            if fi <= self.last_index:
                continue
            if self.last_index >= 0 and fi - self.last_index > 1:
                self.events.append(f"dropped {fi - self.last_index - 1} frame(s) before {fi}")
            # the window is time-ordered; frame n-1 is "now", older ones are back-dated.
            stamp_ns = arrival_ns - (n - 1 - k) * step_ns
            args = (msg["smpl_pose"][k], msg["body_quat_w"][k], msg["smpl_joints"][k], fi / self.fps, stamp_ns)
            frame = self.retargeter.retarget_pico_frame(*args)
            if self.last_index < 0:  # first frame of a session: let the warm-started IK settle
                for _ in range(5):
                    frame = self.retargeter.retarget_pico_frame(*args)
                self.retargeter.reset()
                frame = self.retargeter.retarget_pico_frame(*args)
            self.last_index = fi
            if not frame.is_finite():
                self.events.append("non-finite frame skipped")
                continue
            self.solve_ms.append(self.retargeter.last_solve_ms)
            out.append(frame)
        return out


class A3SmplPoseAugmenter:
    """Pico ``pose`` message -> the same message with ``joint_pos`` = A3 joint angles (IsaacLab order).

    The A3 deploy's SMPL source (``a3_zmq_smpl_source.cpp``) reads the G1 packed pose stream.  Its SMPL tokenizer is
    ``[10 x 24 SMPL joints | 10 x root orientation 6D | 10 x wrist joints]`` where the last 60 floats are the six wrist
    angles (IsaacLab indices 23..28) taken from ``joint_pos``.  The official G1 sender fills those with a G1-specific
    formula (G1 wrist axes / signs); on A3 they must be A3 wrist angles.  They are computed with the same GMR IK
    (``agibot_a3_ultra``) that produces the A3 reference data the SMPL encoder is trained against, so the deploy-time
    input follows the training-time distribution.  All other fields are passed through unchanged.
    """

    def __init__(self, retargeter: A3RealtimeRetargeter | None = None, fps: float = 50.0, cache_size: int = 512, **kw):
        self.retargeter = retargeter or A3RealtimeRetargeter(**kw)
        self.fps = float(fps)
        self._cache: dict[int, np.ndarray] = {}
        self._cache_size = int(cache_size)
        self._last_index = -1
        self._window: tuple[int, int] | None = None
        self.catch_ups = 0

    def reset(self) -> None:
        self._cache.clear()
        self._last_index = -1
        self._window = None
        self.retargeter.reset()

    def _solve(self, msg: dict[str, np.ndarray], k: int, fi: int) -> np.ndarray:
        rt = self.retargeter
        args = (msg["smpl_pose"][k], msg["body_quat_w"][k], msg["smpl_joints"][k], fi / self.fps, time.time_ns())
        if self._last_index < 0:  # first frame: let the warm-started IK settle, as the a3_ref node does
            for _ in range(5):
                rt.retarget_pico_frame(*args)
            rt.reset()
        frame = rt.retarget_pico_frame(*args)
        q_il = np.asarray(frame.q_mj29, dtype=np.float64)[A3_IL_TO_MJ29]
        return q_il if np.all(np.isfinite(q_il)) else np.zeros(NUM_POLICY_DOFS)

    def augment(self, msg: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
        """Return a copy of ``msg`` whose ``joint_pos`` (N, 29) holds the A3 joint angles of each window frame."""
        idx = np.asarray(msg["frame_index"]).reshape(-1)
        # Same rule as G1's receiver (and the C++ StreamedMotionMerger this window is forwarded to): after a catch-up the
        # cached solutions belong to another session -- the sender re-entered POSE mode and numbers its frames from 0.
        if pose_window_catch_up(self._window, idx):
            self.catch_ups += 1
            self.reset()
        self._window = (int(idx[0]), int(idx[-1]))
        rows = []
        for k, fi in enumerate(int(i) for i in idx):
            q = self._cache.get(fi)
            if q is None:
                if fi <= self._last_index and self._cache:  # late/replayed frame not in the cache: reuse the nearest
                    q = self._cache[min(self._cache, key=lambda c: abs(c - fi))]
                else:
                    q = self._solve(msg, k, fi)
                    self._cache[fi] = q
                    self._last_index = max(self._last_index, fi)
                    while len(self._cache) > self._cache_size:
                        del self._cache[min(self._cache)]
            rows.append(q)
        out = dict(msg)
        out["joint_pos"] = np.stack(rows, axis=0).astype(np.float64)
        return out


def unpack_pose_message(message: bytes, topic: bytes = b"pose", header_size: int = 1280) -> dict[str, np.ndarray]:
    """Decode a Pico ``pose`` ZMQ message ([topic][1280B JSON header][field bytes])."""
    import json

    if not message.startswith(topic):
        raise ValueError("unexpected topic prefix")
    header_raw = message[len(topic) : len(topic) + header_size]
    header = json.loads(header_raw.rstrip(b"\x00").decode("utf-8"))
    dtypes = {"f32": "<f4", "f64": "<f8", "i32": "<i4", "i64": "<i8", "bool": "?"}
    out: dict[str, np.ndarray] = {}
    offset = len(topic) + header_size
    for field in header["fields"]:
        dt = np.dtype(dtypes[field["dtype"]])
        shape = tuple(field["shape"])
        n = int(np.prod(shape)) if shape else 1
        nbytes = n * dt.itemsize
        out[field["name"]] = np.frombuffer(message, dtype=dt, count=n, offset=offset).reshape(shape).copy()
        offset += nbytes
    if offset != len(message):
        raise ValueError(f"pose message size mismatch: parsed {offset}, got {len(message)}")
    return out


def default_gmr_env() -> None:
    """Silence noisy GMR/mujoco start-up output unless the user asked for it."""
    os.environ.setdefault("MUJOCO_GL", "egl")
