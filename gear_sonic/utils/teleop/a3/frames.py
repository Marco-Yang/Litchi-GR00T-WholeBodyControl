"""A3 teleop reference frames and the ZMQ wire format shared with the C++ deploy.

One ``A3TeleopFrame`` is the joint-space target the upstream retargeter produces
for the 035 ``g1`` / ``a3_fast`` encoders. It mirrors the C++ struct
``a3_deploy::A3TeleopFrame`` (``a3_teleop_reference.hpp``) and the contents of the
AimRT ``/ta/whole_body_command`` message, so the same downstream buffer consumes
frames from either transport.

Joint order ("mj29", a.k.a. the policy view the C++ code calls ``q_mujoco``):
    waist[0..2] (yaw, roll, pitch)
    left arm [3..9]   (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw)
    right arm [10..16]
    left leg [17..22] (hip pitch/roll/yaw, knee, ankle pitch/roll)
    right leg [23..28]

Wire format (little-endian, no padding), published on ZMQ PUB with topic prefix
``a3_ref``::

    u32 magic = 0x46335441 ("A3TF")
    u32 version = 1
    i64 sender_stamp_ns        (informational only; receivers re-stamp on arrival,
                                exactly like the AimRT /ta/whole_body_command path)
    f64 q[29]                  joint position targets, mj29 order, rad
    f64 dq[29]                 joint velocity targets, mj29 order, rad/s
    f64 pelvis_quat_wxyz[4]    reference pelvis orientation in the TA world frame
    u8  has_head
    f64 head_q[2]              (head yaw, head pitch) rad
    f64 head_dq[2]             rad/s
"""

from __future__ import annotations

from dataclasses import dataclass, field
import struct

import numpy as np

NUM_POLICY_DOFS = 29

# mj29 / "policy view" order -- must match a3_policy_parameters.hpp
# (a3_mujoco_to_isaaclab) and ConvertTaWholeBodyCommand() in the C++ deploy.
MJ29_JOINT_NAMES: tuple[str, ...] = (
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)

HEAD_JOINT_NAMES: tuple[str, ...] = ("head_yaw_joint", "ultra_head_pitch_joint")

# mj29 index -> isaaclab index permutation, copied from a3_policy_parameters.hpp:
# ``a3_mujoco_to_isaaclab[i_il] == i_mj``.
A3_IL_TO_MJ29 = np.array(
    [17, 23, 0, 18, 24, 1, 19, 25, 2, 20, 26, 3, 10, 21, 27, 4, 11, 22, 28, 5, 12, 6, 13, 7, 14, 8, 15, 9, 16],
    dtype=np.int64,
)

# Default standing pose in mj29 order (same constants as sim2sim_a3_mujoco.py /
# a3_policy_parameters.hpp::a3_default_angles).
DEFAULT_Q_BY_NAME = {
    "left_hip_pitch_joint": -0.1311,
    "right_hip_pitch_joint": -0.1311,
    "left_hip_roll_joint": 0.0056,
    "right_hip_roll_joint": -0.0056,
    "left_hip_yaw_joint": -0.0348,
    "right_hip_yaw_joint": 0.0348,
    "left_knee_joint": 0.2468,
    "right_knee_joint": 0.2468,
    "left_ankle_pitch_joint": -0.1204,
    "right_ankle_pitch_joint": -0.1204,
    "left_ankle_roll_joint": -0.0078,
    "right_ankle_roll_joint": 0.0078,
    "waist_yaw_joint": 0.0,
    "waist_roll_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "left_shoulder_pitch_joint": 0.3,
    "right_shoulder_pitch_joint": 0.3,
    "left_shoulder_roll_joint": 0.12,
    "right_shoulder_roll_joint": -0.12,
    "left_shoulder_yaw_joint": 0.0,
    "right_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.8,
    "right_elbow_joint": 0.8,
    "left_wrist_roll_joint": 0.0,
    "right_wrist_roll_joint": 0.0,
    "left_wrist_pitch_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "left_wrist_yaw_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
}
DEFAULT_Q_MJ29 = np.array([DEFAULT_Q_BY_NAME[n] for n in MJ29_JOINT_NAMES], dtype=np.float64)

WIRE_TOPIC = b"a3_ref"
WIRE_MAGIC = 0x46335441
WIRE_VERSION = 1
_WIRE_STRUCT = struct.Struct("<IIq29d29d4dB2d2d")
WIRE_BODY_SIZE = _WIRE_STRUCT.size


@dataclass
class A3TeleopFrame:
    stamp_ns: int = 0
    q_mj29: np.ndarray = field(default_factory=lambda: DEFAULT_Q_MJ29.copy())
    dq_mj29: np.ndarray = field(default_factory=lambda: np.zeros(NUM_POLICY_DOFS))
    pelvis_quat_wxyz: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    head_q: np.ndarray = field(default_factory=lambda: np.zeros(2))
    head_dq: np.ndarray = field(default_factory=lambda: np.zeros(2))
    has_head: bool = False

    def is_finite(self) -> bool:
        arrays = [self.q_mj29, self.dq_mj29, self.pelvis_quat_wxyz]
        if self.has_head:
            arrays += [self.head_q, self.head_dq]
        return all(bool(np.all(np.isfinite(a))) for a in arrays)


def pack_frame(frame: A3TeleopFrame, topic: bytes = WIRE_TOPIC) -> bytes:
    body = _WIRE_STRUCT.pack(
        WIRE_MAGIC,
        WIRE_VERSION,
        int(frame.stamp_ns),
        *np.asarray(frame.q_mj29, dtype=np.float64).tolist(),
        *np.asarray(frame.dq_mj29, dtype=np.float64).tolist(),
        *np.asarray(frame.pelvis_quat_wxyz, dtype=np.float64).tolist(),
        1 if frame.has_head else 0,
        *np.asarray(frame.head_q, dtype=np.float64).tolist(),
        *np.asarray(frame.head_dq, dtype=np.float64).tolist(),
    )
    return topic + body


def unpack_frame(message: bytes, topic: bytes = WIRE_TOPIC) -> A3TeleopFrame:
    if not message.startswith(topic):
        raise ValueError("unexpected topic prefix")
    body = message[len(topic):]
    if len(body) != WIRE_BODY_SIZE:
        raise ValueError(f"bad body size {len(body)} (expected {WIRE_BODY_SIZE})")
    values = _WIRE_STRUCT.unpack(body)
    magic, version, stamp_ns = values[0], values[1], values[2]
    if magic != WIRE_MAGIC or version != WIRE_VERSION:
        raise ValueError(f"bad magic/version {magic:#x}/{version}")
    n = NUM_POLICY_DOFS
    q = np.array(values[3 : 3 + n], dtype=np.float64)
    dq = np.array(values[3 + n : 3 + 2 * n], dtype=np.float64)
    off = 3 + 2 * n
    quat = np.array(values[off : off + 4], dtype=np.float64)
    has_head = bool(values[off + 4])
    head_q = np.array(values[off + 5 : off + 7], dtype=np.float64)
    head_dq = np.array(values[off + 7 : off + 9], dtype=np.float64)
    return A3TeleopFrame(
        stamp_ns=int(stamp_ns),
        q_mj29=q,
        dq_mj29=dq,
        pelvis_quat_wxyz=quat,
        head_q=head_q,
        head_dq=head_dq,
        has_head=has_head,
    )
