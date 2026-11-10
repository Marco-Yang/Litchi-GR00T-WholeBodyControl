"""Diagnostics for MuJoCo sim2sim + ZMQ SMPL pipeline."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R

LOG = logging.getLogger(__name__)

# G1 29-DoF: MuJoCo qpos order -> IsaacLab / deploy motor order.
MUJOCO_TO_ISAACLAB = np.array(
    [0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8, 11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28],
    dtype=np.int32,
)


def isaaclab_to_mujoco_dof(dof_il: np.ndarray) -> np.ndarray:
    """Inverse of ``mujoco_to_isaaclab_dof``."""
    dof_il = np.asarray(dof_il, dtype=np.float64)
    dof_mj = np.empty_like(dof_il)
    for mj in range(29):
        dof_mj[..., mj] = dof_il[..., MUJOCO_TO_ISAACLAB[mj]]
    return dof_mj


def mujoco_to_isaaclab_dof(dof_mj: np.ndarray) -> np.ndarray:
    dof_mj = np.asarray(dof_mj, dtype=np.float64)
    dof_il = np.empty_like(dof_mj)
    for mj in range(29):
        dof_il[..., MUJOCO_TO_ISAACLAB[mj]] = dof_mj[..., mj]
    return dof_il


def pkl_root_quat_to_wxyz(root_rot_xyzw: np.ndarray) -> np.ndarray:
    """Robot PKL stores root quat as xyzw; MuJoCo qpos uses wxyz."""
    return R.from_quat(np.asarray(root_rot_xyzw, dtype=np.float64), scalar_first=False).as_quat(
        scalar_first=True
    )


def pelvis_up_dot(model, data, body_name: str = "pelvis") -> float:
    import mujoco

    pelvis = model.body(body_name).id
    mujoco.mj_forward(model, data)
    z_axis = data.xmat[pelvis].reshape(3, 3)[:, 2]
    return float(np.dot(z_axis, [0.0, 0.0, 1.0]))


def vr_3point_root_relative_to_world(
    vr_pos_root_frame: np.ndarray,
    root_trans: np.ndarray,
    root_quat_wxyz: np.ndarray,
) -> np.ndarray:
    """Convert deploy ``g1_debug`` VR 3-point positions to world frame.

    Deploy stores positions in root frame (translate + inverse root rotation).
    See ``g1_deploy_onnx_ref.cpp`` GatherVR3PointPosition normalization.
    """
    root = np.asarray(root_trans, dtype=np.float64)[:3]
    root_rot = R.from_quat(np.asarray(root_quat_wxyz, dtype=np.float64)[:4], scalar_first=True)
    rel = np.asarray(vr_pos_root_frame, dtype=np.float64).reshape(3)
    return root + root_rot.apply(rel)


def vr_3point_world_to_root_relative(
    vr_pos_world: np.ndarray,
    root_trans: np.ndarray,
    root_quat_wxyz: np.ndarray,
) -> np.ndarray:
    root = np.asarray(root_trans, dtype=np.float64)[:3]
    root_rot = R.from_quat(np.asarray(root_quat_wxyz, dtype=np.float64)[:4], scalar_first=True)
    rel_world = np.asarray(vr_pos_world, dtype=np.float64).reshape(3) - root
    return root_rot.inv().apply(rel_world)


@dataclass
class DiagnosticIssue:
    level: str  # "error" | "warn" | "info"
    code: str
    message: str


@dataclass
class DiagnosticReport:
    issues: list[DiagnosticIssue] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)

    def add(self, level: str, code: str, message: str) -> None:
        self.issues.append(DiagnosticIssue(level, code, message))

    @property
    def ok(self) -> bool:
        return not any(i.level == "error" for i in self.issues)

    def log_summary(self, logger: logging.Logger | None = None) -> None:
        logger = logger or LOG
        for issue in self.issues:
            line = f"[{issue.code}] {issue.message}"
            if issue.level == "error":
                logger.error(line)
            elif issue.level == "warn":
                logger.warning(line)
            else:
                logger.info(line)
        if self.metrics:
            logger.info("metrics=%s", self.metrics)


def check_pkl_root_quat(root_rot_xyzw: np.ndarray) -> DiagnosticReport:
    report = DiagnosticReport()
    q_wrong = np.asarray(root_rot_xyzw, dtype=np.float64)
    up_wrong = float(R.from_quat(q_wrong, scalar_first=True).apply([0, 0, 1])[2])
    q_right = pkl_root_quat_to_wxyz(q_wrong)
    up_right = float(R.from_quat(q_right, scalar_first=True).apply([0, 0, 1])[2])
    report.metrics["up_dot_xyzw_as_wxyz"] = up_wrong
    report.metrics["up_dot_converted_wxyz"] = up_right
    if up_wrong < 0.5 and up_right > 0.9:
        report.add(
            "warn",
            "QUAT_CONVENTION",
            f"PKL root_rot looks xyzw (up_dot={up_wrong:.3f}); convert to wxyz (up_dot={up_right:.3f}) for MuJoCo.",
        )
    elif up_right < 0.5:
        report.add("error", "QUAT_UPRIGHT", f"Converted root quat still not upright: up_dot={up_right:.3f}")
    return report


def check_dof_permutation(dof_mj: np.ndarray) -> DiagnosticReport:
    report = DiagnosticReport()
    dof_mj = np.asarray(dof_mj, dtype=np.float64).reshape(-1)[:29]
    dof_il = mujoco_to_isaaclab_dof(dof_mj)
    back = isaaclab_to_mujoco_dof(dof_il)
    err = float(np.max(np.abs(back - dof_mj)))
    report.metrics["dof_roundtrip_max_err"] = err
    if err > 1e-9:
        report.add("error", "DOF_PERMUTATION", f"MuJoCo<->IsaacLab roundtrip error={err:.6g}")
    return report


def diagnose_g1_debug_msg(msg: dict | None) -> DiagnosticReport:
    report = DiagnosticReport()
    if msg is None:
        report.add("warn", "G1_DEBUG_MISSING", "No g1_debug message received yet.")
        return report

    keys = sorted(msg.keys())
    report.metrics["keys"] = keys
    for required in ("body_q_measured", "base_trans_measured", "base_quat_measured"):
        if required not in msg:
            report.add("warn", "G1_DEBUG_FIELD", f"Missing {required} in g1_debug.")

    if "vr_3point_position" not in msg:
        report.add(
            "warn",
            "VR3POINT_MISSING",
            "No vr_3point_position in g1_debug; command markers cannot be drawn.",
        )
        return report

    root = np.asarray(msg.get("base_trans_measured", [0, 0, 0.78]), dtype=np.float64)[:3]
    quat = np.asarray(msg.get("base_quat_measured", msg.get("base_quat", [1, 0, 0, 0])), dtype=np.float64)[:4]
    vr = np.asarray(msg["vr_3point_position"], dtype=np.float64).reshape(-1)
    if vr.size == 3:
        vr = vr.reshape(1, 3)
    else:
        vr = vr.reshape(3, 3)
    world_naive = vr + root
    world_correct = np.stack(
        [vr_3point_root_relative_to_world(vr[i], root, quat) for i in range(vr.shape[0])],
        axis=0,
    )
    naive_vs_correct = float(np.max(np.abs(world_naive - world_correct)))
    report.metrics["vr_marker_naive_vs_rotated_max_dist"] = naive_vs_correct
    report.metrics["vr_point_z_world"] = world_correct[:, 2].tolist()
    if naive_vs_correct > 0.05:
        report.add(
            "info",
            "VR3POINT_TRANSFORM",
            f"VR markers need root rotation (naive add error={naive_vs_correct:.3f} m).",
        )
    return report


def summarize_sim_state(model, data, *, elastic_band_enabled: bool | None = None) -> dict[str, Any]:
    import mujoco

    mujoco.mj_forward(model, data)
    pelvis_id = model.body("pelvis").id
    return {
        "sim_time": float(data.time),
        "root_z": float(data.qpos[2]),
        "root_xy": data.qpos[:2].tolist(),
        "pelvis_z": float(data.xpos[pelvis_id][2]),
        "pelvis_up_dot": pelvis_up_dot(model, data),
        "elastic_band_enabled": elastic_band_enabled,
    }


def check_zmq_stream_frame(stream: dict, frame_idx: int = 0, robot: dict | None = None) -> DiagnosticReport:
    report = DiagnosticReport()
    for key in ("smpl_pose", "smpl_joints_local", "body_quat_w"):
        if key not in stream:
            report.add("error", "ZMQ_STREAM_FIELD", f"Missing stream field: {key}")
    if not report.ok:
        return report
    quat = np.asarray(stream["body_quat_w"][frame_idx], dtype=np.float64)
    up = float(R.from_quat(quat, scalar_first=True).apply([0, 0, 1])[2])
    report.metrics["body_quat_up_dot"] = up
    report.metrics["smpl_joints_shape"] = np.asarray(stream["smpl_joints_local"][frame_idx]).shape
    if up < 0.5:
        report.add("warn", "BODY_QUAT_UPRIGHT", f"body_quat_w frame {frame_idx} not upright: up_dot={up:.3f}")
    if robot is not None and "root_rot" in robot:
        fi = int(stream.get("frame_indices", [frame_idx])[frame_idx])
        expected = pkl_root_quat_to_wxyz(robot["root_rot"][fi])
        err = float(np.max(np.abs(quat - expected)))
        report.metrics["body_quat_vs_robot_root_max_err"] = err
        if err > 1e-4:
            report.add(
                "error",
                "BODY_QUAT_MISMATCH",
                f"body_quat_w must match robot root_rot (max err={err:.6g}); "
                "using SMPL global_orient causes deploy heading mismatch.",
            )
    return report
