"""ZMQ pose frame conventions and PICO-5 marker transforms."""

from __future__ import annotations

from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R

from gear_sonic.utils.zmq_pose_unpack import PICO5_NAMES, PICO5_SMPL_INDICES
from gear_sonic.utils.zmq_sim_diagnostics import isaaclab_to_mujoco_dof

# SMPL-24 index 0 is pelvis / root in our retargeted PKL convention.
SMPL_PELVIS_INDEX = 0

# G1 semantic PICO-5 proxy bodies (same as demo_five_point_smpl_zmq_pipeline).
G1_PICO5_BODY_NAMES = (
    "torso_link",
    "left_wrist_yaw_link",
    "right_wrist_yaw_link",
    "left_ankle_roll_link",
    "right_ankle_roll_link",
)
HEAD_TORSO_OFFSET = np.array([0.0, 0.0, 0.35], dtype=np.float64)

# Retargeted SMPL PKL joints are pelvis-local but X/Y are rotated vs G1 root-local
# (SMPL +X ≈ robot +Y). Apply this yaw before body_quat when drawing world markers.
SMPL_LOCAL_YAW_TO_ROBOT_DEG = 90.0


def smpl_joints_local_to_world(
    joints_local: np.ndarray,
    root_trans: np.ndarray,
    body_quat_wxyz: np.ndarray,
    *,
    subtract_pelvis: bool = True,
    align_smpl_yaw_to_robot: bool = False,
) -> np.ndarray:
    """Convert pelvis-local SMPL joints to MuJoCo world frame.

    SONIC ZMQ v3 ``smpl_joints`` are in pelvis-local coordinates (same as PICO
    teleop ``smpl_joints_local``). They must NOT be drawn or interpreted as
    world xyz directly.

    Retargeted PKLs keep a constant pelvis offset at index 0; subtract it so
    ``root_trans`` anchors the pelvis in world frame.

    For legacy PKL streams, set ``align_smpl_yaw_to_robot=True`` to rotate SMPL
    local XY into G1 root-local (Y-forward). Streams from ``build_gt_zmq_stream``
    / ``build_fitted_zmq_stream`` are already aligned on publish.
    """
    joints = np.asarray(joints_local, dtype=np.float64).reshape(-1, 3)
    root = np.asarray(root_trans, dtype=np.float64).reshape(3)
    quat = np.asarray(body_quat_wxyz, dtype=np.float64).reshape(4)
    if subtract_pelvis and joints.shape[0] > SMPL_PELVIS_INDEX:
        joints = joints - joints[SMPL_PELVIS_INDEX]
    if align_smpl_yaw_to_robot:
        joints = R.from_euler("z", SMPL_LOCAL_YAW_TO_ROBOT_DEG, degrees=True).apply(joints)
    rot = R.from_quat(quat, scalar_first=True)
    return root + rot.apply(joints)


def extract_pico5_world_points(
    msg: dict[str, np.ndarray],
    *,
    align_smpl_yaw_to_robot: bool = False,
) -> np.ndarray | None:
    """Return (5, 3) PICO semantic points in world frame from a pose message."""
    if "smpl_joints" not in msg:
        return None
    joints = np.asarray(msg["smpl_joints"], dtype=np.float64).reshape(-1, 3)
    if joints.shape[0] <= max(PICO5_SMPL_INDICES):
        return None
    root = np.zeros(3, dtype=np.float64)
    quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if "root_trans" in msg:
        root = np.asarray(msg["root_trans"], dtype=np.float64).reshape(-1)[:3]
    if "body_quat_w" in msg:
        quat = np.asarray(msg["body_quat_w"], dtype=np.float64).reshape(-1)[:4]
    world = smpl_joints_local_to_world(
        joints,
        root,
        quat,
        align_smpl_yaw_to_robot=align_smpl_yaw_to_robot,
    )
    return world[list(PICO5_SMPL_INDICES), :]


def assert_smpl_joints_are_pelvis_local(joints_seq: np.ndarray, *, atol: float = 1e-4) -> None:
    """Pelvis joint (index 0) should be constant in pelvis-local storage."""
    joints_seq = np.asarray(joints_seq, dtype=np.float64)
    pelvis = joints_seq[:, SMPL_PELVIS_INDEX, :]
    if joints_seq.shape[0] < 2:
        return
    if float(np.max(np.abs(pelvis - pelvis[0]))) > atol:
        raise AssertionError(
            f"smpl_joints pelvis index {SMPL_PELVIS_INDEX} varies across frames; "
            "expected pelvis-local coordinates"
        )


def assert_pico5_world_sane(world5: np.ndarray, *, min_z: float = 0.05) -> None:
    """Sanity-check transformed PICO5 points are above ground in MuJoCo Z-up."""
    world5 = np.asarray(world5, dtype=np.float64).reshape(5, 3)
    head_wrist_z = world5[:3, 2]
    ankle_z = world5[3:, 2]
    if float(np.min(head_wrist_z)) < min_z:
        raise AssertionError(f"PICO5 head/wrist world z below {min_z}: {head_wrist_z}")
    # Ankles may sit slightly below ground in retargeted SMPL (foot penetration).
    if float(np.min(ankle_z)) < -0.15:
        raise AssertionError(f"PICO5 ankle world z below -0.15: {ankle_z}")


def g1_pico5_world_from_g1_debug(msg: dict[str, np.ndarray], model, data) -> np.ndarray | None:
    """Return (5, 3) SONIC target skeleton from deploy ``g1_debug`` viz fields."""
    if "body_q_target" not in msg:
        return None
    pose_msg: dict[str, np.ndarray] = {
        "joint_pos": np.asarray(msg["body_q_target"], dtype=np.float64),
    }
    if "base_trans_target" in msg:
        pose_msg["root_trans"] = np.asarray(msg["base_trans_target"], dtype=np.float64)
    if "base_quat_target" in msg:
        pose_msg["body_quat_w"] = np.asarray(msg["base_quat_target"], dtype=np.float64)
    return g1_pico5_world_from_pose_msg(pose_msg, model, data)


def g1_pico5_world_from_pose_msg(msg: dict[str, np.ndarray], model, data) -> np.ndarray | None:
    """Return (5, 3) G1 FK world positions from ZMQ ``joint_pos`` (IsaacLab order)."""
    import mujoco

    if "joint_pos" not in msg:
        return None
    root = np.zeros(3, dtype=np.float64)
    quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if "root_trans" in msg:
        root = np.asarray(msg["root_trans"], dtype=np.float64).reshape(-1)[:3]
    if "body_quat_w" in msg:
        quat = np.asarray(msg["body_quat_w"], dtype=np.float64).reshape(-1)[:4]
    dof_il = np.asarray(msg["joint_pos"], dtype=np.float64).reshape(-1)[:29]
    dof_mj = isaaclab_to_mujoco_dof(dof_il)
    n_dof = min(29, model.nq - 7)
    data.qpos[:3] = root
    data.qpos[3:7] = quat
    data.qpos[7 : 7 + n_dof] = dof_mj[:n_dof]
    mujoco.mj_forward(model, data)

    out = np.zeros((5, 3), dtype=np.float64)
    torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, G1_PICO5_BODY_NAMES[0])
    out[0] = data.xpos[torso_id] + HEAD_TORSO_OFFSET
    for i, name in enumerate(G1_PICO5_BODY_NAMES[1:], start=1):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        out[i] = data.xpos[bid]
    return out


def compare_pico5_smpl_vs_g1(
    smpl_world5: np.ndarray,
    g1_world5: np.ndarray,
    root_trans: np.ndarray,
) -> dict[str, Any]:
    """Per-point height/offset deltas between SMPL command and G1 FK reference."""
    smpl_world5 = np.asarray(smpl_world5, dtype=np.float64).reshape(5, 3)
    g1_world5 = np.asarray(g1_world5, dtype=np.float64).reshape(5, 3)
    root = np.asarray(root_trans, dtype=np.float64).reshape(3)
    rel_smpl = smpl_world5 - root
    rel_g1 = g1_world5 - root
    dist = np.linalg.norm(smpl_world5 - g1_world5, axis=-1)
    xy_dist = np.linalg.norm(smpl_world5[:, :2] - g1_world5[:, :2], axis=-1)
    return {
        "names": PICO5_NAMES,
        "smpl_rel_root_z": rel_smpl[:, 2].tolist(),
        "g1_rel_root_z": rel_g1[:, 2].tolist(),
        "dz_smpl_minus_g1": (rel_smpl[:, 2] - rel_g1[:, 2]).tolist(),
        "world_dist_m": dist.tolist(),
        "world_xy_dist_m": xy_dist.tolist(),
        "ankle_smpl_z": smpl_world5[3:, 2].tolist(),
        "ankle_g1_z": g1_world5[3:, 2].tolist(),
    }


def motion_heading_report(root_trans_seq: np.ndarray, root_quat_wxyz_seq: np.ndarray) -> dict[str, Any]:
    """Explain whether clip walks sideways in world vs robot-local forward."""
    root_trans_seq = np.asarray(root_trans_seq, dtype=np.float64)
    root_quat_wxyz_seq = np.asarray(root_quat_wxyz_seq, dtype=np.float64)
    n = len(root_trans_seq)
    if n < 2:
        return {}
    delta_world = root_trans_seq[-1] - root_trans_seq[0]
    yaw0 = float(R.from_quat(root_quat_wxyz_seq[0], scalar_first=True).as_euler("xyz", degrees=True)[2])
    delta_local = R.from_quat(root_quat_wxyz_seq[0], scalar_first=True).inv().apply(delta_world)
    axis_world = ("x", "y", "z")[int(np.argmax(np.abs(delta_world)))]
    axis_local = ("x", "y", "z")[int(np.argmax(np.abs(delta_local)))]
    return {
        "yaw_deg_frame0": yaw0,
        "displacement_world": delta_world.tolist(),
        "displacement_robot_local": delta_local.tolist(),
        "dominant_world_axis": axis_world,
        "dominant_robot_local_axis": axis_local,
        "walks_sideways_in_world": abs(yaw0) > 45.0 and abs(yaw0) < 135.0,
    }


def tracking_camera_azimuth(root_quat_wxyz: np.ndarray, *, offset_deg: float = 135.0) -> float:
    """Azimuth for MuJoCo tracking cam so robot-local +Y forward faces the camera."""
    yaw = float(R.from_quat(np.asarray(root_quat_wxyz, dtype=np.float64).reshape(4), scalar_first=True).as_euler("xyz", degrees=True)[2])
    return yaw + offset_deg


def calc_deploy_heading_rad(quat_wxyz: np.ndarray) -> float:
    """Match ``calc_heading_d`` in deploy (forward reference is world +X)."""
    quat = np.asarray(quat_wxyz, dtype=np.float64).reshape(4)
    rot_dir = R.from_quat(quat, scalar_first=True).apply([1.0, 0.0, 0.0])
    return float(np.arctan2(rot_dir[1], rot_dir[0]))


def calc_deploy_heading_quat(quat_wxyz: np.ndarray) -> np.ndarray:
    """Match ``calc_heading_quat_d`` (yaw-only component, wxyz)."""
    heading = calc_deploy_heading_rad(quat_wxyz)
    return R.from_euler("z", heading).as_quat(scalar_first=True)


def calc_deploy_heading_quat_inv(quat_wxyz: np.ndarray) -> np.ndarray:
    """Match ``calc_heading_quat_inv_d``."""
    heading = calc_deploy_heading_rad(quat_wxyz)
    return R.from_euler("z", -heading).as_quat(scalar_first=True)


def compute_apply_delta_heading(
    init_base_quat_wxyz: np.ndarray,
    init_ref_quat_wxyz: np.ndarray,
    *,
    delta_heading_rad: float = 0.0,
) -> R:
    """Same heading alignment deploy applies to ZMQ ``root_trans`` / ``body_quat_w``."""
    init_h = R.from_quat(calc_deploy_heading_quat(init_base_quat_wxyz), scalar_first=True)
    ref_inv = R.from_quat(calc_deploy_heading_quat_inv(init_ref_quat_wxyz), scalar_first=True)
    apply_delta = init_h * ref_inv
    if abs(delta_heading_rad) > 1e-12:
        apply_delta = R.from_euler("z", delta_heading_rad) * apply_delta
    return apply_delta


def apply_deploy_heading_to_pose_msg(
    msg: dict[str, np.ndarray],
    apply_delta: R,
) -> dict[str, np.ndarray]:
    """Transform raw ZMQ pose fields into deploy-corrected world targets for viz."""
    out = dict(msg)
    if "root_trans" in msg:
        root = np.asarray(msg["root_trans"], dtype=np.float64).reshape(3)
        out["root_trans"] = apply_delta.apply(root)
    if "body_quat_w" in msg:
        body = R.from_quat(np.asarray(msg["body_quat_w"], dtype=np.float64).reshape(4), scalar_first=True)
        out["body_quat_w"] = (apply_delta * body).as_quat(scalar_first=True).astype(np.float32)
    return out


def lock_deploy_heading_for_viz(
    sim_base_quat_wxyz: np.ndarray,
    ref_body_quat_wxyz: np.ndarray,
    *,
    delta_heading_rad: float = 0.0,
) -> R:
    """Deploy heading lock for marker overlay (same as deploy first ZMQ frame)."""
    return compute_apply_delta_heading(
        sim_base_quat_wxyz,
        ref_body_quat_wxyz,
        delta_heading_rad=delta_heading_rad,
    )


def lock_deploy_heading_once(
    heading_apply: R | None,
    sim_base_quat_wxyz: np.ndarray,
    ref_body_quat_wxyz: np.ndarray,
) -> R:
    """Return existing deploy heading lock, or create one (mirrors deploy first ZMQ frame)."""
    if heading_apply is not None:
        return heading_apply
    return compute_apply_delta_heading(sim_base_quat_wxyz, ref_body_quat_wxyz)


def deploy_corrected_snap_pose(
    sim_quat_on_band: np.ndarray,
    zmq_pose_frame0: dict[str, np.ndarray],
    fallback_root: np.ndarray,
    fallback_quat: np.ndarray,
    fallback_dof: np.ndarray,
    *,
    heading_apply: R | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Snap sim to deploy-corrected frame-0 (matches marker overlay frame)."""
    root = np.asarray(fallback_root, dtype=np.float64).reshape(3)
    quat = np.asarray(fallback_quat, dtype=np.float64).reshape(4)
    dof = np.asarray(fallback_dof, dtype=np.float64).reshape(-1)
    lock = lock_deploy_heading_once(
        heading_apply,
        sim_quat_on_band,
        zmq_pose_frame0["body_quat_w"],
    )
    corrected = apply_deploy_heading_to_pose_msg(
        {
            "root_trans": np.asarray(zmq_pose_frame0["root_trans"], dtype=np.float64).reshape(3),
            "body_quat_w": np.asarray(zmq_pose_frame0["body_quat_w"], dtype=np.float64).reshape(4),
        },
        lock,
    )
    return (
        np.asarray(corrected["root_trans"], dtype=np.float64).reshape(3),
        np.asarray(corrected["body_quat_w"], dtype=np.float64).reshape(4),
        dof,
    )


def marker_root_displacement_xy(
    root_trans_seq: np.ndarray,
    body_quat_seq: np.ndarray,
    sim_quat_at_lock: np.ndarray,
    *,
    heading_apply: R | None = None,
) -> np.ndarray:
    """XY root displacement after deploy heading correction (marker overlay frame)."""
    roots = np.asarray(root_trans_seq, dtype=np.float64)
    quats = np.asarray(body_quat_seq, dtype=np.float64)
    lock = lock_deploy_heading_once(heading_apply, sim_quat_at_lock, quats[0])
    out = []
    for i in range(len(roots)):
        msg = {"root_trans": roots[i], "body_quat_w": quats[i]}
        out.append(apply_deploy_heading_to_pose_msg(msg, lock)["root_trans"].reshape(3))
    out = np.stack(out, axis=0)
    return out[-1, :2] - out[0, :2]


def align_smpl_joints_pelvis_local_to_robot(
    joints: np.ndarray,
    *,
    yaw_deg: float = SMPL_LOCAL_YAW_TO_ROBOT_DEG,
) -> np.ndarray:
    """Rotate pelvis-local SMPL joint offsets into G1 root-local (+Y forward).

    Retargeted PKLs store ``smpl_joints`` with SMPL pelvis axes (+X lateral /
    forward) while ``body_quat_w`` is already the G1 root. Deploy heading only
    corrects ``root_trans`` / ``body_quat_w``; policy still reads ``smpl_joints``
    in pelvis-local frame, so publish must apply this yaw once.
    """
    joints = np.asarray(joints, dtype=np.float64)
    single = joints.ndim == 2
    if single:
        joints = joints[np.newaxis, ...]
    pelvis = joints[..., [SMPL_PELVIS_INDEX], :]
    offsets = joints - pelvis
    rot = R.from_euler("z", yaw_deg, degrees=True)
    rotated = pelvis + rot.apply(offsets.reshape(-1, 3)).reshape(offsets.shape)
    out = rotated.astype(np.float32)
    return out[0] if single else out


def apply_publish_stream_robot_frame_alignment(stream: dict) -> dict:
    """Align ZMQ stream fields for deploy publish (in-place copy returned)."""
    out = dict(stream)
    if "smpl_joints" in out:
        aligned = align_smpl_joints_pelvis_local_to_robot(out["smpl_joints"])
        out["smpl_joints"] = aligned
        out["smpl_joints_local"] = aligned
        out["smpl_joints_robot_aligned"] = True
    return out
