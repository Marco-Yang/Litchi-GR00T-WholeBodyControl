#!/usr/bin/env python3
"""Batch BVH → SONIC motion_lib retarget for AgiBot A3 Ultra (default) or A3 (``--robot a3``).

Processes SOMA BVH files from soma_proportional.tar.gz (or a BVH directory)
and produces SONIC motion_lib pkl files.

Pipeline per file:
  BVH (NOVA/SOMA skeleton) → GMR nokov retarget → robot qpos → SONIC pkl

``--robot a3_ultra`` (default, what produced robot_filtered_v15): the 63-DOF T3D0 model, 31-column ``dof``, 64-body
``pose_aa``.  ``--robot a3``: the same method on the A3 of the 035 contract (``a3_t2d5.xml``, Litchi a3 parameters:
its URDF kinematics, joint limits, foot/ground geometry), written directly in the Litchi ``a3_29`` motion-lib format
(29-column ``dof``, 32-body ``pose_aa``) -- the dataset the A3 training path (``--robot a3`` / ``--robot a3_ultra``) reads.
GMR does not know the A3 yet; it is registered at run time below, no change to the GMR checkout is needed.

SOMA BVH specifics: ROOT is a synthetic "Root" joint (6 channels), "Hips" is
a child with 6 channels, all other joints have 3 channels. The vendor
read_bvh cannot handle two 6-channel joints, so we use the custom parser from
extract_soma_joints_from_bvh.py and replicate GMR's world-space dict format.

Usage:
    # From tar.gz (streaming, no full extraction needed)
    OMP_NUM_THREADS=1 conda run -n gmr python batch_retarget_bvh_soma_a3ultra.py \
        --tar data/bones_seed/soma_proportional.tar.gz \
        --output data/motion_lib_a3ultra/robot_filtered_v15/ \
        --fps 30 --workers 24

    # From a directory of BVH files
    OMP_NUM_THREADS=1 conda run -n gmr python batch_retarget_bvh_soma_a3ultra.py \
        --bvh_dir /path/to/bvh/ \
        --output data/motion_lib_a3ultra/robot_filtered_v15/ \
        --fps 30
"""

import argparse
import contextlib
import io
import os
import re
import sys
import tarfile
import tempfile
import time
from pathlib import Path

import joblib
import numpy as np
from scipy.spatial.transform import Rotation

# Ensure project root on path
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from gear_sonic.data_process.convert_gmr_a3ultra_to_motion_lib import convert_sequence

# --------------------------------------------------------------------------- #
# Target robots                                                               #
# --------------------------------------------------------------------------- #
IK_CONFIG_DIR = Path(__file__).resolve().parent / "gmr_ik_configs"
MJCF_DIR = PROJECT_ROOT / "gear_sonic" / "data" / "assets" / "robot_description" / "mjcf"
ROBOTS = {
    # GMR already ships agibot_a3_ultra (xml + bvh_nokov IK table live in the GMR checkout)
    "a3_ultra": {"gmr": "agibot_a3_ultra"},
    # the A3 (T2D5) of the 035 contract; the IK table is the Ultra's: both robots share the 29 body joints and every
    # IK link frame at qpos0 (tests/test_a3_retarget_batch.py compares them), so the calibrated offsets carry over.
    "a3": {"gmr": "agibot_a3", "mjcf": MJCF_DIR / "a3_t2d5.xml", "ik_config": IK_CONFIG_DIR / "bvh_nokov_to_a3.json"},
}


def _register_robot(robot: str) -> str:
    """Make GMR aware of ``robot`` (a no-op for robots it ships) and return its GMR name."""
    spec = ROBOTS[robot]
    if "mjcf" in spec:
        from general_motion_retargeting import params
        params.ROBOT_XML_DICT[spec["gmr"]] = spec["mjcf"]
        params.IK_CONFIG_DICT["bvh_nokov"][spec["gmr"]] = spec["ik_config"]
        params.ROBOT_BASE_DICT[spec["gmr"]] = "pelvis_link"
        params.VIEWER_CAM_DISTANCE_DICT[spec["gmr"]] = 2.5
    return spec["gmr"]


def convert_for(robot: str, qpos_list: list, fps: float) -> dict:
    """GMR qpos list -> motion-lib entry in the format of ``robot``."""
    if robot == "a3":
        from gear_sonic.data_process.convert_gmr_a3_to_motion_lib import convert_sequence as convert_a3
        return convert_a3(qpos_list, fps)
    return convert_sequence(qpos_list, fps)


# --------------------------------------------------------------------------- #
# SOMA BVH parser → GMR world-space frame dicts                               #
# --------------------------------------------------------------------------- #
# Y-up → Z-up rotation matrix (same as GMR's load_bvh_file)
_R_YUP_TO_ZUP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
_Q_YUP_TO_ZUP = Rotation.from_matrix(_R_YUP_TO_ZUP).as_quat(scalar_first=True)  # wxyz


def _parse_soma_bvh(filepath, max_frames=None, text=None):
    """Parse SOMA BVH; return (joints, motion_data, n_frames, frame_time).

    Handles multiple 6-channel joints (Root + Hips both have XYZ pos + XYZ rot).
    Returns motion_data as (n_frames, n_channels) with raw degrees/cm values; ``max_frames`` reads only the first rows
    (``n_frames`` is still the declared count), ``text`` parses an in-memory BVH instead of a file.
    """
    if text is not None:
        lines = text.splitlines()
    else:
        with open(filepath) as f:
            lines = f.readlines()

    joints = []   # list of {name, offset, pos_channels, rot_channels, rot_order, parent_idx}
    joint_stack = []
    channel_index = 0  # running index into per-frame data vector

    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line == "MOTION":
            i += 1
            break

        m = re.match(r"(ROOT|JOINT)\s+(\S+)", line)
        if m:
            name = m.group(2)
            parent_idx = joint_stack[-1] if joint_stack else -1
            joints.append({
                "name": name,
                "offset": None,
                "pos_ch_idx": [],   # channel indices for Xposition, Yposition, Zposition
                "rot_ch_idx": [],   # channel indices for rotations
                "rot_order": "",
                "parent_idx": parent_idx,
            })
            joint_stack.append(len(joints) - 1)
        elif line.startswith("OFFSET") and joint_stack:
            vals = [float(x) for x in line.split()[1:]]
            joints[joint_stack[-1]]["offset"] = np.array(vals)
        elif line.startswith("CHANNELS") and joint_stack:
            parts = line.split()
            n_ch = int(parts[1])
            ch_names = parts[2:2 + n_ch]
            j = joints[joint_stack[-1]]
            for ch in ch_names:
                if ch.endswith("position"):
                    j["pos_ch_idx"].append(channel_index)
                else:
                    j["rot_ch_idx"].append(channel_index)
                    j["rot_order"] += ch[0].lower()  # x/y/z
                channel_index += 1
        elif line == "}" and joint_stack:
            joint_stack.pop()
        i += 1

    # MOTION section
    n_frames = int(lines[i].strip().split(":")[1])
    i += 1
    frame_time = float(lines[i].strip().split(":")[1])
    i += 1

    n_channels = channel_index
    n_read = n_frames if max_frames is None else min(n_frames, max_frames)
    motion_data = np.empty((n_read, n_channels), dtype=np.float64)
    for fi in range(n_read):
        motion_data[fi] = list(map(float, lines[i].strip().split()))
        i += 1

    return joints, motion_data, n_frames, frame_time


def _quat_mul(q1, q2):
    """Quaternion multiply, both wxyz scalar-first, shape (4,)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _yaw_f0_from_hips_quat(hq_xyzw) -> float:
    """Heading of the performer at frame 0 (rad, Z-up world, +X = 0): the angle by which ``load_soma_bvh_frames`` turns
    the clip so the performer faces +X.  SOMA Hips body_Z is the performer's LEFT (not forward): verified on 40
    Bones-SEED clips via toe direction and LeftLeg-RightLeg."""
    hq_wxyz = np.array([hq_xyzw[3], hq_xyzw[0], hq_xyzw[1], hq_xyzw[2]])
    hq_zup = _quat_mul(_Q_YUP_TO_ZUP, hq_wxyz)
    body_z = Rotation.from_quat(hq_zup, scalar_first=True).as_matrix()[:, 2]
    return float(np.arctan2(-body_z[0], body_z[1]))


def bvh_heading_f0(filepath=None, text=None) -> float:
    """``_yaw_f0`` of a SOMA BVH without loading the clip: the original capture heading (rad) that the retarget removes.

    Reads the header and the first frame only (same forward kinematics as ``load_soma_bvh_frames``).
    """
    joints, motion_data, _, _ = _parse_soma_bvh(filepath, max_frames=1, text=text)
    hips = next((k for k, j in enumerate(joints) if j["name"] == "Hips"), -1)
    if hips < 0:
        return 0.0
    rot = [None] * len(joints)
    for k, joint in enumerate(joints):
        if joint["rot_order"]:
            local = Rotation.from_euler(joint["rot_order"].upper(), motion_data[0, joint["rot_ch_idx"]], degrees=True).as_matrix()
        else:
            local = np.eye(3)
        rot[k] = local if joint["parent_idx"] < 0 else rot[joint["parent_idx"]] @ local
    return _yaw_f0_from_hips_quat(Rotation.from_matrix(rot[hips]).as_quat())


def rotate_qpos_about_z(qpos, psi: float):
    """GMR qpos (root xyz, root quat wxyz, joints) -> the same pose with the whole scene turned by ``psi`` about +Z.

    Joint angles are unchanged: root position and orientation are rotated in the world.
    """
    out = np.array(qpos, dtype=np.float64, copy=True)
    c, s = np.cos(psi), np.sin(psi)
    out[0], out[1] = c * qpos[0] - s * qpos[1], s * qpos[0] + c * qpos[1]
    out[3:7] = _quat_mul(np.array([np.cos(psi / 2), 0.0, 0.0, np.sin(psi / 2)]), qpos[3:7])
    return out


def load_soma_bvh_frames(filepath, return_yaw=False):
    """Load SOMA BVH and return (frames_list, human_height, fps_src) matching GMR format
    (plus the removed capture heading ``_yaw_f0`` as a 4th value with ``return_yaw=True``).

    Each frame is a dict: joint_name → [position_zup_m, orientation_wxyz_zup].
    Adds synthetic LeftFootMod/RightFootMod (foot position + toe orientation).
    """
    joints, motion_data, n_frames, frame_time = _parse_soma_bvh(filepath)
    n_joints = len(joints)
    joint_names = [j["name"] for j in joints]

    # FK: compute world positions and rotations
    world_pos = np.zeros((n_frames, n_joints, 3))
    world_rot_mat = np.tile(np.eye(3), (n_frames, n_joints, 1, 1))

    for j_idx, joint in enumerate(joints):
        offset = joint["offset"] if joint["offset"] is not None else np.zeros(3)
        rot_idx = joint["rot_ch_idx"]
        pos_idx = joint["pos_ch_idx"]
        rot_order = joint["rot_order"]

        # Local position (all frames)
        if pos_idx:
            local_pos = motion_data[:, pos_idx]  # (T, 3)
        else:
            local_pos = np.tile(offset, (n_frames, 1))

        # Local rotation (all frames)
        if rot_order:
            rot_vals = motion_data[:, rot_idx]
            local_rot = Rotation.from_euler(
                rot_order.upper(), rot_vals, degrees=True
            ).as_matrix()  # (T, 3, 3)
        else:
            local_rot = np.tile(np.eye(3), (n_frames, 1, 1))

        p = joint["parent_idx"]
        if p < 0:
            world_rot_mat[:, j_idx] = local_rot
            world_pos[:, j_idx] = local_pos
        else:
            par_rot = world_rot_mat[:, p]
            par_pos = world_pos[:, p]
            world_pos[:, j_idx] = par_pos + np.einsum("fij,fj->fi", par_rot, local_pos)
            world_rot_mat[:, j_idx] = np.einsum("fij,fjk->fik", par_rot, local_rot)

    # Convert to quaternion (wxyz) and apply Y-up → Z-up for each frame
    # Apply rotation: world_pos_zup = R_conv @ world_pos  (broadcast)
    world_pos_zup = world_pos @ _R_YUP_TO_ZUP.T  # (T, N, 3) in meters
    world_pos_zup /= 100.0  # cm → m

    # Orientation: q_zup = Q_conv * q_world  (per joint per frame)
    world_rot_scipy = Rotation.from_matrix(world_rot_mat.reshape(-1, 3, 3))
    world_quat_xyzw = world_rot_scipy.as_quat()  # (T*N, 4) xyzw
    world_quat_xyzw = world_quat_xyzw.reshape(n_frames, n_joints, 4)

    # Yaw-normalize: rotate all bodies so the performer faces +X at frame 0.
    # Different BVH files start with the performer facing arbitrary directions;
    # the fixed IK offset was derived from a walk BVH where yaw≈0. Compensating
    # the frame-0 Hips yaw makes the offset universal across all motions.
    _hips_j_idx = next((k for k, j in enumerate(joints) if j["name"] == "Hips"), -1)
    _q_yaw_comp = None  # wxyz compensation quaternion (identity if Hips not found)
    _yaw_f0 = 0.0
    if _hips_j_idx >= 0:
        # Rotate so Hips body_Z (the performer's LEFT) -> +Y, i.e. the performer faces +X like A3 Ultra at qpos0.
        # Must match calibrate_soma_to_a3ultra.py, or the robot ends up 180°
        # from the performer (crossed legs/arms, hyperextended knees).
        _yaw_f0 = _yaw_f0_from_hips_quat(world_quat_xyzw[0, _hips_j_idx])
        _R_yaw_comp = Rotation.from_euler('Z', -_yaw_f0).as_matrix()
        _q_yaw_comp = Rotation.from_euler('Z', -_yaw_f0).as_quat(scalar_first=True)  # wxyz
        world_pos_zup = np.einsum('ij,tnj->tni', _R_yaw_comp, world_pos_zup)

    # Convert xyzw → wxyz and apply Y-up→Z-up quaternion + yaw compensation
    frames = []
    for fi in range(n_frames):
        result = {}
        for ji, jname in enumerate(joint_names):
            pos = world_pos_zup[fi, ji]
            q_xyzw = world_quat_xyzw[fi, ji]
            q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
            q_wxyz_zup = _quat_mul(_Q_YUP_TO_ZUP, q_wxyz)
            if _q_yaw_comp is not None:
                q_wxyz_zup = _quat_mul(_q_yaw_comp, q_wxyz_zup)
            result[jname] = [pos, q_wxyz_zup]

        # Synthetic foot entries (position from foot, orientation from toe or foot itself)
        if "LeftFoot" in result:
            toe_ori = result["LeftToeBase"][1] if "LeftToeBase" in result else result["LeftFoot"][1]
            result["LeftFootMod"] = [result["LeftFoot"][0], toe_ori]
        if "RightFoot" in result:
            toe_ori = result["RightToeBase"][1] if "RightToeBase" in result else result["RightFoot"][1]
            result["RightFootMod"] = [result["RightFoot"][0], toe_ori]

        frames.append(result)

    # Human height from Hips to head
    if "Head" in joint_names and "Hips" in joint_names:
        hi = joint_names.index("Hips")
        he = joint_names.index("Head")
        hips_h = np.mean(world_pos_zup[:, hi, 2])
        head_h = np.mean(world_pos_zup[:, he, 2])
        human_height = float(head_h - hips_h) + 0.2
        human_height = max(1.5, min(2.2, human_height))
    else:
        human_height = 1.75

    fps_src = round(1.0 / frame_time)
    if return_yaw:
        return frames, human_height, fps_src, _yaw_f0
    return frames, human_height, fps_src


# --------------------------------------------------------------------------- #
# GMR retarget for one BVH file                                               #
# --------------------------------------------------------------------------- #

POSTURE_COST = 1.0  # continuity prior (per rad) vs frame tasks (pos 100/m, rot 5-10/rad)


def make_retargeter(robot: str = "a3_ultra"):
    """Create a GMR instance for SOMA → A3 Ultra (or A3) retargeting (reusable).

    Body scale and position offsets are set per clip by _configure_clip() from
    the performer's own bone lengths, so no global human height is assumed.
    """
    import mink
    from general_motion_retargeting import GeneralMotionRetargeting as GMR
    ret = GMR(src_human="bvh_nokov", tgt_robot=_register_robot(robot))
    ret.max_iter = 50
    # Pull each frame toward the previous solution so the IK does not hop
    # between equivalent branches (e.g. shoulder gimbal at roll ≈ 90°).
    ret.posture_task = mink.PostureTask(ret.configuration.model, cost=POSTURE_COST)
    ret.tasks1.append(ret.posture_task)
    ret.tasks2.append(ret.posture_task)
    ret.robot_geom = _robot_geometry(ret.configuration.model)
    return ret


def _robot_geometry(model) -> dict:
    """Robot reference dimensions at qpos0, where every link frame is world-aligned."""
    import mujoco as mj
    d = mj.MjData(model)
    d.qpos[:] = model.qpos0
    mj.mj_forward(model, d)
    P = lambda n: d.xpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, n)].copy()
    foot = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "left_ankle_roll_Link")
    sole = min(_geom_lowest_z(model, d, g) for g in range(model.ngeom)
               if model.geom_bodyid[g] == foot and model.geom_contype[g])  # spheres: z - r; meshes: lowest vertex
    hip_l, hip_r = P("left_hip_pitch_Link"), P("right_hip_pitch_Link")
    knee, ankle = P("left_knee_Link"), P("left_ankle_roll_Link")
    return {
        "leg": np.linalg.norm(hip_l - knee) + np.linalg.norm(knee - ankle),
        "pelvis_to_hip": (hip_l + hip_r) / 2 - P("pelvis_link"),
        "hip_half_w": (hip_l - hip_r)[1] / 2,
        "hip_yaw_drop": hip_l[2] - P("left_hip_yaw_Link")[2],
        "shoulder_half_w": (P("left_shoulder_roll_Link") - P("right_shoulder_roll_Link"))[1] / 2,
        "shoulder_yaw_drop": P("left_shoulder_roll_Link")[2] - P("left_shoulder_yaw_Link")[2],
        "ankle_h": ankle[2] - sole,
    }


def _configure_clip(ret, frames) -> float:
    """Fit the IK targets to this performer's proportions.

    - one uniform scale s = robot leg length / performer leg length
    - pelvis offset so the robot hip joints land on the scaled human hips
      (the A3 pelvis frame sits 0.18 m above its hips, SOMA Hips only ~0.09 m)
    - outward offsets for leg / arm targets to absorb hip and shoulder width
    Offsets are in each target's robot-convention frame (GMR applies them there).
    Returns the root z shift that puts the robot soles on the floor.
    """
    g = ret.robot_geom
    sample = frames[:: max(1, len(frames) // 50)]
    P = lambda f, n: np.asarray(f[n][0])
    R_tgt = lambda f, n: (Rotation.from_quat(f[n][1], scalar_first=True) * ret.rot_offsets1[n]).as_matrix()

    leg_h = np.mean([np.linalg.norm(P(f, f"{s}Leg") - P(f, f"{s}Shin"))
                     + np.linalg.norm(P(f, f"{s}Shin") - P(f, f"{s}Foot"))
                     for f in sample for s in ("Left", "Right")])
    s = g["leg"] / leg_h
    for k in ret.human_scale_table:
        ret.human_scale_table[k] = s

    hip_c = np.mean([R_tgt(f, "Hips").T @ ((P(f, "LeftLeg") + P(f, "RightLeg")) / 2 - P(f, "Hips"))
                     for f in sample], axis=0)
    hip_hw = np.mean([(R_tgt(f, "Hips").T @ (P(f, "LeftLeg") - P(f, "RightLeg")))[1] / 2 for f in sample])
    sh_hw = np.mean([(R_tgt(f, "Chest").T @ (P(f, "LeftArm") - P(f, "RightArm")))[1] / 2 for f in sample])
    leg_lat = g["hip_half_w"] - s * hip_hw
    arm_lat = g["shoulder_half_w"] - s * sh_hw

    offsets = {"Hips": s * hip_c - g["pelvis_to_hip"]}
    for side, sign in (("Left", 1.0), ("Right", -1.0)):
        offsets[f"{side}Leg"] = [0.0, sign * leg_lat, -g["hip_yaw_drop"]]
        offsets[f"{side}Shin"] = [0.0, sign * leg_lat, 0.0]
        offsets[f"{side}FootMod"] = [0.0, sign * leg_lat, 0.0]
        offsets[f"{side}Arm"] = [0.0, sign * arm_lat, -g["shoulder_yaw_drop"]]
        offsets[f"{side}ForeArm"] = [0.0, sign * arm_lat, 0.0]
        offsets[f"{side}Hand"] = [0.0, sign * arm_lat, 0.0]
    for k, v in offsets.items():
        ret.pos_offsets1[k] = np.asarray(v, dtype=float)

    # BVH floor is z=0; planted-foot ankle height ≈ low percentile over the clip.
    ankle_z = np.percentile([min(P(f, "LeftFoot")[2], P(f, "RightFoot")[2]) for f in frames], 5)
    return g["ankle_h"] - s * ankle_z


FLOOR_TOL = 0.02  # m of floor penetration tolerated before the root is lifted
# Hands are left out: when the performer leans on the floor the robot's longer
# arms push the palm under it, and lifting the whole body for that floats the feet.
HAND_KW = ("wrist", "thumb", "index", "middle", "ring", "pinky")


def _geom_lowest_z(model, data, g) -> float:
    """Exact lowest world z of a sphere / capsule / cylinder / box / mesh geom."""
    import mujoco as mj
    z, Rm, s, t = data.geom_xpos[g][2], data.geom_xmat[g].reshape(3, 3), model.geom_size[g], model.geom_type[g]
    az = abs(Rm[2, 2])
    if t == mj.mjtGeom.mjGEOM_SPHERE:
        return z - s[0]
    if t == mj.mjtGeom.mjGEOM_CAPSULE:
        return z - az * s[1] - s[0]
    if t == mj.mjtGeom.mjGEOM_CYLINDER:
        return z - az * s[1] - s[0] * np.sqrt(max(0.0, 1 - az * az))
    if t == mj.mjtGeom.mjGEOM_BOX:
        return z - np.abs(Rm[2, :]) @ s
    if t == mj.mjtGeom.mjGEOM_MESH:
        i = model.geom_dataid[g]
        v = model.mesh_vert[model.mesh_vertadr[i]: model.mesh_vertadr[i] + model.mesh_vertnum[i]]
        return z + (v @ Rm[2, :]).min()
    return z


def _lift_out_of_floor(qpos_list, model, fps: float):
    """Raise the root on frames where a trunk / leg / head collision geom sinks into the floor.

    Kneeling, crawling and deep crouches put robot knees or toes below
    z=0 because the robot's link volumes differ from the performer's. The lift
    is a moving max followed by a moving mean (±0.2 s), so it is never smaller
    than what a frame needs and never steps abruptly.
    """
    import mujoco as mj
    from scipy.ndimage import maximum_filter1d, uniform_filter1d
    d = mj.MjData(model)
    body = lambda g: (mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, model.geom_bodyid[g]) or "").lower()
    geoms = [g for g in range(model.ngeom)
             if model.geom_bodyid[g] != 0 and (model.geom_contype[g] or model.geom_conaffinity[g])
             and not any(k in body(g) for k in HAND_KW)]
    need = np.zeros(len(qpos_list))
    for t, q in enumerate(qpos_list):
        d.qpos[:] = q
        mj.mj_kinematics(model, d)
        need[t] = max(0.0, -min(_geom_lowest_z(model, d, g) for g in geoms) - FLOOR_TOL)
    if not need.any():
        return
    w = 2 * max(1, round(0.2 * fps)) + 1
    lift = uniform_filter1d(maximum_filter1d(need, w, mode="nearest"), w, mode="nearest")
    for q, dz in zip(qpos_list, lift):
        q[2] += dz


def _warm_start_qpos(model) -> np.ndarray:
    """qpos0 with knees pre-bent 20° so the first frame avoids the -7° hyperextension basin."""
    import mujoco as mj
    q = model.qpos0.copy()
    for i in range(model.njnt):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, i)
        if name in ("left_knee_joint", "right_knee_joint"):
            q[model.jnt_qposadr[i]] = np.radians(20.0)
    return q


def retarget_bvh(bvh_path: str, fps_target: int = 30,
                 retargeter=None, robot: str = "a3_ultra", keep_heading: bool = False):
    """Run GMR nokov retarget on a SOMA BVH file.

    Returns (qpos_list, retargeter). Pass the returned retargeter back on the
    next call to avoid reloading the robot model each time.

    The IK runs on the clip turned to face +X at frame 0 (``load_soma_bvh_frames``).  ``keep_heading=True`` turns the
    result back by the removed capture heading, so the output sits in the BVH's own world frame -- the frame of the
    Bones-SEED SMPL data and of the G1 library (the SMPL encoder's ``smpl_root_ori_b`` needs robot and SMPL to share it).
    Joint angles are identical either way.
    """
    frames, _human_height, fps_src, yaw_f0 = load_soma_bvh_frames(bvh_path, return_yaw=True)

    if len(frames) == 0:
        raise ValueError("BVH produced 0 frames")

    if retargeter is None:
        retargeter = make_retargeter(robot)

    root_dz = _configure_clip(retargeter, frames)
    retargeter.configuration.update(_warm_start_qpos(retargeter.configuration.model))

    stride = max(1, round(fps_src / fps_target))
    qpos_list = []
    for i, frame_data in enumerate(frames):
        if i % stride != 0:
            continue
        retargeter.posture_task.set_target(retargeter.configuration.q)
        if not qpos_list:
            retargeter.retarget(frame_data)  # settle the first frame before anchoring to it
            retargeter.posture_task.set_target(retargeter.configuration.q)
        qpos = retargeter.retarget(frame_data).copy()
        qpos[2] += root_dz
        qpos_list.append(qpos)

    _lift_out_of_floor(qpos_list, retargeter.configuration.model, fps_src / stride)
    if keep_heading:
        qpos_list = [rotate_qpos_about_z(q, yaw_f0) for q in qpos_list]
    return qpos_list, retargeter


# --------------------------------------------------------------------------- #
# Process one BVH file → SONIC pkl                                            #
# --------------------------------------------------------------------------- #

def process_one_bvh(bvh_path: str, out_pkl: str, fps_target: int,
                    retargeter=None, motion_name: str = None, robot: str = "a3_ultra", keep_heading: bool = False):
    """Retarget one BVH file and save as SONIC motion_lib pkl.

    Returns (success, retargeter) — retargeter is returned so callers can
    reuse the loaded model across files.
    """
    try:
        qpos_list, retargeter = retarget_bvh(bvh_path, fps_target, retargeter, robot, keep_heading)
        if len(qpos_list) < 10:
            print(f"  SKIP {motion_name or bvh_path}: too short ({len(qpos_list)} frames)")
            return False, retargeter

        motion_data = convert_for(robot, qpos_list, float(fps_target))
        os.makedirs(os.path.dirname(out_pkl), exist_ok=True)
        tmp_pkl = out_pkl + ".tmp"  # atomic: an interrupted run must not leave a half-written pkl that "skip existing" trusts
        joblib.dump({motion_name or Path(bvh_path).stem: motion_data}, tmp_pkl)
        os.replace(tmp_pkl, out_pkl)
        return True, retargeter
    except Exception as e:
        print(f"  FAILED {motion_name or bvh_path}: {e}")
        return False, retargeter


# --------------------------------------------------------------------------- #
# Parallel batch                                                              #
# --------------------------------------------------------------------------- #

_WORKER_RETARGETER = None


def _worker_job(job):
    """Pool worker: (motion_name, bvh path or bytes, out_pkl, fps, robot, keep_heading) → success."""
    global _WORKER_RETARGETER
    motion_name, src, out_pkl, fps, robot, keep_heading = job
    if _WORKER_RETARGETER is None:
        with contextlib.redirect_stdout(io.StringIO()):  # GMR prints the whole model on load
            _WORKER_RETARGETER = make_retargeter(robot)
    tmp = None
    if isinstance(src, bytes):
        fd, tmp = tempfile.mkstemp(suffix=".bvh")
        with os.fdopen(fd, "wb") as fh:
            fh.write(src)
        src = tmp
    try:
        ok, _WORKER_RETARGETER = process_one_bvh(src, out_pkl, fps, _WORKER_RETARGETER, motion_name, robot, keep_heading)
    finally:
        if tmp:
            os.unlink(tmp)
    return ok


def _run_jobs(jobs, workers: int, counts: dict, out_dir):
    """Run jobs on a process pool, keeping at most 4×workers in flight so the
    45 GB tar is never buffered into memory."""
    from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait

    t0 = time.time()

    def tally(done):
        for fut in done:
            counts["ok" if fut.result() else "fail"] += 1
            n = counts["ok"] + counts["fail"]
            if n % 200 == 0:
                rate = n / (time.time() - t0)
                print(f"  [{n}] ok={counts['ok']} fail={counts['fail']} skip={counts['skip']} "
                      f"({rate:.1f} clips/s)", flush=True)

    with ProcessPoolExecutor(max_workers=workers) as ex:
        pending = set()
        for job in jobs:
            if len(pending) >= 4 * workers:
                done, pending = wait(pending, return_when=FIRST_COMPLETED)
                tally(done)
            pending.add(ex.submit(_worker_job, job))
        tally(wait(pending).done)
    print(f"\nDone: {counts['ok']} converted, {counts['skip']} skipped, {counts['fail']} failed "
          f"in {(time.time() - t0) / 60:.1f} min → {out_dir}")


def _tar_jobs(tar_path, out_dir, fps, skip, max_files, counts, robot="a3_ultra", only=None, keep_heading=False):
    """Yield jobs from tar.gz via pipe streaming (sequential read, no seeking).

    Output keeps the session directory: soma_proportional/bvh/<session>/<name>.bvh
    → <out_dir>/<session>/<name>.pkl
    """
    n_jobs = 0
    with tarfile.open(tar_path, "r|gz") as tf:
        for member in tf:
            if not member.name.endswith(".bvh"):
                continue
            parts = Path(member.name).parts
            session = parts[-2] if len(parts) >= 3 else "session"
            motion_name = Path(member.name).stem
            if only is not None and motion_name not in only:
                continue
            out_pkl = out_dir / session / f"{motion_name}.pkl"
            if skip and out_pkl.exists():
                counts["skip"] += 1
                continue
            f = tf.extractfile(member)
            if f is None:
                continue
            yield motion_name, f.read(), str(out_pkl), fps, robot, keep_heading
            n_jobs += 1
            if max_files > 0 and n_jobs >= max_files:
                print(f"Reached max_files={max_files}, stopping.")
                return


def _dir_jobs(bvh_dir, out_dir, fps, skip, max_files, counts, robot="a3_ultra", only=None, keep_heading=False):
    """Yield jobs for BVH files under a directory (one sub-directory level kept)."""
    import glob
    bvh_files = sorted(glob.glob(os.path.join(bvh_dir, "**", "*.bvh"), recursive=True))
    print(f"Found {len(bvh_files)} BVH files in {bvh_dir}")
    n_jobs = 0
    for bvh_path in bvh_files:
        motion_name = Path(bvh_path).stem
        if only is not None and motion_name not in only:
            continue
        out_pkl = out_dir / Path(bvh_path).parent.name / f"{motion_name}.pkl"
        if skip and out_pkl.exists():
            counts["skip"] += 1
            continue
        yield motion_name, bvh_path, str(out_pkl), fps, robot, keep_heading
        n_jobs += 1
        if max_files > 0 and n_jobs >= max_files:
            return


# --------------------------------------------------------------------------- #
# Main                                                                        #
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(description="Batch BVH SOMA → A3 Ultra / A3 SONIC motion_lib")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--tar", help="Path to soma_proportional.tar.gz")
    src.add_argument("--bvh_dir", help="Directory containing BVH files (flat or nested)")
    parser.add_argument("--output", required=True, help="Output directory for pkl files")
    parser.add_argument("--robot", choices=sorted(ROBOTS), default="a3_ultra",
                        help="a3_ultra: 63-DOF T3D0 model (robot_filtered_v15 format); a3: the 035 contract's A3 (T2D5), "
                             "written as a3_29 (29 dof, 32 bodies)")
    parser.add_argument("--only_from", default=None,
                        help="convert only the clips whose name (file stem) exists under this directory, e.g. the "
                             "filtered robot_filtered_v15_clean library, so both robots cover the same clips")
    parser.add_argument("--keep_heading", action="store_true",
                        help="turn each clip back to its original capture heading after the IK (the frame of the SMPL "
                             "data and of the G1 library); default: every clip faces +X at frame 0")
    parser.add_argument("--fps", type=int, default=30, help="Target FPS (default 30)")
    parser.add_argument("--no_skip", action="store_true",
                        help="Re-convert clips whose output pkl already exists")
    parser.add_argument("--max_files", type=int, default=0, help="Max clips to convert (0=all)")
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 4),
                        help="Parallel worker processes (default: CPU count - 4)")
    args = parser.parse_args()

    skip = not args.no_skip
    out_dir = Path(args.output)
    only = None
    if args.only_from:
        only = {p.stem for p in Path(args.only_from).rglob("*.pkl") if p.name != "metadata.pkl"}
        print(f"--only_from {args.only_from}: {len(only)} clip names", flush=True)
    counts = {"ok": 0, "fail": 0, "skip": 0}
    print(f"Source: {args.tar or args.bvh_dir}\nOutput: {out_dir}  robot={args.robot}  fps={args.fps}  "
          f"skip_existing={skip}  workers={args.workers}  keep_heading={args.keep_heading}", flush=True)
    if args.tar:
        jobs = _tar_jobs(args.tar, out_dir, args.fps, skip, args.max_files, counts, args.robot, only, args.keep_heading)
    else:
        jobs = _dir_jobs(args.bvh_dir, out_dir, args.fps, skip, args.max_files, counts, args.robot, only, args.keep_heading)
    _run_jobs(jobs, args.workers, counts, out_dir)


if __name__ == "__main__":
    main()
