#!/usr/bin/env python3
"""Calibrate rot_offsets for bvh_nokov_to_a3_ultra.json.

  - SOMA canonical: performer faces East (+X) after yaw normalization
    (Hips body_Z = performer's left → +Y)
  - Robot reference pose: qpos0 for trunk and legs (upright, straight legs, flat
    feet). qpos0 is NOT a valid arm reference — A3 Ultra elbow=0 is a 90° bend
    with the forearm pointing forward — so the arm joints are solved to match
    the performer's frame-0 upper-arm / forearm / thumb directions.
  - rot_offset = q_soma_canonical⁻¹ * q_robot_reference
  - The IK tables (body mapping + weights) are written from IK_TABLES; per-clip
    position offsets are computed at retarget time, so pos_offsets here are 0.

Usage:
    conda run -n gmr python gear_sonic/data_process/calibrate_soma_to_a3ultra.py \\
        --bvh data/bones_seed/soma_proportional/bvh/230530_walk/walk_test.bvh \\
        --json /home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json \\
        --out_json /tmp/bvh_nokov_to_a3_ultra_calibrated.json

    # Or extract from tar on the fly:
    conda run -n gmr python gear_sonic/data_process/calibrate_soma_to_a3ultra.py \\
        --tar data/bones_seed/soma_proportional.tar.gz \\
        --json /home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json \\
        --out_json /home/user/gmr_repo/general_motion_retargeting/ik_configs/bvh_nokov_to_a3_ultra.json
"""

import argparse
import json
import os
import re
import sys
import tarfile
import tempfile
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

MJCF_PATH = str(PROJECT_ROOT / "gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml")

# robot link → (SOMA body, (pos_w, rot_w) table1, (pos_w, rot_w) table2).
# Same structure as GMR's bvh_nokov_to_g1.json. SOMA naming: LeftLeg = thigh
# (hip joint), LeftShin = shank (knee joint), LeftFoot = ankle. Each SOMA body's
# position is the joint at the START of its segment, so it maps to the robot
# link whose origin is that joint (LeftShin → knee_Link, not ankle_pitch_Link).
IK_TABLES = {
    "pelvis_link":             ("Hips",         (100, 10), (100, 5)),
    "left_hip_yaw_Link":       ("LeftLeg",      (0, 10),   (10, 5)),
    "left_knee_Link":          ("LeftShin",     (0, 10),   (10, 5)),
    "left_ankle_roll_Link":    ("LeftFootMod",  (100, 10), (100, 10)),
    "right_hip_yaw_Link":      ("RightLeg",     (0, 10),   (10, 5)),
    "right_knee_Link":         ("RightShin",    (0, 10),   (10, 5)),
    "right_ankle_roll_Link":   ("RightFootMod", (100, 10), (100, 10)),
    "torso_Link":              ("Chest",        (0, 10),   (0, 10)),
    "left_shoulder_yaw_Link":  ("LeftArm",      (0, 10),   (10, 5)),
    "left_elbow_Link":         ("LeftForeArm",  (0, 10),   (10, 5)),
    "left_wrist_yaw_Link":     ("LeftHand",     (0, 10),   (10, 5)),
    "right_shoulder_yaw_Link": ("RightArm",     (0, 10),   (10, 5)),
    "right_elbow_Link":        ("RightForeArm", (0, 10),   (10, 5)),
    "right_wrist_yaw_Link":    ("RightHand",    (0, 10),   (10, 5)),
}
SOMA_TO_ROBOT = {soma: link for link, (soma, _, _) in IK_TABLES.items()}

# Arm chain used to solve the robot reference arm pose from SOMA frame-0.
ARM_JOINTS = ("shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint",
              "elbow_joint", "wrist_roll_joint")

# Y-up → Z-up rotation (same as GMR)
_R_YUP_TO_ZUP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
_Q_YUP_TO_ZUP = Rotation.from_matrix(_R_YUP_TO_ZUP).as_quat(scalar_first=True)


def _quat_mul(q1, q2):
    """Quaternion multiply (wxyz scalar-first)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def _quat_conj(q):
    """Conjugate (= inverse for unit quaternion) in wxyz."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


# ---------------------------------------------------------------------------
# Step 1: parse SOMA BVH and get frame-0 body orientations (no +π)
# ---------------------------------------------------------------------------

def _parse_soma_bvh(filepath):
    with open(filepath) as f:
        lines = f.readlines()
    joints = []
    joint_stack = []
    channel_index = 0
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line == "MOTION":
            i += 1
            break
        m = re.match(r"(ROOT|JOINT)\s+(\S+)", line)
        if m:
            parent_idx = joint_stack[-1] if joint_stack else -1
            joints.append({
                "name": m.group(2),
                "offset": None,
                "pos_ch_idx": [],
                "rot_ch_idx": [],
                "rot_order": "",
                "parent_idx": parent_idx,
            })
            joint_stack.append(len(joints) - 1)
        elif line.startswith("OFFSET") and joint_stack:
            joints[joint_stack[-1]]["offset"] = np.array([float(x) for x in line.split()[1:]])
        elif line.startswith("CHANNELS") and joint_stack:
            parts = line.split()
            n_ch = int(parts[1])
            j = joints[joint_stack[-1]]
            for ch in parts[2:2 + n_ch]:
                if ch.endswith("position"):
                    j["pos_ch_idx"].append(channel_index)
                else:
                    j["rot_ch_idx"].append(channel_index)
                    j["rot_order"] += ch[0].lower()
                channel_index += 1
        elif line == "}" and joint_stack:
            joint_stack.pop()
        i += 1

    n_frames = int(lines[i].strip().split(":")[1])
    i += 2  # skip frame time
    n_channels = channel_index
    motion_data = np.empty((n_frames, n_channels), dtype=np.float64)
    for fi in range(n_frames):
        motion_data[fi] = list(map(float, lines[i].strip().split()))
        i += 1
    return joints, motion_data, n_frames


def get_soma_frame0_orientations(bvh_path):
    """Return dict: joint_name → q_wxyz in Z-up frame, with yaw-normalization (no +π)."""
    joints, motion_data, _ = _parse_soma_bvh(bvh_path)
    n_joints = len(joints)
    joint_names = [j["name"] for j in joints]

    # FK for frame 0 only
    world_rot_mat = np.tile(np.eye(3), (n_joints, 1, 1))
    world_pos = np.zeros((n_joints, 3))

    for j_idx, joint in enumerate(joints):
        offset = joint["offset"] if joint["offset"] is not None else np.zeros(3)
        rot_idx = joint["rot_ch_idx"]
        pos_idx = joint["pos_ch_idx"]
        rot_order = joint["rot_order"]

        local_pos = motion_data[0, pos_idx] if pos_idx else offset
        if rot_order:
            local_rot = Rotation.from_euler(rot_order.upper(), motion_data[0, rot_idx], degrees=True).as_matrix()
        else:
            local_rot = np.eye(3)

        p = joint["parent_idx"]
        if p < 0:
            world_rot_mat[j_idx] = local_rot
            world_pos[j_idx] = local_pos
        else:
            world_pos[j_idx] = world_pos[p] + world_rot_mat[p] @ local_pos
            world_rot_mat[j_idx] = world_rot_mat[p] @ local_rot

    # Y-up → Z-up
    world_pos_zup = world_pos @ _R_YUP_TO_ZUP.T / 100.0  # cm → m

    # Yaw-normalize exactly like batch_retarget_bvh_soma_a3ultra.py: Hips body_Z
    # is the performer's LEFT, rotate it to +Y so the performer faces +X (East),
    # the same direction as A3 Ultra at qpos0.
    hips_j_idx = next((k for k, j in enumerate(joints) if j["name"] == "Hips"), -1)
    q_yaw_comp = np.array([1.0, 0.0, 0.0, 0.0])  # identity default
    if hips_j_idx >= 0:
        hq_xyzw = Rotation.from_matrix(world_rot_mat[hips_j_idx]).as_quat()
        hq_wxyz = np.array([hq_xyzw[3], hq_xyzw[0], hq_xyzw[1], hq_xyzw[2]])
        hq_zup = _quat_mul(_Q_YUP_TO_ZUP, hq_wxyz)
        body_z = Rotation.from_quat(hq_zup, scalar_first=True).as_matrix()[:, 2]
        yaw_f0 = float(np.arctan2(-body_z[0], body_z[1]))
        q_yaw_comp = Rotation.from_euler('Z', -yaw_f0).as_quat(scalar_first=True)

    world_pos_zup = Rotation.from_quat(q_yaw_comp, scalar_first=True).apply(world_pos_zup)

    result = {}
    for ji, jname in enumerate(joint_names):
        q_xyzw = Rotation.from_matrix(world_rot_mat[ji]).as_quat()
        q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
        q_wxyz_zup = _quat_mul(_Q_YUP_TO_ZUP, q_wxyz)
        q_wxyz_final = _quat_mul(q_yaw_comp, q_wxyz_zup)
        result[jname] = q_wxyz_final

    # Synthetic foot entries
    if "LeftFoot" in result:
        toe_q = result.get("LeftToeBase", result["LeftFoot"])
        result["LeftFootMod"] = toe_q
    if "RightFoot" in result:
        toe_q = result.get("RightToeBase", result["RightFoot"])
        result["RightFootMod"] = toe_q

    return result, world_pos_zup, joint_names


# ---------------------------------------------------------------------------
# Step 2: robot reference pose (qpos0 trunk/legs + arms solved to SOMA frame-0)
# ---------------------------------------------------------------------------

def _unit(v):
    return v / np.linalg.norm(v)


def _perp(v, axis):
    """Unit component of v perpendicular to axis."""
    a = _unit(axis)
    return _unit(v - (v @ a) * a)


def solve_robot_arm_reference(model, data, soma_pos, soma_names):
    """Set arm joints in data.qpos so robot segment directions match SOMA frame 0.

    Matches upper-arm and forearm directions, plus the thumb direction
    (perpendicular to the forearm) to fix forearm twist.
    """
    from scipy.optimize import least_squares

    P = lambda n: soma_pos[soma_names.index(n)]
    body = lambda n: mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, n)
    for side, S, f in (("left", "Left", "L"), ("right", "Right", "R")):
        adr = [model.jnt_qposadr[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"{side}_{j}")]
               for j in ARM_JOINTS]
        lo = [model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"{side}_{j}")][0] for j in ARM_JOINTS]
        hi = [model.jnt_range[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, f"{side}_{j}")][1] for j in ARM_JOINTS]
        h_up = _unit(P(f"{S}ForeArm") - P(f"{S}Arm"))
        h_fo = _unit(P(f"{S}Hand") - P(f"{S}ForeArm"))
        h_th = _perp(P(f"{S}HandThumb1") - P(f"{S}Hand"), h_fo)
        b_sh, b_el, b_wr, b_th = (body(f"{side}_shoulder_roll_Link"), body(f"{side}_elbow_Link"),
                                  body(f"{side}_wrist_yaw_Link"), body(f"{f}_thumb_roll_link"))

        def residual(x):
            data.qpos[adr] = x
            mj.mj_kinematics(model, data)
            r_up = _unit(data.xpos[b_el] - data.xpos[b_sh])
            r_fo = _unit(data.xpos[b_wr] - data.xpos[b_el])
            r_th = _perp(data.xpos[b_th] - data.xpos[b_wr], r_fo)
            return np.concatenate([2 * (r_up - h_up), 2 * (r_fo - h_fo), r_th - h_th, 0.01 * x])

        x0 = np.clip(np.array([0.0, 0.0, 0.0, np.pi / 2, 0.0]), np.array(lo) + 1e-3, np.array(hi) - 1e-3)
        sol = least_squares(residual, x0, bounds=(lo, hi))
        residual(sol.x)
        print(f"  {side:5s} arm ref (deg): " + ", ".join(
            f"{j.replace('_joint', '')}={np.degrees(v):+.1f}" for j, v in zip(ARM_JOINTS, sol.x))
            + f"  | dir err up={np.degrees(np.arccos(np.clip(_unit(data.xpos[b_el]-data.xpos[b_sh]) @ h_up, -1, 1))):.1f}°"
            + f" fore={np.degrees(np.arccos(np.clip(_unit(data.xpos[b_wr]-data.xpos[b_el]) @ h_fo, -1, 1))):.1f}°")


def get_robot_reference_orientations(soma_pos, soma_names):
    """Return dict: link_name → q_wxyz at the robot reference pose."""
    model = mj.MjModel.from_xml_path(MJCF_PATH)
    data = mj.MjData(model)
    mj.mj_resetData(model, data)
    solve_robot_arm_reference(model, data, soma_pos, soma_names)
    mj.mj_forward(model, data)
    result = {}
    for i in range(model.nbody):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i)
        result[name] = data.xquat[i].copy()  # wxyz
    return result


# ---------------------------------------------------------------------------
# Step 3: compute rot_offsets and write JSON
# ---------------------------------------------------------------------------

def compute_rot_offsets(soma_quats, robot_quats):
    """For each pair, rot_offset = q_soma⁻¹ * q_robot."""
    offsets = {}
    print("\nCalibrated rot_offsets:")
    for soma_body, robot_link in SOMA_TO_ROBOT.items():
        if soma_body not in soma_quats:
            print(f"  WARNING: SOMA body '{soma_body}' not found in BVH")
            continue
        if robot_link not in robot_quats:
            print(f"  WARNING: robot link '{robot_link}' not found in MJCF")
            continue
        q_soma = soma_quats[soma_body]
        q_robot = robot_quats[robot_link]
        # rot_offset = q_soma^{-1} * q_robot
        rot_offset = _quat_mul(_quat_conj(q_soma), q_robot)
        # Normalize
        rot_offset /= np.linalg.norm(rot_offset)
        offsets[soma_body] = rot_offset
        print(f"  {soma_body:15s} → {robot_link:30s}: {rot_offset.round(6)}")
    return offsets


def update_json(json_path, out_path, rot_offsets_by_soma):
    """Load existing JSON and rewrite both IK tables from IK_TABLES."""
    with open(json_path) as f:
        cfg = json.load(f)

    for t, key in ((1, "ik_match_table1"), (2, "ik_match_table2")):
        cfg[key] = {
            link: [soma, *(w1 if t == 1 else w2), [0.0, 0.0, 0.0], rot_offsets_by_soma[soma].tolist()]
            for link, (soma, w1, w2) in IK_TABLES.items()
        }
    cfg["human_scale_table"] = {soma: 1.0 for soma in SOMA_TO_ROBOT}

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(cfg, f, indent=4)
    print(f"\nWrote calibrated JSON → {out_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Calibrate SOMA → A3 Ultra rot_offsets")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bvh", help="Path to a SOMA BVH file (e.g. walk_test.bvh)")
    src.add_argument("--tar", help="Path to soma_proportional.tar.gz (uses first walk BVH)")
    ap.add_argument("--json", required=True,
                    help="Source JSON config (bvh_nokov_to_a3_ultra.json)")
    ap.add_argument("--out_json", required=True,
                    help="Output JSON path (can be same as --json to overwrite in place)")
    args = ap.parse_args()

    # Resolve BVH path
    if args.bvh:
        bvh_path = args.bvh
    else:
        print(f"Extracting first walk BVH from {args.tar}...")
        with tarfile.open(args.tar, "r:gz") as tf:
            for m in tf.getmembers():
                if "walk" in m.name.lower() and m.name.endswith(".bvh"):
                    f = tf.extractfile(m)
                    tmp = tempfile.NamedTemporaryFile(suffix=".bvh", delete=False)
                    tmp.write(f.read())
                    tmp.close()
                    bvh_path = tmp.name
                    print(f"  Using: {m.name}")
                    break
            else:
                raise FileNotFoundError("No walk BVH found in tar")

    print(f"\n=== Step 1: SOMA frame-0 orientations from {Path(bvh_path).name} ===")
    soma_quats, soma_pos, soma_names = get_soma_frame0_orientations(bvh_path)
    hips_q = soma_quats.get("Hips", np.array([1, 0, 0, 0]))
    hips_fwd = Rotation.from_quat(hips_q, scalar_first=True).as_matrix()[:, 2]
    print(f"  Hips body_Z (performer left): {hips_fwd.round(4)}  (expect ≈[0,+1,0])")
    for name in ["Hips", "LeftLeg", "RightLeg", "LeftArm", "RightArm", "LeftFootMod", "RightFootMod"]:
        if name in soma_quats:
            print(f"  {name:15s}: {soma_quats[name].round(4)}")

    print("\n=== Step 2: A3 Ultra reference pose (qpos0 + solved arms) ===")
    robot_quats = get_robot_reference_orientations(soma_pos, soma_names)
    for link in ["pelvis_link", "left_hip_yaw_Link", "right_hip_yaw_Link",
                 "left_ankle_roll_Link", "right_ankle_roll_Link",
                 "left_shoulder_yaw_Link", "right_shoulder_yaw_Link"]:
        if link in robot_quats:
            print(f"  {link:35s}: {robot_quats[link].round(4)}")

    print("\n=== Step 3: Computing rot_offsets ===")
    rot_offsets = compute_rot_offsets(soma_quats, robot_quats)

    print("\n=== Step 4: Writing calibrated JSON ===")
    update_json(args.json, args.out_json, rot_offsets)

    if args.tar and os.path.exists(bvh_path):
        os.unlink(bvh_path)

    print("\nDone.")


if __name__ == "__main__":
    main()
