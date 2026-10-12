#!/usr/bin/env python3
"""Batch BVH → SONIC motion_lib retarget for AgiBot A3 Ultra.

Processes SOMA BVH files from soma_proportional.tar.gz (or a BVH directory)
and produces SONIC motion_lib pkl files.

Pipeline per file:
  BVH (NOVA/SOMA skeleton) → GMR nokov retarget → A3 Ultra qpos → SONIC pkl

SOMA BVH specifics: ROOT is a synthetic "Root" joint (6 channels), "Hips" is
a child with 6 channels, all other joints have 3 channels. The vendor
read_bvh cannot handle two 6-channel joints, so we use the custom parser from
extract_soma_joints_from_bvh.py and replicate GMR's world-space dict format.

Usage:
    # From tar.gz (streaming, no full extraction needed)
    conda run -n gmr python batch_retarget_bvh_soma_a3ultra.py \
        --tar data/bones_seed/soma_proportional.tar.gz \
        --output data/motion_lib_a3ultra/robot_filtered/ \
        --fps 30

    # From a directory of BVH files
    conda run -n gmr python batch_retarget_bvh_soma_a3ultra.py \
        --bvh_dir /path/to/bvh/ \
        --output data/motion_lib_a3ultra/robot_filtered/ \
        --fps 30
"""

import argparse
import os
import re
import sys
import tarfile
import tempfile
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
# SOMA BVH parser → GMR world-space frame dicts                               #
# --------------------------------------------------------------------------- #
# Y-up → Z-up rotation matrix (same as GMR's load_bvh_file)
_R_YUP_TO_ZUP = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64)
_Q_YUP_TO_ZUP = Rotation.from_matrix(_R_YUP_TO_ZUP).as_quat(scalar_first=True)  # wxyz


def _parse_soma_bvh(filepath):
    """Parse SOMA BVH; return (joints, channel_order, motion_data, n_frames, frame_time).

    Handles multiple 6-channel joints (Root + Hips both have XYZ pos + XYZ rot).
    Returns motion_data as (n_frames, n_channels) with raw degrees/cm values.
    """
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
    motion_data = np.empty((n_frames, n_channels), dtype=np.float64)
    for fi in range(n_frames):
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


def load_soma_bvh_frames(filepath):
    """Load SOMA BVH and return (frames_list, human_height) matching GMR format.

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
    if _hips_j_idx >= 0:
        _hq_xyzw = world_quat_xyzw[0, _hips_j_idx]
        _hq_wxyz = np.array([_hq_xyzw[3], _hq_xyzw[0], _hq_xyzw[1], _hq_xyzw[2]])
        _hq_zup = _quat_mul(_Q_YUP_TO_ZUP, _hq_wxyz)
        # body_Z (third column of R) = facing direction for SOMA Hips in Z-up.
        # BVH forward (+Z in Y-up) maps to -Y in Z-up after R_x(+90°).
        # Canonical forward = -Y_world (south), so yaw = angle FROM -Y TO body_Z_xy.
        # Solve: R_Z(yaw) @ [0,-1] = [body_Z[0], body_Z[1]]
        #   => yaw = atan2(body_Z[0], -body_Z[1])
        # Rotate all frames so Hips faces canonical south (-Y) at frame 0;
        # rot_offsets in bvh_nokov_to_a3_ultra.json were calibrated for this
        # canonical direction, and A3 Ultra at qpos0 also faces south (-Y).
        _body_z = Rotation.from_quat(_hq_zup, scalar_first=True).as_matrix()[:, 2]
        _yaw_f0 = float(np.arctan2(_body_z[0], -_body_z[1]))
        # +π flips the canonical SOMA orientation by 180° around Z so the IK
        # targets an East-facing robot (same as qpos0).  A post-processing step
        # in retarget_bvh() then flips the root to West-facing.  This is
        # cleaner than targeting West directly in the IK, because the mink
        # differential IK converges reliably for East-facing from the East
        # warm-start but stalls in a local minimum when the target is West.
        _yaw_f0 += np.pi
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
    return frames, human_height, fps_src


# --------------------------------------------------------------------------- #
# GMR retarget for one BVH file                                               #
# --------------------------------------------------------------------------- #

def make_retargeter():
    """Create a GMR instance for SOMA → A3 Ultra retargeting (reusable)."""
    from general_motion_retargeting import GeneralMotionRetargeting as GMR
    ret = GMR(
        src_human="bvh_nokov",
        tgt_robot="agibot_a3_ultra",
        # 1.90 rather than the SOMA performer's true height (~1.7-1.8 m).
        # GMR multiplies all human_scale_table entries by ratio=actual/assumption.
        # At 1.75 the ratio is 0.972 and the effective Hips scale = 0.875, which
        # places the pelvis IK target at 0.88 × SOMA_hip_Z ≈ 0.90 m — far below
        # A3 Ultra's natural standing pelvis height (1.30 m at qpos0), forcing
        # 63° of knee flexion.  At 1.90 (ratio=1.056, Hips scale=0.950) the pelvis
        # target rises to ≈0.97 m, giving ≈44° mean knee for normal walking
        # (range 32-61°, no frames stuck at the −7° joint lower limit).
        # NOTE: 1.95 would give ~23° mean but causes 78/120 frames to fall into
        # the −7° hyperextension local minimum via sequential warm-start drift.
        actual_human_height=1.90,
    )
    ret.max_iter = 50  # 30 was insufficient to rotate 180° to West-facing target
    return ret


def _warm_start_qpos(model) -> np.ndarray:
    """Return qpos0 with knees pre-bent to 20° to escape IK hyperextension basin.

    Starting from qpos0 (knees=0°) the mink IK falls into the lower joint-limit
    basin (-7°, hyperextension).  Any init ≥ 15° is far enough from that basin
    to converge to the correct solution (~23° mean for walking at actual_h=1.95).
    """
    import mujoco as mj
    q = model.qpos0.copy()
    for i in range(model.njnt):
        name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, i)
        if name in ("left_knee_joint", "right_knee_joint"):
            q[model.jnt_qposadr[i]] = np.radians(20.0)
    return q


def retarget_bvh(bvh_path: str, fps_target: int = 30,
                 retargeter=None):
    """Run GMR nokov retarget on a SOMA BVH file.

    Returns (qpos_list, retargeter). Pass the returned retargeter back on the
    next call to avoid reloading the robot model each time.
    """
    frames, _human_height, fps_src = load_soma_bvh_frames(bvh_path)

    if len(frames) == 0:
        raise ValueError("BVH produced 0 frames")

    if retargeter is None:
        retargeter = make_retargeter()

    # load_soma_bvh_frames applies +π yaw_comp, which flips the IK target from
    # East-facing to West-facing (R_Z(π) * East = West).  Use a West-facing
    # warm-start so the IK starts in the correct basin and converges in ~50 iters.
    _R180Z_wxyz = np.array([0.0, 0.0, 0.0, 1.0])  # wxyz: Rz(180°)
    q_warm = _warm_start_qpos(retargeter.configuration.model)
    q_warm[3:7] = _quat_mul(_R180Z_wxyz, q_warm[3:7])
    retargeter.configuration.update(q_warm)

    stride = max(1, round(fps_src / fps_target))
    qpos_list = []
    for i, frame_data in enumerate(frames):
        if i % stride != 0:
            continue
        qpos = retargeter.retarget(frame_data)
        qpos_list.append(qpos.copy())

    return qpos_list, retargeter


# --------------------------------------------------------------------------- #
# Process one BVH file → SONIC pkl                                            #
# --------------------------------------------------------------------------- #

def process_one_bvh(bvh_path: str, out_pkl: str, fps_target: int,
                    retargeter=None):
    """Retarget one BVH file and save as SONIC motion_lib pkl.

    Returns (success, retargeter) — retargeter is returned so callers can
    reuse the loaded model across files.
    """
    try:
        qpos_list, retargeter = retarget_bvh(bvh_path, fps_target, retargeter)
        if len(qpos_list) < 10:
            print(f"  SKIP {bvh_path}: too short ({len(qpos_list)} frames)")
            return False, retargeter

        motion_data = convert_sequence(qpos_list, fps=float(fps_target))
        motion_name = Path(bvh_path).stem
        os.makedirs(os.path.dirname(out_pkl), exist_ok=True)
        joblib.dump({motion_name: motion_data}, out_pkl)
        return True, retargeter
    except Exception as e:
        print(f"  FAILED {bvh_path}: {e}")
        return False, retargeter


# --------------------------------------------------------------------------- #
# Tar streaming                                                               #
# --------------------------------------------------------------------------- #

def iter_bvh_from_tar(tar_path: str):
    """Yield (motion_name, tmp_bvh_path) from tar.gz using pipe streaming.

    Uses r|gz (pipe) mode: reads sequentially without seeking. Each BVH is
    written to a temp file, yielded, then deleted.
    """
    with tarfile.open(tar_path, "r|gz") as tf:
        for member in tf:
            if not member.name.endswith(".bvh"):
                continue
            f = tf.extractfile(member)
            if f is None:
                continue
            content = f.read()
            motion_name = Path(member.name).stem
            # Write to temp file (GMR needs a file path, not bytes)
            tmp = tempfile.NamedTemporaryFile(
                suffix=".bvh", delete=False, mode="wb"
            )
            tmp.write(content)
            tmp.flush()
            tmp.close()
            yield motion_name, tmp.name, member.name


# --------------------------------------------------------------------------- #
# Main                                                                        #
# --------------------------------------------------------------------------- #

def main():
    parser = argparse.ArgumentParser(description="Batch BVH SOMA → A3 Ultra SONIC motion_lib")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--tar", help="Path to soma_proportional.tar.gz")
    src.add_argument("--bvh_dir", help="Directory containing BVH files (flat or nested)")
    parser.add_argument("--output", required=True, help="Output directory for pkl files")
    parser.add_argument("--fps", type=int, default=30, help="Target FPS (default 30)")
    parser.add_argument("--skip_existing", action="store_true", default=True,
                        help="Skip already-converted files (default on)")
    parser.add_argument("--no_skip", action="store_true", help="Disable skip_existing")
    parser.add_argument("--max_files", type=int, default=0, help="Max files (0=all)")
    parser.add_argument("--workers", type=int, default=1,
                        help="Parallel workers (default 1; GMR uses its own IK solver)")
    args = parser.parse_args()

    skip = args.skip_existing and not args.no_skip
    out_dir = Path(args.output)

    if args.tar:
        _run_from_tar(args.tar, out_dir, args.fps, skip, args.max_files, args.workers)
    else:
        _run_from_dir(args.bvh_dir, out_dir, args.fps, skip, args.max_files, args.workers)


def _run_from_tar(tar_path, out_dir, fps, skip, max_files, workers):
    print(f"Streaming BVH files from {tar_path}")
    print(f"Output: {out_dir}, FPS: {fps}, skip_existing: {skip}")

    n_ok = n_skip = n_fail = 0
    tmp_files = []
    retargeter = None  # created on first file, reused thereafter

    try:
        for i, (motion_name, tmp_bvh, archive_path) in enumerate(
            iter_bvh_from_tar(tar_path)
        ):
            tmp_files.append(tmp_bvh)

            # Preserve session subdirectory structure: soma_proportional/bvh/DATE/NAME.bvh
            parts = Path(archive_path).parts
            session = parts[-2] if len(parts) >= 3 else "session"
            out_pkl = out_dir / session / f"{motion_name}.pkl"

            if skip and out_pkl.exists():
                n_skip += 1
                os.unlink(tmp_bvh)
                tmp_files.pop()
                if (n_ok + n_skip + n_fail) % 100 == 0:
                    print(f"  [{n_ok + n_skip + n_fail}] ok={n_ok} skip={n_skip} fail={n_fail}")
                continue

            ok, retargeter = process_one_bvh(tmp_bvh, str(out_pkl), fps, retargeter)
            os.unlink(tmp_bvh)
            tmp_files.pop()

            if ok:
                n_ok += 1
            else:
                n_fail += 1

            if (n_ok + n_skip + n_fail) % 100 == 0 or n_ok + n_skip + n_fail == 1:
                print(f"  [{n_ok + n_skip + n_fail}] ok={n_ok} skip={n_skip} fail={n_fail}")

            if max_files > 0 and (n_ok + n_skip + n_fail) >= max_files:
                print(f"Reached max_files={max_files}, stopping.")
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Clean up any remaining temp files
        for f in tmp_files:
            if os.path.exists(f):
                os.unlink(f)

    print(f"\nDone: {n_ok} converted, {n_skip} skipped, {n_fail} failed → {out_dir}")


def _run_from_dir(bvh_dir, out_dir, fps, skip, max_files, workers):
    import glob
    bvh_files = sorted(glob.glob(os.path.join(bvh_dir, "**", "*.bvh"), recursive=True))
    print(f"Found {len(bvh_files)} BVH files in {bvh_dir}")
    print(f"Output: {out_dir}, FPS: {fps}")

    n_ok = n_skip = n_fail = 0
    retargeter = None
    for bvh_path in bvh_files:
        motion_name = Path(bvh_path).stem
        # Preserve one level of sub-directory
        parent = Path(bvh_path).parent.name
        out_pkl = out_dir / parent / f"{motion_name}.pkl"

        if skip and out_pkl.exists():
            n_skip += 1
            continue

        ok, retargeter = process_one_bvh(bvh_path, str(out_pkl), fps, retargeter)
        if ok:
            n_ok += 1
        else:
            n_fail += 1

        if (n_ok + n_skip + n_fail) % 100 == 0:
            print(f"  [{n_ok + n_skip + n_fail}] ok={n_ok} skip={n_skip} fail={n_fail}")

        if max_files > 0 and (n_ok + n_skip + n_fail) >= max_files:
            break

    print(f"\nDone: {n_ok} converted, {n_skip} skipped, {n_fail} failed → {out_dir}")


if __name__ == "__main__":
    main()
