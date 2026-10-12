#!/usr/bin/env python3
"""Batch retarget SMPL motion data to A3 Ultra robot for SONIC training.

Pipeline per input file:
  1. Load smpl_filtered pkl (pose_aa N×72, transl N×3, fps in Y-up SMPL frame)
  2. Apply Y-up → Z-up coordinate rotation (R_x(+90°)) to trans and root_orient
  3. Save as SMPL-X npz (GMR input format)
  4. Run GMR retarget → robot qpos list
  5. Convert qpos to SONIC motion_lib format via convert_gmr_a3ultra_to_motion_lib.py

Usage (single-file test):
    conda run -n gmr python gear_sonic/data_process/batch_retarget_a3ultra.py \\
        --input sample_data/smpl_filtered/ \\
        --output data/motion_lib_a3ultra/robot_filtered/ \\
        --fps 30 --max_files 1

Usage (full batch from Bones-SEED extract):
    conda run -n gmr python gear_sonic/data_process/batch_retarget_a3ultra.py \\
        --input /path/to/bones_seed_smpl/ \\
        --output data/motion_lib_a3ultra/robot_filtered/ \\
        --fps 30

Environment requirements:
    conda activate gmr
    (gmr env has: mujoco, smplx, joblib, scipy, the GMR repo at /home/user/gmr_repo)
"""

import argparse
import os
import pathlib
import pickle
import sys
import tempfile
import traceback

import joblib
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, "/home/user/gmr_repo")

from general_motion_retargeting import GeneralMotionRetargeting as GMR
from general_motion_retargeting.utils.smpl import load_smplx_file, get_smplx_data_offline_fast

# Import the qpos → SONIC format converter from this directory
_THIS_DIR = pathlib.Path(__file__).parent
sys.path.insert(0, str(_THIS_DIR))
from convert_gmr_a3ultra_to_motion_lib import convert_sequence

SMPLX_FOLDER = pathlib.Path("/home/user/gmr_repo/assets/body_models")
ROBOT_NAME = "agibot_a3_ultra"

# Rotation to convert SMPL Y-up frame to Z-up robotics frame
_R_CONV = Rotation.from_euler("x", 90, degrees=True)


def smpl_pkl_to_smplx_npz(pkl_path: str, npz_path: str) -> float:
    """Convert a smpl_filtered pkl to a GMR-compatible SMPL-X npz.

    Returns the fps from the source file.
    """
    d = joblib.load(pkl_path)
    m = d if "pose_aa" in d else list(d.values())[0]

    pose_aa = m["pose_aa"].astype(np.float64)   # (N, 72)
    transl = m["transl"].astype(np.float64)      # (N, 3) Y-up
    fps = float(m.get("fps") or 30.0)

    root_orient_yup = pose_aa[:, 0:3]
    pose_body = pose_aa[:, 3:66].astype(np.float32)   # (N, 63) local joints unchanged

    # Y-up → Z-up: new_z = old_y (height), new_y = -old_z
    trans_zup = _R_CONV.apply(transl).astype(np.float32)
    root_orient_zup = (_R_CONV * Rotation.from_rotvec(root_orient_yup)).as_rotvec().astype(np.float32)

    np.savez(
        npz_path,
        root_orient=root_orient_zup,
        pose_body=pose_body,
        trans=trans_zup,
        betas=np.zeros((1, 16), dtype=np.float32),
        gender="neutral",
        mocap_frame_rate=fps,
    )
    return fps


def retarget_smplx_npz(npz_path: str, tgt_fps: int = 30) -> list:
    """Run GMR retarget on a SMPL-X npz. Returns list of qpos arrays."""
    smplx_data, body_model, smplx_output, actual_human_height = load_smplx_file(
        npz_path, SMPLX_FOLDER
    )
    smplx_frames, _ = get_smplx_data_offline_fast(
        smplx_data, body_model, smplx_output, tgt_fps=tgt_fps
    )
    retargeter = GMR(
        actual_human_height=actual_human_height,
        src_human="smplx",
        tgt_robot=ROBOT_NAME,
    )
    return [retargeter.retarget(frame) for frame in smplx_frames]


def process_one(pkl_path: str, out_pkl: str, tgt_fps: int, verbose: bool = True) -> bool:
    """Full pipeline: smpl_filtered pkl → SONIC motion_lib pkl. Returns True on success."""
    with tempfile.NamedTemporaryFile(suffix=".npz", delete=False) as tmp:
        tmp_npz = tmp.name
    try:
        src_fps = smpl_pkl_to_smplx_npz(pkl_path, tmp_npz)
        qpos_list = retarget_smplx_npz(tmp_npz, tgt_fps=tgt_fps)
        T = len(qpos_list)
        if T == 0:
            raise RuntimeError("GMR returned 0 frames")

        entry = convert_sequence(qpos_list, fps=float(tgt_fps))
        seq_name = pathlib.Path(pkl_path).stem

        os.makedirs(os.path.dirname(os.path.abspath(out_pkl)), exist_ok=True)
        joblib.dump({seq_name: entry}, out_pkl)

        if verbose:
            root_z = entry["root_trans_offset"][:, 2]
            print(f"  OK: {T} frames, root_z [{root_z.min():.3f}, {root_z.max():.3f}]"
                  f", pose_aa{entry['pose_aa'].shape}, dof{entry['dof'].shape}")
        return True
    except Exception:
        traceback.print_exc()
        return False
    finally:
        if os.path.exists(tmp_npz):
            os.unlink(tmp_npz)


def main():
    ap = argparse.ArgumentParser(description="Batch retarget SMPL → A3 Ultra SONIC training data")
    ap.add_argument("--input", required=True, help="Input dir (smpl_filtered pkls) or single pkl")
    ap.add_argument("--output", required=True, help="Output directory for SONIC motion_lib pkls")
    ap.add_argument("--fps", type=int, default=30, help="Target fps (default 30)")
    ap.add_argument("--max_files", type=int, default=0, help="Process at most N files (0=all)")
    ap.add_argument("--pattern", default="*.pkl", help="Glob pattern for input files")
    args = ap.parse_args()

    src = pathlib.Path(args.input)
    if src.is_file():
        input_files = [src]
    else:
        input_files = sorted(src.glob(args.pattern))

    if args.max_files > 0:
        input_files = input_files[: args.max_files]

    if not input_files:
        print(f"No files found in {args.input} matching '{args.pattern}'", file=sys.stderr)
        sys.exit(1)

    out_dir = pathlib.Path(args.output)
    print(f"Processing {len(input_files)} file(s) → {out_dir}")

    ok, fail = 0, 0
    for pkl_path in input_files:
        out_pkl = out_dir / (pkl_path.stem + ".pkl")
        print(f"[{ok+fail+1}/{len(input_files)}] {pkl_path.name}")
        if process_one(str(pkl_path), str(out_pkl), args.fps):
            ok += 1
        else:
            fail += 1
            print(f"  FAILED: {pkl_path.name}")

    print(f"\nDone: {ok} succeeded, {fail} failed")
    if fail:
        sys.exit(1)


if __name__ == "__main__":
    main()
