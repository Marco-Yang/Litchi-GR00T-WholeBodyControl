"""SMPL <-> robot clip pairing checker (frame counts at the 50 fps training rate)."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("joblib")

from gear_sonic.data_process.check_smpl_pairing import check_pair, robot_frames_at_target  # noqa: E402

REPO = Path(__file__).resolve().parents[1]


def _robot(t, fps=30.0):
    return {"root_trans_offset": np.zeros((t, 3), np.float32), "fps": int(fps)}


def _smpl(frames, orig=None, fps=50.0):
    out = {"pose_aa": np.zeros((frames, 72)), "smpl_joints": np.zeros((frames, 24, 3)), "fps": fps}
    if orig is not None:
        out["original_pose_aa"] = np.zeros((orig, 72))
    return out


def test_frame_arithmetic_matches_the_official_sample():
    assert robot_frames_at_target(1202, 30.0) == 2002  # walk_forward_amateur_001__A001: 1202 @ 30 fps -> 2002 @ 50 fps
    # T - 1 a multiple of 3: the library's arange excludes the end point (one third of all clips)
    assert robot_frames_at_target(1201, 30.0) == 2000
    assert robot_frames_at_target(161, 30.0) == 267  # body_stretch_4_002__A066_M: 161 @ 30 fps, 267 SMPL frames on disk


def test_frame_arithmetic_is_the_motion_library_interpolation():
    """Same count as ``TorchHumanoidBatch.interploate_pose`` (``torch.arange(0, (T - 1) / fps, 1 / target_fps)``)."""
    torch = pytest.importorskip("torch")
    for t in range(10, 400):
        assert robot_frames_at_target(t, 30.0) == len(torch.arange(0, (t - 1) * 1 / 30.0, 1 / 50.0, dtype=torch.float32)), t


def test_pairing_accepts_matching_and_reports_mismatches():
    assert check_pair("clip", _robot(1202), _smpl(2002, orig=1202)) == []
    bad = check_pair("clip", _robot(1202), _smpl(2001, orig=1201))
    assert len(bad) == 3 and all("clip" in b for b in bad)
    assert any("fps" in b for b in check_pair("clip", _robot(1202), _smpl(2002, fps=30.0)))


@pytest.mark.skipif(not (REPO / "sample_data/smpl_filtered/walk_forward_amateur_001__A001.pkl").exists(), reason="sample SMPL data missing")
def test_official_sample_clip_pairs_with_the_a3_clip():
    import joblib

    smpl = joblib.load(REPO / "sample_data/smpl_filtered/walk_forward_amateur_001__A001.pkl")
    g1 = next(iter(joblib.load(REPO / "sample_data/robot_filtered/210531/walk_forward_amateur_001__A001.pkl").values()))
    assert check_pair("walk_forward_amateur_001__A001", g1, smpl) == []  # the G1 library pairs; the A3 clips have the same length


def test_heading_offset_flags_a_library_that_faces_plus_x():
    from scipy.spatial.transform import Rotation

    from gear_sonic.data_process.check_smpl_pairing import heading_offset_deg

    base = Rotation.from_quat([0.5, 0.5, 0.5, 0.5], scalar_first=True)
    ytoz = Rotation.from_rotvec([np.pi / 2, 0, 0])
    capture = np.radians(-87.0)  # the performer's capture heading (Z-up world)
    pose = np.zeros((51, 72))
    pose[:, :3] = (ytoz.inv() * (Rotation.from_euler("z", capture) * base)).as_rotvec()
    smpl = {"pose_aa": pose}
    plain = {"root_rot": Rotation.from_euler("z", np.zeros(31)).as_quat(), "fps": 30}  # turned to +X, as the batch retarget solves
    kept = {"root_rot": Rotation.from_euler("z", np.full(31, capture)).as_quat(), "fps": 30}  # --keep_heading / restore_a3_heading.py
    assert heading_offset_deg(plain, smpl) == pytest.approx(87.0, abs=0.1)
    assert heading_offset_deg(kept, smpl) == pytest.approx(0.0, abs=0.1)
    assert heading_offset_deg({"fps": 30}, smpl) is None
