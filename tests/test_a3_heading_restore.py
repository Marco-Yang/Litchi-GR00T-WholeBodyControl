"""Heading of the A3 retarget library: the IK runs on clips turned to face +X, the SMPL data keeps the capture heading.

``restore_a3_heading.py`` (existing libraries) and ``--keep_heading`` (new runs) turn the result back by the removed capture
heading ``_yaw_f0``.  These tests pin the sign and the invariants: joint angles and heights untouched, the root rotated about
Z only, the three ways of reading ``_yaw_f0`` identical.  See plan section 17.8.
"""

from __future__ import annotations

from pathlib import Path
import tarfile

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import gear_sonic.data_process.batch_retarget_bvh_soma_a3ultra as batch
from gear_sonic.data_process import restore_a3_heading as restore

REPO = Path(__file__).resolve().parents[1]
BVH_TAR = REPO / "data/bones_seed/soma_proportional.tar.gz"


def _entry(t=12, seed=0):
    rng = np.random.default_rng(seed)
    rot = Rotation.from_euler("zyx", rng.uniform(-0.3, 0.3, (t, 3)))
    pose = rng.normal(size=(t, 32, 3)).astype(np.float32)
    pose[:, 0] = rot.as_rotvec()
    return {
        "root_trans_offset": rng.normal(size=(t, 3)).astype(np.float32),
        "pose_aa": pose,
        "dof": rng.normal(size=(t, 29)).astype(np.float32),
        "root_rot": rot.as_quat().astype(np.float32),
        "smpl_joints": np.zeros((t, 24, 3), np.float32),
        "fps": 30,
    }


def test_rotate_entry_turns_the_scene_about_z_only():
    e, psi = _entry(), np.radians(-87.8)
    r = restore.rotate_entry(e, psi)
    assert np.array_equal(r["dof"], e["dof"]) and r["fps"] == 30
    assert np.array_equal(r["pose_aa"][:, 1:], e["pose_aa"][:, 1:])  # only the root slot changes
    assert np.allclose(r["root_trans_offset"][:, 2], e["root_trans_offset"][:, 2])
    assert np.allclose(np.linalg.norm(r["root_trans_offset"][:, :2], axis=1), np.linalg.norm(e["root_trans_offset"][:, :2], axis=1), atol=1e-5)
    turn = Rotation.from_quat(r["root_rot"]) * Rotation.from_quat(e["root_rot"]).inv()  # world turn = new * old^-1
    assert np.allclose(turn.as_rotvec(), [[0, 0, psi]] * len(e["dof"]), atol=1e-5)
    # pose_aa[:, 0] is the same rotation as root_rot, and every dtype survives
    assert np.allclose(Rotation.from_rotvec(r["pose_aa"][:, 0]).as_matrix(), Rotation.from_quat(r["root_rot"]).as_matrix(), atol=1e-5)
    assert all(r[k].dtype == e[k].dtype for k in ("root_trans_offset", "root_rot", "pose_aa", "dof"))
    back = restore.rotate_entry(r, -psi)
    assert np.allclose(back["root_trans_offset"], e["root_trans_offset"], atol=1e-5)
    assert np.allclose(Rotation.from_quat(back["root_rot"]).as_matrix(), Rotation.from_quat(e["root_rot"]).as_matrix(), atol=1e-5)


def test_rotate_qpos_about_z_is_the_same_turn_as_rotate_entry():
    e, psi = _entry(), 1.1
    q = np.zeros((len(e["dof"]), 38))
    q[:, :3] = e["root_trans_offset"]
    q[:, 3:7] = e["root_rot"][:, [3, 0, 1, 2]]  # wxyz, as GMR
    q[:, 7:] = np.random.default_rng(1).normal(size=(len(q), 31))
    turned = np.array([batch.rotate_qpos_about_z(row, psi) for row in q])
    r = restore.rotate_entry(e, psi)
    assert np.allclose(turned[:, :3], r["root_trans_offset"], atol=1e-5)
    assert np.allclose(Rotation.from_quat(turned[:, [4, 5, 6, 3]]).as_matrix(), Rotation.from_quat(r["root_rot"]).as_matrix(), atol=1e-5)
    assert np.array_equal(turned[:, 7:], q[:, 7:])


def _bvh(angle_deg: float) -> str:
    """Root (6 ch) + Hips (6 ch) + one child; the Hips rotated about the Y (up) axis of the BVH by ``angle_deg``."""
    return f"""HIERARCHY
ROOT Root
{{
  OFFSET 0 0 0
  CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
  JOINT Hips
  {{
    OFFSET 0 0 0
    CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation
    JOINT Spine
    {{
      OFFSET 0 10 0
      CHANNELS 3 Zrotation Xrotation Yrotation
      End Site
      {{
        OFFSET 0 10 0
      }}
    }}
  }}
}}
MOTION
Frames: 3
Frame Time: 0.0083333
0 0 0 0 0 0 0 90 0 0 0 {angle_deg} 0 0 0
0 0 0 0 0 0 0 90 0 0 0 {angle_deg} 0 0 0
0 0 0 0 0 0 0 90 0 0 0 {angle_deg} 0 0 0
"""


def test_capture_heading_follows_the_hips_and_all_readers_agree(tmp_path):
    wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi  # noqa: E731
    base = batch.bvh_heading_f0(text=_bvh(0.0))
    for a in (-170.0, -87.8, 0.0, 30.0, 135.0):
        text = _bvh(a)
        h = batch.bvh_heading_f0(text=text)
        assert wrap(h - base) == pytest.approx(wrap(np.radians(a)), abs=1e-9), a  # a turn of the hips about the BVH up axis is the same turn of the heading
        path = tmp_path / "clip.bvh"
        path.write_text(text)
        assert batch.bvh_heading_f0(str(path)) == pytest.approx(h, abs=1e-12)
        assert batch.bvh_heading_f0(text=restore._head_of_bvh(text.encode())) == pytest.approx(h, abs=1e-12)  # header + first frame is enough
        frames, _, fps, yaw = batch.load_soma_bvh_frames(str(path), return_yaw=True)
        assert yaw == pytest.approx(h, abs=1e-9) and fps == 120 and len(frames) == 3  # the loader removes exactly this angle
        assert len(batch.load_soma_bvh_frames(str(path))) == 3  # default signature unchanged


def test_loader_faces_the_performer_to_plus_x_whatever_the_capture_heading(tmp_path):
    """After ``load_soma_bvh_frames`` every clip has the same Hips heading: that is what ``keep_heading`` undoes."""
    out = []
    for a in (-120.0, 0.0, 77.0):
        path = tmp_path / f"c{int(a)}.bvh"
        path.write_text(_bvh(a))
        frames, _, _ = batch.load_soma_bvh_frames(str(path))
        q = frames[0]["Hips"][1]  # wxyz, Z-up, after the yaw compensation
        left = Rotation.from_quat(q, scalar_first=True).as_matrix()[:, 2]
        out.append(np.degrees(np.arctan2(left[1], left[0])))
    assert np.allclose(out, out[0], atol=1e-6)


@pytest.mark.skipif(not BVH_TAR.exists(), reason="Bones-SEED SOMA BVH archive not on this machine")
def test_keep_heading_only_turns_the_result(tmp_path):
    pytest.importorskip("mink")
    pytest.importorskip("general_motion_retargeting")
    with tarfile.open(BVH_TAR, "r|gz") as tf:
        for member in tf:
            if member.name.endswith(".bvh"):
                bvh = tmp_path / Path(member.name).name
                bvh.write_bytes(tf.extractfile(member).read())
                break
    yaw = batch.bvh_heading_f0(str(bvh))
    plain, ret = batch.retarget_bvh(str(bvh), 30, None, "a3")
    kept, _ = batch.retarget_bvh(str(bvh), 30, ret, "a3", keep_heading=True)
    assert len(plain) == len(kept)
    plain, kept = np.array(plain), np.array(kept)
    assert np.allclose(plain[:, 7:], kept[:, 7:], atol=1e-6) and np.allclose(plain[:, 2], kept[:, 2], atol=1e-6)
    turn = Rotation.from_quat(kept[:, [4, 5, 6, 3]]) * Rotation.from_quat(plain[:, [4, 5, 6, 3]]).inv()
    assert np.allclose(turn.as_rotvec()[:, 2], yaw, atol=1e-5) and np.allclose(turn.as_rotvec()[:, :2], 0, atol=1e-5)
