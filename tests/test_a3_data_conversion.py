"""A3 Ultra motion library -> Litchi ``a3_29`` format (``gear_sonic/data_process/convert_a3ultra_to_a3_29d.py``).

The conversion is only legitimate because the 29 body joints of the two robots are the same kinematic chain; these tests
keep that assumption, the A3 joint limits, the re-grounding and the output format pinned (needs mujoco, skipped without).
"""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import sys

import joblib
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

mj = pytest.importorskip("mujoco")

REPO = Path(__file__).resolve().parents[1]


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


conv_mod = _load(REPO / "gear_sonic/data_process/convert_a3ultra_to_a3_29d.py", "convert_a3ultra_to_a3_29d")
litchi = _load(REPO / "gear_sonic/data_process/convert_a3_pkl_to_motion_lib_29d.py", "convert_a3_pkl_to_motion_lib_29d")


@pytest.fixture(scope="module")
def conv():
    return conv_mod.build_conversion()


def _ultra_clip(conv, n=12, seed=0, knee_deg=None):
    """A synthetic clip in the layout of data/motion_lib_a3ultra/robot_filtered_v15_clean (random in-range poses)."""
    rng = np.random.default_rng(seed)
    a3u = conv_mod.load_spec(conv_mod.DEFAULT_A3U_MJCF)
    names = [a3u.joint_names[b - 1] for b in conv.src_body]
    dof = np.zeros((n, 31))
    for c, name in enumerate(names):
        if name.startswith(("head", "ultra_head")):
            continue  # the Ultra library has zeros here
        lim = conv.a3.limits[conv.src_dof_to_a3_joint[c]]
        lo, hi = max(lim[0], -1.0), min(lim[1], 1.0)
        dof[:, c] = rng.uniform(lo, hi, n)
    if knee_deg is not None:
        for c, name in enumerate(names):
            if "knee" in name:
                dof[:, c] = np.radians(knee_deg)
    pose = np.zeros((n, 64, 3))
    root_rot = Rotation.from_euler("xyz", rng.uniform(-0.2, 0.2, (n, 3))).as_quat()
    pose[:, 0] = Rotation.from_quat(root_rot).as_rotvec()
    for c in range(31):
        pose[:, conv.src_body[c]] = dof[:, c, None] * conv.src_axes[c][None, :]
    trans = np.stack([np.linspace(0, 1, n), np.zeros(n), np.full(n, 1.07)], axis=1)
    return {
        "root_trans_offset": trans.astype(np.float32), "pose_aa": pose.astype(np.float32), "dof": dof.astype(np.float32),
        "root_rot": root_rot.astype(np.float32), "smpl_joints": np.zeros((n, 24, 3), np.float32), "fps": 30.0,
    }


def test_derived_spec_matches_litchis_a3_spec(conv):
    """The MuJoCo-derived A3 spec is Litchi's (ElementTree, body DFS order) spec: bodies, joints, axes, head indices."""
    spec = litchi.load_a3_mjcf_spec(conv_mod.DEFAULT_A3_MJCF)
    assert conv.a3.body_names == spec.body_names
    assert conv.a3.joint_names == spec.joint_names
    assert np.allclose(conv.a3.axes, spec.dof_axis)
    assert list(conv.a3_body_joint) == spec.dof_output_indices
    assert list(conv.a3_head_joint) == spec.head_dof_indices
    assert len(conv.a3_body_joint) == 29 and len(conv.a3.body_names) == 32


def test_shared_joints_have_one_to_one_columns(conv):
    """Source column order = Ultra depth-first actuated joints; every A3 body joint is fed by the column of the same name."""
    a3u = conv_mod.load_spec(conv_mod.DEFAULT_A3U_MJCF)
    names = [a3u.joint_names[b - 1] for b in conv.src_body]
    assert len(names) == 31 and names[3:5] == ["head_yaw_joint", "ultra_head_pitch_joint"]
    for k, j in enumerate(conv.a3_body_joint):
        assert names[conv.keep_src_cols[k]] == conv.a3.joint_names[j]


def test_root_is_lowered_by_the_sole_height_difference(conv):
    """A3 foot = mesh, Ultra foot = 13 spheres: the A3 ankle is 4.54 mm closer to the sole."""
    assert conv.root_z_shift == pytest.approx(-0.00454, abs=2e-4)
    assert conv_mod.sole_height(conv_mod.DEFAULT_A3_MJCF) < conv_mod.sole_height(conv_mod.DEFAULT_A3U_MJCF)


def test_converted_motion_has_the_litchi_a3_29_layout(conv):
    entry = _ultra_clip(conv)
    out, stats = conv_mod.convert_entry("clip", entry, conv)
    assert set(out) == {"root_trans_offset", "pose_aa", "dof", "root_rot", "smpl_joints", "fps"}
    assert out["pose_aa"].shape == (12, 32, 3) and out["dof"].shape == (12, 29)
    assert all(out[k].dtype == np.float32 for k in ("root_trans_offset", "pose_aa", "dof", "root_rot", "smpl_joints"))
    assert out["fps"] == 30 and isinstance(out["fps"], int)
    assert np.all(out["pose_aa"][:, 4:6] == 0), "head bodies are not policy joints"
    assert conv_mod.check_a3_29d_entry("clip", out, conv, 12) == []
    assert stats["clipped"].sum() == 0
    # xy of the root is kept, z follows the new sole height
    assert np.allclose(out["root_trans_offset"][:, :2], entry["root_trans_offset"][:, :2])
    assert np.allclose(out["root_trans_offset"][:, 2], entry["root_trans_offset"][:, 2] + conv.root_z_shift, atol=1e-6)


def test_litchis_own_converter_accepts_the_output_unchanged(conv):
    """Feeding the result through Litchi's convert_a3_pkl_to_motion_lib_29d normaliser changes nothing."""
    out, _ = conv_mod.convert_entry("clip", _ultra_clip(conv, seed=3), conv)
    spec = litchi.load_a3_mjcf_spec(conv_mod.DEFAULT_A3_MJCF)
    again, _, notes = litchi.normalize_entry("clip", out, spec, target_fps=30)
    assert notes == []
    for key in ("root_trans_offset", "pose_aa", "dof"):
        assert np.allclose(again[key], out[key], atol=1e-6), key
    q_a, q_b = again["root_rot"], out["root_rot"]
    assert np.allclose(np.minimum(np.abs(q_a - q_b).max(1), np.abs(q_a + q_b).max(1)), 0, atol=1e-5)  # same rotation


def test_forward_kinematics_of_the_shared_bodies_is_preserved(conv):
    """After conversion the A3 model, driven with the converted data, is where the Ultra model was (re-grounding aside)."""
    entry = _ultra_clip(conv, n=20, seed=5)
    out, _ = conv_mod.convert_entry("clip", entry, conv)
    A = mj.MjModel.from_xml_path(str(conv_mod.DEFAULT_A3_MJCF))
    U = mj.MjModel.from_xml_path(str(conv_mod.DEFAULT_A3U_MJCF))
    dA, dU = mj.MjData(A), mj.MjData(U)
    a3u = conv_mod.load_spec(conv_mod.DEFAULT_A3U_MJCF)
    worst = 0.0
    for t in range(20):
        qu = np.zeros(U.nq)
        qu[:3] = entry["root_trans_offset"][t]
        x, y, z, w = entry["root_rot"][t]
        qu[3:7] = [w, x, y, z]
        for c in range(31):
            qu[U.jnt_qposadr[mj.mj_name2id(U, mj.mjtObj.mjOBJ_JOINT, a3u.joint_names[conv.src_body[c] - 1])]] = entry["dof"][t, c]
        qa = np.zeros(A.nq)
        qa[:3] = out["root_trans_offset"][t]
        x, y, z, w = out["root_rot"][t]
        qa[3:7] = [w, x, y, z]
        for k, j in enumerate(conv.a3_body_joint):
            qa[A.jnt_qposadr[mj.mj_name2id(A, mj.mjtObj.mjOBJ_JOINT, conv.a3.joint_names[j])]] = out["dof"][t, k]
        dU.qpos[:] = qu
        dA.qpos[:] = qa
        mj.mj_forward(U, dU)
        mj.mj_forward(A, dA)
        for name in (n for i, n in enumerate(conv.a3.body_names) if i not in (4, 5)):  # not the head bodies (renamed)
            iu = mj.mj_name2id(U, mj.mjtObj.mjOBJ_BODY, name)
            ia = mj.mj_name2id(A, mj.mjtObj.mjOBJ_BODY, name)
            worst = max(worst, float(np.linalg.norm(dU.xpos[iu] + [0, 0, conv.root_z_shift] - dA.xpos[ia])))
    assert worst < 0.002, f"max body position difference {worst * 1000:.2f} mm"


def test_knee_beyond_the_a3_range_is_clipped_and_counted(conv):
    entry = _ultra_clip(conv, knee_deg=144.5)  # allowed on the Ultra (145 deg), not on the A3 (143 deg)
    out, stats = conv_mod.convert_entry("clip", entry, conv)
    names = [conv.a3.joint_names[j] for j in conv.a3_body_joint]
    knees = [i for i, n in enumerate(names) if "knee" in n]
    assert np.allclose(np.degrees(out["dof"][:, knees]), 143.0, atol=1e-3)
    assert list(stats["clipped"][knees]) == [12, 12] and stats["clipped"].sum() == 24
    # the clipped angle is also what pose_aa says (pose_aa is regenerated from dof, not copied)
    assert conv_mod.check_a3_29d_entry("clip", out, conv, 12) == []


def test_inconsistent_source_is_refused(conv):
    entry = _ultra_clip(conv)
    entry["dof"] = entry["dof"][:, ::-1].copy()  # columns no longer match pose_aa
    with pytest.raises(ValueError, match="joint order"):
        conv_mod.convert_entry("clip", entry, conv)
    bad = _ultra_clip(conv)
    bad["dof"][0, 0] = np.nan
    with pytest.raises(ValueError, match="non-finite"):
        conv_mod.convert_entry("clip", bad, conv)


def test_batch_run_keeps_layout_reports_failures_and_check_mode_agrees(tmp_path, conv):
    src, dst = tmp_path / "src", tmp_path / "dst"
    (src / "sessionA").mkdir(parents=True)
    (src / "sessionB").mkdir()
    for rel, seed in (("sessionA/walk_001.pkl", 1), ("sessionB/jump_001_M.pkl", 2)):
        joblib.dump({Path(rel).stem: _ultra_clip(conv, seed=seed, knee_deg=144.0)}, src / rel)
    joblib.dump({"broken": {"dof": np.zeros((3, 31))}}, src / "sessionA/broken.pkl")
    args = argparse.Namespace(
        input=src, output=dst, a3_mjcf=conv_mod.DEFAULT_A3_MJCF, a3u_mjcf=conv_mod.DEFAULT_A3U_MJCF, num_workers=2,
        limit=0, overwrite=False, report=tmp_path / "report.json", check=False,
    )
    assert conv_mod.run(args) == 1  # one failure -> non-zero exit, the others are written
    assert sorted(p.relative_to(dst).as_posix() for p in dst.rglob("*.pkl")) == ["sessionA/walk_001.pkl", "sessionB/jump_001_M.pkl"]
    report = __import__("json").loads((tmp_path / "report.json").read_text())
    assert report["ok"] == 2 and report["failed"] == 1 and report["clipped_frames_per_joint"]["left_knee_joint"] == 24
    (name, entry), = joblib.load(dst / "sessionB/jump_001_M.pkl").items()
    assert name == "jump_001_M" and entry["dof"].shape == (12, 29)  # one-item dict keyed by the file name, like the source
    # --check: the two good outputs are fine; the unconverted broken source is reported as missing
    (src / "sessionA/broken.pkl").unlink()
    args.check = True
    assert conv_mod.run(args) == 0
    joblib.dump({"walk_001": {"dof": np.zeros((2, 29), np.float32)}}, dst / "sessionA/walk_001.pkl")  # corrupt one output
    assert conv_mod.run(args) == 1
