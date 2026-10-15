"""Batch retargeting of the Bones-SEED SOMA BVH onto the A3 (T2D5, Litchi a3 parameters): ``--robot a3``.

The method is the one that produced the A3 Ultra library (GMR + calibrated IK table + per-clip proportions + floor lift);
only the robot model and the output format change.  These tests pin what makes that legitimate: the IK table's link frames
are the same on both robots, the foot geometry (a mesh on the A3) is measured exactly, and the output is an ``a3_29`` entry.
Needs mujoco (and GMR + mink for the retargeter tests); skipped without.
"""

from __future__ import annotations

import json
from pathlib import Path
import tarfile

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

mj = pytest.importorskip("mujoco")

REPO = Path(__file__).resolve().parents[1]
IK_DIR = REPO / "gear_sonic/data_process/gmr_ik_configs"
MJCF = REPO / "gear_sonic/data/assets/robot_description/mjcf"
BVH_TAR = REPO / "data/bones_seed/soma_proportional.tar.gz"


def _tables(path: Path) -> dict:
    cfg = json.loads(path.read_text())
    return {**cfg["ik_match_table1"], **cfg["ik_match_table2"]}


def test_a3_ik_table_is_the_calibrated_ultra_table():
    """Same table; the two robots share every IK link frame at qpos0 (checked in the next test)."""
    a3 = json.loads((IK_DIR / "bvh_nokov_to_a3.json").read_text())
    ultra = json.loads((IK_DIR / "bvh_nokov_to_a3_ultra.json").read_text())
    assert a3 == ultra
    assert a3["robot_root_name"] == "pelvis_link" and a3["human_root_name"] == "Hips"


def test_ik_link_frames_at_qpos0_are_identical_on_a3_and_a3_ultra():
    """rot_offset = q_soma_canonical^-1 * q_robot_qpos0 was calibrated on the Ultra; it only transfers if the A3 agrees."""
    A = mj.MjModel.from_xml_path(str(MJCF / "a3_t2d5.xml"))
    U = mj.MjModel.from_xml_path(str(MJCF / "a3_ultra_t3d0.xml"))
    dA, dU = mj.MjData(A), mj.MjData(U)
    for m, d in ((A, dA), (U, dU)):
        d.qpos[:] = m.qpos0
        mj.mj_forward(m, d)
    bodies = sorted(_tables(IK_DIR / "bvh_nokov_to_a3.json"))
    assert len(bodies) == 14
    for name in bodies:
        ia, iu = mj.mj_name2id(A, mj.mjtObj.mjOBJ_BODY, name), mj.mj_name2id(U, mj.mjtObj.mjOBJ_BODY, name)
        assert ia >= 0 and iu >= 0, f"{name} must exist in both models"
        rot = (Rotation.from_quat(dA.xquat[ia], scalar_first=True).inv() * Rotation.from_quat(dU.xquat[iu], scalar_first=True)).magnitude()
        assert rot < 1e-4, f"{name}: link frame differs by {np.degrees(rot):.4f} deg"
        assert np.linalg.norm(dA.xpos[ia] - dU.xpos[iu]) < 0.0025, name


def _gmr():
    pytest.importorskip("mink")
    pytest.importorskip("general_motion_retargeting")
    import gear_sonic.data_process.batch_retarget_bvh_soma_a3ultra as b

    return b


def test_gmr_gets_the_a3_registered_at_run_time():
    b = _gmr()
    ret = b.make_retargeter("a3")
    model = ret.configuration.model
    assert model.nq == 38 and model.nbody == 33
    assert ret.robot_geom["ankle_h"] == pytest.approx(0.0675, abs=2e-4), "A3 foot is a mesh: use its lowest vertex, not geom_size"
    ultra = b.make_retargeter("a3_ultra")
    assert ultra.robot_geom["ankle_h"] == pytest.approx(0.0720, abs=2e-4)  # sphere feet: unchanged by the mesh-aware sole
    assert ultra.configuration.model.nq == 70


def test_a3_qpos_becomes_an_a3_29_entry():
    pytest.importorskip("joblib")
    from gear_sonic.data_process.convert_a3ultra_to_a3_29d import build_conversion, check_a3_29d_entry
    from gear_sonic.data_process.convert_gmr_a3_to_motion_lib import convert_sequence

    conv = build_conversion()
    rng = np.random.default_rng(0)
    qpos = []
    for _ in range(9):
        q = np.zeros(38)
        q[:3] = [0.1, -0.2, 1.05]
        q[3:7] = Rotation.from_euler("z", rng.uniform(-1, 1)).as_quat(scalar_first=True)
        q[7:] = rng.uniform(-0.3, 0.3, 31)
        q[7 + 3] = q[7 + 4] = 0.0  # head joints are locked in the A3 MJCF
        q[7 + 22] = 3.0  # left knee (column 22 of the 31 joints), 172 deg: beyond the A3 range, must be clipped to 143 deg
        qpos.append(q)
    entry = convert_sequence(qpos, 30.0)
    assert check_a3_29d_entry("clip", entry, conv, 9) == []
    assert entry["fps"] == 30 and entry["dof"].shape == (9, 29) and entry["pose_aa"].shape == (9, 32, 3)
    assert np.allclose(entry["root_trans_offset"][0], [0.1, -0.2, 1.05])
    # wxyz in, xyzw out, and pose_aa[:, 0] is the same rotation as root_rot
    assert np.allclose(Rotation.from_rotvec(entry["pose_aa"][:, 0]).as_matrix(), Rotation.from_quat(entry["root_rot"]).as_matrix(), atol=1e-5)
    assert np.all(entry["pose_aa"][:, 4:6] == 0)
    assert np.allclose(np.degrees(entry["dof"][:, 20]), 143.0, atol=1e-3)  # the clipped knee is joint 20 of the 29 policy joints


@pytest.mark.skipif(not BVH_TAR.exists(), reason="Bones-SEED SOMA BVH archive not on this machine")
def test_one_real_bvh_retargets_onto_the_a3_and_matches_the_ultra_pipeline(tmp_path):
    b = _gmr()
    from gear_sonic.data_process.convert_a3ultra_to_a3_29d import build_conversion, check_a3_29d_entry
    import joblib

    with tarfile.open(BVH_TAR, "r|gz") as tf:
        for member in tf:
            if member.name.endswith(".bvh"):
                bvh = tmp_path / Path(member.name).name
                bvh.write_bytes(tf.extractfile(member).read())
                break
    out_a3, out_ultra = tmp_path / "a3.pkl", tmp_path / "ultra.pkl"
    ok_a3, _ = b.process_one_bvh(str(bvh), str(out_a3), 30, None, "clip", "a3")
    ok_ultra, _ = b.process_one_bvh(str(bvh), str(out_ultra), 30, None, "clip", "a3_ultra")
    assert ok_a3 and ok_ultra and not list(tmp_path.glob("*.tmp"))
    (_, a), = joblib.load(out_a3).items()
    (_, u), = joblib.load(out_ultra).items()
    assert check_a3_29d_entry("clip", a, build_conversion(), u["dof"].shape[0]) == []
    cols = [i for i in range(31) if i not in (3, 4)]  # drop the head columns of the Ultra's 31-column dof
    assert np.sqrt(np.mean((a["dof"] - u["dof"][:, cols]) ** 2)) < 0.01  # same method, same human, same joint angles
    assert abs(float(np.mean(a["root_trans_offset"][:, 2] - u["root_trans_offset"][:, 2])) + 0.0045) < 0.003  # sole height
