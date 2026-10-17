"""Left/right IK-offset calibration of the A3 retarget library: mirror FK, measurement, half-correction, table edit."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("scipy")
from scipy.spatial.transform import Rotation  # noqa: E402

from gear_sonic.data_process.calibrate_a3_lr_symmetry import (  # noqa: E402
    LINKS,
    A3Kinematics,
    apply_to_table,
    corrections,
    half_inverse,
    measure,
    mirror_name,
    summarize,
)


@pytest.fixture(scope="module")
def kin():
    return A3Kinematics()


def _random_q(kin, t=12, seed=0):
    rng = np.random.default_rng(seed)
    q = rng.uniform(-0.5, 0.5, size=(t, 29))
    q[:, kin.col["left_wrist_roll_joint"]] = 0.0
    q[:, kin.col["right_wrist_roll_joint"]] = 0.0
    return q


def test_mirror_name_is_an_involution_and_signs_follow_the_axes(kin):
    assert mirror_name("left_wrist_roll_joint") == "right_wrist_roll_joint"
    assert mirror_name(mirror_name("right_hip_yaw_joint")) == "right_hip_yaw_joint"
    s = lambda n: kin.sign[kin.col[n]]  # noqa: E731
    # roll (x) and yaw (z) flip under a left/right mirror, pitch (y) does not
    assert (s("left_wrist_roll_joint"), s("left_wrist_pitch_joint"), s("left_wrist_yaw_joint")) == (-1, 1, -1)
    assert (s("left_hip_yaw_joint"), s("left_hip_pitch_joint")) == (-1, 1)
    assert (s("waist_yaw_joint"), s("waist_roll_joint"), s("waist_pitch_joint")) == (-1, -1, 1)
    q = _random_q(kin)
    assert np.allclose(kin.mirrored(kin.mirrored(q)), q)


def test_the_a3_model_is_left_right_symmetric_under_the_mirror_map(kin):
    """Random whole-body configurations: every mirrored link sits at the mirrored position with the mirrored frame."""
    import mujoco as mj

    mdl, dat = kin.model, kin.data
    lo = np.array([mdl.jnt_range[kin._jid(n)][0] for n in kin.names])
    hi = np.array([mdl.jnt_range[kin._jid(n)][1] for n in kin.names])
    rng = np.random.default_rng(1)
    mirror = np.diag([1.0, -1.0, 1.0])
    bodies = [f"{s}_{v}" for s in ("left", "right") for v in LINKS.values()]
    worst_pos = worst_rot = 0.0
    for _ in range(100):
        q = rng.uniform(lo, hi)[None]
        a, b = kin.link_rotations(q, bodies), kin.link_rotations(kin.mirrored(q), bodies)
        for v in LINKS.values():
            rot = Rotation.from_matrix(mirror @ a[f"left_{v}"][0] @ mirror).inv() * Rotation.from_matrix(b[f"right_{v}"][0])
            worst_rot = max(worst_rot, np.degrees(rot.magnitude()))
        qpos = np.zeros(mdl.nq)
        qpos[3] = 1.0
        pos = {}
        for tag, qq in (("a", q[0]), ("b", kin.mirrored(q)[0])):
            qpos[kin._qadr] = qq
            dat.qpos[:] = qpos
            mj.mj_kinematics(mdl, dat)
            pos[tag] = {x: dat.xpos[mj.mj_name2id(mdl, mj.mjtObj.mjOBJ_BODY, x)].copy() for x in bodies}
        for v in LINKS.values():
            worst_pos = max(worst_pos, np.linalg.norm(mirror @ pos["a"][f"left_{v}"] - pos["b"][f"right_{v}"]))
    assert worst_rot < 1e-3  # degrees
    assert worst_pos < 2e-3  # metres (the MJCF elbow offsets differ by 0.5 mm)


def test_exact_mirror_twins_have_no_asymmetry(kin):
    qa = _random_q(kin)
    summ = summarize(measure(kin, [(qa, kin.mirrored(qa))]))
    assert max(s["deg"] for s in summ.values()) < 1e-3


def test_a_constant_wrist_roll_error_is_measured_and_halved_per_side(kin):
    qa = _random_q(kin)
    for n in ("left_wrist_pitch_joint", "left_wrist_yaw_joint"):  # a fixed wrist axis keeps the per-frame error rotation identical
        qa[:, kin.col[n]] = 0.0
    qb = kin.mirrored(qa)
    qb[:, kin.col["right_wrist_roll_joint"]] += 0.2  # the right hand of the twin is rolled 0.2 rad too far
    summ = summarize(measure(kin, [(qa, qb)]))
    assert np.degrees(0.2) == pytest.approx(summ[("right", "wrist_yaw")]["deg"], abs=0.5)
    assert np.degrees(0.2) == pytest.approx(summ[("left", "wrist_yaw")]["deg"], abs=0.5)
    assert summ[("right", "elbow")]["deg"] < 1e-3  # links before the wrist are untouched
    corr = corrections(summ, min_deg=5.0)
    assert set(corr) == {("left", "wrist_yaw"), ("right", "wrist_yaw")}
    assert np.degrees(corr[("right", "wrist_yaw")].magnitude()) == pytest.approx(0.5 * np.degrees(0.2), abs=0.3)


def test_half_inverse_halves_the_angle_and_flips_the_sense():
    e = half_inverse(Rotation.from_euler("z", 20, degrees=True))
    assert np.degrees(e.as_rotvec()) == pytest.approx([0, 0, -10], abs=1e-9)


def test_apply_to_table_right_multiplies_the_offset_of_both_tables_only_for_corrected_links():
    ident = [1.0, 0.0, 0.0, 0.0]
    row = lambda: ["x", 0, 10, [0.0, 0.0, 0.0], list(ident)]  # noqa: E731
    keys = [f"{s}_{v}" for s in ("left", "right") for v in LINKS.values()]
    cfg = {"ik_match_table1": {k: row() for k in keys}, "ik_match_table2": {k: row() for k in keys}}
    e = Rotation.from_euler("x", 10, degrees=True)
    done = apply_to_table(cfg, {("left", "wrist_yaw"): e})
    assert done == ["left_wrist_yaw_Link"]
    for table in ("ik_match_table1", "ik_match_table2"):
        got = Rotation.from_quat(cfg[table]["left_wrist_yaw_Link"][4], scalar_first=True)
        assert got.magnitude() == pytest.approx(e.magnitude())
        assert cfg[table]["right_wrist_yaw_Link"][4] == ident
        assert cfg[table]["left_knee_Link"][4] == ident
    # right-multiplication: applying a second correction composes in the link frame
    cfg["ik_match_table1"]["left_wrist_yaw_Link"][4] = Rotation.from_euler("z", 90, degrees=True).as_quat(scalar_first=True).tolist()
    apply_to_table(cfg, {("left", "wrist_yaw"): e})
    got = Rotation.from_quat(cfg["ik_match_table1"]["left_wrist_yaw_Link"][4], scalar_first=True)
    assert got.as_matrix() == pytest.approx((Rotation.from_euler("z", 90, degrees=True) * e).as_matrix())
