"""``compare_smpl_vs_a3``: the heading-offset measure follows the training code's SMPL root convention.

The offset is only meaningful if SMPL's root quaternion is built exactly as ``TrackingCommand.smpl_root_quat_w`` builds
it (Y-up -> Z-up, then ``remove_smpl_base_rot``); these tests pin that against an independent construction.
"""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

import gear_sonic.data_process.compare_smpl_vs_a3 as cmp


def _pose_aa_facing(yaw: np.ndarray) -> np.ndarray:
    """SMPL (Y-up) global orient of a person who stands upright and faces world heading ``yaw`` (Z-up, +X forward)."""
    world = Rotation.from_euler("z", yaw)  # the root orientation we want in the Z-up world
    base = Rotation.from_quat([0.5, 0.5, 0.5, 0.5], scalar_first=True)
    ytoz = Rotation.from_rotvec([np.pi / 2, 0, 0])
    y_up = ytoz.inv() * (world * base)  # invert smpl_root_ytoz_up(remove_smpl_base_rot^-1(...))
    pose = np.zeros((len(yaw), 72))
    pose[:, :3] = y_up.as_rotvec()
    return pose


def test_smpl_root_world_roundtrips_a_known_heading():
    yaw = np.radians([-170.0, -87.0, 0.0, 45.0, 120.0])
    rot = cmp.smpl_root_world(_pose_aa_facing(yaw))
    assert np.allclose(cmp.wrap(cmp.heading(rot) - yaw), 0, atol=1e-6)
    assert np.allclose(rot.apply([0, 0, 1.0]), [0, 0, 1.0], atol=1e-6)  # upright stays upright


def _clips(robot_yaw: np.ndarray, smpl_yaw: np.ndarray, robot_fps=30, n_smpl=None):
    smpl = {"pose_aa": _pose_aa_facing(smpl_yaw)}
    robot = {"root_rot": Rotation.from_euler("z", robot_yaw).as_quat(), "fps": robot_fps}
    return smpl, robot


def test_yaw_offset_is_robot_minus_smpl_and_survives_the_fps_mapping():
    t = 31  # 1 s of robot frames at 30 fps -> 51 SMPL frames at 50 fps
    sec = np.linspace(0, 1.0, t)
    spin = np.radians(40.0) * sec  # both turn the same way at the same rate
    smpl_t = np.linspace(0, 1.0, 51)
    smpl_yaw = np.radians(40.0) * smpl_t - np.radians(87.0)
    robot_yaw = spin - np.radians(2.0)
    smpl, robot = _clips(robot_yaw, smpl_yaw)
    mean, std = cmp.yaw_offset(smpl, robot)
    assert np.degrees(mean) == pytest.approx(85.0, abs=0.2) and np.degrees(std) < 0.2


def test_consistent_pair_has_zero_offset_and_wraps_around_180():
    yaw = np.full(31, np.radians(179.0))
    smpl, robot = _clips(yaw + np.radians(4.0), np.full(51, np.radians(179.0)))
    mean, _ = cmp.yaw_offset(smpl, robot)
    assert np.degrees(mean) == pytest.approx(4.0, abs=0.01)  # +4 deg, not a -356 deg artefact of the wrap at +-180


def test_smpl_world_floor_and_pelvis_height():
    t = 100
    rel = np.zeros((t, 24, 3))
    rel[:, :, 2] = np.linspace(0, -0.95, 24)[None]  # pelvis at 0, feet 0.95 m below
    transl = np.zeros((t, 3))
    transl[:, 1] = 1.2  # Y-up height parameter, offset by an arbitrary constant
    transl[40:60, 1] += 0.3  # a jump
    out_rel, h, xy = cmp.smpl_world({"smpl_joints": rel, "transl": transl})
    assert h[0] == pytest.approx(0.95, abs=1e-6)  # standing: the pelvis is 0.95 m above the lowest joint
    assert h[50] == pytest.approx(1.25, abs=1e-6)  # the jump lifts it by 0.3 m
    assert out_rel.shape == (t, 24, 3) and xy.shape == (t, 2)
