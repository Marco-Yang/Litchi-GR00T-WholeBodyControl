"""A3 / A3 Ultra MuJoCo simulator model checks (mujoco only, no ROS) and the --robot registry."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import yaml

mujoco = pytest.importorskip("mujoco")

from gear_sonic.utils.mujoco_sim.a3_sim import A3Simulator, GROUPS  # noqa: E402

WBC_DIR = Path(__file__).resolve().parents[1] / "gear_sonic" / "utils" / "mujoco_sim" / "wbc_configs"


def _config(robot: str) -> dict:
    from gear_sonic.utils.mujoco_sim.configs import ROBOT_WBC_YAML

    return yaml.safe_load((WBC_DIR / ROBOT_WBC_YAML[robot]).read_text())


@pytest.mark.parametrize("robot", ["a3", "a3_ultra"])
def test_every_topic_joint_resolves(robot):
    sim = A3Simulator(_config(robot), onscreen=False, bridge=None)
    assert sum(len(v) for v in GROUPS.values()) == 31  # 12 leg + 3 waist + 14 arm + 2 neck
    for names in GROUPS.values():
        for n in names:
            assert n in sim.qadr and n in sim.actid


def test_a3_ultra_head_pitch_alias():
    sim = A3Simulator(_config("a3_ultra"), onscreen=False, bridge=None)
    assert sim.mj_name["head_pitch_joint"] == "ultra_head_pitch_joint"
    # 20 finger position actuators are held, not driven by the body-drive topics
    assert len(sim.hold_act) == 20
    assert not A3Simulator(_config("a3"), onscreen=False, bridge=None).hold_act


@pytest.mark.parametrize("robot", ["a3", "a3_ultra"])
def test_floor_collides_and_robot_rests_on_it(robot):
    """Regression: the A3 Ultra MJCF floor once had contype=conaffinity=0 and the robot fell through it."""
    cfg = _config(robot)
    cfg.update(ENABLE_ELASTIC_BAND=False, FALL_RESET_Z=-10.0)  # limp robot: let it collapse onto the floor
    sim = A3Simulator(cfg, onscreen=False, bridge=None)
    for _ in range(int(3.0 / sim.dt)):
        sim.step()
    z = float(sim.data.qpos[2])
    assert z > 0.05, f"pelvis z={z}: robot fell through the floor"
    assert np.isfinite(sim.data.qpos).all()


@pytest.mark.parametrize("robot", ["a3", "a3_ultra"])
def test_imu_measures_gravity_when_at_rest(robot):
    cfg = _config(robot)
    cfg.update(ENABLE_ELASTIC_BAND=False, FALL_RESET_Z=-10.0)
    sim = A3Simulator(cfg, onscreen=False, bridge=None)
    for _ in range(int(8.0 / sim.dt)):
        sim.step()
    mujoco.mj_rnePostConstraint(sim.model, sim.data)
    quat, gyro, acc = sim._imu("torso")
    assert abs(np.linalg.norm(acc) - 9.81) < 1.5
    assert np.linalg.norm(gyro) < 0.5
    assert abs(np.linalg.norm(quat) - 1.0) < 1e-6


def test_robot_registry_has_all_robots():
    pytest.importorskip("pinocchio")
    from gear_sonic.data.robot_model.instantiation import ROBOT_MODEL_REGISTRY, instantiate_robot_model

    assert set(ROBOT_MODEL_REGISTRY) == {"g1", "a3", "a3_ultra", "h2"}
    for robot in ROBOT_MODEL_REGISTRY:
        model = instantiate_robot_model(robot)
        assert len(model.get_body_actuated_joint_indices()) == 29, robot
        assert len(model.get_joint_group_indices("arms")) == 14, robot
        assert len(model.get_joint_group_indices("legs")) == 12, robot


def test_key_frames_follow_the_robot():
    pytest.importorskip("pinocchio")
    from gear_sonic.data.robot_model.instantiation import instantiate_robot_model
    from gear_sonic.utils.teleop.vis.vr3pt_pose_visualizer import get_g1_key_frame_poses

    for robot in ("g1", "a3", "a3_ultra", "h2"):
        poses = get_g1_key_frame_poses(instantiate_robot_model(robot))
        assert set(poses) == {"left_wrist", "right_wrist", "torso"}, robot
