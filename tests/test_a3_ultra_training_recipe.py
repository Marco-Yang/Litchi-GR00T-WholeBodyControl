"""``--robot a3_ultra`` trains the A3 contract (A3 parameters) with domain randomization that covers A3 AND A3 Ultra.

The 035 policy was trained on the A3 URDF with randomization "tuned to support both A3 and A3 Ultra"; ``sonic_a3_ultra.yaml``
is that recipe for the A3 Ultra's own motion library.  These tests keep (1) the CLI mapping, (2) the composed config
identical to the 035 contract except for the data / names, and (3) the claim that the randomization covers the real
A3 -> A3 Ultra differences, derived from the two MJCFs instead of asserted by hand.
"""

from __future__ import annotations

import ast
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
EXP_DIR = REPO / "gear_sonic/config/exp/manager/universal_token/all_modes"
EXP = "manager/universal_token/all_modes/"


def _robot_exp_map() -> dict[str, str]:
    tree = ast.parse((REPO / "gear_sonic/train_agent_trl.py").read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == "ROBOT_EXP_MAP" for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError("ROBOT_EXP_MAP not found")


def test_robot_flag_maps_a3_ultra_to_the_a3_contract_recipe():
    exp_map = _robot_exp_map()
    assert exp_map["a3"] == EXP + "sonic_a3_035"
    assert exp_map["a3_ultra"] == EXP + "sonic_a3_ultra"
    for name in ("sonic_a3_035", "sonic_a3_ultra", "sonic_a3_ultra_t3d0"):
        assert (EXP_DIR / f"{name}.yaml").is_file(), name


@pytest.fixture(scope="module")
def composed():
    hydra = pytest.importorskip("hydra")
    from hydra import compose, initialize_config_dir

    out = {}
    with initialize_config_dir(config_dir=str(REPO / "gear_sonic/config"), version_base="1.1"):
        for key, exp in {"a3": "sonic_a3_035", "a3_ultra": "sonic_a3_ultra", "t3d0": "sonic_a3_ultra_t3d0",
                         "smpl": "sonic_a3_ultra_smpl"}.items():
            out[key] = compose(config_name="base", overrides=[f"+exp={EXP}{exp}"])
    assert hydra is not None
    return out


def _flat(cfg, prefix=""):
    from omegaconf import OmegaConf

    out = {}
    for k, v in OmegaConf.to_container(cfg, resolve=False).items():
        if isinstance(v, dict):
            out.update(_flat(OmegaConf.create(v), prefix + k + "."))
        else:
            out[prefix + k] = v
    return out


def test_a3_ultra_recipe_is_the_035_contract_with_the_converted_library(composed):
    a, u = _flat(composed["a3"]), _flat(composed["a3_ultra"])
    differing = sorted(k for k in set(a) | set(u) if a.get(k) != u.get(k))
    assert differing == ["exp_var", "manager_env.commands.motion.motion_lib_cfg.motion_file", "project_name"]
    assert u["manager_env.commands.motion.motion_lib_cfg.motion_file"] == "data/motion_lib_a3_29d/robot_filtered_v15_clean"
    # the A3 asset / contract (29 policy joints come from it), not the Ultra's 63-DOF model
    assert u["manager_env.config.robot.type"] == "a3_024"
    assert u["manager_env.commands.motion.motion_lib_cfg.asset.assetFileName"] == "a3_t2d5.xml"
    assert u["project_name"] == "TRL_A3Ultra_Track"


def test_the_own_t3d0_contract_stays_available_and_separate(composed):
    t = composed["t3d0"]
    assert t.manager_env.config.robot.type == "a3_ultra"
    assert t.manager_env.commands.motion.motion_lib_cfg.asset.assetFileName == "a3_ultra_t3d0.xml"
    assert len(t.manager_env.actions.joint_pos.joint_names) == 31
    assert t.project_name != composed["a3_ultra"].project_name


def test_smpl_recipe_adds_exactly_the_encoder_the_a3_deploy_loads(composed):
    """`sonic_a3_ultra_smpl`: the a3_ultra recipe + the SMPL encoder with the deploy's tokenizer layout (840 floats)."""
    base, smpl = composed["a3_ultra"], composed["smpl"]
    mo = smpl.manager_env.commands.motion
    b = smpl.algo.config.actor.backbone
    assert list(b.active_encoders) == ["g1", "a3_fast", "smpl"] and b.reencode_smpl_g1_recon is True
    assert dict(mo.encoder_sample_probs) == {"g1": 1.0, "teleop": 0.0, "smpl": 1.0, "a3_fast": 1.0}  # no teleop on A3
    assert set(b.aux_loss_func) == {"g1_recon", "g1_smpl_latent", "reencoded_smpl_g1_latent", "a3_fast_g1_latent", "reencoded_a3_fast_g1_latent"}
    # the SMPL window of the training encoder is the one A3ZmqSmplSource builds: 10 frames, 0.02 s apart
    header = (REPO / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_deploy/a3_obs_builder.hpp").read_text()
    assert "kA3SmplTokenizerTotalFloats = 840" in header
    assert (mo.smpl_num_future_frames, mo.smpl_dt_future_ref_frames) == (10, 0.02)
    # tokenizer order = 24 joints x 3 | root orientation 6D | six wrist joints (IsaacLab 23..28), 10 frames each
    assert list(b.encoders.smpl.inputs) == [
        "smpl_joints_multi_future_local_nonflat", "smpl_root_ori_b_multi_future", "joint_pos_multi_future_wrist_for_smpl",
    ]
    wrist = smpl.manager_env.observations.tokenizer.joint_pos_multi_future_wrist_for_smpl.params.joints_idx
    assert list(wrist) == [23, 24, 25, 26, 27, 28]
    assert 10 * (24 * 3 + 6 + len(wrist)) == 840
    # everything else is the a3_ultra recipe
    assert smpl.manager_env.commands.motion.motion_lib_cfg.motion_file == base.manager_env.commands.motion.motion_lib_cfg.motion_file
    assert smpl.manager_env.config.robot.type == "a3_024"
    assert smpl.project_name == "TRL_A3Ultra_SMPL"


# ---------------------------------------------------------------------------------------------- domain randomization
mj = pytest.importorskip("mujoco")
MJCF = REPO / "gear_sonic/data/assets/robot_description/mjcf"
ALIAS = {"head_pitch_Link": "ultra_head_pitch_Link"}  # A3 body name -> A3 Ultra body name


def _models():
    return (
        mj.MjModel.from_xml_path(str(MJCF / "a3_t2d5.xml")),
        mj.MjModel.from_xml_path(str(MJCF / "a3_ultra_t3d0.xml")),
    )


def _mass_ranges(cfg) -> dict[str, list[float]]:
    ev = cfg.manager_env.events
    return {
        "torso": list(ev.physics_mass_torso.params.mass_distribution_params),
        "other": list(ev.physics_mass_other_links.params.mass_distribution_params),
        "hand_payload": list(ev.hand_payload_mass.params.mass_distribution_params),
    }


def test_domain_randomization_covers_the_a3_ultra_mass_differences(composed):
    r = _mass_ranges(composed["a3_ultra"])
    assert r["torso"] == [0.8, 1.3] and r["other"] == [0.9, 1.1] and r["hand_payload"] == [-0.2, 2.0]
    assert composed["a3_ultra"].manager_env.events.hand_payload_mass.params.operation == "add"
    A, U = _models()
    name = lambda m, i: mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, i)  # noqa: E731
    mass_a = {name(A, i): A.body_mass[i] for i in range(1, A.nbody)}
    mass_u = {name(U, i): U.body_mass[i] for i in range(1, U.nbody)}
    outside = {}
    for body, ma in mass_a.items():
        mu = mass_u[ALIAS.get(body, body)]
        if body.endswith("wrist_yaw_Link"):
            continue  # the Ultra's hand hangs off this body: checked as an additive payload below
        lo, hi = r["torso"] if body == "torso_Link" else r["other"]
        if not lo <= mu / ma <= hi:
            outside[body] = mu - ma
    # the only body outside its scale range is the head yaw link, +0.235 kg (the head joints are held at q = 0)
    assert list(outside) == ["head_yaw_Link"], outside
    assert 0.0 < outside["head_yaw_Link"] < 0.25
    for side in ("left", "right"):
        wrist = f"{side}_wrist_yaw_Link"
        extra = U.body_subtreemass[mj.mj_name2id(U, mj.mjtObj.mjOBJ_BODY, wrist)] - mass_a[wrist]
        assert r["hand_payload"][0] < extra < r["hand_payload"][1], f"{wrist}: +{extra:.3f} kg vs payload range {r['hand_payload']}"
        assert extra == pytest.approx(0.50, abs=0.05)
    assert (sum(mass_u.values()) - sum(mass_a.values())) / sum(mass_a.values()) < 0.04  # +3.5 % overall


def test_joint_ranges_of_the_ultra_stay_inside_what_the_a3_policy_trained_on():
    A, U = _models()
    alias = {"head_pitch_joint": "ultra_head_pitch_joint"}
    worse = {}
    for j in range(1, A.njnt):
        n = mj.mj_id2name(A, mj.mjtObj.mjOBJ_JOINT, j)
        if n.startswith("head"):
            continue
        ra = np.degrees(A.jnt_range[j])
        ru = np.degrees(U.jnt_range[mj.mj_name2id(U, mj.mjtObj.mjOBJ_JOINT, alias.get(n, n))])
        if ru[0] < ra[0] - 1e-6 or ru[1] > ra[1] + 1e-6:
            worse[n] = (tuple(ra.round(1)), tuple(ru.round(1)))
    # the Ultra is wider than the A3 only at the knee (145 vs 143 deg); the converter clips the data to the A3 range
    assert sorted(worse) == ["left_knee_joint", "right_knee_joint"], worse
