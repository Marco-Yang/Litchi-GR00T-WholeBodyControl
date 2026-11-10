#!/usr/bin/env python3
"""Headless MuJoCo sim + mp4 record on Litchi-GR00T-WholeBodyControl gear_sonic (no Phi_0)."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import tyro

import gear_sonic.utils.mujoco_sim.gpu_gl  # noqa: F401

from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
from gear_sonic.scripts.run_sim_loop import SimWrapper
from gear_sonic.scripts.sim_mp4_recorder import SimMp4Recorder
from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
from gear_sonic.utils.mujoco_sim.revo2_hand_config import apply_revo2_hand_wbc_config
from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory


@dataclass
class RecordConfig(SimLoopConfig):
    record_mp4: str = ""
    record_start_flag: str = ""
    record_stop_flag: str = ""
    record_fps: float = 30.0
    revo2_hand: bool = True
    revo2_scene: str = ""
    # Ground walking: elastic band OFF by default.
    disable_elastic_band: bool = True
    disable_fall_reset: bool = True
    g1_debug_host: str = "127.0.0.1"
    g1_debug_port: int = 5557
    # GT RSI npz: keys reference_root_xyz / reference_root_quat_wxyz / observation_qpos(body29)
    gt_rsi_npz: str = ""
    standing_root_z: float = 0.793


def _load_gt_rsi_npz(npz_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    d = np.load(npz_path)
    if "reference_root_xyz" in d.files:
        root = np.asarray(d["reference_root_xyz"], dtype=np.float64).reshape(-1, 3)[0]
    else:
        root = np.array([0.0, 0.0, float(os.environ.get("STANDING_ROOT_Z", "0.793"))], dtype=np.float64)
    if "reference_root_quat_wxyz" in d.files:
        quat = np.asarray(d["reference_root_quat_wxyz"], dtype=np.float64).reshape(-1, 4)[0]
    else:
        quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if "observation_qpos" not in d.files:
        raise ValueError(f"{npz_path}: missing observation_qpos (body29)")
    obs = np.asarray(d["observation_qpos"], dtype=np.float64)
    body = obs.reshape(-1)[:29] if obs.ndim == 1 else obs[0].reshape(-1)[:29]
    return root, quat, body


def main(config: RecordConfig) -> None:
    os.environ.setdefault("MUJOCO_GL", "egl")
    from gear_sonic.utils.network.network_utils import resolve_interface

    iface, _resolved_env = resolve_interface(config.interface)
    # MuJoCo recorder is always sim; eth1/lo only picks the DDS NIC.
    env_type = "sim"
    config.interface = iface
    config.env_type = env_type

    wbc = config.load_wbc_yaml()
    wbc["ENV_NAME"] = config.env_name
    wbc["INTERFACE"] = iface
    wbc["ENV_TYPE"] = env_type
    wbc["ENABLE_ONSCREEN"] = False
    wbc["ENABLE_OFFSCREEN"] = True
    wbc["ENABLE_ELASTIC_BAND"] = not bool(config.disable_elastic_band)
    wbc["DISABLE_FALL_RESET"] = bool(config.disable_fall_reset)
    wbc["STANDING_ROOT_POS"] = [0.0, 0.0, float(config.standing_root_z)]
    print(
        f"[sim_record] INTERFACE={iface} ENV_TYPE={env_type} "
        f"ELASTIC_BAND={wbc['ENABLE_ELASTIC_BAND']} FALL_RESET={not wbc['DISABLE_FALL_RESET']}",
        flush=True,
    )

    if config.revo2_hand:
        apply_revo2_hand_wbc_config(wbc, revo2_scene=config.revo2_scene)
        print(f"[revo2_hand] ROBOT_SCENE={wbc.get('ROBOT_SCENE')}", flush=True)

    robot_model = instantiate_g1_robot_model()
    wrapper = SimWrapper(
        robot_model=robot_model,
        env_name=config.env_name,
        config=wbc,
        onscreen=False,
        offscreen=True,
        enable_image_publish=False,
    )
    sim_env = wrapper.sim.sim_env

    # Apply GT RSI immediately so LowState matches dataset pose before deploy.
    rsi_pose = None
    if config.gt_rsi_npz:
        os.environ["STANDING_ROOT_Z"] = str(config.standing_root_z)
        root, quat, body = _load_gt_rsi_npz(config.gt_rsi_npz)
        root = root.copy()
        root[2] = float(config.standing_root_z)
        rsi_pose = (root, quat, body)
        sim_env.set_standing_pose_on_ground(root, quat, body)
        if sim_env.elastic_band is not None:
            sim_env.elastic_band.enable = False
        print(
            f"[sim_record] GT RSI npz={config.gt_rsi_npz} "
            f"root_z={float(sim_env.mj_data.qpos[2]):.3f} "
            f"L_elbow={float(body[18]):.3f}rad R_elbow={float(body[25]):.3f}rad "
            f"band={getattr(sim_env.elastic_band, 'enable', None)}",
            flush=True,
        )

    if config.record_mp4:
        rec = SimMp4Recorder(
            sim_env.mj_model,
            Path(config.record_mp4),
            fps=config.record_fps,
            start_flag=Path(config.record_start_flag) if config.record_start_flag else None,
            stop_flag=Path(config.record_stop_flag) if config.record_stop_flag else None,
            pose_zmq_host="127.0.0.1",
            pose_zmq_port=5556,
            draw_command_markers=True,
            g1_debug_host=config.g1_debug_host or None,
            g1_debug_port=config.g1_debug_port,
            draw_g1_debug_markers=True,
            draw_g1_debug_hud=True,
            camera_host=None,
            # Re-apply standing snap + ensure band stays off when recording starts.
            snap_on_record_start=True,
            keep_elastic_band=False,
        )
        # Wire snap target so recorder re-applies GT RSI and releases band at record start.
        rec._snap_sim_env = sim_env
        if rsi_pose is not None:
            rec._snap_fallback_pose = rsi_pose
        sim_env.mp4_recorder = rec

    print("[sim] sim loop starting", flush=True)
    SimulatorFactory.start_simulator(
        wrapper.sim,
        as_thread=False,
        enable_image_publish=False,
        mp_start_method=config.mp_start_method,
        camera_port=config.camera_port,
    )


if __name__ == "__main__":
    main(tyro.cli(RecordConfig))
