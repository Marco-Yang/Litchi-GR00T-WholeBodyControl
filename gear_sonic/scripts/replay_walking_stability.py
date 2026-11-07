"""Replay one walking episode in MuJoCo to check dynamic stability.

The floating base is free. Recorded ``action.wbc`` joint targets are tracked
with the SONIC sim PD gains, so a drop in pelvis height means the recorded
whole-body command is not stable in this model. ``ego_view`` and ``head`` from
the same episode play beside the viewer, then freeze for one second and repeat.
"""

from __future__ import annotations

import argparse
import json
import random
import time
from pathlib import Path

import cv2
import mujoco
import mujoco.viewer
import numpy as np
import pyarrow.parquet as pq
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCENE_XML = (
    REPO_ROOT
    / "gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml"
)
WBC_YAML = (
    REPO_ROOT
    / "gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml"
)
DEFAULT_DATASET = (
    REPO_ROOT / "datasets/walking/2026-11-05-15-57-52"
)

BODY_JOINTS = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
HAND_KP = 2.0
HAND_KD = 0.05
FALL_HEIGHT = 0.35
CONTROL_HZ = 50.0
SIM_DT = 0.005
PAUSE_S = 1.0


def _column(table, name: str) -> np.ndarray:
    values = table.column(name).to_pylist()
    return np.asarray(values, dtype=np.float64)


def episodes_with_both_views(dataset: Path) -> list[int]:
    ego = {
        path.stem
        for path in (dataset / "videos/chunk-000/observation.images.ego_view").glob(
            "episode_*.mp4"
        )
    }
    head = {
        path.stem
        for path in (dataset / "videos/chunk-000/observation.images.head").glob(
            "episode_*.mp4"
        )
    }
    indexes = []
    for stem in sorted(ego & head):
        indexes.append(int(stem.split("_")[-1]))
    if not indexes:
        raise FileNotFoundError(f"No episode has both ego_view and head under {dataset}")
    return indexes


def load_episode(dataset: Path, episode: int) -> dict:
    info = json.loads((dataset / "meta/info.json").read_text())
    names = info["features"]["action.wbc"]["names"]
    parquet = dataset / "data/chunk-000" / f"episode_{episode:06d}.parquet"
    table = pq.read_table(
        parquet,
        columns=["action.wbc", "observation.state", "observation.root_orientation"],
    )
    action = _column(table, "action.wbc")
    state = _column(table, "observation.state")
    quat = _column(table, "observation.root_orientation")
    if action.ndim != 2 or action.shape[1] != len(names):
        raise ValueError(f"Unexpected action.wbc shape {action.shape} in {parquet}")

    views = {}
    for key in ("ego_view", "head"):
        video = (
            dataset
            / "videos/chunk-000"
            / f"observation.images.{key}"
            / f"episode_{episode:06d}.mp4"
        )
        views[key] = read_video(video)
    n = min(len(action), len(state), len(quat), *(len(frames) for frames in views.values()))
    if n == 0:
        raise RuntimeError(f"Episode {episode} has no aligned frames")
    return {
        "episode": episode,
        "names": names,
        "action": action[:n],
        "state": state[:n],
        "quat": quat[:n],
        "views": {key: frames[:n] for key, frames in views.items()},
        "fps": float(info["fps"]),
    }


def read_video(path: Path) -> list[np.ndarray]:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open {path}")
    frames = []
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        frames.append(frame)
    capture.release()
    if not frames:
        raise RuntimeError(f"No frames in {path}")
    return frames


def build_pd(model: mujoco.MjModel, names: list[str]) -> dict:
    with open(WBC_YAML) as handle:
        config = yaml.load(handle, Loader=yaml.FullLoader)
    body_kp = {name: float(kp) for name, kp in zip(BODY_JOINTS, config["MOTOR_KP"])}
    body_kd = {name: float(kd) for name, kd in zip(BODY_JOINTS, config["MOTOR_KD"])}
    name_to_col = {name: index for index, name in enumerate(names)}

    actuators = []
    for act_id in range(model.nu):
        joint_id = int(model.actuator_trnid[act_id, 0])
        joint_name = model.joint(joint_id).name
        if joint_name not in name_to_col:
            raise KeyError(f"Actuator joint {joint_name} is missing from action.wbc")
        lo, hi = model.actuator_ctrlrange[act_id]
        actuators.append(
            {
                "act": act_id,
                "col": name_to_col[joint_name],
                "qadr": int(model.jnt_qposadr[joint_id]),
                "vadr": int(model.jnt_dofadr[joint_id]),
                "kp": body_kp.get(joint_name, HAND_KP),
                "kd": body_kd.get(joint_name, HAND_KD),
                "lo": float(lo),
                "hi": float(hi),
            }
        )
    return {"actuators": actuators}


def apply_pd(model, data, targets: np.ndarray, pd: dict) -> float:
    error = 0.0
    for item in pd["actuators"]:
        q = float(data.qpos[item["qadr"]])
        dq = float(data.qvel[item["vadr"]])
        q_des = float(targets[item["col"]])
        tau = item["kp"] * (q_des - q) + item["kd"] * (0.0 - dq)
        data.ctrl[item["act"]] = np.clip(tau, item["lo"], item["hi"])
        error += abs(q_des - q)
    return error / len(pd["actuators"])


def initial_state(model, data, episode: dict) -> np.ndarray:
    mujoco.mj_resetData(model, data)
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    quat = episode["quat"][0].copy()
    quat /= np.linalg.norm(quat) + 1e-12
    data.qpos[3:7] = quat
    name_to_col = {name: index for index, name in enumerate(episode["names"])}
    q0 = episode["state"][0]
    for joint_id in range(model.njnt):
        name = model.joint(joint_id).name
        if name not in name_to_col:
            continue
        data.qpos[int(model.jnt_qposadr[joint_id])] = q0[name_to_col[name]]
    mujoco.mj_forward(model, data)
    return data.qpos.copy()


def compose_views(ego: np.ndarray, head: np.ndarray, paused: bool, label: str) -> np.ndarray:
    if ego.shape[:2] != head.shape[:2]:
        head = cv2.resize(head, (ego.shape[1], ego.shape[0]))
    canvas = np.concatenate([ego, head], axis=1)
    cv2.putText(
        canvas,
        "ego_view",
        (16, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "head",
        (ego.shape[1] + 16, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    status = "暂停 1s" if paused else label
    cv2.putText(
        canvas,
        status,
        (16, canvas.shape[0] - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 220, 255) if paused else (180, 255, 180),
        2,
        cv2.LINE_AA,
    )
    return canvas


def run(args: argparse.Namespace) -> None:
    dataset = Path(args.dataset)
    choices = episodes_with_both_views(dataset)
    episode_index = args.episode if args.episode is not None else random.choice(choices)
    if episode_index not in choices:
        raise SystemExit(f"Episode {episode_index} does not have both camera views")
    episode = load_episode(dataset, episode_index)
    n = len(episode["action"])
    print(
        f"episode {episode_index:06d}  frames {n}  "
        f"views ego_view+head  control {CONTROL_HZ:.0f} Hz  pause {PAUSE_S:.0f}s"
    )

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    model.opt.timestep = SIM_DT
    pd = build_pd(model, episode["names"])
    qpos0 = initial_state(model, data, episode)
    substeps = max(1, int(round((1.0 / CONTROL_HZ) / SIM_DT)))
    pause_ticks = int(round(PAUSE_S * CONTROL_HZ))

    viewer = None
    if not args.headless:
        viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
        viewer.cam.distance = 2.5
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 140
        viewer.cam.lookat[:] = [0.0, 0.0, 0.6]
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = model.body("pelvis").id

    loop = 0
    frame = 0
    pausing = False
    pause_left = 0
    min_z = float(data.qpos[2])
    err_sum = 0.0
    err_count = 0
    fell = False
    next_tick = time.perf_counter()
    tick_dt = 1.0 / CONTROL_HZ

    try:
        while viewer is None or viewer.is_running():
            if pausing:
                targets = episode["action"][-1]
                image_index = n - 1
            else:
                targets = episode["action"][frame]
                image_index = frame

            step_err = 0.0
            for _ in range(substeps):
                step_err = apply_pd(model, data, targets, pd)
                mujoco.mj_step(model, data)
            err_sum += step_err
            err_count += 1
            min_z = min(min_z, float(data.qpos[2]))
            if data.qpos[2] < FALL_HEIGHT and not fell:
                fell = True
                print(f"跌倒  pelvis_z={data.qpos[2]:.3f} m  frame={image_index}")

            if not args.headless:
                canvas = compose_views(
                    episode["views"]["ego_view"][image_index],
                    episode["views"]["head"][image_index],
                    pausing,
                    f"ep {episode_index:06d}  {image_index + 1}/{n}  z={data.qpos[2]:.2f}m",
                )
                cv2.imshow("walking cameras", canvas)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                viewer.sync()

            if pausing:
                pause_left -= 1
                if pause_left <= 0:
                    report(loop, episode_index, min_z, err_sum / max(err_count, 1), fell)
                    if args.loops and loop >= args.loops:
                        break
                    data.qpos[:] = qpos0
                    data.qvel[:] = 0.0
                    mujoco.mj_forward(model, data)
                    frame = 0
                    pausing = False
                    min_z = float(data.qpos[2])
                    err_sum = 0.0
                    err_count = 0
                    fell = False
            else:
                frame += 1
                if frame >= n:
                    loop += 1
                    pausing = True
                    pause_left = pause_ticks

            if args.headless and loop >= args.loops and not pausing:
                report(loop, episode_index, min_z, err_sum / max(err_count, 1), fell)
                break

            if not args.headless:
                next_tick += tick_dt
                delay = next_tick - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.perf_counter()
    finally:
        if viewer is not None:
            viewer.close()
        cv2.destroyAllWindows()


def report(loop: int, episode: int, min_z: float, mean_err: float, fell: bool) -> None:
    status = "跌倒" if fell else "稳定"
    print(
        f"第 {loop} 遍  episode {episode:06d}  {status}  "
        f"最低骨盆 {min_z:.3f} m  关节跟踪误差 {mean_err:.4f} rad"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default=str(DEFAULT_DATASET))
    parser.add_argument("--episode", type=int, default=None, help="固定某一条；默认随机")
    parser.add_argument("--loops", type=int, default=0, help="播放遍数，0 表示一直循环")
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
