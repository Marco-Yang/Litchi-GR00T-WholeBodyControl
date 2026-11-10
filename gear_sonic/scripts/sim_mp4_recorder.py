"""Offscreen MuJoCo frame recorder (sim qpos + ZMQ pose command markers)."""

from __future__ import annotations

import logging
import os
import time
from pathlib import Path

import cv2
import msgpack
import mujoco
import numpy as np
import zmq
from scipy.spatial.transform import Rotation as R

from gear_sonic.utils.data_collection.zmq_state_subscriber import STATE_ZMQ_TOPIC
from gear_sonic.utils.zmq_pose_frames import (
    apply_deploy_heading_to_pose_msg,
    calc_deploy_heading_rad,
    deploy_corrected_snap_pose,
    lock_deploy_heading_once,
    extract_pico5_world_points,
    g1_pico5_world_from_g1_debug,
    g1_pico5_world_from_pose_msg,
    tracking_camera_azimuth,
)
from gear_sonic.utils.zmq_pose_unpack import summarize_pose_command, unpack_pose_message

# PICO 5-point SMPL command markers (world frame from ZMQ smpl_joints).
_PICO5_RGBA = (
    np.array([0.2, 0.85, 1.0, 0.95], dtype=np.float32),   # head
    np.array([1.0, 0.9, 0.1, 0.95], dtype=np.float32),     # left wrist
    np.array([1.0, 0.45, 0.05, 0.95], dtype=np.float32),  # right wrist
    np.array([0.2, 1.0, 0.35, 0.95], dtype=np.float32),   # left ankle
    np.array([0.35, 0.55, 1.0, 0.95], dtype=np.float32),  # right ankle
)
_ROOT_CMD_RGBA = np.array([0.95, 0.15, 0.95, 0.9], dtype=np.float32)
_G1_REF_RGBA = np.array([1.0, 1.0, 1.0, 0.55], dtype=np.float32)
_G1_CMD_RGBA = np.array([0.15, 0.95, 0.95, 0.85], dtype=np.float32)
LOG = logging.getLogger(__name__)


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name, "").strip()
    return float(value) if value else float(default)


def _clip_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    if max_norm <= 0:
        return vec
    norm = float(np.linalg.norm(vec))
    if norm <= max_norm or norm <= 1e-9:
        return vec
    return vec * (max_norm / norm)


def _wxyz_to_xyzw(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float64).reshape(4)
    return quat[[1, 2, 3, 0]]


def root_pose_sample(qpos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return simulator root position and measured WXYZ orientation."""
    values = np.asarray(qpos, dtype=np.float64).reshape(-1)
    if values.size < 7:
        raise ValueError(f"expected qpos with at least 7 values, got {values.size}")
    return values[:3].copy(), values[3:7].copy()


def _unpack_g1_debug(raw: bytes, topic: str) -> dict | None:
    try:
        payload = raw[len(topic) :]
        data = msgpack.unpackb(payload, raw=False)
    except Exception:
        return None
    if not isinstance(data, dict):
        return None
    out = {}
    for key, value in data.items():
        if isinstance(value, (list, tuple)):
            out[key] = np.asarray(value)
        else:
            out[key] = value
    return out


def _poll_g1_debug(sock: zmq.Socket | None, topic: str, latest: dict | None) -> dict | None:
    if sock is None:
        return latest
    while True:
        try:
            raw = sock.recv(zmq.NOBLOCK)
        except zmq.Again:
            break
        parsed = _unpack_g1_debug(raw, topic)
        if parsed is not None:
            latest = parsed
    return latest


def _add_g1_debug_command_markers(
    scene: mujoco.MjvScene,
    debug_msg: dict | None,
    *,
    model: mujoco.MjModel | None = None,
    fk_data: mujoco.MjData | None = None,
) -> None:
    if debug_msg is None or scene.ngeom >= scene.maxgeom:
        return

    if "base_trans_target" in debug_msg and scene.ngeom < scene.maxgeom:
        root = np.asarray(debug_msg["base_trans_target"], dtype=np.float64).reshape(-1)[:3]
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.045, 0.0, 0.0],
            pos=root,
            mat=np.eye(3).flatten(),
            rgba=_ROOT_CMD_RGBA,
        )
        scene.ngeom += 1

    if model is not None and fk_data is not None:
        g1_cmd = g1_pico5_world_from_g1_debug(debug_msg, model, fk_data)
        if g1_cmd is not None:
            for i, pos in enumerate(g1_cmd):
                if scene.ngeom >= scene.maxgeom:
                    break
                rgba = _G1_CMD_RGBA.copy()
                rgba[3] = 0.75 if i == 0 else 0.9
                mujoco.mjv_initGeom(
                    scene.geoms[scene.ngeom],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.032 if i else 0.04, 0.0, 0.0],
                    pos=pos,
                    mat=np.eye(3).flatten(),
                    rgba=rgba,
                )
                scene.ngeom += 1


def _apply_heading_to_g1_debug(
    debug_msg: dict | None,
    heading_apply,
) -> dict | None:
    """Align deploy g1_debug root targets to MuJoCo viz frame (fixes ~90° yaw offset)."""
    if debug_msg is None or heading_apply is None:
        return debug_msg
    out = dict(debug_msg)
    pose_part: dict[str, np.ndarray] = {}
    if "base_trans_target" in debug_msg:
        pose_part["root_trans"] = np.asarray(debug_msg["base_trans_target"], dtype=np.float64)
    if "base_quat_target" in debug_msg:
        pose_part["body_quat_w"] = np.asarray(debug_msg["base_quat_target"], dtype=np.float64)
    if not pose_part:
        return out
    corrected = apply_deploy_heading_to_pose_msg(pose_part, heading_apply)
    if "root_trans" in corrected:
        out["base_trans_target"] = corrected["root_trans"]
    if "body_quat_w" in corrected:
        out["base_quat_target"] = corrected["body_quat_w"]
    return out


def _composite_ego_inset(
    frame_bgr: np.ndarray,
    ego_rgb: np.ndarray,
    *,
    scale: float = 0.38,
    label: str = "ego_view (VLA input)",
    from_rgb: bool = True,
) -> None:
    h, w = frame_bgr.shape[:2]
    ih = max(64, int(h * scale))
    iw = max(64, int(w * scale))
    ego = _to_bgr(ego_rgb, from_rgb=from_rgb)
    ego_small = cv2.resize(ego, (iw, ih), interpolation=cv2.INTER_AREA)
    x0, y0 = w - iw - 10, 10
    frame_bgr[y0 : y0 + ih, x0 : x0 + iw] = ego_small
    cv2.rectangle(frame_bgr, (x0 - 2, y0 - 2), (x0 + iw + 2, y0 + ih + 2), (255, 255, 255), 1)
    if label:
        cv2.putText(
            frame_bgr,
            label,
            (x0, y0 + ih + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )


def _to_bgr(img: np.ndarray, *, from_rgb: bool = True) -> np.ndarray:
    """Convert to BGR for OpenCV write/composite.

    VideoCapture / already-BGR panels must pass ``from_rgb=False`` (avoid R↔B swap).
    Live camera RGB tensors keep default ``from_rgb=True``.
    """
    arr = np.asarray(img)
    if arr.ndim == 2:
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    if arr.shape[-1] == 3 and from_rgb:
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    return arr


def advance_record_schedule(
    next_t: float,
    now: float,
    interval: float,
) -> tuple[float, bool]:
    """Advance record clock; drop stale frames when sim fell behind."""
    if now < next_t:
        return next_t, False
    if now - next_t > interval * 2:
        return now, True
    return next_t + interval, True


def gt_zmq_record_gate(
    frame_index: int | None,
    *,
    seen_fi: bool,
    seen_advance: bool,
    last_fi: int | None,
    last_advance_t: float | None,
    now: float,
    stall_s: float = 0.4,
) -> tuple[str, bool, bool, int | None, float | None]:
    """Gate GT-panel mp4 writes to the ZMQ control clock.

    Returns ``(action, seen_fi, seen_advance, last_fi, last_advance_t)``.
    action: ``skip`` | ``write`` | ``stop``.

    # ponytail: stall-stop only after frame_index advanced once; ceiling ~0.4s
    # frozen tail if EOS is quiet. Upgrade: explicit end-of-stream flag from publisher.
    """
    if frame_index is None:
        if (
            seen_advance
            and last_advance_t is not None
            and (now - last_advance_t) >= stall_s
        ):
            return "stop", seen_fi, seen_advance, last_fi, last_advance_t
        return ("write" if seen_fi else "skip"), seen_fi, seen_advance, last_fi, last_advance_t

    fi = int(frame_index)
    if not seen_fi:
        return "write", True, False, fi, now

    if last_fi is None or fi != int(last_fi):
        return "write", True, True, fi, now

    if (
        seen_advance
        and last_advance_t is not None
        and (now - last_advance_t) >= stall_s
    ):
        return "stop", seen_fi, seen_advance, last_fi, last_advance_t
    return "write", seen_fi, seen_advance, last_fi, last_advance_t


def _sync_gt_frame(
    cap: cv2.VideoCapture | None,
    cap_idx: int,
    target_idx: int,
    last_bgr: np.ndarray | None,
) -> tuple[int, np.ndarray | None, bool]:
    """Advance GT video sequentially; seek only on backward jumps."""
    if cap is None or target_idx < 0:
        return cap_idx, last_bgr, False
    if target_idx < cap_idx:
        cap.set(cv2.CAP_PROP_POS_FRAMES, target_idx)
        cap_idx = target_idx
    while cap_idx <= target_idx:
        ok, bgr = cap.read()
        if not ok:
            return cap_idx, last_bgr, False
        last_bgr = bgr
        cap_idx += 1
    return cap_idx, last_bgr, True


def _composite_top_gt_panel(
    main_bgr: np.ndarray,
    panels: list[tuple[np.ndarray, str]],
    *,
    panel_h: int,
) -> np.ndarray:
    """Stack GT camera row above clean sim view (no overlay on main_bgr)."""
    main_h, main_w = main_bgr.shape[:2]
    top = np.zeros((panel_h, main_w, 3), dtype=np.uint8)
    n = max(len(panels), 1)
    slot_w = main_w // n
    for i, (img, label) in enumerate(panels):
        # GT VideoCapture frames are already BGR.
        tile = _to_bgr(img, from_rgb=False)
        ih, iw = tile.shape[:2]
        scale = min((slot_w - 8) / max(iw, 1), (panel_h - 24) / max(ih, 1))
        tw, th = max(1, int(iw * scale)), max(1, int(ih * scale))
        small = cv2.resize(tile, (tw, th), interpolation=cv2.INTER_AREA)
        x0 = i * slot_w + (slot_w - tw) // 2
        y0 = 18 + (panel_h - 18 - th) // 2
        top[y0 : y0 + th, x0 : x0 + tw] = small
        if label:
            cv2.putText(
                top, label, (i * slot_w + 6, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA,
            )
    return np.vstack([top, main_bgr])


def _draw_g1_debug_hud(
    frame_bgr: np.ndarray,
    debug_msg: dict | None,
    *,
    latent_mse: float | None = None,
    gt_frame_idx: int | None = None,
) -> None:
    if debug_msg is None and latent_mse is None:
        return
    y = 24
    if latent_mse is not None:
        cv2.putText(
            frame_bgr,
            f"latent MSE={latent_mse:.4f}"
            + (f"  gt_f={gt_frame_idx}" if gt_frame_idx is not None else ""),
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 220, 255),
            2,
            cv2.LINE_AA,
        )
        y += 26
    if debug_msg is None:
        return
    token = debug_msg.get("token_state")
    if token is not None:
        token = np.asarray(token, dtype=np.float64).reshape(-1)
        if token.size > 0:
            cv2.putText(
                frame_bgr,
                f"SONIC token dim={token.size} [0]={token[0]:+.3f}",
                (8, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (40, 255, 40),
                2,
                cv2.LINE_AA,
            )
            y += 22
            if token.size >= 4:
                cv2.putText(
                    frame_bgr,
                    f"  [1:4]=[{token[1]:+.2f}, {token[2]:+.2f}, {token[3]:+.2f}]",
                    (8, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (40, 255, 40),
                    1,
                    cv2.LINE_AA,
                )
                y += 22
    if "base_trans_target" in debug_msg:
        root = np.asarray(debug_msg["base_trans_target"], dtype=np.float64).reshape(-1)[:3]
        cv2.putText(
            frame_bgr,
            f"SONIC target root z={root[2]:.3f} xy=({root[0]:.2f},{root[1]:.2f})",
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 220, 80),
            1,
            cv2.LINE_AA,
        )
        y += 22
    for key, label in (
        ("last_left_hand_action", "L gripper cmd"),
        ("last_right_hand_action", "R gripper cmd"),
    ):
        if key not in debug_msg:
            continue
        vals = np.asarray(debug_msg[key], dtype=np.float64).reshape(-1)
        if vals.size == 0:
            continue
        preview = ", ".join(f"{v:+.2f}" for v in vals[:3])
        cv2.putText(
            frame_bgr,
            f"{label} [0:3]=[{preview}] max={np.max(np.abs(vals)):.3f}",
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (120, 200, 255),
            1,
            cv2.LINE_AA,
        )
        y += 20
    for key, label in (
        ("left_hand_q", "L hand sim"),
        ("right_hand_q", "R hand sim"),
    ):
        if key not in debug_msg:
            continue
        vals = np.asarray(debug_msg[key], dtype=np.float64).reshape(-1)
        if vals.size == 0:
            continue
        preview = ", ".join(f"{v:+.2f}" for v in vals[:3])
        cv2.putText(
            frame_bgr,
            f"{label} [0:3]=[{preview}] max={np.max(np.abs(vals)):.3f}",
            (8, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (180, 180, 255),
            1,
            cv2.LINE_AA,
        )
        y += 20


def current_pose_frame(message: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Reduce a batched pose message to the recorder's current frame."""
    frame_index = message.get("frame_index")
    if frame_index is None:
        return message
    frame_count = np.asarray(frame_index).reshape(-1).size
    if frame_count <= 1:
        return message

    return {
        key: value[:1]
        if isinstance(value, np.ndarray) and value.ndim > 0 and value.shape[0] == frame_count
        else value
        for key, value in message.items()
    }


def _poll_pose(sock: zmq.Socket | None, topic: str, latest: dict | None) -> dict | None:
    if sock is None:
        return latest
    while True:
        try:
            raw = sock.recv(zmq.NOBLOCK)
        except zmq.Again:
            break
        if raw.startswith(topic.encode("utf-8")):
            latest = current_pose_frame(unpack_pose_message(raw, topic=topic))
    return latest


def _add_pico5_command_markers(
    scene: mujoco.MjvScene,
    pose_msg: dict | None,
    *,
    model: mujoco.MjModel | None = None,
    fk_data: mujoco.MjData | None = None,
) -> None:
    if pose_msg is None or scene.ngeom >= scene.maxgeom:
        return

    if "root_trans" in pose_msg and scene.ngeom < scene.maxgeom:
        root = np.asarray(pose_msg["root_trans"], dtype=np.float64).reshape(-1)[:3]
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.045, 0.0, 0.0],
            pos=root,
            mat=np.eye(3).flatten(),
            rgba=_ROOT_CMD_RGBA,
        )
        scene.ngeom += 1

    # G1 FK reference (ground-aligned robot skeleton from joint_pos in the message).
    if model is not None and fk_data is not None:
        g1_ref = g1_pico5_world_from_pose_msg(pose_msg, model, fk_data)
        if g1_ref is not None:
            for pos in g1_ref:
                if scene.ngeom >= scene.maxgeom:
                    break
                mujoco.mjv_initGeom(
                    scene.geoms[scene.ngeom],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.028, 0.0, 0.0],
                    pos=pos,
                    mat=np.eye(3).flatten(),
                    rgba=_G1_REF_RGBA,
                )
                scene.ngeom += 1

    # Publish path pre-aligns smpl_joints to G1 pelvis-local at stream build time.
    p5 = extract_pico5_world_points(pose_msg)
    if p5 is None:
        return

    for i, pos in enumerate(p5):
        if scene.ngeom >= scene.maxgeom:
            break
        rgba = _PICO5_RGBA[i].copy()
        rgba[3] = 0.75
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.035, 0.0, 0.0],
            pos=pos,
            mat=np.eye(3).flatten(),
            rgba=rgba,
        )
        scene.ngeom += 1


class SimMp4Recorder:
    def __init__(
        self,
        model: mujoco.MjModel,
        output: Path,
        *,
        fps: float = 30.0,
        width: int = 640,
        height: int = 480,
        start_flag: Path | None = None,
        stop_flag: Path | None = None,
        pose_ready_flag: Path | None = None,
        pose_zmq_host: str | None = "127.0.0.1",
        pose_zmq_port: int = 5556,
        pose_zmq_topic: str = "pose",
        draw_command_markers: bool = True,
        g1_debug_host: str | None = None,
        g1_debug_port: int = 5557,
        g1_debug_topic: str = STATE_ZMQ_TOPIC,
        draw_g1_debug_markers: bool = False,
        draw_g1_debug_hud: bool = False,
        camera_host: str | None = None,
        camera_port: int = 5555,
        camera_inset_scale: float = 0.38,
        ego_gt_video: str | Path | None = None,
        wrist_gt_video: str | Path | None = None,
        gt_panel_layout: str = "inset",
        gt_tokens_npz: str | Path | None = None,
        top_panel_height_ratio: float = 0.32,
        ego_sync_pose_zmq_host: str = "127.0.0.1",
        ego_sync_pose_zmq_port: int = 5556,
        ego_inset_scale: float = 0.12,
        snap_sim_env=None,
        snap_fallback_pose: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None,
        snap_on_record_start: bool = False,
        record_init_yaw_rad: float | None = None,
        keep_elastic_band: bool = False,
        world_fixed_camera: bool = False,
        record_video: bool = True,
    ) -> None:
        os.environ.setdefault("MUJOCO_GL", "egl")
        self.output = Path(output)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.start_flag = Path(start_flag) if start_flag else None
        self.stop_flag = Path(stop_flag) if stop_flag else None
        self.pose_ready_flag = Path(pose_ready_flag) if pose_ready_flag else None
        self.frame_interval = 1.0 / max(fps, 1.0)
        self._next_t = 0.0
        self._started = start_flag is None
        self._closed = False
        self._frames = 0
        self._draw_markers = draw_command_markers and pose_zmq_host is not None
        self._g1_debug_subscribe = g1_debug_host is not None
        self._draw_g1_debug_markers = draw_g1_debug_markers and self._g1_debug_subscribe
        self._draw_g1_debug_hud = draw_g1_debug_hud and self._g1_debug_subscribe
        self._snap_on_record_start = snap_on_record_start
        self._g1_debug_topic = g1_debug_topic
        self._latest_g1_debug: dict | None = None
        self._pose_topic = pose_zmq_topic
        self._latest_pose: dict | None = None
        self._debug_reported = False
        self._marker_miss_logged = False
        self._heading_apply = None
        self._pose_ready = False
        self._snap_sim_env = snap_sim_env
        self._snap_fallback_pose = snap_fallback_pose
        self._record_init_yaw_rad = record_init_yaw_rad
        self._keep_elastic_band = keep_elastic_band
        self._world_fixed_camera = world_fixed_camera
        self._record_video = record_video
        self._pelvis_bid = model.body("pelvis").id
        self._zmq_sock: zmq.Socket | None = None
        self._g1_debug_sock: zmq.Socket | None = None
        self._zmq_ctx: zmq.Context | None = None
        self._camera_host = camera_host
        self._camera_port = camera_port
        self._camera_inset_scale = camera_inset_scale
        self._camera_client = None
        self._ego_gt_video = Path(ego_gt_video) if ego_gt_video else None
        self._wrist_gt_video = Path(wrist_gt_video) if wrist_gt_video else None
        self._gt_panel_layout = gt_panel_layout
        self._gt_tokens = None
        if gt_tokens_npz:
            import numpy as _np

            _path = Path(gt_tokens_npz)
            _data = _np.load(_path)
            _tok = _data["tokens"] if "tokens" in _data.files else _data[_data.files[0]]
            self._gt_tokens = _np.asarray(_tok, dtype=_np.float32)
            if self._gt_tokens.ndim != 2 or self._gt_tokens.shape[1] != 64:
                raise ValueError(f"gt_tokens_npz expect (T,64), got {self._gt_tokens.shape}")
            print(f"[sim_record] GT tokens for latent MSE HUD: {_path} T={self._gt_tokens.shape[0]}")
        # ponytail: top layout reserves panel height only when GT video sources exist;
        # otherwise VideoWriter (H+panel) vs render (H) mismatch yields empty mp4.
        has_gt_panel = bool(ego_gt_video or wrist_gt_video)
        self._top_panel_h = (
            max(96, int(height * top_panel_height_ratio))
            if gt_panel_layout == "top" and has_gt_panel
            else 0
        )
        self._ego_cap: cv2.VideoCapture | None = None
        self._wrist_cap: cv2.VideoCapture | None = None
        self._root_track_csv = os.environ.get("ROOT_TRACK_CSV")
        self._root_track_fp = None
        self._kinematic_root = os.environ.get("KINEMATIC_ROOT_REPLAY") == "1"
        root_min_z = os.environ.get("KINEMATIC_ROOT_MIN_Z", "").strip()
        self._kinematic_root_min_z = float(root_min_z) if root_min_z else None
        self._root_assist = (
            os.environ.get("ROOT_ASSIST_REPLAY") == "1" and not self._kinematic_root
        )
        self._root_assist_kp_pos = _env_float("ROOT_ASSIST_KP_POS", 1800.0)
        self._root_assist_kd_pos = _env_float("ROOT_ASSIST_KD_POS", 180.0)
        self._root_assist_kp_ang = _env_float("ROOT_ASSIST_KP_ANG", 900.0)
        self._root_assist_kd_ang = _env_float("ROOT_ASSIST_KD_ANG", 90.0)
        self._root_assist_max_force = _env_float("ROOT_ASSIST_MAX_FORCE", 900.0)
        self._root_assist_max_torque = _env_float("ROOT_ASSIST_MAX_TORQUE", 450.0)
        assist_min_z = os.environ.get("ROOT_ASSIST_MIN_Z", "").strip()
        self._root_assist_min_z = float(assist_min_z) if assist_min_z else None
        self._body_dof = 29
        self._ego_cap_idx = 0
        self._wrist_cap_idx = 0
        self._ego_last_bgr: np.ndarray | None = None
        self._wrist_last_bgr: np.ndarray | None = None
        self._ego_inset_scale = ego_inset_scale
        self._ego_sync_host = ego_sync_pose_zmq_host
        self._ego_sync_port = ego_sync_pose_zmq_port
        self._ego_frame_index = 0
        self._ego_video_offset = int(os.environ.get("EGO_VIDEO_OFFSET", "0"))
        _go_flag = os.environ.get("RECORD_GO_FLAG", "")
        self._record_go_flag = Path(_go_flag) if _go_flag else None
        # Arm GT stream on RECORD_GO (replay ready) so panel clock matches token start.
        self._gt_stream_armed = self._record_go_flag is None
        self._gt_stream_arm_t = time.monotonic() if self._gt_stream_armed else 0.0
        self._gt_local_frame = 0  # legacy; GT index now follows ZMQ/control clock
        self._gt_sync_logged = False
        # GT-panel mp4 gate: skip until first ZMQ frame_index; stall-stop after advance.
        self._zmq_fi_seen = False
        self._zmq_fi_advanced = False
        self._zmq_fi_last: int | None = None
        self._zmq_fi_advance_t: float | None = None
        self._zmq_fi_stall_s = float(os.environ.get("GT_ZMQ_RECORD_STALL_S", "0.4"))

        if (
            self._draw_markers
            or self._g1_debug_subscribe
            or self._ego_gt_video is not None
            or self._wrist_gt_video is not None
            or self._kinematic_root
            or self._root_track_csv
        ):
            self._zmq_ctx = zmq.Context()

        if self._draw_markers:
            self._zmq_sock = self._zmq_ctx.socket(zmq.SUB)
            self._zmq_sock.connect(f"tcp://{pose_zmq_host}:{pose_zmq_port}")
            self._zmq_sock.setsockopt_string(zmq.SUBSCRIBE, pose_zmq_topic)
            print(
                f"[sim_record] ZMQ markers tcp://{pose_zmq_host}:{pose_zmq_port} topic={pose_zmq_topic} "
                f"(SMPL=colored, G1 FK=gray, root=purple)"
            )
        elif (
            self._ego_gt_video is not None
            or self._wrist_gt_video is not None
            or self._kinematic_root
            or self._root_track_csv
        ):
            self._zmq_sock = self._zmq_ctx.socket(zmq.SUB)
            host = self._ego_sync_host
            port = self._ego_sync_port
            self._zmq_sock.connect(f"tcp://{host}:{port}")
            self._zmq_sock.setsockopt_string(zmq.SUBSCRIBE, pose_zmq_topic)
            self._zmq_sock.setsockopt(zmq.CONFLATE, 1)
            label = "ego sync" if self._ego_gt_video or self._wrist_gt_video else "pose track"
            print(f"[sim_record] {label} tcp://{host}:{port} topic={pose_zmq_topic}")
            if self._kinematic_root:
                print("[sim_record] KINEMATIC_ROOT_REPLAY=1 — pin qpos[0:7] to GT each step")
                if self._kinematic_root_min_z is not None:
                    print(
                        "[sim_record] KINEMATIC_ROOT_MIN_Z="
                        f"{self._kinematic_root_min_z:.3f} — clamp visual root height"
                    )
            if self._root_assist:
                print(
                    "[sim_record] ROOT_ASSIST_REPLAY=1 — soft external pelvis force/torque "
                    f"kp_pos={self._root_assist_kp_pos:.1f} kd_pos={self._root_assist_kd_pos:.1f} "
                    f"kp_ang={self._root_assist_kp_ang:.1f} kd_ang={self._root_assist_kd_ang:.1f} "
                    f"max_force={self._root_assist_max_force:.1f} "
                    f"max_torque={self._root_assist_max_torque:.1f}"
                )
                if self._root_assist_min_z is not None:
                    print(
                        "[sim_record] ROOT_ASSIST_MIN_Z="
                        f"{self._root_assist_min_z:.3f} — clamp root assist target height"
                    )

        if self._g1_debug_subscribe:
            self._g1_debug_sock = self._zmq_ctx.socket(zmq.SUB)
            self._g1_debug_sock.connect(f"tcp://{g1_debug_host}:{g1_debug_port}")
            self._g1_debug_sock.setsockopt_string(zmq.SUBSCRIBE, g1_debug_topic)
            self._g1_debug_sock.setsockopt(zmq.CONFLATE, 1)
            if self._draw_g1_debug_markers or self._draw_g1_debug_hud:
                print(
                    f"[sim_record] g1_debug markers tcp://{g1_debug_host}:{g1_debug_port} "
                    f"topic={g1_debug_topic} (cyan=SONIC target, purple=root, HUD=token_state)"
                )
            else:
                print(
                    f"[sim_record] g1_debug snap tcp://{g1_debug_host}:{g1_debug_port} "
                    f"topic={g1_debug_topic} (heading only, no overlay)"
                )
        if self._record_video and self._camera_host and self._ego_gt_video is None:
            from gear_sonic.camera.composed_camera import ComposedCameraClientSensor

            self._camera_client = ComposedCameraClientSensor(
                server_ip=self._camera_host, port=self._camera_port
            )
            print(
                f"[sim_record] ego inset tcp://{self._camera_host}:{self._camera_port} "
                f"(sim camera / VLA closed-loop)"
            )
        if self._record_video and self._ego_gt_video is not None:
            self._ego_cap = cv2.VideoCapture(str(self._ego_gt_video))
            if not self._ego_cap.isOpened():
                raise RuntimeError(f"Cannot open HE GT ego video: {self._ego_gt_video}")
            print(f"[sim_record] HE GT ego {'panel' if self._gt_panel_layout == 'top' else 'inset'} {self._ego_gt_video}")
        if self._record_video and self._wrist_gt_video is not None:
            self._wrist_cap = cv2.VideoCapture(str(self._wrist_gt_video))
            if not self._wrist_cap.isOpened():
                raise RuntimeError(f"Cannot open GT wrist video: {self._wrist_gt_video}")
            print(f"[sim_record] GT wrist panel {self._wrist_gt_video}")

        self._fk_data = mujoco.MjData(model)
        self._model = model

        self._renderer = None
        self._cam = None
        self._writer = None
        if self._record_video:
            out_h = height + self._top_panel_h
            self._renderer = mujoco.Renderer(model, height=height, width=width)
            self._cam = mujoco.MjvCamera()
            self._cam.distance = 3.8
            self._cam.elevation = -15
            # Front-ish (+X-facing robot): SIM_RECORD_CAM_AZIMUTH_OFFSET=90
            # Default 135 = robot-local +Y faces cam (legacy 3/4).
            self._cam_az_offset = float(os.environ.get("SIM_RECORD_CAM_AZIMUTH_OFFSET", "135"))
            self._cam_az_fixed = os.environ.get("SIM_RECORD_CAM_AZIMUTH", "").strip()
            if world_fixed_camera:
                self._cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                if self._cam_az_fixed:
                    self._cam.azimuth = float(self._cam_az_fixed)
                elif record_init_yaw_rad is not None:
                    import math

                    yaw_deg = math.degrees(float(record_init_yaw_rad))
                    self._cam.azimuth = yaw_deg + self._cam_az_offset
                else:
                    self._cam.azimuth = self._cam_az_offset
                print(
                    f"[sim_record] world-fixed camera azimuth={self._cam.azimuth:.1f}° "
                    f"(lookat=pelvis, no yaw tracking)"
                )
            else:
                self._cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                self._cam.trackbodyid = self._pelvis_bid
                self._cam.azimuth = (
                    float(self._cam_az_fixed) if self._cam_az_fixed else self._cam_az_offset
                )
                print(
                    f"[sim_record] tracking camera az_offset={self._cam_az_offset:g} "
                    f"az_fixed={self._cam_az_fixed or 'none'} (front≈offset 90)"
                )

            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(str(self.output), fourcc, fps, (width, out_h))
            if not self._writer.isOpened():
                raise RuntimeError(f"Cannot open video writer: {self.output}")
        else:
            print("[sim_record] video writer disabled; root_track/logging only")

    def _poll_latest_pose(self) -> None:
        if (
            not self._draw_markers
            and self._ego_gt_video is None
            and self._wrist_gt_video is None
            and not self._kinematic_root
            and not self._root_track_csv
        ):
            return
        self._latest_pose = _poll_pose(self._zmq_sock, self._pose_topic, self._latest_pose)
        if self._latest_pose is not None and "frame_index" in self._latest_pose:
            fi = np.asarray(self._latest_pose["frame_index"], dtype=np.int64).reshape(-1)
            if fi.size:
                self._ego_frame_index = int(fi[0])
        if not self._draw_markers:
            return
        if self._pose_ready or self._latest_pose is None:
            return
        if extract_pico5_world_points(self._latest_pose) is None:
            return
        self._pose_ready = True
        if self.pose_ready_flag is not None:
            self.pose_ready_flag.parent.mkdir(parents=True, exist_ok=True)
            self.pose_ready_flag.touch()
        summary = summarize_pose_command(self._latest_pose)
        LOG.info("[sim_record] pose stream ready %s", summary)

    def _poll_latest_g1_debug(self) -> None:
        if not self._g1_debug_subscribe:
            return
        self._latest_g1_debug = _poll_g1_debug(
            self._g1_debug_sock, self._g1_debug_topic, self._latest_g1_debug
        )

    def _ensure_heading_locked(self, sim_quat: np.ndarray) -> None:
        if self._heading_apply is not None:
            return
        ref_quat = None
        if self._latest_pose is not None and "body_quat_w" in self._latest_pose:
            ref_quat = self._latest_pose["body_quat_w"]
        elif self._latest_g1_debug is not None:
            if "base_quat_target" in self._latest_g1_debug:
                ref_quat = self._latest_g1_debug["base_quat_target"]
            elif "base_quat" in self._latest_g1_debug:
                ref_quat = self._latest_g1_debug["base_quat"]
        if ref_quat is None:
            return
        self._heading_apply = lock_deploy_heading_once(None, sim_quat, ref_quat)
        LOG.info(
            "[sim_record] locked deploy heading delta (sim=%.1f° ref=%.1f°)",
            float(np.degrees(calc_deploy_heading_rad(sim_quat))),
            float(np.degrees(calc_deploy_heading_rad(ref_quat))),
        )

    def _snap_to_ground(self, sim_quat: np.ndarray) -> None:
        if self._snap_sim_env is None:
            return
        if self._snap_fallback_pose is None:
            root_quat = None
            if self._record_init_yaw_rad is not None:
                import math

                yaw = float(self._record_init_yaw_rad)
                root_quat = [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]
            self._snap_sim_env.set_standing_pose_on_ground(root_quat=root_quat)
            if (
                not self._keep_elastic_band
                and getattr(self._snap_sim_env, "elastic_band", None) is not None
            ):
                self._snap_sim_env.elastic_band.enable = False
            yaw_note = (
                f" yaw={np.degrees(float(self._record_init_yaw_rad)):.1f}°"
                if self._record_init_yaw_rad is not None
                else ""
            )
            LOG.info(
                "[sim_record] snap default standing pose%s (no PKL fallback, band=%s)",
                yaw_note,
                getattr(getattr(self._snap_sim_env, "elastic_band", None), "enable", None),
            )
            return
        root, quat, dof = self._snap_fallback_pose
        heading_pose = self._latest_pose
        if heading_pose is None and self._latest_g1_debug is not None:
            ref_quat = self._latest_g1_debug.get("base_quat_target")
            if ref_quat is None:
                ref_quat = self._latest_g1_debug.get("base_quat")
            root_trans = self._latest_g1_debug.get("base_trans_target")
            if ref_quat is not None and root_trans is not None:
                heading_pose = {
                    "body_quat_w": ref_quat,
                    "root_trans": root_trans,
                }
        if heading_pose is not None and "body_quat_w" in heading_pose and "root_trans" in heading_pose:
            root, quat, dof = deploy_corrected_snap_pose(
                sim_quat,
                heading_pose,
                root,
                quat,
                dof,
                heading_apply=self._heading_apply,
            )
            LOG.info(
                "[sim_record] snap deploy-corrected pose yaw=%.1f° root_z=%.3f",
                float(np.degrees(calc_deploy_heading_rad(quat))),
                float(root[2]),
            )
        else:
            LOG.info("[sim_record] snap fallback pose root_z=%.3f (no heading correction)", float(root[2]))
        self._snap_sim_env.apply_record_snap_to_ground(
            root, quat, dof, release_elastic_band=not self._keep_elastic_band
        )


    def _joint_cmd_err(self, data: mujoco.MjData) -> tuple[float, float]:
        """Mean/max abs error between sim body qpos and deploy body_q_target."""
        if self._latest_g1_debug is None or "body_q_target" not in self._latest_g1_debug:
            return float("nan"), float("nan")
        cmd = np.asarray(self._latest_g1_debug["body_q_target"], dtype=np.float64).reshape(-1)
        n = min(self._body_dof, cmd.size, data.qpos.size - 7)
        if n <= 0:
            return float("nan"), float("nan")
        sim = np.asarray(data.qpos[7 : 7 + n], dtype=np.float64)
        err = np.abs(sim - cmd[:n])
        return float(err.mean()), float(err.max())

    def _apply_kinematic_root(self, data: mujoco.MjData) -> None:
        pose = self._latest_pose
        if pose is None or "root_trans" not in pose or "body_quat_w" not in pose:
            return
        if self._heading_apply is not None:
            pose = apply_deploy_heading_to_pose_msg(pose, self._heading_apply)
        root = np.asarray(pose["root_trans"], dtype=np.float64).reshape(-1)[:3]
        quat = np.asarray(pose["body_quat_w"], dtype=np.float64).reshape(-1)[:4]
        if self._kinematic_root_min_z is not None:
            root = root.copy()
            root[2] = max(root[2], self._kinematic_root_min_z)
        data.qpos[0:3] = root
        data.qpos[3:7] = quat
        data.qvel[0:6] = 0.0
        mujoco.mj_forward(self._model, data)

    def _apply_root_assist(self, data: mujoco.MjData) -> None:
        pose = self._latest_pose
        data.xfrc_applied[self._pelvis_bid, :] = 0.0
        if pose is None or "root_trans" not in pose or "body_quat_w" not in pose:
            return
        if self._heading_apply is not None:
            pose = apply_deploy_heading_to_pose_msg(pose, self._heading_apply)

        ref_pos = np.asarray(pose["root_trans"], dtype=np.float64).reshape(-1)[:3].copy()
        if self._root_assist_min_z is not None:
            ref_pos[2] = max(ref_pos[2], self._root_assist_min_z)
        ref_quat = np.asarray(pose["body_quat_w"], dtype=np.float64).reshape(-1)[:4]

        pos_err = ref_pos - np.asarray(data.qpos[0:3], dtype=np.float64)
        force = self._root_assist_kp_pos * pos_err - self._root_assist_kd_pos * np.asarray(
            data.qvel[0:3], dtype=np.float64
        )
        force = _clip_norm(force, self._root_assist_max_force)

        cur_rot = R.from_quat(_wxyz_to_xyzw(data.qpos[3:7]))
        ref_rot = R.from_quat(_wxyz_to_xyzw(ref_quat))
        rotvec = (ref_rot * cur_rot.inv()).as_rotvec()
        torque = self._root_assist_kp_ang * rotvec - self._root_assist_kd_ang * np.asarray(
            data.qvel[3:6], dtype=np.float64
        )
        torque = _clip_norm(torque, self._root_assist_max_torque)

        data.xfrc_applied[self._pelvis_bid, 0:3] = force
        data.xfrc_applied[self._pelvis_bid, 3:6] = torque

    def sim_tick(self, data: mujoco.MjData, *, phase: str = "post") -> None:
        """Poll ZMQ; kinematic mode pins root to GT before physics while recording."""
        if phase == "pre":
            self._poll_latest_pose()
            self._poll_latest_g1_debug()
            if self._root_assist:
                if self._heading_apply is None:
                    self._ensure_heading_locked(data.qpos[3:7])
                self._apply_root_assist(data)
            if not self._kinematic_root or not self._started:
                return
            if self._heading_apply is None:
                self._ensure_heading_locked(data.qpos[3:7])
            self._apply_kinematic_root(data)
        elif phase == "post":
            if self._kinematic_root and self._started:
                data.qvel[0:6] = 0.0
            if self._root_assist:
                data.xfrc_applied[self._pelvis_bid, :] = 0.0

    def _maybe_arm_gt_stream(self) -> None:
        if self._gt_stream_armed or self._record_go_flag is None:
            return
        if not self._record_go_flag.is_file():
            return
        self._gt_stream_armed = True
        self._gt_stream_arm_t = time.monotonic()
        self._gt_local_frame = 0
        for cap in (self._ego_cap, self._wrist_cap):
            if cap is not None:
                cap.set(cv2.CAP_PROP_POS_FRAMES, self._ego_video_offset)
        self._ego_cap_idx = self._ego_video_offset
        self._wrist_cap_idx = self._ego_video_offset
        self._ego_last_bgr = None
        self._wrist_last_bgr = None
        print(
            f"[sim_record] GT stream armed at video f={self._ego_video_offset} "
            f"(flag={self._record_go_flag})"
        )

    def _gt_video_target_index(self) -> int:
        """GT panel frame index on the *control* clock (token frame_index), not record fps.

        Prefer ZMQ pose ``frame_index`` so panels track robot tokens. Do **not** free-run
        on wall clock — when infer is late, wall-clock GT races ahead and the robot
        looks slow. Hold last index until the first pose arrives.
        """
        if not self._gt_stream_armed:
            return -1
        if self._latest_pose is not None and "frame_index" in self._latest_pose:
            idx = self._ego_video_offset + max(0, int(self._ego_frame_index))
            if getattr(self, "_gt_sync_mode", "") != "zmq":
                self._gt_sync_mode = "zmq"
                print(f"[sim_record] GT panel sync=zmq_frame_index (f={idx})")
            self._gt_last_idx = idx
            return idx
        # Hold until ZMQ pose arrives (avoids wall-clock race vs slow infer).
        idx = int(getattr(self, "_gt_last_idx", self._ego_video_offset))
        if getattr(self, "_gt_sync_mode", "") != "hold":
            self._gt_sync_mode = "hold"
            print("[sim_record] GT panel sync=hold_until_zmq_frame_index")
        return idx

    def maybe_capture(self, data: mujoco.MjData) -> bool:
        if self._closed:
            return False

        self._poll_latest_pose()
        self._poll_latest_g1_debug()

        if not self._started:
            if self._draw_markers or self._g1_debug_subscribe:
                self._ensure_heading_locked(data.qpos[3:7])
            if self.start_flag is None or not self.start_flag.exists():
                return True
            if self._snap_sim_env is not None and self._snap_on_record_start:
                self._snap_to_ground(data.qpos[3:7])
                self._snap_sim_env = None
                self._snap_fallback_pose = None
            self._started = True
            self._next_t = time.monotonic()
            if self._root_track_csv and self._root_track_fp is None:
                self._root_track_fp = open(self._root_track_csv, "w", encoding="utf-8")
                self._root_track_fp.write(
                    "frame,sim_x,sim_y,sim_z,sim_qw,sim_qx,sim_qy,sim_qz,"
                    "cmd_x,cmd_y,cmd_z,gt_x,gt_y,gt_z,"
                    "joint_mae,joint_max_abs\n"
                )
                print(f"[sim_record] root_track -> {self._root_track_csv}")
            print(f"[sim_record] started -> {self.output}")
        elif self._root_track_csv and self._root_track_fp is None:
            self._root_track_fp = open(self._root_track_csv, "w", encoding="utf-8")
            self._root_track_fp.write(
                "frame,sim_x,sim_y,sim_z,sim_qw,sim_qx,sim_qy,sim_qz,"
                "cmd_x,cmd_y,cmd_z,gt_x,gt_y,gt_z,"
                "joint_mae,joint_max_abs\n"
            )
            print(f"[sim_record] root_track -> {self._root_track_csv}")

        if self.stop_flag is not None and self.stop_flag.exists():
            self.close()
            return False

        if not self._started:
            return True

        now = time.monotonic()
        self._next_t, capture = advance_record_schedule(
            self._next_t, now, self.frame_interval
        )
        if not capture:
            return True

        has_gt_panel = self._ego_gt_video is not None or self._wrist_gt_video is not None
        if has_gt_panel and self._record_video:
            fi = None
            if self._latest_pose is not None and "frame_index" in self._latest_pose:
                fi = int(self._ego_frame_index)
            action, self._zmq_fi_seen, self._zmq_fi_advanced, self._zmq_fi_last, self._zmq_fi_advance_t = (
                gt_zmq_record_gate(
                    fi,
                    seen_fi=self._zmq_fi_seen,
                    seen_advance=self._zmq_fi_advanced,
                    last_fi=self._zmq_fi_last,
                    last_advance_t=self._zmq_fi_advance_t,
                    now=now,
                    stall_s=self._zmq_fi_stall_s,
                )
            )
            if action == "stop":
                print(
                    f"[sim_record] GT ZMQ frame_index stalled "
                    f"(last={self._zmq_fi_last}, stall_s={self._zmq_fi_stall_s:g}); stopping"
                )
                self.close()
                return False
            if action == "skip":
                return True

        if not self._record_video:
            self._maybe_arm_gt_stream()
            self._frames += 1
            if self._root_track_fp is not None:
                sim, sim_quat = root_pose_sample(data.qpos)
                cmd = np.full(3, np.nan)
                gt = np.full(3, np.nan)
                if self._latest_g1_debug is not None and "base_trans_target" in self._latest_g1_debug:
                    cmd = np.asarray(
                        self._latest_g1_debug["base_trans_target"], dtype=np.float64
                    ).reshape(-1)[:3]
                if self._latest_pose is not None and "root_trans" in self._latest_pose:
                    gt = np.asarray(self._latest_pose["root_trans"], dtype=np.float64).reshape(-1)[:3]
                j_mae, j_max = self._joint_cmd_err(data)
                self._root_track_fp.write(
                    f"{self._frames},{sim[0]:.6f},{sim[1]:.6f},{sim[2]:.6f},"
                    f"{sim_quat[0]:.9f},{sim_quat[1]:.9f},"
                    f"{sim_quat[2]:.9f},{sim_quat[3]:.9f},"
                    f"{cmd[0]:.6f},{cmd[1]:.6f},{cmd[2]:.6f},"
                    f"{gt[0]:.6f},{gt[1]:.6f},{gt[2]:.6f},"
                    f"{j_mae:.6f},{j_max:.6f}\n"
                )
                self._root_track_fp.flush()
            if self._frames == 1:
                root_z = float(data.qpos[2])
                LOG.info("[sim_record] frame=1 sim_root_z=%.3f (video disabled)", root_z)
            return True

        if self._draw_markers and not self._debug_reported and self._frames == 0:
            summary = summarize_pose_command(self._latest_pose)
            LOG.info("[sim_record] first pose command: %s", summary)
            if summary.get("ok") and "root_trans" not in summary.get("fields", []):
                LOG.warning("[sim_record] pose stream missing root_trans -> robot may not walk forward")
            self._debug_reported = True
        if (
            self._draw_markers
            and self._latest_pose is not None
            and extract_pico5_world_points(self._latest_pose) is None
            and not self._marker_miss_logged
        ):
            LOG.warning("[sim_record] pose has no smpl_joints PICO5; markers skipped")
            self._marker_miss_logged = True

        if self._world_fixed_camera:
            self._cam.lookat[:] = data.xpos[self._pelvis_bid]
        else:
            if getattr(self, "_cam_az_fixed", ""):
                self._cam.azimuth = float(self._cam_az_fixed)
            else:
                az_off = float(getattr(self, "_cam_az_offset", 135.0))
                self._cam.azimuth = tracking_camera_azimuth(
                    data.qpos[3:7], offset_deg=az_off
                )
        self._renderer.update_scene(data, camera=self._cam)
        if self._draw_markers:
            pose_for_markers = self._latest_pose
            if pose_for_markers is not None and self._heading_apply is not None:
                pose_for_markers = apply_deploy_heading_to_pose_msg(pose_for_markers, self._heading_apply)
            _add_pico5_command_markers(
                self._renderer.scene,
                pose_for_markers,
                model=self._model,
                fk_data=self._fk_data,
            )
        if self._draw_g1_debug_markers:
            g1_debug_viz = _apply_heading_to_g1_debug(self._latest_g1_debug, self._heading_apply)
            _add_g1_debug_command_markers(
                self._renderer.scene,
                g1_debug_viz,
                model=self._model,
                fk_data=self._fk_data,
            )

        rgb = self._renderer.render()
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        # HUD (incl. latent MSE) drawn after gt_idx is known
        if self._camera_client is not None:
            cam_msg = self._camera_client.read(blocking=False)
            if cam_msg is not None and "images" in cam_msg and "ego_view" in cam_msg["images"]:
                _composite_ego_inset(
                    bgr,
                    cam_msg["images"]["ego_view"],
                    scale=self._camera_inset_scale,
                    label="ego_view (sim)",
                )
        self._maybe_arm_gt_stream()
        gt_idx = (
            self._gt_video_target_index()
            if (self._ego_gt_video is not None or self._wrist_gt_video is not None)
            else self._ego_frame_index
        )
        show_gt_labels = os.environ.get("GT_PANEL_LABELS", "1").strip() != "0"
        gt_panels: list[tuple[np.ndarray, str]] = []
        gt_synced = False
        if self._ego_cap is not None and gt_idx >= 0:
            self._ego_cap_idx, self._ego_last_bgr, ok = _sync_gt_frame(
                self._ego_cap,
                self._ego_cap_idx,
                gt_idx,
                self._ego_last_bgr,
            )
            gt_synced = gt_synced or ok
            if ok and self._ego_last_bgr is not None:
                ego_label = f"ego_view (GT f={gt_idx})" if show_gt_labels else ""
                if self._gt_panel_layout == "top":
                    gt_panels.append((self._ego_last_bgr, ego_label))
                else:
                    _composite_ego_inset(
                        bgr,
                        self._ego_last_bgr,
                        scale=self._ego_inset_scale,
                        label=ego_label,
                        from_rgb=False,
                    )
        if self._wrist_cap is not None and gt_idx >= 0:
            self._wrist_cap_idx, self._wrist_last_bgr, ok = _sync_gt_frame(
                self._wrist_cap,
                self._wrist_cap_idx,
                gt_idx,
                self._wrist_last_bgr,
            )
            gt_synced = gt_synced or ok
            if ok and self._wrist_last_bgr is not None:
                wrist_label = f"left_wrist (GT f={gt_idx})" if show_gt_labels else ""
                if self._gt_panel_layout == "top":
                    gt_panels.append((self._wrist_last_bgr, wrist_label))
                else:
                    _composite_ego_inset(
                        bgr,
                        self._wrist_last_bgr,
                        scale=self._ego_inset_scale,
                        label=wrist_label,
                        from_rgb=False,
                    )
        if self._gt_panel_layout == "top" and gt_panels:
            bgr = _composite_top_gt_panel(bgr, gt_panels, panel_h=self._top_panel_h)
        latent_mse = None
        if getattr(self, "_gt_tokens", None) is not None:
            # Prefer commanded pose ZMQ token (always present on v4); g1_debug may omit it.
            tok = None
            if self._latest_pose is not None:
                tok = self._latest_pose.get("token_state")
            if tok is None and self._latest_g1_debug is not None:
                tok = self._latest_g1_debug.get("token_state")
            if tok is not None:
                tok = np.asarray(tok, dtype=np.float64).reshape(-1)
                gi = int(gt_idx) if gt_idx is not None and int(gt_idx) >= 0 else -1
                if tok.size >= 64 and 0 <= gi < len(self._gt_tokens):
                    gt = self._gt_tokens[gi].astype(np.float64)
                    latent_mse = float(np.mean((tok[:64] - gt) ** 2))
        if self._draw_g1_debug_hud:
            _draw_g1_debug_hud(
                bgr,
                self._latest_g1_debug,
                latent_mse=latent_mse,
                gt_frame_idx=int(gt_idx) if gt_idx is not None else None,
            )
        self._writer.write(bgr)
        self._frames += 1
        if gt_synced:
            self._gt_local_frame += 1
        if self._root_track_fp is not None:
            sim, sim_quat = root_pose_sample(data.qpos)
            cmd = np.full(3, np.nan)
            gt = np.full(3, np.nan)
            if self._latest_g1_debug is not None and "base_trans_target" in self._latest_g1_debug:
                cmd = np.asarray(
                    self._latest_g1_debug["base_trans_target"], dtype=np.float64
                ).reshape(-1)[:3]
            if self._latest_pose is not None and "root_trans" in self._latest_pose:
                gt = np.asarray(self._latest_pose["root_trans"], dtype=np.float64).reshape(-1)[:3]
            j_mae, j_max = self._joint_cmd_err(data)
            self._root_track_fp.write(
                f"{self._frames},{sim[0]:.6f},{sim[1]:.6f},{sim[2]:.6f},"
                f"{sim_quat[0]:.9f},{sim_quat[1]:.9f},"
                f"{sim_quat[2]:.9f},{sim_quat[3]:.9f},"
                f"{cmd[0]:.6f},{cmd[1]:.6f},{cmd[2]:.6f},"
                f"{gt[0]:.6f},{gt[1]:.6f},{gt[2]:.6f},"
                f"{j_mae:.6f},{j_max:.6f}\n"
            )
            self._root_track_fp.flush()
        if self._frames == 1:
            root_z = float(data.qpos[2])
            if self._latest_g1_debug is not None and "base_trans_target" in self._latest_g1_debug:
                cmd_root = np.asarray(self._latest_g1_debug["base_trans_target"], dtype=np.float64).reshape(-1)[:3]
                LOG.info(
                    "[sim_record] frame=1 sim_root_z=%.3f cmd_root_z=%.3f",
                    root_z,
                    float(cmd_root[2]),
                )
            else:
                summary = summarize_pose_command(self._latest_pose)
                LOG.info(
                    "[sim_record] frame=1 sim_root_z=%.3f cmd_root=%s",
                    root_z,
                    summary.get("root_trans"),
                )
        return True

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._writer is not None:
            self._writer.release()
        if self._root_track_fp is not None:
            self._root_track_fp.close()
            self._root_track_fp = None
        if self._renderer is not None:
            self._renderer.close()
        if self._zmq_sock is not None:
            self._zmq_sock.close()
        if self._g1_debug_sock is not None:
            self._g1_debug_sock.close()
        if self._camera_client is not None:
            self._camera_client.close()
            self._camera_client = None
        if self._ego_cap is not None:
            self._ego_cap.release()
            self._ego_cap = None
        if self._wrist_cap is not None:
            self._wrist_cap.release()
            self._wrist_cap = None
        if self._zmq_ctx is not None:
            self._zmq_ctx.term()
        print(f"[sim_record] saved {self._frames} frames -> {self.output}")
