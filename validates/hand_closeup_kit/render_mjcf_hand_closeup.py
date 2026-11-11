#!/usr/bin/env python3
"""Render a left/right Revo2 (or MJCF hand) close-up open/close video from a MuJoCo scene XML.

Headless: set MUJOCO_GL=egl (preferred) or osmesa.
Deliverable should be re-encoded to H.264 externally (see AGENT.md in this kit; also docs/notes/g1/mujoco_xml_render_hand_closeup.md).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def _set_q(model, data, name: str, val: float) -> None:
    import mujoco

    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if jid < 0:
        return
    lo, hi = model.jnt_range[jid]
    data.qpos[model.jnt_qposadr[jid]] = float(np.clip(val, lo, hi))


def _body_id(model, *names: str) -> int:
    import mujoco

    for name in names:
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if i >= 0:
            return i
    raise RuntimeError(f"none of bodies found: {names}")


def _hand_joints(model) -> list[tuple[str, int, np.ndarray]]:
    import mujoco

    out: list[tuple[str, int, np.ndarray]] = []
    keys = ("thumb", "index", "middle", "ring", "pinky")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) or ""
        if any(k in name for k in keys):
            out.append((name, model.jnt_qposadr[i], model.jnt_range[i].copy()))
    return out


def _hand_center(data, body_ids: list[int]) -> np.ndarray:
    return np.mean(np.stack([data.xpos[i] for i in body_ids], axis=0), axis=0)


def render_hand_closeup(
    scene_xml: Path,
    out_mp4: Path,
    *,
    fps: int = 30,
    seconds: float = 5.0,
    width: int = 480,
    height: int = 480,
    mid_png: Path | None = None,
) -> dict:
    import mujoco
    import imageio.v2 as imageio

    scene_xml = scene_xml.resolve()
    out_mp4 = out_mp4.resolve()
    out_mp4.parent.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)

    # Bring both hands in front of torso for visibility.
    for name, val in {
        "left_shoulder_pitch_joint": 0.55,
        "right_shoulder_pitch_joint": 0.55,
        "left_shoulder_roll_joint": 0.35,
        "right_shoulder_roll_joint": -0.35,
        "left_shoulder_yaw_joint": 0.15,
        "right_shoulder_yaw_joint": -0.15,
        "left_elbow_joint": 1.45,
        "right_elbow_joint": 1.45,
        "left_wrist_pitch_joint": 0.0,
        "right_wrist_pitch_joint": 0.0,
        "left_wrist_roll_joint": 0.0,
        "right_wrist_roll_joint": 0.0,
        "left_wrist_yaw_joint": 0.0,
        "right_wrist_yaw_joint": 0.0,
    }.items():
        _set_q(model, data, name, val)

    hands = _hand_joints(model)
    if not hands:
        raise RuntimeError(f"no finger joints found in {scene_xml}")

    left_ids = [
        _body_id(model, "left_index_proximal_Link", "left_wrist_yaw_link"),
        _body_id(model, "left_middle_proximal_Link", "left_wrist_yaw_link"),
        _body_id(model, "left_thumb_proximal_Link", "left_wrist_yaw_link"),
        _body_id(model, "left_wrist_yaw_link"),
    ]
    right_ids = [
        _body_id(model, "right_index_proximal_link", "right_wrist_yaw_link"),
        _body_id(model, "right_middle_proximal_link", "right_wrist_yaw_link"),
        _body_id(model, "right_thumb_proximal_link", "right_wrist_yaw_link"),
        _body_id(model, "right_wrist_yaw_link"),
    ]

    # Offscreen framebuffer in this MJCF is often 480; keep within limits.
    height = min(height, 480)
    width = min(width, 480)

    renderer = mujoco.Renderer(model, height=height, width=width)
    nframes = int(round(fps * seconds))
    frames: list[np.ndarray] = []

    cam_l = mujoco.MjvCamera()
    cam_r = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam_l)
    mujoco.mjv_defaultCamera(cam_r)
    cam_l.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam_r.type = mujoco.mjtCamera.mjCAMERA_FREE

    for fi in range(nframes):
        t = fi / max(nframes - 1, 1)
        # Smooth open -> close -> open.
        amt = (1.0 - np.cos(2.0 * np.pi * t)) * 0.5
        for _name, adr, (lo, hi) in hands:
            data.qpos[adr] = lo + amt * (hi - lo)
        mujoco.mj_forward(model, data)

        c_l = _hand_center(data, left_ids)
        c_r = _hand_center(data, right_ids)

        # Left: exterior / frontal. Right: MUST NOT aim along the forearm toward
        # the shoulder (old az≈30 put the hand out of frame).
        cam_l.lookat[:] = c_l
        cam_l.distance = 0.18
        cam_l.azimuth = 160.0
        cam_l.elevation = -20.0

        cam_r.lookat[:] = c_r
        cam_r.distance = 0.18
        cam_r.azimuth = -20.0
        cam_r.elevation = -20.0

        renderer.update_scene(data, camera=cam_l)
        img_l = renderer.render().copy()
        renderer.update_scene(data, camera=cam_r)
        img_r = renderer.render().copy()
        sep = np.full((height, 4, 3), 20, dtype=np.uint8)
        frames.append(np.concatenate([img_l, sep, img_r], axis=1))

    renderer.close()
    imageio.mimsave(out_mp4, frames, fps=fps, quality=8, macro_block_size=1)

    if mid_png is not None:
        import cv2

        mid_png = mid_png.resolve()
        mid_png.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(mid_png), frames[len(frames) // 2][:, :, ::-1])

    rubber = any(
        (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").endswith("rubber_hand")
        for i in range(model.nbody)
    )
    return {
        "scene_xml": str(scene_xml),
        "out_mp4": str(out_mp4),
        "nframes": len(frames),
        "hand_joints": len(hands),
        "nq": int(model.nq),
        "has_rubber_hand_body": rubber,
        "mid_png": str(mid_png) if mid_png is not None else None,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    kit = Path(__file__).resolve().parent
    ap.add_argument(
        "--scene-xml",
        type=Path,
        default=kit / "assets" / "scene_43dof_revo2.xml",
        help="MuJoCo scene XML (default: kit assets/scene_43dof_revo2.xml)",
    )
    ap.add_argument(
        "--out-mp4",
        type=Path,
        default=kit / "out" / "revo2_hand_closeup_raw.mp4",
        help="Raw/intermediate mp4 path",
    )
    ap.add_argument("--mid-png", type=Path, default=kit / "out" / "revo2_hand_closeup_mid.png", help="Optional mid-frame PNG")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--width", type=int, default=480)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    meta = render_hand_closeup(
        args.scene_xml,
        args.out_mp4,
        fps=args.fps,
        seconds=args.seconds,
        width=args.width,
        height=args.height,
        mid_png=args.mid_png,
    )
    for k, v in meta.items():
        print(f"{k}={v}")


if __name__ == "__main__":
    main()
