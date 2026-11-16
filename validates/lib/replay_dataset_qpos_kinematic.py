#!/usr/bin/env python3
"""Kinematic-only MuJoCo replay of LeRobot episode observation.state (qpos).

Maps joints by name into MJCF (body 29). No physics, no SONIC, no hands remap
(Dex3 names in data ≠ Revo2 in scene — hands stay at model default).

Root: xyz fixed at (0,0,root_z); quat from observation.root_orientation each frame.
Use this to visually check joint order / sign conventions vs your recollection.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
import pyarrow.parquet as pq


_DEFAULT_MJCF = Path(
    "/home/zhenyu/workspace/code/HumanoidWBM/demo/RL_controller/"
    "Litchi-GR00T-WholeBodyControl/gear_sonic_deploy/g1/g1_29dof_with_revo2.xml"
)
_DEFAULT_FFMPEG = Path("/home/zhenyu/workspace/code/HumanoidWBM/demo/VLA/.tools/bin/ffmpeg")


def _load_episode(ds: Path, episode: int) -> tuple[np.ndarray, np.ndarray, list[str], float]:
    info = json.loads((ds / "meta" / "info.json").read_text())
    names = list(info["features"]["observation.state"]["names"])
    fps = float(info.get("fps", 50))
    table = pq.read_table(ds / "data" / "chunk-000" / f"episode_{episode:06d}.parquet")
    state = np.stack(table.column("observation.state").to_pylist()).astype(np.float64)
    if "observation.root_orientation" in table.column_names:
        root_quat = np.stack(table.column("observation.root_orientation").to_pylist()).astype(np.float64)
    else:
        root_quat = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (state.shape[0], 1))
    return state, root_quat, names, fps


def _joint_qpos_map(model: mujoco.MjModel, names: list[str]) -> list[tuple[int, int]]:
    """Return list of (state_idx, qpos_adr) for names present in MJCF."""
    out: list[tuple[int, int]] = []
    missing: list[str] = []
    for i, name in enumerate(names):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            missing.append(name)
            continue
        out.append((i, int(model.jnt_qposadr[jid])))
    if missing:
        print(f"[qpos_replay] skip {len(missing)} joints not in MJCF (likely hands): {missing[:6]}...", flush=True)
    print(f"[qpos_replay] mapped {len(out)}/{len(names)} joints by name", flush=True)
    return out


def _read_rgb_frame(cap, last: np.ndarray | None) -> np.ndarray | None:
    if cap is None:
        return last
    ok, bgr = cap.read()
    if not ok:
        return last
    import cv2

    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def _stack_dual_cam(
    main_rgb: np.ndarray,
    panels: list[tuple[np.ndarray, str]],
    *,
    panel_h: int,
    below: bool = True,
) -> np.ndarray:
    import cv2

    main_h, main_w = main_rgb.shape[:2]
    row = np.zeros((panel_h, main_w, 3), dtype=np.uint8)
    n = max(len(panels), 1)
    slot_w = main_w // n
    for i, (img, label) in enumerate(panels):
        tile = np.asarray(img)
        ih, iw = tile.shape[:2]
        scale = min((slot_w - 8) / max(iw, 1), (panel_h - 24) / max(ih, 1))
        tw, th = max(1, int(iw * scale)), max(1, int(ih * scale))
        small = cv2.resize(tile, (tw, th), interpolation=cv2.INTER_AREA)
        x0 = i * slot_w + (slot_w - tw) // 2
        y0 = 18 + (panel_h - 18 - th) // 2
        row[y0 : y0 + th, x0 : x0 + tw] = small
        if label:
            cv2.putText(
                row,
                label,
                (i * slot_w + 6, 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )
    return np.vstack([main_rgb, row] if below else [row, main_rgb])


def render(
    ds: Path,
    episode: int,
    out_mp4: Path,
    *,
    mjcf: Path,
    root_z: float,
    fps_override: float | None,
    max_frames: int,
    ffmpeg: Path,
    width: int = 640,
    height: int = 480,
    dual_cam: bool = True,
    dual_cam_below: bool = True,
    cam_panel_h: int = 160,
) -> Path:
    state, root_quat, names, fps = _load_episode(ds, episode)
    if fps_override is not None:
        fps = float(fps_override)
    if max_frames > 0:
        state = state[:max_frames]
        root_quat = root_quat[:max_frames]
    t = int(state.shape[0])

    # Resolve relative mesh paths from MJCF directory.
    mjcf = mjcf.resolve()
    cwd = Path.cwd()
    os.chdir(mjcf.parent)
    try:
        model = mujoco.MjModel.from_xml_path(str(mjcf))
    finally:
        os.chdir(cwd)

    data = mujoco.MjData(model)
    mapping = _joint_qpos_map(model, names)

    renderer = mujoco.Renderer(model, height=height, width=width)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance = 3.2
    cam.elevation = -12.0
    cam.azimuth = 135.0

    ego_cap = head_cap = None
    ego_last = head_last = None
    if dual_cam:
        import cv2

        ep_pad = f"{int(episode):06d}"
        ego_p = ds / "videos/chunk-000/observation.images.ego_view" / f"episode_{ep_pad}.mp4"
        head_p = ds / "videos/chunk-000/observation.images.head" / f"episode_{ep_pad}.mp4"
        if ego_p.is_file():
            ego_cap = cv2.VideoCapture(str(ego_p))
            print(f"[qpos_replay] dual-cam ego ← {ego_p}", flush=True)
        if head_p.is_file():
            head_cap = cv2.VideoCapture(str(head_p))
            print(f"[qpos_replay] dual-cam head ← {head_p}", flush=True)

    out_h = height + (cam_panel_h if dual_cam and (ego_cap or head_cap) else 0)
    out_mp4 = Path(out_mp4)
    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    ff = str(ffmpeg) if ffmpeg.is_file() else "ffmpeg"
    proc = subprocess.Popen(
        [
            ff, "-y", "-loglevel", "error",
            "-f", "rawvideo", "-vcodec", "rawvideo",
            "-pix_fmt", "rgb24", "-s", f"{width}x{out_h}", "-r", str(fps), "-i", "-",
            "-an", "-vcodec", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            str(out_mp4),
        ],
        stdin=subprocess.PIPE,
    )
    assert proc.stdin is not None

    # elbow indices in state (body naming)
    try:
        i_le = names.index("left_elbow_joint")
        i_re = names.index("right_elbow_joint")
        i_lsp = names.index("left_shoulder_pitch_joint")
        i_rsp = names.index("right_shoulder_pitch_joint")
    except ValueError:
        i_le = i_re = i_lsp = i_rsp = -1

    for i in range(t):
        q = model.qpos0.copy()
        q[0:3] = [0.0, 0.0, float(root_z)]
        q[3:7] = root_quat[i]
        for si, adr in mapping:
            q[adr] = state[i, si]
        data.qpos[:] = q
        mujoco.mj_forward(model, data)
        cam.lookat[:] = [0.0, 0.0, float(root_z)]
        renderer.update_scene(data, camera=cam)
        frame = renderer.render()

        # simple HUD via numpy (green text would need PIL; stamp numbers in corner as bars)
        # Use a tiny overlay: encode key angles as colored pixels strip at top.
        # Prefer readable text via OpenCV if available; else raw frame.
        try:
            import cv2

            img = frame.copy()
            le = float(state[i, i_le]) if i_le >= 0 else 0.0
            re = float(state[i, i_re]) if i_re >= 0 else 0.0
            lsp = float(state[i, i_lsp]) if i_lsp >= 0 else 0.0
            rsp = float(state[i, i_rsp]) if i_rsp >= 0 else 0.0
            lines = [
                f"ep{episode:03d} fi={i}/{t - 1}  kinematic observation.state",
                f"L_elbow={le:+.3f}rad ({np.degrees(le):+.1f}deg)  R_elbow={re:+.3f}rad ({np.degrees(re):+.1f}deg)",
                f"L_sh_pitch={lsp:+.3f}  R_sh_pitch={rsp:+.3f}  root_z={root_z:.3f} (xy fixed)",
            ]
            y = 22
            for line in lines:
                cv2.putText(img, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 80), 1, cv2.LINE_AA)
                y += 18
            if dual_cam and (ego_cap is not None or head_cap is not None):
                panels: list[tuple[np.ndarray, str]] = []
                ego_last = _read_rgb_frame(ego_cap, ego_last)
                head_last = _read_rgb_frame(head_cap, head_last)
                if ego_last is not None:
                    panels.append((ego_last, f"ego_view (GT f={i})"))
                if head_last is not None:
                    panels.append((head_last, f"head (GT f={i})"))
                if panels:
                    img = _stack_dual_cam(
                        img, panels, panel_h=cam_panel_h, below=dual_cam_below
                    )
            proc.stdin.write(img.tobytes())
        except Exception:
            proc.stdin.write(frame.tobytes())

        if i % 50 == 0 or i == t - 1:
            print(f"[qpos_replay] frame {i}/{t - 1}", flush=True)

    if ego_cap is not None:
        ego_cap.release()
    if head_cap is not None:
        head_cap.release()
    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError(f"ffmpeg failed writing {out_mp4}")
    print(f"[ok] {out_mp4} frames={t} fps={fps} mjcf={mjcf.name}", flush=True)
    return out_mp4


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dataset", type=Path, required=True)
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--mjcf", type=Path, default=_DEFAULT_MJCF)
    ap.add_argument("--root-z", type=float, default=0.793)
    ap.add_argument("--fps", type=float, default=None)
    ap.add_argument("--max-frames", type=int, default=0, help="0 = all frames")
    ap.add_argument("--ffmpeg", type=Path, default=_DEFAULT_FFMPEG)
    ap.add_argument("--no-dual-cam", action="store_true", help="Disable ego+head strip")
    ap.add_argument(
        "--dual-cam-above",
        action="store_true",
        help="Put ego+head above MuJoCo (default: below)",
    )
    args = ap.parse_args()
    render(
        args.dataset,
        args.episode,
        args.out,
        mjcf=args.mjcf,
        root_z=args.root_z,
        fps_override=args.fps,
        max_frames=args.max_frames,
        ffmpeg=args.ffmpeg,
        dual_cam=not bool(args.no_dual_cam),
        dual_cam_below=not bool(args.dual_cam_above),
    )


if __name__ == "__main__":
    main()
