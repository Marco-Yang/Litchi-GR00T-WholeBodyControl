#!/usr/bin/env python3
"""Side-by-side visualization: SOMA BVH skeleton vs A3 Ultra MuJoCo retarget.

Renders both animations frame-by-frame and saves as an MP4 (or GIF) so you
can verify that the retargeted robot pose matches the human motion.

Usage:
    # From a local BVH file
    conda run -n gmr python gear_sonic/data_process/visualize_retarget.py \
        --bvh data/walk_test.bvh --out /tmp/retarget_check.mp4

    # From tar (streams; specify the BVH name inside the tar)
    conda run -n gmr python gear_sonic/data_process/visualize_retarget.py \
        --tar data/bones_seed/soma_proportional.tar.gz \
        --bvh_name walking_on_edge_180_R_001__A410.bvh \
        --out /tmp/walking_on_edge_check.mp4

    # Show live (requires X11 display)
    conda run -n gmr python gear_sonic/data_process/visualize_retarget.py \
        --bvh data/walk_test.bvh --live
"""

import argparse
import os
import sys
import tarfile
import tempfile
from pathlib import Path

import cv2
import mujoco as mj
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
GMR_ROOT = Path.home() / "gmr_repo"
if str(GMR_ROOT) not in sys.path:
    sys.path.insert(0, str(GMR_ROOT))

from gear_sonic.data_process.batch_retarget_bvh_soma_a3ultra import (
    load_soma_bvh_frames,
    retarget_bvh,
    make_retargeter,
)

# ---------------------------------------------------------------------------
# SOMA skeleton connections (parent → child in the BVH hierarchy)
# ---------------------------------------------------------------------------
SOMA_EDGES = [
    ("Hips", "Chest"),
    ("Chest", "Spine"),
    ("Spine", "Head"),
    ("Hips", "LeftLeg"),
    ("LeftLeg", "LeftShin"),
    ("LeftShin", "LeftFoot"),
    ("LeftFoot", "LeftToeBase"),
    ("Hips", "RightLeg"),
    ("RightLeg", "RightShin"),
    ("RightShin", "RightFoot"),
    ("RightFoot", "RightToeBase"),
    ("Chest", "LeftArm"),
    ("LeftArm", "LeftForeArm"),
    ("LeftForeArm", "LeftHand"),
    ("Chest", "RightArm"),
    ("RightArm", "RightForeArm"),
    ("RightForeArm", "RightHand"),
]

# Color palette (BGR)
COL_SPINE   = (255, 200, 100)
COL_LEFT    = (100, 200, 255)
COL_RIGHT   = (100, 255, 150)
COL_JOINT   = (255, 255, 255)

EDGE_COLORS = {
    "Hips":       COL_SPINE,
    "Chest":      COL_SPINE,
    "Spine":      COL_SPINE,
    "LeftLeg":    COL_LEFT,
    "LeftShin":   COL_LEFT,
    "LeftFoot":   COL_LEFT,
    "RightLeg":   COL_RIGHT,
    "RightShin":  COL_RIGHT,
    "RightFoot":  COL_RIGHT,
    "LeftArm":    COL_LEFT,
    "LeftForeArm":COL_LEFT,
    "RightArm":   COL_RIGHT,
    "RightForeArm":COL_RIGHT,
}


# ---------------------------------------------------------------------------
# Render SOMA skeleton frame
# ---------------------------------------------------------------------------
def _proj3d(pts3d: np.ndarray, img_size: int, scale: float, center: np.ndarray) -> np.ndarray:
    """Front-view 2D projection for an East-facing (+X) character.

    Same viewpoint as the MuJoCo az=180 camera: at +X looking toward −X, so
    image right = +Y = the character's LEFT side. Projects onto YZ (ignores X).
    """
    pts = pts3d - center
    px = (pts[:, 1] * scale + img_size // 2).astype(int)
    py = (img_size - (pts[:, 2] * scale + img_size * 0.35)).astype(int)
    return np.stack([px, py], axis=1)


def motion_label(vel_xy: np.ndarray, left_xy: np.ndarray, min_speed: float = 0.05) -> str:
    """Describe a displacement in the body's own frame (left_xy = left-side vector)."""
    if np.linalg.norm(vel_xy) < min_speed:
        return "moving: still"
    left = left_xy / max(np.linalg.norm(left_xy), 1e-9)
    fwd = np.array([left[1], -left[0]])
    lat, lon = float(vel_xy @ left), float(vel_xy @ fwd)
    if abs(lat) >= abs(lon):
        return "moving: to its LEFT" if lat > 0 else "moving: to its RIGHT"
    return "moving: FORWARD" if lon > 0 else "moving: BACKWARD"


def _put_bottom_text(img: np.ndarray, text: str):
    if text:
        cv2.putText(img, text, (10, img.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 255), 1, cv2.LINE_AA)


def render_soma_frame(frame: dict, img_size: int = 512, motion_text: str = "",
                      view: tuple = None) -> np.ndarray:
    """Draw SOMA skeleton as colored stick figure on black background.

    view = (px_per_m, floor_row): fixed scale and floor line (e.g. from
    MuJoCoRenderer.soma_view) so crouches and jumps read the same as in the
    robot panel. Without it the figure is re-fitted to the panel every frame.
    """
    img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

    # "Root" is a synthetic joint that stays at the BVH origin.
    pts = {name: np.array(val[0]) for name, val in frame.items()
           if not name.endswith("Mod") and name != "Root"}

    if len(pts) < 3:
        return img

    if view is not None:
        px_per_m, floor_row = view
        hips_y = pts["Hips"][1]
        proj = {name: np.array([int(img_size / 2 + (p[1] - hips_y) * px_per_m),
                                int(floor_row - p[2] * px_per_m)])
                for name, p in pts.items()}
        cv2.line(img, (0, int(floor_row)), (img_size, int(floor_row)), (70, 70, 70), 1)
    else:
        all_pos = np.stack(list(pts.values()))
        center = all_pos.mean(axis=0)
        spread = np.ptp(all_pos, axis=0).max()
        scale = (img_size * 0.65) / max(spread, 0.01)
        proj = {name: _proj3d(np.array(pos)[None], img_size, scale, center)[0]
                for name, pos in pts.items()}
        cv2.putText(img, "auto-zoom", (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)

    # Draw edges
    for src, dst in SOMA_EDGES:
        if src not in proj or dst not in proj:
            continue
        p1, p2 = tuple(proj[src]), tuple(proj[dst])
        color = EDGE_COLORS.get(src, COL_SPINE)
        cv2.line(img, p1, p2, color, 3, cv2.LINE_AA)

    # Draw joints
    for name, p in proj.items():
        cv2.circle(img, tuple(p), 4, COL_JOINT, -1, cv2.LINE_AA)

    # Stick figure has no depth cue, so label sides explicitly.
    for name, tag, col in (("LeftHand", "L", COL_LEFT), ("LeftFoot", "L", COL_LEFT),
                           ("RightHand", "R", COL_RIGHT), ("RightFoot", "R", COL_RIGHT)):
        if name in proj:
            x, y = proj[name]
            cv2.putText(img, tag, (int(x) + 8, int(y) + 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, col, 2, cv2.LINE_AA)

    cv2.putText(img, "SOMA BVH (front view)", (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (200, 200, 200), 1, cv2.LINE_AA)
    _put_bottom_text(img, motion_text)
    return img


# ---------------------------------------------------------------------------
# Render A3 Ultra MuJoCo frame
# ---------------------------------------------------------------------------
class MuJoCoRenderer:
    def __init__(self, xml_path: str, width: int = 512, height: int = 512, title: str = "A3 Ultra (front view)"):
        self.title = title
        self.model = mj.MjModel.from_xml_path(xml_path)
        self.data = mj.MjData(self.model)
        self.renderer = mj.Renderer(self.model, height=height, width=width)
        self.width = width
        self.height = height

        # az=180: camera at +X looking −X — front view of the East-facing robot,
        # the same viewpoint as the SOMA panel (_proj3d).
        self._cam = mj.MjvCamera()
        self._cam.type = mj.mjtCamera.mjCAMERA_FREE
        self._cam.distance = 4.0
        self._cam.azimuth = 180.0
        self._cam.elevation = -15.0
        self._cam.lookat[0] = 0.0
        self._cam.lookat[1] = 0.0
        self._cam.lookat[2] = 0.8  # focus on torso height

    def soma_view(self, human_scale: float = 1.0):
        """(px_per_m, floor_row) matching this camera at the tracked root.

        human_scale (the retarget scale) shrinks the human to robot size so
        the two panels compare one-to-one.
        """
        f_px = 0.5 * self.height / np.tan(np.radians(self.model.vis.global_.fovy) / 2)
        el, d, h = np.radians(self._cam.elevation), self._cam.distance, self._cam.lookat[2]
        return f_px / d * human_scale, self.height / 2 + f_px * h * np.cos(el) / (d - h * np.sin(el))

    def body_pos(self, name: str) -> np.ndarray:
        return self.data.xpos[mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_BODY, name)].copy()

    def _project(self, p: np.ndarray):
        """World point → pixel, using the GL camera of the last update_scene."""
        c = self.renderer.scene.camera[0]
        fwd, up = np.array(c.forward), np.array(c.up)
        d = p - np.array(c.pos)
        z = d @ fwd
        if z <= 1e-6:
            return None
        x = (d @ np.cross(fwd, up)) * c.frustum_near / z
        y = (d @ up) * c.frustum_near / z
        # frustum_width == 0 means "derive from viewport aspect" (MuJoCo convention).
        half_w = c.frustum_width or 0.5 * (c.frustum_top - c.frustum_bottom) * self.width / self.height
        u = (x - (c.frustum_center - half_w)) / (2 * half_w) * self.width
        v = (c.frustum_top - y) / (c.frustum_top - c.frustum_bottom) * self.height
        return int(u), int(v)

    def render(self, qpos: np.ndarray, motion_text: str = "") -> np.ndarray:
        self.data.qpos[:] = qpos
        mj.mj_kinematics(self.model, self.data)
        self._cam.lookat[0] = qpos[0]
        self._cam.lookat[1] = qpos[1]
        self.renderer.update_scene(self.data, camera=self._cam)
        img = self.renderer.render()  # (H, W, 3) RGB
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        for body, tag, col in (("left_wrist_yaw_Link", "L", COL_LEFT), ("left_ankle_roll_Link", "L", COL_LEFT),
                               ("right_wrist_yaw_Link", "R", COL_RIGHT), ("right_ankle_roll_Link", "R", COL_RIGHT)):
            uv = self._project(self.body_pos(body))
            if uv is not None:
                cv2.putText(img_bgr, tag, (uv[0] + 8, uv[1] + 6), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, col, 2, cv2.LINE_AA)
        cv2.putText(img_bgr, self.title, (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1, cv2.LINE_AA)
        _put_bottom_text(img_bgr, motion_text)
        return img_bgr

    def close(self):
        self.renderer.close()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def load_bvh(args) -> str:
    """Return path to a temporary or permanent BVH file."""
    if args.bvh and os.path.exists(args.bvh):
        return args.bvh, False
    if args.tar and args.bvh_name:
        with tarfile.open(args.tar, "r|gz") as tf:
            for m in tf:
                if args.bvh_name in m.name:
                    fobj = tf.extractfile(m)
                    tmp = tempfile.NamedTemporaryFile(
                        suffix=".bvh", delete=False, mode="wb")
                    tmp.write(fobj.read()); tmp.flush(); tmp.close()
                    return tmp.name, True
        raise FileNotFoundError(f"{args.bvh_name} not found in tar")
    raise ValueError("Provide --bvh FILE or --tar TAR --bvh_name NAME")


def main():
    parser = argparse.ArgumentParser(description="BVH↔robot visualizer (separate or combined)")
    parser.add_argument("--bvh",  help="Path to local BVH file")
    parser.add_argument("--tar",  help="Path to tar.gz archive")
    parser.add_argument("--bvh_name", help="BVH filename inside tar to extract")
    parser.add_argument("--out",  default="",
                        help="Combined side-by-side output path (.mp4); empty = skip combined")
    parser.add_argument("--out_soma", default="",
                        help="SOMA-only output path (.mp4); empty = skip")
    parser.add_argument("--out_robot", default="",
                        help="Robot-only MuJoCo output path (.mp4); empty = skip")
    parser.add_argument("--fps",  type=int, default=30)
    parser.add_argument("--max_frames", type=int, default=300,
                        help="Cap output frames (default 300 = 10s at 30fps)")
    parser.add_argument("--live", action="store_true",
                        help="Show live with cv2.imshow (requires X11)")
    parser.add_argument("--size", type=int, default=400,
                        help="Width/height of each panel in pixels (MuJoCo offscreen max 480)")
    args = parser.parse_args()

    # Default to combined if no output specified
    if not args.out and not args.out_soma and not args.out_robot:
        args.out = "/tmp/retarget_check.mp4"

    bvh_path, is_tmp = load_bvh(args)
    print(f"Loading BVH: {bvh_path}")

    frames, _, fps_src = load_soma_bvh_frames(bvh_path)
    print(f"  {len(frames)} frames at {fps_src} fps")

    print("Running retarget...")
    import io
    from contextlib import redirect_stdout
    buf = io.StringIO()
    with redirect_stdout(buf):
        qpos_list, retargeter = retarget_bvh(bvh_path, fps_target=args.fps)
    print(f"  {len(qpos_list)} qpos frames at {args.fps} fps")

    if is_tmp:
        os.unlink(bvh_path)

    MJCF = str(PROJECT_ROOT / "gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml")
    print(f"Loading MuJoCo model: {MJCF}")
    mj_renderer = MuJoCoRenderer(MJCF, width=args.size, height=args.size)
    view = mj_renderer.soma_view(retargeter.human_scale_table["Hips"])

    # Match BVH frames to retarget frames (same stride)
    stride = max(1, round(fps_src / args.fps))
    bvh_frames_ds = [frames[i] for i in range(0, len(frames), stride)]

    n = min(len(bvh_frames_ds), len(qpos_list), args.max_frames)
    print(f"Rendering {n} frames...")

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer_combined = (cv2.VideoWriter(args.out, fourcc, args.fps,
                                       (args.size * 2, args.size))
                       if args.out else None)
    writer_soma  = (cv2.VideoWriter(args.out_soma,  fourcc, args.fps,
                                    (args.size, args.size))
                    if args.out_soma else None)
    writer_robot = (cv2.VideoWriter(args.out_robot, fourcc, args.fps,
                                    (args.size, args.size))
                    if args.out_robot else None)

    # Per-frame "moving to its LEFT/RIGHT" labels, each in its own body frame.
    soma_hips = np.array([f["Hips"][0][:2] for f in bvh_frames_ds[:n]])
    soma_left = np.array([np.asarray(f["LeftLeg"][0][:2]) - np.asarray(f["RightLeg"][0][:2])
                          for f in bvh_frames_ds[:n]])
    robot_root = np.array([q[:2] for q in qpos_list[:n]])
    robot_left = []
    for q in qpos_list[:n]:
        mj_renderer.data.qpos[:] = q
        mj.mj_kinematics(mj_renderer.model, mj_renderer.data)
        robot_left.append((mj_renderer.body_pos("left_hip_yaw_Link")
                           - mj_renderer.body_pos("right_hip_yaw_Link"))[:2])
    robot_left = np.array(robot_left)
    win = max(1, args.fps // 3)

    def _label(pos, left_vec, i):
        a, b = max(0, i - win), min(n - 1, i + win)
        return motion_label(pos[b] - pos[a], left_vec[i])

    for i in range(n):
        left  = render_soma_frame(bvh_frames_ds[i], img_size=args.size,
                                  motion_text=_label(soma_hips, soma_left, i), view=view)
        right = mj_renderer.render(qpos_list[i], motion_text=_label(robot_root, robot_left, i))

        if writer_soma:
            writer_soma.write(left)
        if writer_robot:
            writer_robot.write(right)
        if writer_combined:
            combined = np.hstack([left, right])
            cv2.putText(combined, f"frame {i+1}/{n}", (args.size - 90, args.size - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            writer_combined.write(combined)
        if args.live:
            cv2.imshow("BVH vs A3 Ultra", np.hstack([left, right]))
            if cv2.waitKey(int(1000 / args.fps)) & 0xFF == ord("q"):
                break

        if (i + 1) % 50 == 0:
            print(f"  {i+1}/{n} frames rendered")

    for w in (writer_combined, writer_soma, writer_robot):
        if w:
            w.release()
    mj_renderer.close()
    if args.live:
        cv2.destroyAllWindows()
    if args.out:
        print(f"Combined saved: {args.out}")
    if args.out_soma:
        print(f"SOMA saved:     {args.out_soma}")
    if args.out_robot:
        print(f"Robot saved:    {args.out_robot}")

    # Also print some per-frame stats for quick sanity check
    import sys as _sys
    _sys.path.insert(0, str(GMR_ROOT))
    from gear_sonic.data_process.convert_gmr_a3ultra_to_motion_lib import convert_sequence
    motion = convert_sequence(qpos_list, fps=float(args.fps))
    dof = motion["dof"]
    kl = np.degrees(dof[:, 22])  # L_knee
    kr = np.degrees(dof[:, 28])  # R_knee
    print(f"\nRetarget stats (first {n} frames):")
    print(f"  L_knee: mean={kl[:n].mean():+.1f}°  std={kl[:n].std():.1f}°  "
          f"min={kl[:n].min():.1f}°  max={kl[:n].max():.1f}°")
    print(f"  R_knee: mean={kr[:n].mean():+.1f}°  std={kr[:n].std():.1f}°  "
          f"min={kr[:n].min():.1f}°  max={kr[:n].max():.1f}°")
    stuck = (kl[:n].std() < 0.5 and kl[:n].mean() < -5) or \
            (kr[:n].std() < 0.5 and kr[:n].mean() < -5)
    print(f"  IK status: {'STUCK at lower limit ← PROBLEM' if stuck else 'OK ✓'}")


if __name__ == "__main__":
    main()
