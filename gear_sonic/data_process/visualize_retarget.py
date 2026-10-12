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
    """Front-view 2D projection for a West-facing (−X) character.

    Camera is to the West (−X), looking East (+X), so the character's front
    faces the camera.  In this view:
      • camera right = −Y  → image right = character's LEFT arm  (standard mirror)
      • up = +Z            → image up
    Project onto YZ plane (ignore X depth).
    """
    pts = pts3d - center
    # −Y → image right (mirrors left/right, standard face-to-face view)
    px = ((-pts[:, 1]) * scale + img_size // 2).astype(int)
    py = (img_size - (pts[:, 2] * scale + img_size * 0.35)).astype(int)
    return np.stack([px, py], axis=1)


def render_soma_frame(frame: dict, img_size: int = 512) -> np.ndarray:
    """Draw SOMA skeleton as colored stick figure on black background."""
    img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

    # Collect all 3D positions
    pts = {name: np.array(val[0]) for name, val in frame.items()
           if not name.endswith("Mod")}

    if len(pts) < 3:
        return img

    all_pos = np.stack(list(pts.values()))
    center = all_pos.mean(axis=0)
    # Dynamic scale: fit skeleton to ~70% of image
    spread = np.ptp(all_pos, axis=0).max()
    scale = (img_size * 0.65) / max(spread, 0.01)

    proj = {name: _proj3d(np.array(pos)[None], img_size, scale, center)[0]
            for name, pos in pts.items()}

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

    cv2.putText(img, "SOMA BVH", (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (200, 200, 200), 1, cv2.LINE_AA)
    return img


# ---------------------------------------------------------------------------
# Render A3 Ultra MuJoCo frame
# ---------------------------------------------------------------------------
class MuJoCoRenderer:
    def __init__(self, xml_path: str, width: int = 512, height: int = 512):
        self.model = mj.MjModel.from_xml_path(xml_path)
        self.data = mj.MjData(self.model)
        self.renderer = mj.Renderer(self.model, height=height, width=width)
        self.width = width
        self.height = height

        # West-side camera (az=0): camera is at the −X (west) side, looking east.
        # The robot now faces west (−X) after the +π yaw-comp fix, so az=0
        # gives a front view.  SOMA's orthographic projection (YZ plane, −Y→right)
        # also corresponds to a camera looking from the west, so both panels
        # render the subject from the same viewpoint.
        self._cam = mj.MjvCamera()
        self._cam.type = mj.mjtCamera.mjCAMERA_FREE
        self._cam.distance = 4.0
        self._cam.azimuth = 0.0
        self._cam.elevation = -15.0
        self._cam.lookat[0] = 0.0
        self._cam.lookat[1] = 0.0
        self._cam.lookat[2] = 0.8  # focus on torso height

    def render(self, qpos: np.ndarray) -> np.ndarray:
        self.data.qpos[:] = qpos
        mj.mj_kinematics(self.model, self.data)
        self.renderer.update_scene(self.data, camera=self._cam)
        img = self.renderer.render()  # (H, W, 3) RGB
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.putText(img_bgr, "A3 Ultra (MuJoCo)", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1, cv2.LINE_AA)
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
        qpos_list, _ = retarget_bvh(bvh_path, fps_target=args.fps)
    print(f"  {len(qpos_list)} qpos frames at {args.fps} fps")

    if is_tmp:
        os.unlink(bvh_path)

    MJCF = str(PROJECT_ROOT / "gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml")
    print(f"Loading MuJoCo model: {MJCF}")
    mj_renderer = MuJoCoRenderer(MJCF, width=args.size, height=args.size)

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

    for i in range(n):
        left  = render_soma_frame(bvh_frames_ds[i], img_size=args.size)
        right = mj_renderer.render(qpos_list[i])

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
