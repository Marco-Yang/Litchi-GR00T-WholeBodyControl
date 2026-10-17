#!/usr/bin/env python3
# ruff: noqa: T201
"""Score the legs of the real-time retarget on a REAL Pico session recorded with ``a3_pose_recorder.py`` (ankle trackers included), offline, no robot.

    PYTHONPATH=. ~/miniconda3/envs/gmr/bin/python gear_sonic_deploy/scripts/a3_eval_pose_recording.py pico_session1.pkl [--foot-rot-weight 8] [--ankle-knee-rot-weight W] [--lower-body track]

It runs the recorded ``pose`` windows through the same retargeter as the deploy chain (same dedupe, same warm start), so every setting can be compared on
the SAME human motion.  Printed:
  * ankle roll / pitch over the still intervals (the human's feet barely move): a plainly standing person should give an ankle roll near 0, not at the
    joint limit (+-0.349), and the share of ALL frames with an ankle roll within 0.05 rad of the limit;
  * hip yaw range (a human walking in place does not turn the hips by +-1 rad);
  * foot tracking: the human foot positions relative to the pelvis (``smpl_joints`` 10 / 11, from the ankle trackers) against the robot foot sites (forward
    kinematics of the retargeted legs): best linear map human -> robot per foot, the residual rms in cm and the R^2, and the delay (cross-correlation of the
    vertical foot motion) in ms.  A good retarget has a small residual and R^2 close to 1 while the feet move.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import pickle
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
MJCF = ROOT / "gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml"
ANKLE_ROLL_LIMIT = 0.349066
LEFT_FOOT, RIGHT_FOOT = 10, 11  # SMPL joint indices (left_foot, right_foot)
HZ = 50.0


def load_pose_messages(path: Path) -> list[tuple[int, bytes]]:
    with open(path, "rb") as f:
        rec = pickle.load(f)
    return [(t, m) for t, m in rec["messages"] if m.startswith(b"pose")]


def retarget_recording(msgs, retargeter) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(q (T,29), human feet (T,2,3), t seconds (T,)) for every new frame of the recorded windows, in order (the A3RefStream logic with the joints kept)."""
    from gear_sonic.utils.teleop.a3.retarget import pose_window_catch_up, unpack_pose_message

    qs, feet, ts = [], [], []
    window, last = None, -1
    for t_ns, raw in msgs:
        msg = unpack_pose_message(raw)
        idx = np.asarray(msg["frame_index"]).reshape(-1)
        n = len(idx)
        if pose_window_catch_up(window, idx):
            last = -1
        window = (int(idx[0]), int(idx[-1]))
        for k in range(n):
            fi = int(idx[k])
            if fi <= last:
                continue
            args = (msg["smpl_pose"][k], msg["body_quat_w"][k], msg["smpl_joints"][k], fi / HZ, 1)
            frame = retargeter.retarget_pico_frame(*args)
            if last < 0:  # first frame of a session: let the warm-started IK settle (as A3RefStream does)
                for _ in range(5):
                    frame = retargeter.retarget_pico_frame(*args)
                retargeter.reset()
                frame = retargeter.retarget_pico_frame(*args)
            last = fi
            if not frame.is_finite():
                continue
            qs.append(frame.q_mj29.copy())
            feet.append(np.asarray(msg["smpl_joints"][k], dtype=np.float64)[[LEFT_FOOT, RIGHT_FOOT]])
            ts.append(t_ns / 1e9 - (n - 1 - k) / HZ)
    return np.array(qs), np.array(feet), np.array(ts)


def robot_feet(q: np.ndarray) -> np.ndarray:
    """(T,2,3) robot left / right foot site positions relative to the pelvis, forward kinematics of the 29 retargeted joints with the pelvis upright."""
    import mujoco as mj

    sys.path.insert(0, str(ROOT))
    from gear_sonic.utils.teleop.a3.frames import MJ29_JOINT_NAMES

    m = mj.MjModel.from_xml_path(str(MJCF))
    d = mj.MjData(m)
    adr = [m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in MJ29_JOINT_NAMES]
    sid = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, n) for n in ("left_foot", "right_foot")]
    pel = mj.mj_name2id(m, mj.mjtObj.mjOBJ_BODY, "pelvis_link")
    out = np.zeros((len(q), 2, 3))
    for i, row in enumerate(q):
        d.qpos[:] = m.qpos0
        d.qpos[:3], d.qpos[3:7] = [0, 0, 1.0], [1, 0, 0, 0]
        d.qpos[adr] = row
        mj.mj_kinematics(m, d)
        for s in range(2):
            out[i, s] = d.site_xpos[sid[s]] - d.xpos[pel]  # pelvis upright and unrotated: world axes = pelvis axes
    return out


def still_mask(feet: np.ndarray, thresh_mps: float = 0.05, win: int = 25) -> np.ndarray:
    """True where both human feet move slower than ``thresh_mps`` (m/s, smoothed over ``win`` frames): the person is standing."""
    v = np.linalg.norm(np.diff(feet, axis=0, prepend=feet[:1]), axis=2) * HZ  # (T,2)
    sm = np.stack([np.convolve(v[:, s], np.ones(win) / win, mode="same") for s in range(2)], 1)
    return np.max(sm, axis=1) < thresh_mps


def linear_fit(h: np.ndarray, r: np.ndarray) -> tuple[float, float]:
    """Best affine map human (T,3) -> robot (T,3): residual rms (m) and R^2 (1 - residual variance / robot variance)."""
    a = np.hstack([h, np.ones((len(h), 1))])
    coef, *_ = np.linalg.lstsq(a, r, rcond=None)
    res = r - a @ coef
    var = np.sum((r - r.mean(0)) ** 2)
    return float(np.sqrt(np.mean(np.sum(res**2, axis=1)))), float(1 - np.sum(res**2) / var) if var > 1e-9 else float("nan")


def lag_ms(h: np.ndarray, r: np.ndarray, max_lag: int = 25) -> float:
    """Delay (ms) of the robot vertical foot motion relative to the human's, by cross-correlation; positive = robot lags."""
    h, r = h - h.mean(), r - r.mean()
    if h.std() < 1e-6 or r.std() < 1e-6:
        return float("nan")
    best = (-2.0, 0)
    for lag in range(-max_lag, max_lag + 1):
        x = h[max(0, -lag): len(h) - max(0, lag)]
        y = r[max(0, lag): len(r) - max(0, -lag)]
        c = np.corrcoef(x, y)[0, 1]
        if c > best[0]:
            best = (c, lag)
    return best[1] / HZ * 1000.0


def report(q: np.ndarray, hfeet: np.ndarray) -> dict:
    rfeet = robot_feet(q)
    still = still_mask(hfeet)
    out = dict(frames=len(q), seconds=len(q) / HZ, still_share=float(still.mean()))
    sat = np.abs(q[:, [22, 28]]) > ANKLE_ROLL_LIMIT - 0.05
    out["ankle_roll_sat_all"] = float(sat.mean())
    if still.sum() > 50:
        out["still_ankle_roll_mean"] = q[still][:, [22, 28]].mean(0)
        out["still_ankle_roll_sat"] = float(sat[still].mean())
        out["still_ankle_pitch_mean"] = q[still][:, [21, 27]].mean(0)
    out["hip_yaw_range"] = (float(q[:, [19, 25]].min()), float(q[:, [19, 25]].max()))
    moving = ~still
    for s, nm in ((0, "left"), (1, "right")):
        h, r = hfeet[:, s], rfeet[:, s]
        if moving.sum() > 100:
            res, r2 = linear_fit(h[moving], r[moving])
            out[f"{nm}_foot_resid_cm"], out[f"{nm}_foot_r2"] = res * 100, r2
            out[f"{nm}_foot_lag_ms"] = lag_ms(np.linalg.norm(h - h.mean(0), axis=1)[moving], np.linalg.norm(r - r.mean(0), axis=1)[moving])
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", type=Path)
    ap.add_argument("--foot-rot-weight", type=float, default=None)
    ap.add_argument("--ankle-knee-rot-weight", type=float, default=None)
    ap.add_argument("--lower-body", default="track")
    ap.add_argument("--start", type=float, default=0.0, help="skip this many seconds of the recording")
    ap.add_argument("--end", type=float, default=0.0)
    a = ap.parse_args()
    from gear_sonic.utils.teleop.a3.retarget import A3RealtimeRetargeter

    msgs = load_pose_messages(a.recording)
    if not msgs:
        print("no pose messages in the recording")
        return 1
    t0 = msgs[0][0]
    msgs = [(t, m) for t, m in msgs if (t - t0) / 1e9 >= a.start and (a.end <= 0 or (t - t0) / 1e9 <= a.end)]
    ret = A3RealtimeRetargeter(foot_rot_weight=a.foot_rot_weight, ankle_knee_rot_weight=a.ankle_knee_rot_weight, lower_body=a.lower_body)
    q, hfeet, _ = retarget_recording(msgs, ret)
    r = report(q, hfeet)
    print(f"{a.recording.name}: {r['frames']} frames = {r['seconds']:.1f} s; foot_rot_weight={a.foot_rot_weight} ankle_knee_rot_weight={a.ankle_knee_rot_weight}")
    print(f"  feet still {r['still_share'] * 100:.0f}% of the time")
    if "still_ankle_roll_mean" in r:
        print(f"  STILL: ankle roll mean L/R {r['still_ankle_roll_mean'][0]:+.3f}/{r['still_ankle_roll_mean'][1]:+.3f} rad (limit +-{ANKLE_ROLL_LIMIT:.3f}), at the limit in "
              f"{r['still_ankle_roll_sat'] * 100:.0f}% of the still frames; ankle pitch mean L/R {r['still_ankle_pitch_mean'][0]:+.3f}/{r['still_ankle_pitch_mean'][1]:+.3f}")
    print(f"  ankle roll within 0.05 rad of the limit in {r['ankle_roll_sat_all'] * 100:.1f}% of ALL frames; hip yaw range {r['hip_yaw_range'][0]:+.2f} .. {r['hip_yaw_range'][1]:+.2f} rad")
    for nm in ("left", "right"):
        if f"{nm}_foot_resid_cm" in r:
            print(f"  {nm} foot while moving: residual {r[f'{nm}_foot_resid_cm']:.1f} cm, R^2 {r[f'{nm}_foot_r2']:.2f}, delay {r[f'{nm}_foot_lag_ms']:+.0f} ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
