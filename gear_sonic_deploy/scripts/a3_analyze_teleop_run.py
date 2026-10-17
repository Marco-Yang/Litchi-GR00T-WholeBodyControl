#!/usr/bin/env python3
# ruff: noqa: T201
"""Read-only analysis of one teleop run of the A3 deploy: the per-tick dump (and, optionally, the full-rate relay recording).

    python3 gear_sonic_deploy/scripts/a3_analyze_teleop_run.py dump_teleop_real.bin [--record run.npz] [--skip 4] [--stand-tol 0.08]

``dump_teleop_real.bin`` is written by the deploy when ``logging.dump_ticks`` is > 0 in the runtime config (one record per POLICY tick, 50 Hz, only
while the policy runs, i.e. in teleop: follow and policy stand; not in PD stand).  Layout (a3_deploy/main.cpp, "Binary dump layout"):
obs[1570] f32, raw_action[29] f32, q_des[29] f64, jp[29] f64 (measured joint position), jv[29] f64, gdir[3] f64 (projected gravity), bav[3] f64
(base angular velocity); joint order = mj29 (waist 0-2, left arm 3-9, right arm 10-16, left leg 17-22, right leg 23-28).

It labels every tick "follow" or "stand" from the REFERENCE the policy received (decoded from obs[0:290], see decode_reference): a tick is "stand"
when the reference of all 29 joints equals the deploy's policy-stand reference (the mean of teleop_motions/BMD_..._stand_Skeleton0.csv) to
within ``--stand-tol`` (0.08 rad), else "follow".  (Labelling from the commanded elbows, my first version, was wrong: the real operator's reference
elbows were ~0.2 rad, not the ~1.5 rad of the synthetic test person, so the labels came out reversed.)  It cuts the run into segments and prints per
segment what matters for the sway / shaking question: measured ankle / knee mean, base pitch / roll sway (amplitude and dominant frequency), base
angular velocity, leg command jitter, the largest steady command-vs-measured error of a leg joint (a joint that is commanded far from where it can go
is pushing against something: planted foot, limit), the transient after each switch, and (with --record) the mean and peak leg torques against the
MJCF limits.  The same script run on a MuJoCo dump gives the baseline to compare against.
"""

from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import re
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml"
HZ = 50.0
REC = np.dtype([("obs", "<f4", 1570), ("act", "<f4", 29), ("qdes", "<f8", 29), ("jp", "<f8", 29), ("jv", "<f8", 29), ("gdir", "<f8", 3), ("bav", "<f8", 3)])
LEG = slice(17, 29)
LEG_NAMES = ["L_hip_p", "L_hip_r", "L_hip_y", "L_knee", "L_ank_p", "L_ank_r", "R_hip_p", "R_hip_r", "R_hip_y", "R_knee", "R_ank_p", "R_ank_r"]
# IsaacLab joint order -> mj29: isaac position k holds mj joint PERM[k] (a3_policy_parameters.hpp, a3_mujoco_to_isaaclab)
PERM = np.array([17, 23, 0, 18, 24, 1, 19, 25, 2, 20, 26, 3, 10, 21, 27, 4, 11, 22, 28, 5, 12, 6, 13, 7, 14, 8, 15, 9, 16])
STAND_CSV = ROOT / "gear_sonic_deploy/dist/a3_deploy_x86_64/teleop_motions/BMD_0319_a3_filtered_20260319_164305__stand_Skeleton0.csv"
SDK_LEG = slice(19, 31)  # the same twelve leg joints in the relay's 31-slot order


def load_dump(path: Path) -> np.ndarray:
    size = Path(path).stat().st_size
    n = size // REC.itemsize
    if n == 0:
        raise SystemExit(f"{path}: empty (the deploy writes the dump only while the policy runs; was dump_ticks > 0 and the run clean-exited with Ctrl-C?)")
    if size % REC.itemsize:
        print(f"note: {path} has {size % REC.itemsize} trailing bytes (an unfinished last record: the deploy was killed); using {n} whole records")
    return np.memmap(path, dtype=REC, mode="r", shape=(n,))


def decode_reference(obs: np.ndarray) -> np.ndarray:
    """(T, 10, 29) reference joint positions in mj29 order, frames 0..9 (frame 0 = now).  The first 580 floats of the observation are the future command:
    290 positions (10 frames x 29 joints, IsaacLab order) then 290 velocities (verified against the measured arms of a real run: 0.037 rad at frames 0 and 9)."""
    pos = np.asarray(obs[:, :290], dtype=np.float64).reshape(-1, 10, 29)
    ref = np.zeros_like(pos)
    ref[..., PERM] = pos
    return ref


def stand_reference(csv_path: Path = STAND_CSV) -> np.ndarray | None:
    """Mean pose (rad, mj29 order) of the policy-stand CSV, or None when the file is not there."""
    import csv
    import math

    if not Path(csv_path).exists():
        return None
    rows = list(csv.reader(open(csv_path)))
    hdr, data = rows[0], [[float(x) for x in r] for r in rows[1:] if r]
    names = ["waist_yaw", "waist_roll", "waist_pitch", "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll",
             "left_wrist_pitch", "left_wrist_yaw", "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll",
             "right_wrist_pitch", "right_wrist_yaw", "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
             "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll"]
    return np.array([np.mean([math.radians(r[hdr.index(n + "_joint")]) for r in data]) for n in names])


def label_ticks(ref0: np.ndarray, stand: np.ndarray | None, tol: float = 0.08) -> np.ndarray:
    """True = follow, False = policy stand.  ``ref0`` (T, 29) is the reference of the current frame; without a stand reference everything is follow."""
    if stand is None:
        return np.ones(len(ref0), dtype=bool)
    return np.max(np.abs(ref0 - stand), axis=1) >= tol


def segments_from_labels(follow: np.ndarray, min_ticks: int = 100) -> list[tuple[int, int, str]]:
    segs, start = [], 0
    for i in range(1, len(follow) + 1):
        if i == len(follow) or follow[i] != follow[start]:
            segs.append((start, i, "follow" if follow[start] else "stand"))
            start = i
    merged: list[tuple[int, int, str]] = []
    for s in segs:  # a label flicker shorter than min_ticks is absorbed by its neighbour
        if merged and (s[1] - s[0] < min_ticks or merged[-1][2] == s[2]):
            merged[-1] = (merged[-1][0], s[1], merged[-1][2])
        else:
            merged.append(s)
    if len(merged) > 1 and merged[0][1] - merged[0][0] < min_ticks:
        merged[1] = (merged[0][0], merged[1][1], merged[1][2])
        merged.pop(0)
    return merged


def sway(gdir: np.ndarray) -> dict:
    g = gdir / np.maximum(np.linalg.norm(gdir, axis=1, keepdims=True), 1e-9)
    pitch, roll = np.degrees(np.arcsin(np.clip(g[:, 0], -1, 1))), np.degrees(np.arcsin(np.clip(g[:, 1], -1, 1)))
    f = float("nan")
    if len(pitch) >= 100:
        x = pitch - pitch.mean()
        spec = np.abs(np.fft.rfft(x * np.hanning(len(x))))
        fr = np.fft.rfftfreq(len(x), 1.0 / HZ)
        ok = fr >= 0.2
        if ok.any() and spec[ok].max() > 0:
            f = float(fr[ok][np.argmax(spec[ok])])
    return dict(pitch_std=float(pitch.std()), pitch_ptp=float(np.ptp(pitch)), roll_std=float(roll.std()), roll_ptp=float(np.ptp(roll)), pitch_freq=f)


def mjcf_leg_limits(path: Path = MJCF) -> np.ndarray:
    """Actuator force limit (N*m) of the twelve leg joints in the mj29 leg order, nan where the MJCF has none."""
    text = path.read_text()
    names = ["left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
             "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll"]
    out = np.full(12, np.nan)
    for i, n in enumerate(names):
        m = re.search(rf'<joint\b[^>]*\bname="{n}_joint"[^>]*>', text)
        r = re.search(r'actuatorfrcrange="\s*[-+\d.eE]+\s+([-+\d.eE]+)\s*"', m.group(0)) if m else None
        if r:
            out[i] = float(r.group(1))
    return out


def align_to_record(jp_leg: np.ndarray, rec_q_leg: np.ndarray, window: int = 300) -> tuple[np.ndarray, float]:
    """Map every dump tick to a row of the 200 Hz record by matching the measured leg joints.  Returns (row index per tick, residual rad)."""
    n = len(jp_leg)
    step = 4
    sub = rec_q_leg[::step]
    w = min(window, n)

    def best(ticks: slice, lo: int, hi: int) -> tuple[int, float]:
        ref = jp_leg[ticks]
        bi, be = lo, np.inf
        for k in range(max(lo, 0), max(min(hi, len(sub) - len(ref)), lo) + 1):
            e = float(np.mean(np.abs(sub[k:k + len(ref)] - ref)))
            if e < be:
                bi, be = k, e
        return bi, be

    k0, e0 = best(slice(0, w), 0, len(sub) - n)
    k1, e1 = best(slice(n - w, n), max(0, k0 + n - w - 100), k0 + n - w + 100)
    # tick i -> sub row k0 + i + (k1 - (k0 + n - w)) * i / (n - w): linear drift correction between the two anchors
    drift = (k1 - (k0 + (n - w))) / max(n - w, 1)
    sub_idx = k0 + np.arange(n) * (1.0 + drift)
    rows = np.clip(np.round(sub_idx).astype(int) * step, 0, len(rec_q_leg) - 1)
    return rows, float(max(e0, e1))


HEADER = ROOT / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_policy_parameters.hpp"
ANKLE_KP_A3_024 = 60.0  # policy profile a3_024 (the deploy log prints ankle_kp=60), the header table still has the older 50


def policy_leg_kps(header: Path = HEADER) -> np.ndarray | None:
    """Policy Kp of the twelve leg joints (N*m/rad, mj29 leg order) from the deploy's parameter header, ankles at the a3_024 value; None if unreadable."""
    try:
        m = re.search(r"constexpr\s+std::array<double,\s*29>\s+a3_kps\s*=\s*\{(.*?)\};", Path(header).read_text(), flags=re.S)
        vals = [float(x) for x in re.findall(r"[-+]?\d+(?:\.\d*)?", re.sub(r"//[^\n]*", "", m.group(1)))]
        kps = np.array(vals[17:29], dtype=np.float64)
    except (OSError, AttributeError, ValueError):
        return None
    if kps.shape != (12,):
        return None
    kps[[4, 5, 10, 11]] = ANKLE_KP_A3_024
    return kps


def worst_leg_error(qd: np.ndarray, jp: np.ndarray, kps: np.ndarray | None = None, limits: np.ndarray | None = None) -> str:
    """The leg joint with the largest MEAN command-minus-measured, as the torque it implies (error x policy Kp) against the actuator limit, e.g.
    'L_hip_r +0.52 rad = 62 N*m (28% of limit)'.  A joint commanded far from where it is carries that torque steadily: planted foot, joint limit.
    A large angle on a soft joint (ankle, Kp 60) is just gravity compensation; the torque share tells the two apart.  Without Kp / limits it falls
    back to the angle alone (flag above 0.3 rad)."""
    e = (qd[:, LEG] - jp[:, LEG]).mean(axis=0)
    if kps is not None and limits is not None and np.all(np.isfinite(limits)):
        share = np.abs(e) * kps / limits
        j = int(np.argmax(share))
        return f"{LEG_NAMES[j]} {e[j]:+.2f} rad = {abs(e[j]) * kps[j]:.0f} N*m ({share[j] * 100:.0f}% of limit)" + ("  <== LARGE" if share[j] > 0.25 else "")
    j = int(np.argmax(np.abs(e)))
    return f"{LEG_NAMES[j]} {e[j]:+.2f} rad" + ("  <== LARGE" if abs(e[j]) > 0.3 else "")


KPS = policy_leg_kps()
LIMITS = mjcf_leg_limits()


def describe(d: np.ndarray, segs, skip_s: float, rec=None, rows=None, limits=None) -> None:
    qd, jp, jv, gdir, bav = d["qdes"], d["jp"], d["jv"], d["gdir"], d["bav"]
    sk = int(skip_s * HZ)
    print(f"{len(d)} policy ticks = {len(d) / HZ:.1f} s; segments (tick ranges are 50 Hz policy ticks; the first {skip_s:g} s of each are skipped for the settled statistics):")
    hdr = (f"{'#':>2s} {'kind':6s} {'secs':>6s} | {'L ank r':>8s} {'R ank r':>8s} {'L ank p':>8s} {'R ank p':>8s} {'L knee':>7s} {'R knee':>7s} | "
           f"{'pitch std':>9s} {'ptp':>6s} {'f Hz':>5s} {'roll std':>8s} {'bav rms':>8s} | {'qdes jit':>8s} {'track rms':>9s} {'ank jv':>7s} | {'worst leg cmd error (mean) -> torque':>40s}")
    print(hdr)
    for i, (a, b, kind) in enumerate(segs):
        a2 = min(a + sk, b - 1)
        s = slice(a2, b)
        if b - a2 < 10:
            print(f"{i:2d} {kind:6s} {(b - a) / HZ:6.1f} | (too short for statistics)")
            continue
        sw = sway(gdir[s])
        jit = float(np.sqrt(np.mean(np.diff(qd[s][:, LEG], axis=0) ** 2)) * HZ)
        trk = float(np.sqrt(np.mean((qd[s][:, LEG] - jp[s][:, LEG]) ** 2)))
        print(f"{i:2d} {kind:6s} {(b - a) / HZ:6.1f} | {jp[s, 22].mean():+8.3f} {jp[s, 28].mean():+8.3f} {jp[s, 21].mean():+8.3f} {jp[s, 27].mean():+8.3f} "
              f"{jp[s, 20].mean():+7.3f} {jp[s, 26].mean():+7.3f} | {sw['pitch_std']:9.2f} {sw['pitch_ptp']:6.1f} {sw['pitch_freq']:5.2f} {sw['roll_std']:8.2f} "
              f"{float(np.sqrt(np.mean(bav[s] ** 2))):8.3f} | {jit:8.3f} {trk:9.3f} {float(np.sqrt(np.mean(jv[s][:, [22, 28]] ** 2))):7.3f} | {worst_leg_error(qd[s], jp[s], KPS, LIMITS)}")
    print("\nunits: ankle / knee in rad (measured), base pitch / roll in degrees (from the projected gravity), bav rad/s, qdes jit = rms of the leg command")
    print("difference per tick x 50 (rad/s), track rms = rms(leg q_des - measured) rad, ank jv = rms ankle-roll speed rad/s.")
    print("\nthe 3 s after each switch (max over the twelve leg joints):")
    print(f"  {'switch':22s} {'max |d q_des|/tick':>19s} {'max |jv|':>9s} {'max |q_des - q|':>16s} {'bav max':>8s}")
    for i in range(1, len(segs)):
        a = segs[i][0]
        w = slice(a, min(a + int(3 * HZ), len(d)))
        step = float(np.max(np.abs(np.diff(qd[w][:, LEG], axis=0)))) if w.stop - w.start > 1 else float("nan")
        print(f"  {segs[i - 1][2] + ' -> ' + segs[i][2]:22s} {step:19.3f} {float(np.max(np.abs(jv[w][:, LEG]))):9.2f} "
              f"{float(np.max(np.abs(qd[w][:, LEG] - jp[w][:, LEG]))):16.3f} {float(np.max(np.linalg.norm(bav[w], axis=1))):8.2f}")
    if rec is not None and rows is not None:
        print("\nleg torque from the relay recording (N*m), per segment: max |tau| and the share of ticks above 80% of the MJCF actuator limit:")
        tau = rec["tau"][:, SDK_LEG]
        for i, (a, b, kind) in enumerate(segs):
            r = rows[min(a + sk, b - 1):b]
            if len(r) < 10:
                continue
            t = np.abs(tau[r])
            mean_share = [f"{LEG_NAMES[j]} {np.mean(t[:, j]) / limits[j] * 100:.0f}%" for j in range(12) if np.isfinite(limits[j]) and np.mean(t[:, j]) / limits[j] > 0.15]
            if mean_share:
                print(f"  {i:2d} {kind:6s} STEADY torque above 15% of the limit (mean |tau|): " + ", ".join(mean_share))
            hot = [f"{LEG_NAMES[j]} {np.mean(t[:, j] > 0.8 * limits[j]) * 100:.0f}%" for j in range(12) if np.isfinite(limits[j]) and np.mean(t[:, j] > 0.8 * limits[j]) > 0.01]
            print(f"  {i:2d} {kind:6s} max |tau| " + " ".join(f"{LEG_NAMES[j]} {t[:, j].max():.0f}" for j in (3, 4, 5, 9, 10, 11)) + ("   >80% limit: " + ", ".join(hot) if hot else ""))


def foot_task_space(ref_legs: np.ndarray, meas_legs: np.ndarray) -> dict | None:
    """Foot positions relative to the pelvis (forward kinematics, MJCF) of the REFERENCE legs against the MEASURED legs: how well the policy executes the foot
    motion it is given.  Per foot: rms residual (cm), the vertical range of the reference vs the executed foot (cm: a step the policy flattens shows here) and
    the delay (ms).  Returns None when mujoco is not importable."""
    try:
        spec = importlib.util.spec_from_file_location("a3_eval_pose_recording", Path(__file__).with_name("a3_eval_pose_recording.py"))
        ev = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ev)

        def full(q12: np.ndarray) -> np.ndarray:
            q = np.zeros((len(q12), 29))
            q[:, 17:29] = q12
            return q

        fr, fm = ev.robot_feet(full(ref_legs)), ev.robot_feet(full(meas_legs))
    except ImportError:
        return None
    out = {}
    for s, nm in ((0, "L"), (1, "R")):
        e = fm[:, s] - fr[:, s]
        out[nm] = dict(resid=float(np.sqrt(np.mean(np.sum(e**2, axis=1))) * 100), xyz=np.sqrt(np.mean(e**2, axis=0)) * 100,
                       lift_ref=float(np.ptp(fr[:, s, 2]) * 100), lift_meas=float(np.ptp(fm[:, s, 2]) * 100),
                       lag=ev.lag_ms(np.linalg.norm(fr[:, s] - fr[:, s].mean(0), axis=1), np.linalg.norm(fm[:, s] - fm[:, s].mean(0), axis=1)))
    return out


def print_feet(d: np.ndarray, ref: np.ndarray, segs, skip_s: float) -> None:
    jp = np.asarray(d["jp"])
    print("\nfoot execution in task space (pelvis-relative foot position, reference legs vs measured legs; the policy flattens or delays what it cannot do):")
    for i, (a, b, kind) in enumerate(segs):
        a2 = a + int(skip_s * HZ)
        if kind != "follow" or b - a2 < 250:
            continue
        r = foot_task_space(ref[a2:b, 0, 17:29], jp[a2:b, 17:29])
        if r is None:
            print("  (mujoco not importable: skipped)")
            return
        for nm in "LR":
            v = r[nm]
            print(f"  seg {i:2d} {nm} foot: residual {v['resid']:5.1f} cm (x/y/z rms {v['xyz'][0]:.1f}/{v['xyz'][1]:.1f}/{v['xyz'][2]:.1f}), vertical range reference {v['lift_ref']:.0f} cm vs executed {v['lift_meas']:.0f} cm, delay {v['lag']:+.0f} ms")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump", type=Path)
    ap.add_argument("--record", type=Path, help="the .npz written by a3_relay_tool.py record (adds the torque statistics)")
    ap.add_argument("--stand-tol", type=float, default=0.08, help="max |reference - policy-stand reference| over the 29 joints for a tick to count as policy stand (rad)")
    ap.add_argument("--stand-csv", type=Path, default=STAND_CSV, help="the policy-stand reference CSV of the deploy package")
    ap.add_argument("--skip", type=float, default=4.0, help="seconds skipped at the start of every segment for the settled statistics")
    a = ap.parse_args()
    d = load_dump(a.dump)
    stand = stand_reference(a.stand_csv)
    if stand is None:
        print(f"note: {a.stand_csv} not found: cannot tell follow from policy stand, the whole run is one segment")
    ref = decode_reference(np.asarray(d["obs"]))
    segs = segments_from_labels(label_ticks(ref[:, 0, :], stand, a.stand_tol))
    rec = rows = limits = None
    if a.record:
        rec = dict(np.load(a.record))
        rows, resid = align_to_record(np.asarray(d["jp"])[:, LEG], rec["q"][:, SDK_LEG])
        print(f"alignment of the dump to the relay recording: residual {resid:.4f} rad ({'ok' if resid < 0.01 else 'POOR: the torque statistics below are unreliable'})\n")
        limits = mjcf_leg_limits()
    describe(d, segs, a.skip, rec, rows, limits)
    print_feet(d, ref, segs, a.skip)
    return 0


if __name__ == "__main__":
    sys.exit(main())
