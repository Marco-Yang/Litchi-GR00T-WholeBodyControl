#!/usr/bin/env python3
# ruff: noqa: T201
"""Score one MuJoCo run that replayed a recorded Pico session: falls, how well the legs execute the reference, base sway and foot slip.  Read-only.

    PYTHONPATH=. ~/miniconda3/envs/gmr/bin/python gear_sonic_deploy/scripts/a3_score_sim_replay.py --csv sim_x.csv --dump dump_sim_x.bin [--label name] [--json out.json]

Inputs are the two files a replay run leaves: the sim log (``A3_SIM_LOG_CSV``: sim time, pelvis pose, joint positions at 50 Hz of SIM time) and the deploy
dump (``logging.dump_ticks``: one record per policy tick at 50 Hz of WALL time).  They are lined up by matching the measured leg / waist joints (the dump
has ``jp``, the csv has the same joints): rows != ticks when the sim runs slower than real time, so a row number is never used as a tick number.
The match quality (mean joint difference, rad) is printed; above 0.03 the alignment is not trusted.

Printed, over the FOLLOW ticks (the reference differs from the policy-stand reference, see a3_analyze_teleop_run.py), the first ``--skip`` s dropped:
  falls      the sim resets (robot fell, band re-enabled) and rows with the pelvis below 0.6 m, inside the follow window
  legs       rms of (measured - reference frame 0) per joint group; large values are normal while a foot is planted and the reference asks otherwise
  feet       foot positions relative to the pelvis (forward kinematics) of the reference legs vs the measured legs: residual (cm), vertical range of the
             reference vs the executed foot (a step the policy flattens or a foot it retracts shows here), delay (ms)
  sway       base tilt (from the projected gravity) std / 95th percentile, and the peak base angular velocity
  slip       world-frame sliding of a planted foot (lowest 2 cm of its travel, contiguous >= 0.3 s, first / last 0.1 s dropped): median / 90th percentile
             (cm) over all stance periods and the share of periods that slid more than 3 cm
Compare runs of the SAME recording; a single run is noisy (about 10 % on the sway numbers of a still stand).
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
SIM_MJCF = ROOT / "gear_sonic/data/robot_model/model_data/a3_ultra/a3_ultra_t3d0.xml"
HZ = 50.0
LEG, WAIST = slice(17, 29), slice(0, 3)


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).with_name(f"{name}.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load_sim_csv(path: Path, joint_names: list[str]):
    """(t, pelvis pos (R,3), pelvis quat wxyz (R,4), q (R,29) in mj29 order); rows cut off by a kill are dropped."""
    with open(path) as f:
        rows = list(csv.reader(f))
    hdr = rows[0]
    s = np.array([[float(x) for x in r] for r in rows[1:] if len(r) == len(hdr)])
    c = {h: i for i, h in enumerate(hdr)}
    return s[:, 0], s[:, [c["px"], c["py"], c["pz"]]], s[:, [c["qw"], c["qx"], c["qy"], c["qz"]]], s[:, [c[f"q:{n}"] for n in joint_names]]


def align_rows(csv_q: np.ndarray, jp: np.ndarray, joints=LEG, n_samples: int = 120) -> tuple[float, float, float]:
    """(row of tick 0, rows per tick, mean |csv - dump| in rad): the offset and rate that make the csv joints follow the dump's measured joints best."""
    t_n, r_n = len(jp), len(csv_q)
    ks = np.linspace(0, t_n - 1, n_samples).astype(int)
    target = jp[ks][:, joints].astype(np.float32)
    q = csv_q[:, joints].astype(np.float32)
    best = (np.inf, 0.0, 0.0)
    # A windowed GUI sim can run at ~0.8 rows per policy tick.  A search that stops at 0.90 then clamps and reports a large residual.
    for ratio in np.arange(0.50, 1.2001, 0.01):
        step = np.round(ratio * ks).astype(int)
        cand = np.arange(0, r_n - int(step[-1]))
        if len(cand) == 0:
            continue
        err = np.empty(len(cand), dtype=np.float32)
        for lo in range(0, len(cand), 1000):  # chunks: C x samples x joints floats would not fit at once
            idx = cand[lo : lo + 1000, None] + step[None, :]
            err[lo : lo + 1000] = np.abs(q[idx] - target[None]).mean(axis=(1, 2))
        j = int(np.argmin(err))
        if err[j] < best[0]:
            best = (float(err[j]), float(cand[j]), float(ratio))
    return best[1], best[2], best[0]


def stance_slip(foot_xyz: np.ndarray, min_rows: int = 15, trim: int = 5, band: float = 0.02) -> list[float]:
    """World-frame sliding path (m) of every stance period of one foot: z within ``band`` of the foot's lowest height, contiguous >= ``min_rows`` rows."""
    z = foot_xyz[:, 2]
    low = z < np.percentile(z, 2) + band
    out, i = [], 0
    while i < len(low):
        if not low[i]:
            i += 1
            continue
        j = i
        while j < len(low) and low[j]:
            j += 1
        if j - i >= min_rows:
            seg = foot_xyz[i + trim : j - trim, :2]
            if len(seg) > 1:
                out.append(float(np.linalg.norm(np.diff(seg, axis=0), axis=1).sum()))
        i = j
    return out


def foot_world(pos: np.ndarray, quat: np.ndarray, q: np.ndarray, joint_names: list[str]) -> np.ndarray:
    """(R,2,3) world positions of the left / right foot sites from the logged pelvis pose and joint positions."""
    import mujoco as mj

    m = mj.MjModel.from_xml_path(str(SIM_MJCF))
    d = mj.MjData(m)
    adr = [m.jnt_qposadr[mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)] for n in joint_names]
    sid = [mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, n) for n in ("left_foot", "right_foot")]
    out = np.zeros((len(q), 2, 3))
    for i in range(len(q)):
        d.qpos[:] = m.qpos0
        d.qpos[:3], d.qpos[3:7] = pos[i], quat[i]
        d.qpos[adr] = q[i]
        mj.mj_kinematics(m, d)
        out[i] = d.site_xpos[sid]
    return out


def score(csv_path: Path, dump_path: Path, skip_s: float = 4.0) -> dict:
    from gear_sonic.utils.teleop.a3.frames import MJ29_JOINT_NAMES

    an = _load("a3_analyze_teleop_run")
    t, pos, quat, q = load_sim_csv(csv_path, MJ29_JOINT_NAMES)
    d = an.load_dump(dump_path)
    jp = np.asarray(d["jp"], dtype=np.float64)
    ref0 = an.decode_reference(np.asarray(d["obs"]))[:, 0]
    follow = an.label_ticks(ref0, an.stand_reference(), 0.08)  # True = follow
    out: dict = dict(ticks=len(d), follow_ticks=int(follow.sum()))
    r0, ratio, aerr = align_rows(q, jp)
    out["align"] = dict(row0=r0, rows_per_tick=ratio, mean_joint_diff=aerr)
    resets = np.where(np.diff(t) < 0)[0]
    ks = np.where(follow)[0]
    if len(ks) < int(skip_s * HZ) + 100:
        out["error"] = "no follow window in the dump"
        return out
    ks = ks[ks >= ks[0] + int(skip_s * HZ)]
    k_lo, k_hi = int(ks[0]), int(ks[-1])
    row_lo, row_hi = int(r0 + ratio * k_lo), min(int(r0 + ratio * k_hi), len(t) - 1)
    out["follow_seconds"] = (k_hi - k_lo) / HZ
    out["falls"] = dict(resets_in_window=int(np.sum((resets >= row_lo) & (resets <= row_hi))), pelvis_below_0p6_rows=int(np.sum(pos[row_lo : row_hi + 1, 2] < 0.6)),
                        pelvis_z_min=float(pos[row_lo : row_hi + 1, 2].min()))
    sl = slice(k_lo, k_hi + 1)
    err = jp[sl][:, LEG] - ref0[sl][:, LEG]
    out["legs"] = dict(rms_all=float(np.sqrt(np.mean(err**2))), hip_knee=float(np.sqrt(np.mean(err[:, [0, 1, 3, 6, 7, 9]] ** 2))),
                       ankle=float(np.sqrt(np.mean(err[:, [4, 5, 10, 11]] ** 2))), per_joint=np.sqrt(np.mean(err**2, axis=0)).tolist())
    feet = an.foot_task_space(ref0[sl][:, LEG], jp[sl][:, LEG])
    out["feet"] = feet
    g = np.asarray(d["gdir"], dtype=np.float64)[sl]
    pitch, roll = np.degrees(np.arctan2(-g[:, 0], -g[:, 2])), np.degrees(np.arctan2(g[:, 1], -g[:, 2]))
    bav = np.linalg.norm(np.asarray(d["bav"], dtype=np.float64)[sl], axis=1)
    out["sway"] = dict(pitch_std=float(pitch.std()), roll_std=float(roll.std()), tilt_p95=float(np.percentile(np.hypot(pitch, roll), 95)), bav_peak=float(bav.max()))
    fw = foot_world(pos[row_lo : row_hi + 1], quat[row_lo : row_hi + 1], q[row_lo : row_hi + 1], MJ29_JOINT_NAMES)
    slips = [s for k in (0, 1) for s in stance_slip(fw[:, k])]
    out["slip"] = dict(periods=len(slips), median_cm=float(np.median(slips) * 100) if slips else float("nan"),
                       p90_cm=float(np.percentile(slips, 90) * 100) if slips else float("nan"), share_over_3cm=float(np.mean(np.array(slips) > 0.03)) if slips else float("nan"))
    return out


def print_report(label: str, r: dict) -> None:
    print(f"== {label}: {r['ticks']} policy ticks, {r['follow_ticks']} follow ticks")
    a = r["align"]
    print(f"   alignment: tick 0 = csv row {a['row0']:.0f}, {a['rows_per_tick']:.2f} rows/tick, mean joint difference {a['mean_joint_diff']:.4f} rad" + ("  <-- NOT TRUSTED" if a["mean_joint_diff"] > 0.03 else ""))
    if "error" in r:
        print("   " + r["error"])
        return
    f = r["falls"]
    print(f"   window {r['follow_seconds']:.0f} s | falls: resets {f['resets_in_window']}, rows with pelvis < 0.6 m {f['pelvis_below_0p6_rows']}, lowest pelvis {f['pelvis_z_min']:.2f} m")
    lg = r["legs"]
    print(f"   legs rms(measured - reference): all {lg['rms_all']:.3f} rad | hip+knee {lg['hip_knee']:.3f} | ankle {lg['ankle']:.3f}")
    if r["feet"]:
        for nm in ("L", "R"):
            x = r["feet"][nm]
            print(f"   foot {nm}: residual {x['resid']:.1f} cm (x/y/z {x['xyz'][0]:.1f}/{x['xyz'][1]:.1f}/{x['xyz'][2]:.1f}) | vertical range reference {x['lift_ref']:.0f} cm vs executed {x['lift_meas']:.0f} cm | delay {x['lag']:+.0f} ms")
    s = r["sway"]
    print(f"   sway: pitch std {s['pitch_std']:.2f} deg, roll std {s['roll_std']:.2f} deg, tilt p95 {s['tilt_p95']:.2f} deg, peak base angular velocity {s['bav_peak']:.2f} rad/s")
    sp = r["slip"]
    print(f"   slip: {sp['periods']} stance periods, median {sp['median_cm']:.1f} cm, p90 {sp['p90_cm']:.1f} cm, {sp['share_over_3cm'] * 100:.0f}% over 3 cm")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", type=Path, required=True)
    ap.add_argument("--dump", type=Path, required=True)
    ap.add_argument("--label", default="")
    ap.add_argument("--skip", type=float, default=4.0, help="seconds dropped at the start of the follow window")
    ap.add_argument("--json", type=Path, default=None)
    a = ap.parse_args()
    r = score(a.csv, a.dump, a.skip)
    print_report(a.label or a.csv.parent.name, r)
    if a.json:
        a.json.write_text(json.dumps(r, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o), indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
