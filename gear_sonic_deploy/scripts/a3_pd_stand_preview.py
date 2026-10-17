#!/usr/bin/env python3
# ruff: noqa: T201
"""Offline preview of what the deploy's PD stand will do to a robot that is hanging limp in its CURRENT posture.  Sends and reads nothing.

    python3 gear_sonic_deploy/scripts/a3_pd_stand_preview.py <snapshot.txt> [--ticks 150] [--hz 50] [--warn-rad 0.5] [--collision]

``snapshot.txt`` is the output of ``a3_relay_tool.py snapshot --host <relay> --seconds 5`` (it only reads the per-slot mean of q).

The deploy (src/a3_deploy/main.cpp, DeployMode::kPdStand) latches the measured policy-view pose when PD stand is entered and then moves every
joint LINEARLY to ``a3_default_angles`` over ``pd_stand_ticks`` control ticks (150 ticks at 50 Hz = 3 s, config/a3_runtime_config.yaml) with the
PD-stand gains.  The relay accepts the first command because it equals the measured pose; nothing checks how far the rest of the ramp goes.
This prints, per joint: start, target, distance, mean speed over the ramp, the PD-stand kp, and flags the joints that travel far, so that
the person at the robot knows what will move, how far, and whether the space around the robot is clear.  ``--collision`` also steps the linear
joint path through the A3 Ultra T3D0 MuJoCo model (needs ``mujoco``) and reports the smallest clearance between an arm and the rest of the body
(the deploy has NO self-collision check).  Links at most two apart in the kinematic tree and the inside of a hand overlap by design of the
simplified collision shapes and are left out; the method was checked with random full-range arm postures, which give penetrations of >100 mm.  The neck (SDK slots 3-4) is not part
of the 29-joint policy view: the deploy holds it at q = 0 with kp 40 / kd 2 (``kA3HeadKp`` / ``kA3HeadKd``).
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re
import sys

HEADER = Path(__file__).resolve().parents[1] / "src/g1/g1_deploy_onnx_ref/include/a3_policy_parameters.hpp"
# policy-view index -> SDK slot (include/robot_io/a3_layout_extra.hpp kA3PolicyToSdkIdx): the two neck slots 3-4 are skipped
POLICY_TO_SDK = [0, 1, 2] + list(range(5, 12)) + list(range(12, 19)) + list(range(19, 25)) + list(range(25, 31))


def _block(text: str, name: str) -> str:
    m = re.search(rf"constexpr\s+std::array<double,\s*29>\s+{name}\s*=\s*\{{(.*?)\}};", text, flags=re.S)
    if not m:
        raise ValueError(f"{name} not found in {HEADER}")
    return m.group(1)


def default_angles(text: str) -> tuple[list[float], list[str]]:
    """The 29 default angles and their names, in the policy view order (``// [ idx] name`` comments of the C++ table)."""
    vals: dict[int, tuple[float, str]] = {}
    for m in re.finditer(r"([-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)\s*,\s*//\s*\[\s*(\d+)\]\s*(\w+)", _block(text, "a3_default_angles")):
        vals[int(m.group(2))] = (float(m.group(1)), m.group(3))
    if sorted(vals) != list(range(29)):
        raise ValueError(f"expected 29 indexed default angles, got {sorted(vals)}")
    return [vals[i][0] for i in range(29)], [vals[i][1] for i in range(29)]


def pd_stand_kps(text: str) -> list[float]:
    nums = [float(x) for x in re.findall(r"[-+]?\d+(?:\.\d*)?", re.sub(r"//[^\n]*", "", _block(text, "a3_pd_stand_kps")))]
    if len(nums) != 29:
        raise ValueError(f"expected 29 PD-stand kp values, got {len(nums)}")
    return nums


def read_snapshot(path: Path) -> list[float]:
    """q mean of the 31 SDK slots from an ``a3_relay_tool.py snapshot`` text."""
    q: dict[int, float] = {}
    for line in path.read_text().splitlines():
        m = re.match(r"^\s*(\d+)\s+\S+\s+([-+]?\d+\.\d+)\s+[-+]?\d", line)
        if m and int(m.group(1)) < 31:
            q[int(m.group(1))] = float(m.group(2))
    if sorted(q) != list(range(31)):
        raise ValueError(f"{path}: expected 31 slot lines, found {len(q)} (is it the output of a3_relay_tool.py snapshot?)")
    return [q[i] for i in range(31)]


def preview(q_sdk: list[float], ticks: int, hz: float, warn_rad: float, text: str | None = None):
    text = text if text is not None else HEADER.read_text()
    target, names = default_angles(text)
    kps = pd_stand_kps(text)
    seconds = ticks / hz
    rows = []
    for i in range(29):
        start = q_sdk[POLICY_TO_SDK[i]]
        d = target[i] - start
        rows.append(dict(i=i, name=names[i], start=start, target=target[i], delta=d, speed=abs(d) / seconds, kp=kps[i]))
    neck = [dict(name=n, start=q_sdk[s], delta=-q_sdk[s]) for n, s in (("neck_yaw", 3), ("neck_pitch", 4))]
    return rows, neck, seconds


MJCF = Path(__file__).resolve().parents[2] / "gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml"
ARM_WORDS = ("shoulder", "elbow", "wrist", "thumb", "index", "middle", "ring", "pinky", "hand")


def _side(body: str):
    arm = any(w in body for w in ARM_WORDS)
    if not arm:
        return None
    return "L" if body.startswith(("left_", "L_")) else "R" if body.startswith(("right_", "R_")) else None


def collision_scan(q_sdk: list[float], text: str | None = None, steps: int = 51, mjcf: Path = MJCF):
    """Tightest arm-vs-body clearances (metres, signed) along the linear PD-stand path.  Returns [(distance, ramp_fraction, body1, body2), ...]."""
    import mujoco as mj
    import numpy as np

    text = text if text is not None else HEADER.read_text()
    target, names = default_angles(text)
    m = mj.MjModel.from_xml_path(str(mjcf))
    d = mj.MjData(m)
    jn = lambda n: mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
    start = np.array([q_sdk[POLICY_TO_SDK[i]] for i in range(29)])
    tgt = np.array(target)
    parent = m.body_parentid

    def anc(b, k):
        for _ in range(k):
            b = parent[b]
        return b

    def near(b1, b2):
        return b1 == b2 or anc(b1, 1) == b2 or anc(b2, 1) == b1 or anc(b1, 2) == b2 or anc(b2, 2) == b1

    bname = lambda b: mj.mj_id2name(m, mj.mjtObj.mjOBJ_BODY, b)  # noqa: E731
    geoms = [g for g in range(m.ngeom) if (m.geom_contype[g] or m.geom_conaffinity[g]) and m.geom_bodyid[g] != 0]
    pairs = []
    for i, g1 in enumerate(geoms):
        for g2 in geoms[i + 1:]:
            b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
            s1, s2 = _side(bname(b1)), _side(bname(b2))
            if near(b1, b2) or (s1 is None and s2 is None) or (s1 is not None and s1 == s2):
                continue  # only arm <-> body / other arm pairs
            pairs.append((g1, g2, bname(b1), bname(b2)))
    tightest: dict = {}
    fromto = np.zeros(6)
    for a in np.linspace(0.0, 1.0, steps):
        qpos = m.qpos0.copy()
        qpos[:3] = [0, 0, 1.3]
        qpos[3:7] = [1, 0, 0, 0]
        for i, n in enumerate(names):
            qpos[m.jnt_qposadr[jn(n)]] = start[i] + a * (tgt[i] - start[i])
        d.qpos[:] = qpos
        mj.mj_kinematics(m, d)
        mj.mj_comPos(m, d)
        mj.mj_collision(m, d)
        for g1, g2, n1, n2 in pairs:
            dist = mj.mj_geomDistance(m, d, g1, g2, 0.3, fromto)
            key = tuple(sorted((n1, n2)))
            if key not in tightest or dist < tightest[key][0]:
                tightest[key] = (float(dist), float(a))
    return sorted(((v[0], v[1], k[0], k[1]) for k, v in tightest.items()))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("snapshot", type=Path)
    ap.add_argument("--ticks", type=int, default=150, help="pd_stand_ticks of the runtime config (default 150)")
    ap.add_argument("--hz", type=float, default=50.0, help="control rate (default 50 Hz)")
    ap.add_argument("--warn-rad", type=float, default=0.5, help="flag joints that travel farther than this (default 0.5 rad = 29 deg)")
    ap.add_argument("--collision", action="store_true", help="also scan the path for arm-vs-body self-collisions in the MuJoCo model (needs mujoco)")
    a = ap.parse_args()
    q_sdk = read_snapshot(a.snapshot)
    rows, neck, seconds = preview(q_sdk, a.ticks, a.hz, a.warn_rad)
    print(f"PD stand from the posture in {a.snapshot.name}: linear ramp over {a.ticks} ticks at {a.hz:g} Hz = {seconds:.1f} s, then hold the default pose")
    print(f"{'joint':26s} {'start':>8s} {'target':>8s} {'travel':>8s} {'deg':>6s} {'mean speed':>11s} {'kp':>6s}")
    for r in sorted(rows, key=lambda r: -abs(r["delta"])):
        flag = "  <== far" if abs(r["delta"]) > a.warn_rad else ""
        print(f"{r['name']:26s} {r['start']:+8.3f} {r['target']:+8.3f} {r['delta']:+8.3f} {math.degrees(r['delta']):+6.0f} {r['speed']:8.2f} r/s {r['kp']:6.0f}{flag}")
    print("neck (held at 0, kp 40): " + ", ".join(f"{n['name']} {n['start']:+.3f} -> 0 ({math.degrees(n['delta']):+.0f} deg)" for n in neck))
    far = [r for r in rows if abs(r["delta"]) > a.warn_rad]
    print(f"\n{len(far)} joint(s) travel more than {a.warn_rad} rad; the largest is {max(rows, key=lambda r: abs(r['delta']))['name']} "
          f"({math.degrees(max(abs(r['delta']) for r in rows)):.0f} deg, {max(r['speed'] for r in rows):.2f} rad/s mean).")
    if a.collision:
        print("\narm-vs-body clearance along the linear path (A3 Ultra T3D0 collision shapes; negative = the shapes would penetrate):")
        for dist, frac, b1, b2 in collision_scan(q_sdk)[:6]:
            print(f"  {b1:26s} {b2:26s} {dist * 1000:8.1f} mm  at {frac:.2f} of the ramp")
        print("  (no self-collision check exists in the deploy; this is a model, not a guarantee: the real hands and cables are not modelled)")
    print("Everything moves at once and the gains are the PD-stand ones; with the robot hanging the limbs swing freely: keep the space clear, "
          "and put the posture near the default by hand first if the travel is large and not intended.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
