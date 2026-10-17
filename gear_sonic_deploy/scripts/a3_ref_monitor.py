#!/usr/bin/env python3
# ruff: noqa: T201
"""Read-only monitor of the A3 reference stream (Pico -> GMR retarget -> ZMQ ``a3_ref`` on tcp://localhost:5560).

    python3 gear_sonic_deploy/scripts/a3_ref_monitor.py [--host localhost] [--port 5560] [--seconds 30]

It only SUBSCRIBES (a ZMQ PUB socket has any number of subscribers, so the deploy is not disturbed) and sends nothing to the robot.  Once per
second it prints the frame rate, the age of the newest frame, the largest frame-to-frame joint jump and how many joints are outside the A3
Ultra joint range; at the end it prints, per joint, the min / max seen and the distance to the default standing pose, and a verdict:

    rate      the frames arrive at ~50 Hz (the policy rate) without gaps
    finite    every value is finite
    range     no joint is outside its mechanical range (the deploy does not clip the reference)
    jump      no frame-to-frame jump above --jump-rad (a retarget glitch would be a step in the policy's reference)

Use it for stage A of docs/notes/a3/pico_real_teleop.md 4.7 (real headset, robot not following yet): wear the headset, stand in the calibration
pose, move the arms slowly, and check that the numbers look like what you did.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from gear_sonic.utils.teleop.a3.frames import (  # noqa: E402
    DEFAULT_Q_MJ29,
    MJ29_JOINT_NAMES,
    WIRE_TOPIC,
    unpack_frame,
)

MJCF = ROOT / "gear_sonic/data/assets/robot_description/mjcf/a3_ultra_t3d0.xml"
NOMINAL_HZ = 50.0


def joint_ranges(mjcf: Path = MJCF) -> np.ndarray:
    """(29, 2) joint ranges in the mj29 order, read from the A3 Ultra MJCF without importing mujoco."""
    text = mjcf.read_text()
    out = np.full((len(MJ29_JOINT_NAMES), 2), np.nan)
    for i, name in enumerate(MJ29_JOINT_NAMES):
        m = re.search(rf'<joint\b[^>]*\bname="{re.escape(name)}"[^>]*>', text)
        r = re.search(r'\brange="\s*([-+\d.eE]+)\s+([-+\d.eE]+)\s*"', m.group(0)) if m else None
        if r:
            out[i] = [float(r.group(1)), float(r.group(2))]
    return out


class Stats:
    """Accumulates the stream; ``feed`` takes (arrival_time_s, q[29], stamp_ns) and returns nothing."""

    def __init__(self, ranges: np.ndarray, jump_rad: float):
        self.ranges, self.jump_rad = ranges, jump_rad
        self.times: list[float] = []
        self.qmin = np.full(len(MJ29_JOINT_NAMES), np.inf)
        self.qmax = np.full(len(MJ29_JOINT_NAMES), -np.inf)
        self.qsum = np.zeros(len(MJ29_JOINT_NAMES))
        self.qsq = np.zeros(len(MJ29_JOINT_NAMES))
        self.n_finite = 0
        self.last_q: np.ndarray | None = None
        self.max_jump, self.max_jump_joint = 0.0, -1
        self.n_nonfinite = 0
        self.n_out_of_range = 0
        self.n_jumps = 0
        self.bad = 0

    def feed(self, t: float, q: np.ndarray) -> None:
        self.times.append(t)
        if not np.all(np.isfinite(q)):
            self.n_nonfinite += 1
            return
        self.qmin = np.minimum(self.qmin, q)
        self.qmax = np.maximum(self.qmax, q)
        self.qsum += q
        self.qsq += q * q
        self.n_finite += 1
        lo, hi = self.ranges[:, 0], self.ranges[:, 1]
        self.n_out_of_range += int(np.any((q < lo - 1e-6) | (q > hi + 1e-6)))
        if self.last_q is not None:
            d = np.abs(q - self.last_q)
            j = int(np.argmax(d))
            if d[j] > self.max_jump:
                self.max_jump, self.max_jump_joint = float(d[j]), j
            self.n_jumps += int(d[j] > self.jump_rad)
        self.last_q = q.copy()

    def rate_hz(self) -> float:
        if len(self.times) < 2:
            return 0.0
        return (len(self.times) - 1) / max(self.times[-1] - self.times[0], 1e-9)

    def max_gap_ms(self) -> float:
        if len(self.times) < 2:
            return 0.0
        return float(np.max(np.diff(self.times))) * 1000.0

    def verdict(self) -> list[tuple[str, bool, str]]:
        n = len(self.times)
        rate = self.rate_hz()
        gap = self.max_gap_ms()
        return [
            ("rate", n > 10 and 45.0 <= rate <= 55.0 and gap < 100.0, f"{rate:.1f} Hz over {n} frames, longest gap {gap:.0f} ms (want ~50 Hz, gap < 100 ms)"),
            ("finite", n > 0 and self.n_nonfinite == 0, f"{self.n_nonfinite} frame(s) with NaN / inf"),
            ("range", n > 0 and self.n_out_of_range == 0, f"{self.n_out_of_range} frame(s) with a joint outside its mechanical range"),
            ("jump", n > 1 and self.n_jumps == 0, f"{self.n_jumps} frame(s) with a jump > {self.jump_rad} rad (largest {self.max_jump:.3f} rad"
                                               f"{' on ' + MJ29_JOINT_NAMES[self.max_jump_joint] if self.max_jump_joint >= 0 else ''})"),
        ]

    def mean_std(self) -> tuple[np.ndarray, np.ndarray]:
        n = max(self.n_finite, 1)
        mean = self.qsum / n
        return mean, np.sqrt(np.maximum(self.qsq / n - mean * mean, 0.0))

    def table(self) -> str:
        mean, std = self.mean_std()
        lines = [f"{'joint':28s} {'min':>8s} {'max':>8s} {'mean':>8s} {'std':>7s} {'default':>8s} {'range lo':>9s} {'range hi':>9s}"]
        for i, name in enumerate(MJ29_JOINT_NAMES):
            if not np.isfinite(self.qmin[i]):
                continue
            lo, hi = self.ranges[i]
            near = "  <-- within 0.05 rad of a limit" if (self.qmin[i] < lo + 0.05 or self.qmax[i] > hi - 0.05) else ""
            lines.append(f"{name:28s} {self.qmin[i]:+8.3f} {self.qmax[i]:+8.3f} {mean[i]:+8.3f} {std[i]:7.3f} {DEFAULT_Q_MJ29[i]:+8.3f} {lo:+9.3f} {hi:+9.3f}{near}")
        return "\n".join(lines)

    def group_summary(self) -> str:
        """Per body group: the largest std and the largest |mean - default| (a still operator should give ~0.01 rad std in every group)."""
        mean, std = self.mean_std()
        groups = {"waist": range(0, 3), "left arm": range(3, 10), "right arm": range(10, 17), "left leg": range(17, 23), "right leg": range(23, 29)}
        out = []
        for g, idx in groups.items():
            idx = list(idx)
            j = idx[int(np.argmax(std[idx]))]
            k = idx[int(np.argmax(np.abs(mean[idx] - DEFAULT_Q_MJ29[idx])))]
            out.append(f"  {g:10s} largest std {std[j]:.3f} rad ({MJ29_JOINT_NAMES[j]});  largest |mean - default| {abs(mean[k] - DEFAULT_Q_MJ29[k]):.3f} rad ({MJ29_JOINT_NAMES[k]})")
        return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=5560)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--jump-rad", type=float, default=0.5, help="flag a frame-to-frame jump above this (default 0.5 rad = 10 rad/s at 50 Hz)")
    a = ap.parse_args()

    import zmq

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SUBSCRIBE, WIRE_TOPIC)
    sock.connect(f"tcp://{a.host}:{a.port}")
    stats = Stats(joint_ranges(), a.jump_rad)
    t0 = time.time()
    t_end = t0 + a.seconds
    last_print, win_n, newest_age_ms = t0, 0, float("nan")
    print(f"a3_ref_monitor: listening on tcp://{a.host}:{a.port} topic 'a3_ref' for {a.seconds:.0f} s (read-only)", flush=True)
    while time.time() < t_end:
        if sock.poll(200):
            try:
                frame = unpack_frame(sock.recv(zmq.NOBLOCK))
            except ValueError as e:
                stats.bad += 1
                print("bad frame:", e, flush=True)
                continue
            now = time.time()
            stats.feed(now, frame.q_mj29)
            win_n += 1
            newest_age_ms = (time.time_ns() - frame.stamp_ns) / 1e6 if frame.stamp_ns else float("nan")
        now = time.time()
        if now - last_print >= 1.0:
            dt = now - last_print
            print(f"t={now - t0:5.1f}s  {win_n / dt:5.1f} Hz  newest-frame age {newest_age_ms:6.1f} ms  out-of-range frames {stats.n_out_of_range}  "
                  f"jumps>{a.jump_rad} {stats.n_jumps}  largest jump so far {stats.max_jump:.3f} rad", flush=True)
            last_print, win_n = now, 0
    print()
    print(stats.table())
    print()
    print("per group (standing still: std should be ~0.01 rad, mean close to default; the legs matter most for the robot's balance):")
    print(stats.group_summary())
    print()
    ok = True
    for name, passed, detail in stats.verdict():
        ok &= passed
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}: {detail}")
    if not stats.times:
        print("  no frames at all: is the Pico stream / the retarget chain running (deploy.sh starts the chain)?")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
