"""a3_ref_monitor.py: the read-only monitor of the Pico -> retarget reference stream (stage A of pico_real_teleop.md 4.7)."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def _tool():
    spec = importlib.util.spec_from_file_location("a3_ref_monitor", ROOT / "gear_sonic_deploy/scripts/a3_ref_monitor.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _stream(tool, hz=50.0, n=200, mutate=None):
    stats = tool.Stats(tool.joint_ranges(), jump_rad=0.5)
    q0 = tool.DEFAULT_Q_MJ29.copy()
    for k in range(n):
        q = q0.copy()
        q[3] += 0.2 * np.sin(2 * np.pi * 0.5 * k / hz)  # a slow left shoulder pitch wave
        if mutate:
            q = mutate(k, q)
        stats.feed(k / hz, q)
    return stats


def test_joint_ranges_cover_all_29_joints():
    r = _tool().joint_ranges()
    assert r.shape == (29, 2) and np.all(np.isfinite(r)) and np.all(r[:, 0] < r[:, 1])


def test_a_smooth_50hz_stream_passes_every_check():
    tool = _tool()
    v = {n: ok for n, ok, _ in _stream(tool).verdict()}
    assert v == {"rate": True, "finite": True, "range": True, "jump": True}


def test_each_failure_is_reported_by_its_own_check():
    tool = _tool()

    def glitch(k, q):
        if k == 100:
            q[10] += 1.2  # a retarget glitch: one frame, one joint
        return q

    v = {n: ok for n, ok, _ in _stream(tool, mutate=glitch).verdict()}
    assert v["jump"] is False and v["range"] is True and v["finite"] is True

    def beyond(k, q):
        q[5] = 9.0  # far outside any joint range
        return q

    v = {n: ok for n, ok, _ in _stream(tool, mutate=beyond).verdict()}
    assert v["range"] is False

    def nan(k, q):
        if k == 50:
            q[0] = float("nan")
        return q

    v = {n: ok for n, ok, _ in _stream(tool, mutate=nan).verdict()}
    assert v["finite"] is False

    v = {n: ok for n, ok, _ in _stream(tool, hz=20.0).verdict()}
    assert v["rate"] is False  # a 20 Hz stream is not the 50 Hz the policy expects


def test_no_frames_is_a_failure_with_a_hint():
    tool = _tool()
    stats = tool.Stats(tool.joint_ranges(), jump_rad=0.5)
    assert not any(ok for _, ok, _ in stats.verdict())


def test_live_subscription_to_a_zmq_publisher():
    zmq = pytest.importorskip("zmq")
    tool = _tool()
    from gear_sonic.utils.teleop.a3.frames import A3TeleopFrame, pack_frame

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    port = pub.bind_to_random_port("tcp://127.0.0.1")
    stop = threading.Event()

    def publish():
        k = 0
        while not stop.is_set():
            f = A3TeleopFrame(stamp_ns=time.time_ns())
            f.q_mj29[3] = 0.2 * np.sin(k / 25.0)
            pub.send(pack_frame(f))
            k += 1
            time.sleep(0.02)

    t = threading.Thread(target=publish, daemon=True)
    t.start()
    try:
        out = subprocess.run(
            [sys.executable, str(ROOT / "gear_sonic_deploy/scripts/a3_ref_monitor.py"), "--host", "127.0.0.1", "--port", str(port), "--seconds", "3"],
            capture_output=True,
            text=True,
            timeout=30,
        )
    finally:
        stop.set()
        t.join(timeout=2)
        pub.close(0)
        ctx.term()
    assert out.returncode == 0, out.stdout + out.stderr
    assert "[PASS] rate" in out.stdout and "[PASS] jump" in out.stdout and "left_shoulder_pitch_joint" in out.stdout


def test_group_summary_tells_a_noisy_leg_from_a_still_one():
    tool = _tool()
    rng = np.random.default_rng(0)

    def noisy_right_knee(k, q):
        q[26] += 0.05 * rng.standard_normal()  # right knee (mj29 index 26) jitters, everything else is still
        return q

    stats = _stream(tool, n=400, mutate=noisy_right_knee)
    mean, std = stats.mean_std()
    assert std[26] > 0.03 and std[25] < 0.001
    text = stats.group_summary()
    right_leg = [line for line in text.splitlines() if line.strip().startswith("right leg")][0]
    assert "right_knee_joint" in right_leg
    assert "mean" in stats.table().splitlines()[0] and "std" in stats.table().splitlines()[0]
