"""Pico session tools: recorder (read-only SUB), raw replayer (PUB), and the offline scorer of a recorded real session."""

from __future__ import annotations

from pathlib import Path
import pickle
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "gear_sonic_deploy/scripts"
zmq = pytest.importorskip("zmq")


def _pose_message(i: int, pm, n=10) -> bytes:
    rng = np.random.default_rng(i)
    msg = {
        "smpl_pose": rng.standard_normal((n, 21, 3)).astype(np.float32) * 0.01,
        "smpl_joints": np.tile(np.arange(24 * 3, dtype=np.float32).reshape(24, 3) * 0.01, (n, 1, 1)),
        "body_quat_w": np.tile(np.array([1, 0, 0, 0], dtype=np.float32), (n, 1)),
        "frame_index": np.arange(i, i + n, dtype=np.int64),
        "timestamp_realtime": np.array([time.time()], dtype=np.float64),
        "timestamp_monotonic": np.array([time.monotonic()], dtype=np.float64),
    }
    return pm.pack_pose_message(msg, topic="pose")


def test_recorder_keeps_every_message_and_the_replayer_reproduces_them_in_time(tmp_path):
    from gear_sonic.scripts import pico_pose_zmq_minimal as pm

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    port = pub.bind_to_random_port("tcp://127.0.0.1")
    sent: list[bytes] = []
    stop = threading.Event()

    def publish():
        i = 0
        while not stop.is_set():
            m = b"command" + b"\x01\x02\x03" if i == 20 else _pose_message(i, pm)
            sent.append(m)
            pub.send(m)
            pub.send(b"manager_status" + b"\x00" * 1300)  # the manager's high-rate status stream: must NOT be recorded
            i += 1
            time.sleep(0.02)

    th = threading.Thread(target=publish, daemon=True)
    th.start()
    out = tmp_path / "rec.pkl"
    proc = subprocess.Popen([sys.executable, str(SCRIPTS / "a3_pose_recorder.py"), "--out", str(out), "--host", "127.0.0.1", "--port", str(port), "--seconds", "2.5"],
                            stdout=subprocess.PIPE, text=True)
    text, _ = proc.communicate(timeout=30)
    stop.set()
    th.join(timeout=2)
    pub.close(0)
    ctx.term()
    assert "saved" in text and out.exists(), text
    rec = pickle.load(open(out, "rb"))
    msgs = rec["messages"]
    assert 60 < len(msgs) <= len(sent)
    assert any(m.startswith(b"command") for _, m in msgs) and all(m in sent for _, m in msgs[:20])  # lossless: the very bytes that were sent
    assert not any(m.startswith(b"manager_status") for _, m in msgs)
    assert all(b >= a for (a, _), (b, _) in zip(msgs, msgs[1:]))

    # replay at 4x into a subscriber: same messages, about a quarter of the duration
    ctx2 = zmq.Context()
    sub = ctx2.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 5000)
    rp = subprocess.Popen([sys.executable, str(SCRIPTS / "a3_pose_replay_raw.py"), str(out), "--port", "5599", "--speed", "4"], stdout=subprocess.PIPE, text=True)
    sub.connect("tcp://127.0.0.1:5599")
    got, t0 = [], time.time()
    try:
        while len(got) < len(msgs) and time.time() - t0 < 8:
            try:
                got.append(sub.recv())
            except zmq.Again:
                break
    finally:
        rp.wait(timeout=20)
        sub.close(0)
        ctx2.term()
    assert len(got) >= len(msgs) * 0.9, (len(got), len(msgs))
    assert any(m.startswith(b"command") for m in got)
    assert time.time() - t0 < 6.0  # 2.5 s of recording at 4x, plus connect time


def test_scorer_runs_on_a_recorded_session_and_sees_the_stock_ankle_roll_drift(tmp_path):
    pytest.importorskip("mujoco")
    pytest.importorskip("general_motion_retargeting")
    from gear_sonic.scripts import pico_pose_zmq_minimal as pm
    from gear_sonic.scripts.pico_pose_zmq_replay import frame_from_pose_aa, synthetic_clip

    fk = pm.HumanJointsFK()
    pose, _ = synthetic_clip("stand", 30.0, seconds=1.0)
    sp, sj, bq = frame_from_pose_aa(pose[0], fk)
    msgs, t = [], time.time_ns()
    for i in range(0, 400):  # 8 s of a perfectly still person, 10-frame windows sliding by one frame like the manager's
        w = {
            "smpl_pose": np.tile(sp, (10, 1, 1)).astype(np.float32),
            "smpl_joints": np.tile(sj, (10, 1, 1)).astype(np.float32),
            "body_quat_w": np.tile(bq, (10, 1)).astype(np.float32),
            "frame_index": np.arange(i, i + 10, dtype=np.int64),
        }
        msgs.append((t + i * 20_000_000, pm.pack_pose_message(w, topic="pose")))
    rec = tmp_path / "still.pkl"
    pickle.dump({"version": 1, "messages": msgs}, open(rec, "wb"))
    env_cmd = [sys.executable, str(SCRIPTS / "a3_eval_pose_recording.py"), str(rec)]
    stock = subprocess.run(env_cmd, capture_output=True, text=True, timeout=300, cwd=ROOT, env={**__import__("os").environ, "PYTHONPATH": str(ROOT)})
    fixed = subprocess.run(env_cmd + ["--foot-rot-weight", "8"], capture_output=True, text=True, timeout=300, cwd=ROOT, env={**__import__("os").environ, "PYTHONPATH": str(ROOT)})
    assert stock.returncode == 0 and fixed.returncode == 0, stock.stdout + stock.stderr + fixed.stderr
    # still person: the stock table lets the ankle roll creep to its limit within a few seconds, the foot rotation weight does not
    assert "STILL: ankle roll mean" in stock.stdout
    def sat(text):
        line = [ln for ln in text.splitlines() if "ankle roll within 0.05 rad of the limit in" in ln][0]
        return float(line.split("limit in ")[1].split("%")[0])
    assert sat(stock.stdout) > 30.0 and sat(fixed.stdout) < 1.0, (stock.stdout, fixed.stdout)


def test_dump_cfg_helper_changes_only_the_dump_lines(tmp_path):
    base = tmp_path / "base.yaml"
    base.write_text("a: 1\nlogging:\n  dump_ticks: 0\n  dump_path: logs/x.bin\n  debug_print_ticks: 0\nb: 2\n")
    out = tmp_path / "out.yaml"
    r = subprocess.run([sys.executable, str(SCRIPTS / "a3_make_dump_cfg.py"), "--dump", str(tmp_path / "run1.bin"), "--ticks", "123", "--base", str(base), "--out", str(out)],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    new = out.read_text().splitlines()
    old = base.read_text().splitlines()
    diff = [(o, n) for o, n in zip(old, new) if o != n]
    assert len(diff) == 2 and "dump_ticks: 123" in diff[0][1] and str(tmp_path / "run1.bin") in diff[1][1]
    bad = tmp_path / "bad.yaml"
    bad.write_text("x: 1\n")
    r2 = subprocess.run([sys.executable, str(SCRIPTS / "a3_make_dump_cfg.py"), "--dump", "d.bin", "--base", str(bad), "--out", str(tmp_path / "o2.yaml")], capture_output=True, text=True, timeout=30)
    assert r2.returncode != 0 and "no logging.dump_ticks" in (r2.stdout + r2.stderr)


def test_replayer_waits_for_the_trigger_and_sends_fifo_commands(tmp_path):
    from gear_sonic.scripts import pico_pose_zmq_minimal as pm

    t0 = time.time_ns()
    msgs = [(t0 + i * 20_000_000, _pose_message(i, pm)) for i in range(60)]
    rec = tmp_path / "r.pkl"
    pickle.dump({"version": 1, "messages": msgs}, open(rec, "wb"))
    fifo, trig = tmp_path / "f", tmp_path / "go"
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 300)
    sub.connect("tcp://127.0.0.1:5598")
    rp = subprocess.Popen([sys.executable, str(SCRIPTS / "a3_pose_replay_raw.py"), str(rec), "--port", "5598", "--no-commands", "--command-fifo", str(fifo), "--wait-for", str(trig)],
                          stdout=subprocess.PIPE, text=True, env={**__import__("os").environ, "PYTHONPATH": str(ROOT)})
    got = []

    def drain(seconds):
        end = time.time() + seconds
        while time.time() < end:
            try:
                got.append(sub.recv())
            except zmq.Again:
                pass

    try:
        drain(2.0)  # started, waiting: only the manager-style first command message may arrive, no pose
        assert not any(m.startswith(b"pose") for m in got), "poses were sent before the trigger"
        n_cmd0 = sum(m.startswith(b"command") for m in got)
        with open(fifo, "w") as f:
            f.write("y")
        drain(1.0)
        assert sum(m.startswith(b"command") for m in got) == n_cmd0 + 1  # A+B+X+Y -> one start command
        trig.write_text("go")
        drain(3.0)
        assert sum(m.startswith(b"pose") for m in got) == 60
    finally:
        rp.wait(timeout=20)
        sub.close(0)
        ctx.term()


def _load_replay_module():
    import importlib.util

    spec = importlib.util.spec_from_file_location("a3_pose_replay_raw", SCRIPTS / "a3_pose_replay_raw.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_replay_schedule_by_frame_index_removes_recorder_stalls():
    """A recorder stall (a silent gap in the arrival times with continuous frame indexes) must not become a dead stream in the replay."""
    from gear_sonic.scripts import pico_pose_zmq_minimal as pm

    rp = _load_replay_module()
    ns = 1_000_000_000
    msgs = []
    for i in range(300):  # 50 Hz windows advancing one frame each; arrival: 3 s silence after window 100, then the buffered ones in a burst
        arrival = int(i * 0.02 * ns) + (3 * ns if i >= 101 else 0) - (int((i - 101) * 0.02 * ns) - int((i - 101) * 0.002 * ns) if 101 <= i < 151 else 0)
        msgs.append((arrival, _pose_message(i, pm)))
    msgs.insert(150, (msgs[149][0], b"command" + b"\x01"))
    assert rp.pose_last_frame_index(msgs[0][1]) == 9 and rp.pose_last_frame_index(b"command\x01") is None
    arr = rp.schedule(msgs, "arrival")
    assert max(b[0] - a[0] for a, b in zip(arr, arr[1:])) > 2.5 * ns  # the stall is there in the recorded times
    frm = rp.schedule(msgs, "frame")
    pose_t = [t for t, m in frm if m.startswith(b"pose")]
    steps = np.diff(pose_t)
    assert np.allclose(steps, 0.02 * ns) and len(pose_t) == 300  # uniform 50 Hz, nothing lost
    assert [m for _, m in frm if m.startswith(b"command")] == [b"command\x01"]
    assert all(b[0] >= a[0] for a, b in zip(frm, frm[1:]))


def _load_script(name: str):
    import importlib.util

    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_score_sim_replay_aligns_by_joints_not_by_row_and_measures_stance_slip():
    sc = _load_script("a3_score_sim_replay")
    rng = np.random.default_rng(0)
    rows = 3000
    base = np.cumsum(rng.standard_normal((rows, 29)) * 0.01, axis=0)  # a wandering joint trajectory: every window of it is distinct
    # the dump saw 1500 ticks starting at csv row 700 and the sim ran at 0.97 rows per tick
    ticks = 1500
    jp = base[np.round(700 + 0.97 * np.arange(ticks)).astype(int)]
    r0, ratio, err = sc.align_rows(base, jp)
    assert abs(r0 - 700) <= 2 and abs(ratio - 0.97) < 0.011 and err < 0.02
    jp_slow = base[np.round(400 + 0.82 * np.arange(ticks)).astype(int)]  # a GUI sim slower than real time (leg2 was 0.82)
    r0, ratio, err = sc.align_rows(base, jp_slow)
    assert abs(r0 - 400) <= 2 and abs(ratio - 0.82) < 0.011 and err < 0.02

    # a foot that lands, slides 5 cm, lifts, and lands again without sliding
    n = 120
    foot = np.zeros((n, 3))
    foot[:, 2] = 0.30  # in the air
    foot[10:50, 2] = 0.0  # stance 1 (40 rows = 0.8 s)
    foot[10:50, 0] = np.linspace(0.0, 0.05, 40)  # slides 5 cm (3.1 cm ends trimmed)
    foot[70:110, 2] = 0.0
    foot[70:110, 0] = 0.4
    slips = sc.stance_slip(foot)
    assert len(slips) == 2 and slips[1] < 1e-9 and 0.02 < slips[0] <= 0.05
