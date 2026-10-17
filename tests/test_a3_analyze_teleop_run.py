"""a3_analyze_teleop_run.py: analysis of the deploy's per-tick dump (+ the relay recording) for the real-robot teleop diagnosis."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "gear_sonic_deploy/scripts/a3_analyze_teleop_run.py"
JOINTS = ["waist_yaw", "waist_roll", "waist_pitch", "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll",
          "left_wrist_pitch", "left_wrist_yaw", "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow", "right_wrist_roll",
          "right_wrist_pitch", "right_wrist_yaw", "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
          "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll"]


def _tool():
    spec = importlib.util.spec_from_file_location("a3_analyze_teleop_run", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _stand_csv(path: Path, values_rad: np.ndarray) -> None:
    """A minimal policy-stand CSV: header 'Frame,<joint>_joint,...', 3 rows of the given pose in DEGREES (what the deploy's CSVs use)."""
    with open(path, "w") as f:
        f.write("Frame," + ",".join(f"{n}_joint" for n in JOINTS) + "\n")
        for k in range(3):
            f.write(f"{k}," + ",".join(f"{math.degrees(v):.6f}" for v in values_rad) + "\n")


def _obs_with_reference(tool, ref_mj: np.ndarray) -> np.ndarray:
    """obs rows whose first 290 floats carry the reference (IsaacLab order, 10 identical frames), as the deploy lays it out."""
    n = len(ref_mj)
    obs = np.zeros((n, 1570), dtype=np.float32)
    isaac = ref_mj[:, tool.PERM]  # isaac position k holds mj joint PERM[k]
    obs[:, :290] = np.tile(isaac, (1, 10))
    return obs


def _fake_dump(tool, path, stand, n_follow=500, n_stand=400, seed=0):
    rng = np.random.default_rng(seed)
    n = n_follow + n_stand + n_follow
    d = np.zeros(n, dtype=tool.REC)
    ref = np.tile(stand, (n, 1))
    ref[:n_follow, 22] = 0.348  # follow: the operator reference has the ankle roll at the limit
    ref[n_follow + n_stand:, 22] = 0.348
    d["obs"] = _obs_with_reference(tool, ref)
    d["jp"][:, tool.LEG] = np.cumsum(0.002 * rng.standard_normal((n, 12)), axis=0)
    d["qdes"][:, tool.LEG] = d["jp"][:, tool.LEG]
    d["qdes"][:n_follow, 18] += 0.5  # follow: a hip roll commanded 0.5 rad away from where the joint is
    d["qdes"][n_follow + n_stand:, 18] += 0.5
    t = np.arange(n) / tool.HZ
    d["gdir"][:, 0] = np.sin(np.radians(2.0) * np.sin(2 * np.pi * 0.5 * t))
    d["gdir"][:, 2] = -1.0
    d.tofile(path)
    return d


def test_reference_decoding_matches_the_deploys_isaaclab_layout():
    tool = _tool()
    ref = np.arange(29, dtype=np.float64)[None, :] * 0.01  # joint j has the value 0.01 * j
    got = tool.decode_reference(_obs_with_reference(tool, ref))
    assert got.shape == (1, 10, 29)
    assert np.allclose(got[0, 0], ref[0], atol=1e-6) and np.allclose(got[0, 9], ref[0], atol=1e-6)


def test_labels_come_from_the_reference_not_from_the_commanded_elbows():
    tool = _tool()
    stand = np.linspace(-0.2, 0.3, 29)
    ref = np.tile(stand, (1300, 1))
    ref[:500, 13] += 0.5  # follow
    ref[900:, 22] = 0.348  # follow again, with a different joint off
    ref[1000:1010] = stand  # a 10-tick return to the stand pose inside a follow phase is a flicker
    segs = tool.segments_from_labels(tool.label_ticks(ref, stand))
    assert [k for _, _, k in segs] == ["follow", "stand", "follow"], segs
    assert abs(segs[1][0] - 500) < 5 and abs(segs[1][1] - 900) < 5
    # without a stand reference nothing can be told apart: one follow segment
    assert [k for _, _, k in tool.segments_from_labels(tool.label_ticks(ref, None))] == ["follow"]


def test_sway_reports_amplitude_and_frequency():
    tool = _tool()
    t = np.arange(1000) / tool.HZ
    g = np.zeros((1000, 3))
    g[:, 0] = np.sin(np.radians(3.0) * np.sin(2 * np.pi * 1.0 * t))
    g[:, 2] = -1.0
    s = tool.sway(g)
    assert abs(s["pitch_ptp"] - 6.0) < 0.3 and abs(s["pitch_freq"] - 1.0) < 0.1 and s["roll_std"] < 1e-6


def test_worst_leg_error_is_judged_by_the_torque_it_implies_not_by_the_angle():
    tool = _tool()
    kps, lim = tool.policy_leg_kps(), tool.mjcf_leg_limits()
    assert kps is not None and kps.shape == (12,) and kps[1] == 120.0 and kps[4] == 60.0  # hip roll Kp 120, ankle at the a3_024 value
    jp = np.zeros((100, 29))
    qd = np.zeros((100, 29))
    qd[:, 18] = 0.5  # left hip roll 0.5 rad away: 60 N*m of 220 = 27%  -> the planted-foot push of the first real run
    text = tool.worst_leg_error(qd, jp, kps, lim)
    assert "L_hip_r +0.50" in text and "60 N*m" in text and "LARGE" in text
    qd[:, 18] = 0.0
    qd[:, 21] = 0.33  # left ankle pitch 0.33 rad away: 20 N*m of 118 = 17%  -> gravity compensation of a soft joint, not a problem
    text = tool.worst_leg_error(qd, jp, kps, lim)
    assert "L_ank_p" in text and "LARGE" not in text
    # without Kp / limits it falls back to the angle alone
    assert "LARGE" in tool.worst_leg_error(qd, jp)


def test_alignment_to_the_200hz_record_with_an_offset_and_a_slow_drift():
    tool = _tool()
    rng = np.random.default_rng(1)
    n = 1500
    leg = np.cumsum(0.01 * rng.standard_normal((n, 12)), axis=0)
    rows_per_tick = np.full(n, 4)
    rows_per_tick[::500] += 1  # the two clocks drift by one row every 500 ticks
    rec = np.concatenate([np.repeat(leg[:1], 400, axis=0)] + [np.repeat(leg[i:i + 1], rows_per_tick[i], axis=0) for i in range(n)])
    rows, resid = tool.align_to_record(leg, rec)
    first = 400 + np.r_[0, np.cumsum(rows_per_tick)[:-1]]
    assert resid < 0.02
    assert np.max(np.abs(rows - first)) <= 8, np.max(np.abs(rows - first))


def test_mjcf_leg_limits_are_found():
    lim = _tool().mjcf_leg_limits()
    assert lim.shape == (12,) and np.all(np.isfinite(lim)) and lim[4] > lim[5] > 0  # ankle pitch limit above ankle roll limit


def test_cli_labels_segments_reports_the_steady_hip_roll_error_and_survives_a_truncated_record(tmp_path):
    tool = _tool()
    stand = np.linspace(-0.1, 0.1, 29)
    csv = tmp_path / "stand.csv"
    _stand_csv(csv, stand)
    p = tmp_path / "dump.bin"
    _fake_dump(tool, p, stand)
    with open(p, "ab") as f:
        f.write(b"\x00" * 100)  # a killed deploy leaves a partial record
    out = subprocess.run([sys.executable, str(SCRIPT), str(p), "--stand-csv", str(csv)], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stdout + out.stderr
    assert "trailing bytes" in out.stdout
    kinds = [line.split()[1] for line in out.stdout.splitlines() if line[:2].strip().isdigit() and len(line.split()) > 2 and line.split()[1] in ("follow", "stand")]
    assert kinds == ["follow", "stand", "follow"], out.stdout
    follow_lines = [line for line in out.stdout.splitlines() if " follow " in line and "LARGE" in line]
    assert len(follow_lines) == 2 and "L_hip_r +0.50" in follow_lines[0] and "N*m" in follow_lines[0], out.stdout
    assert "follow -> stand" in out.stdout and "stand -> follow" in out.stdout


def test_a_missing_stand_csv_is_explained_and_gives_one_segment(tmp_path):
    tool = _tool()
    p = tmp_path / "dump.bin"
    _fake_dump(tool, p, np.zeros(29))
    out = subprocess.run([sys.executable, str(SCRIPT), str(p), "--stand-csv", str(tmp_path / "nope.csv")], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0 and "cannot tell follow from policy stand" in out.stdout


def test_empty_dump_is_explained(tmp_path):
    p = tmp_path / "dump.bin"
    p.write_bytes(b"")
    out = subprocess.run([sys.executable, str(SCRIPT), str(p)], capture_output=True, text=True, timeout=60)
    assert out.returncode != 0 and "dump_ticks" in (out.stdout + out.stderr)


def test_foot_task_space_is_zero_for_identical_legs_and_grows_with_a_leg_error():
    tool = _tool()
    pytest = __import__("pytest")
    pytest.importorskip("mujoco")
    rng = np.random.default_rng(0)
    base = np.zeros((300, 12))
    base[:, [0, 6]] = -0.131 + 0.3 * np.sin(np.linspace(0, 12, 300))[:, None]  # hip pitch swing
    base[:, [3, 9]] = 0.25 + 0.2 * np.sin(np.linspace(0, 12, 300))[:, None]    # knee
    same = tool.foot_task_space(base, base)
    assert same is not None and same["L"]["resid"] < 1e-6 and same["R"]["resid"] < 1e-6
    off = base.copy()
    off[:, 0] += 0.2  # left hip pitch executed 0.2 rad away from the reference
    r = tool.foot_task_space(base, off)
    assert r["L"]["resid"] > 5.0 and r["R"]["resid"] < 1e-6, r  # ~0.2 rad x ~0.8 m of leg = ~16 cm at the left foot only
    assert r["L"]["lift_ref"] > 1.0
