"""PD-stand preview: the default pose / gain tables are read from the C++ header, the snapshot parser, the policy->SDK mapping and the
arm-vs-body clearance scan (the deploy itself has no self-collision check)."""

from __future__ import annotations

from pathlib import Path
import sys

import pytest

DEPLOY = Path(__file__).resolve().parents[1] / "gear_sonic_deploy"
sys.path.insert(0, str(DEPLOY / "scripts"))
import a3_pd_stand_preview as pv  # noqa: E402

SLOT_NAMES = ["waist_yaw", "waist_roll", "waist_pitch", "neck_yaw", "neck_pitch"] + [f"arm{i}" for i in range(14)] + [f"leg{i}" for i in range(12)]


def _snapshot_text(q):
    lines = ["2000 frames in 10 s (200 Hz), seq gaps 0, valid=2000/2000", "slot name             q mean      q std      q min      q max   dq mean  tau mean"]
    lines += [f"{i:3d} {SLOT_NAMES[i]:12s} {q[i]:10.5f} {0.0:10.2e} {q[i]:10.5f} {q[i]:10.5f} {0.0:9.4f} {0.0:9.3f}" for i in range(31)]
    lines.append("pelvis  quat wxyz [1, 0, 0, 0] gyro [0, 0, 0] accel [0, 0, 9.8] |accel| 9.800")
    return "\n".join(lines)


def _default_sdk_pose():
    target, _ = pv.default_angles(pv.HEADER.read_text())
    q = [0.0] * 31
    for i, v in enumerate(target):
        q[pv.POLICY_TO_SDK[i]] = v
    return q


def test_tables_are_read_from_the_header_and_have_29_entries():
    text = pv.HEADER.read_text()
    target, names = pv.default_angles(text)
    assert len(target) == len(names) == 29 and len(pv.pd_stand_kps(text)) == 29
    assert names[0] == "waist_yaw_joint" and names[3] == "left_shoulder_pitch_joint" and names[28].startswith("right_ankle")
    assert target[3] == pytest.approx(0.3) and target[4] == pytest.approx(0.12)  # left shoulder pitch / roll of the default stand
    assert pv.pd_stand_kps(text)[20] == pytest.approx(2000.0)  # left knee


def test_policy_to_sdk_skips_the_two_neck_slots_and_is_a_permutation_of_the_other_29():
    assert pv.POLICY_TO_SDK[:4] == [0, 1, 2, 5] and pv.POLICY_TO_SDK[-1] == 30
    assert sorted(pv.POLICY_TO_SDK) == [i for i in range(31) if i not in (3, 4)]


def test_snapshot_round_trip_and_zero_travel_at_the_default_pose(tmp_path):
    q = _default_sdk_pose()
    f = tmp_path / "snap.txt"
    f.write_text(_snapshot_text(q))
    back = pv.read_snapshot(f)
    assert max(abs(a - b) for a, b in zip(q, back)) < 1e-5
    rows, neck, seconds = pv.preview(back, 150, 50.0, 0.5)
    assert seconds == pytest.approx(3.0) and max(abs(r["delta"]) for r in rows) < 1e-5 and all(abs(n["delta"]) < 1e-9 for n in neck)


def test_a_joint_far_from_the_default_is_the_largest_mover_with_the_right_speed(tmp_path):
    q = _default_sdk_pose()
    q[21] = -1.176  # SDK slot 21 = left hip yaw (the posture seen on 2026-10-16 22:25)
    rows, _, seconds = pv.preview(q, 150, 50.0, 0.5)
    top = max(rows, key=lambda r: abs(r["delta"]))
    assert top["name"] == "left_hip_yaw_joint" and top["delta"] == pytest.approx(-0.035 + 1.176, abs=1e-3)
    assert top["speed"] == pytest.approx(abs(top["delta"]) / seconds)


def test_a_malformed_snapshot_is_rejected(tmp_path):
    f = tmp_path / "bad.txt"
    f.write_text("nothing useful here\n")
    with pytest.raises(ValueError):
        pv.read_snapshot(f)


def test_collision_scan_clears_the_default_pose_and_detects_a_folded_arm():
    pytest.importorskip("mujoco")
    if not pv.MJCF.exists():
        pytest.skip("needs the A3 Ultra MJCF")
    q = _default_sdk_pose()
    ok = pv.collision_scan(q, steps=3)
    assert ok and ok[0][0] > 0.0  # default pose to default pose: every arm-vs-body pair has positive clearance
    bad = list(q)
    for slot, v in ((5, 0.9), (6, -0.08), (7, 1.6), (8, 1.7)):  # left arm folded across the body: shoulder pitch / roll / yaw, elbow
        bad[slot] = v
    # the tool scans the path from `bad` to the default pose: at ramp 0 the folded posture itself is evaluated
    worst = pv.collision_scan(bad, steps=3)
    assert worst[0][0] < ok[0][0]  # the folded start is strictly tighter than the clear default
