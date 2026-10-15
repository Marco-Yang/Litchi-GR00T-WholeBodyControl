"""Accuracy checks for the real-time Pico -> A3 retargeter (needs GMR + mujoco, skipped otherwise)."""

from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

pytest.importorskip("mujoco")
pytest.importorskip("general_motion_retargeting")

from gear_sonic.scripts import pico_pose_zmq_minimal as pm  # noqa: E402
from gear_sonic.scripts.pico_pose_zmq_replay import frame_from_pose_aa, synthetic_clip  # noqa: E402
from gear_sonic.utils.teleop.a3.frames import MJ29_JOINT_NAMES, pack_frame, unpack_frame  # noqa: E402
from gear_sonic.utils.teleop.a3.retarget import (  # noqa: E402
    A3RealtimeRetargeter,
    A3RefStream,
    A3SmplPoseAugmenter,
    pose_window_catch_up,
    unpack_pose_message,
)


def _tilt_deg(q_wxyz: np.ndarray) -> float:
    z = R.from_quat(q_wxyz, scalar_first=True).apply([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(np.clip(z[2], -1.0, 1.0))))


@pytest.fixture(scope="module")
def fk():
    return pm.HumanJointsFK()


def _solve(retargeter, pose_row, fk, iters=10):
    sp, sj, bq = frame_from_pose_aa(pose_row, fk)
    frame = None
    for i in range(iters):
        frame = retargeter.retarget_pico_frame(sp, bq, sj, i / 50.0, 1 + i)
    return frame


def test_standing_human_gives_upright_symmetric_robot(fk):
    pose, _ = synthetic_clip("stand", 30.0, seconds=1.0)
    frame = _solve(A3RealtimeRetargeter(), pose[0], fk)
    assert frame.is_finite()
    assert _tilt_deg(frame.pelvis_quat_wxyz) < 3.0
    q = dict(zip(MJ29_JOINT_NAMES, frame.q_mj29))
    assert abs(q["waist_yaw_joint"]) < 0.08 and abs(q["waist_roll_joint"]) < 0.08 and abs(q["waist_pitch_joint"]) < 0.08
    # symmetric human -> left/right legs mirror each other (pitch/knee/ankle-pitch equal)
    for joint in ("hip_pitch", "knee", "ankle_pitch"):
        assert abs(q[f"left_{joint}_joint"] - q[f"right_{joint}_joint"]) < 0.12, joint


def test_default_scale_reference_gives_the_policy_default_stand(fk):
    """A standing SMPL-template human must come out standing (knees ~ the 035 default 14 deg), not crouched.

    With the literal template height (1.66) the knees were ~84 deg and the policy crouched 20 cm; the pelvis-tilt
    the stock GMR weights used to show was largely an artifact of that crouch (see plan 12.5).
    """
    pose, _ = synthetic_clip("stand", 30.0, seconds=1.0)
    for weight in (None, 50.0):
        frame = _solve(A3RealtimeRetargeter(pelvis_rot_weight=weight), pose[0], fk)
        q = dict(zip(MJ29_JOINT_NAMES, frame.q_mj29))
        for side in ("left", "right"):
            knee = np.degrees(q[f"{side}_knee_joint"])
            assert 3.0 < knee < 30.0, (weight, side, knee)
        assert _tilt_deg(frame.pelvis_quat_wxyz) < 5.0, weight


def test_literal_template_height_crouches(fk):
    """Documents why the scale reference is 2.18 instead of the template's real 1.66."""
    pose, _ = synthetic_clip("stand", 30.0, seconds=1.0)
    frame = _solve(A3RealtimeRetargeter(human_height=1.66), pose[0], fk)
    q = dict(zip(MJ29_JOINT_NAMES, frame.q_mj29))
    assert np.degrees(q["left_knee_joint"]) > 60.0


def test_head_stays_at_its_default_pose_unless_tracking_is_requested(fk):
    """The 035 weights drive 29 body joints; the deploy holds the head at q=0 (like the A3), so frames carry no head."""
    pose, _ = synthetic_clip("wave", 30.0, seconds=2.0)
    frame = _solve(A3RealtimeRetargeter(), pose[45], fk)
    assert frame.has_head is False
    assert np.allclose(frame.head_q, 0.0)
    assert _solve(A3RealtimeRetargeter(track_head=True), pose[45], fk).has_head is True


def test_raised_right_arm_moves_only_right_arm(fk):
    pose, _ = synthetic_clip("wave", 30.0, seconds=8.0)
    retargeter = A3RealtimeRetargeter()
    rest = _solve(retargeter, pose[0], fk)
    raised = _solve(retargeter, pose[int(2.0 * 30.0)], fk)  # right arm fully lifted at t = 2 s
    delta = dict(zip(MJ29_JOINT_NAMES, raised.q_mj29 - rest.q_mj29))
    right = max(abs(v) for k, v in delta.items() if k.startswith("right_shoulder") or k.startswith("right_elbow"))
    left = max(abs(v) for k, v in delta.items() if k.startswith("left_shoulder") or k.startswith("left_elbow"))
    assert right > 0.6
    assert left < 0.25
    assert _tilt_deg(raised.pelvis_quat_wxyz) < 5.0


def test_retargeted_frame_survives_the_wire(fk):
    pose, _ = synthetic_clip("stand", 30.0, seconds=1.0)
    frame = _solve(A3RealtimeRetargeter(), pose[0], fk)
    back = unpack_frame(pack_frame(frame))
    assert np.allclose(back.q_mj29, frame.q_mj29) and np.allclose(back.pelvis_quat_wxyz, frame.pelvis_quat_wxyz)


def _window_msg(pose, fk, first, n=5, index_start=None):
    """A Pico-manager style window message: n consecutive frames, the G1 field set.

    ``first`` is the clip time index of the first frame, ``index_start`` the ``frame_index`` the sender stamps on it
    (default: the same number; differs after the sender restarted its frame counter)."""
    index_start = first if index_start is None else index_start
    rows = [frame_from_pose_aa(pose[t], fk) for t in range(first, first + n)]
    return {
        "smpl_pose": np.stack([r[0] for r in rows]).astype(np.float32),
        "smpl_joints": np.stack([r[1] for r in rows]).astype(np.float32),
        "body_quat_w": np.stack([r[2] for r in rows]).astype(np.float32),
        "joint_pos": np.zeros((n, 29)),  # the G1 sender's G1-axis wrist formula would be here
        "joint_vel": np.zeros((n, 29)),
        "frame_index": np.arange(index_start, index_start + n, dtype=np.int64),
        "left_trigger": np.array([0.25], dtype=np.float32),
        "heading_increment": np.array([0.0], dtype=np.float32),
    }


def test_smpl_pose_augmenter_puts_a3_wrist_angles_in_the_isaaclab_wrist_slots(fk):
    pose, _ = synthetic_clip("wave", 30.0, seconds=6.0)
    aug = A3SmplPoseAugmenter()
    msg = _window_msg(pose, fk, 58)  # t = 2 s: the right arm is at the top of its lift
    out = aug.augment(msg)

    assert out["joint_pos"].shape == (5, 29) and out["joint_pos"].dtype == np.float64
    assert np.all(np.isfinite(out["joint_pos"]))
    for key in msg:  # everything but joint_pos is passed through untouched
        if key != "joint_pos":
            assert np.array_equal(out[key], msg[key]), key
    # the right arm is raised: its IsaacLab-order shoulder roll (16) differs clearly from the left one (15)
    jp = out["joint_pos"][-1]
    assert abs(jp[16]) > 0.5 and abs(jp[16] - jp[15]) > 0.5, jp[[15, 16]]
    # the wrist slots 23..28 are the A3 wrist angles of the same IK solution
    q_mj = aug.retargeter.retarget_pico_frame(*_frame_args(pose, fk, 62), 62 / 50.0, 62).q_mj29
    names = dict(zip(MJ29_JOINT_NAMES, q_mj))
    il_wrists = [
        "left_wrist_roll_joint", "right_wrist_roll_joint", "left_wrist_pitch_joint",
        "right_wrist_pitch_joint", "left_wrist_yaw_joint", "right_wrist_yaw_joint",
    ]  # fmt: skip
    assert np.allclose(jp[23:29], [names[n] for n in il_wrists], atol=0.05)


def _frame_args(pose, fk, t):
    sp, sj, bq = frame_from_pose_aa(pose[t], fk)
    return (sp, bq, sj)


def test_smpl_pose_augmenter_solves_each_frame_once_across_sliding_windows(fk):
    pose, _ = synthetic_clip("wave", 30.0, seconds=6.0)
    aug = A3SmplPoseAugmenter()
    calls = []
    real = aug.retargeter.retarget_pico_frame
    aug.retargeter.retarget_pico_frame = lambda *a, **k: calls.append(1) or real(*a, **k)
    aug.augment(_window_msg(pose, fk, 10))
    first = len(calls)
    assert first >= 5  # 5 new frames (+ the first-frame settling iterations)
    aug.augment(_window_msg(pose, fk, 11))  # window slid by one frame -> exactly one new solve
    assert len(calls) - first == 1


def test_smpl_pose_message_survives_the_wire_and_keeps_the_g1_field_set(fk):
    from gear_sonic.scripts.pico_pose_zmq_minimal import pack_pose_message

    pose, _ = synthetic_clip("stand", 30.0, seconds=2.0)
    msg = _window_msg(pose, fk, 5)
    out = A3SmplPoseAugmenter().augment(msg)
    back = unpack_pose_message(pack_pose_message(out, topic="pose"))
    assert set(back) == set(msg)
    assert np.allclose(back["joint_pos"], out["joint_pos"])
    assert back["frame_index"].tolist() == msg["frame_index"].tolist()


# ---------------------------------------------------------------------------------------------------------------------
# The official sender restarts its frame counter every time the operator leaves / re-enters POSE mode
# (PoseStreamer.on_mode_exit: step = 0).  G1's deploy handles it in StreamedMotionMerger::CalculateSlidingWindow
# ("catch-up"); both A3 chain modes must use the very same rule and keep streaming afterwards.
def test_pose_window_catch_up_rule_is_g1s_sliding_window_rule():
    assert pose_window_catch_up(None, [0, 1, 2, 3, 4]) is None  # first packet: a session starts anyway
    assert pose_window_catch_up((96, 100), [97, 98, 99, 100, 101]) is None  # window slid by one frame
    assert pose_window_catch_up((96, 100), [99, 100, 101, 102, 103]) is None  # slid by 3: overlapping, nothing lost
    assert pose_window_catch_up((96, 100), [101, 102, 103, 104, 105]) is None  # adjacent windows: no frame missing
    # G1: incoming_start <= window start  -> catch-up (the sender restarted its counter, or an old / repeated window)
    assert pose_window_catch_up((96, 100), [0, 1, 2, 3, 4]) == "restart"
    assert pose_window_catch_up((96, 100), [96, 97, 98, 99, 100]) == "restart"
    assert pose_window_catch_up((96, 100), [90, 91, 92, 93, 94]) == "restart"
    assert pose_window_catch_up((0, 4), [0, 1, 2, 3, 4]) == "restart"  # restart after a very short session (equality)
    # G1: incoming_end <= window end  -> catch-up
    assert pose_window_catch_up((96, 100), [97, 98, 99, 100]) == "stale"
    assert pose_window_catch_up((96, 100), [98]) == "stale"
    # G1: incoming_start > window end + frame_step  -> frames were lost between the windows
    assert pose_window_catch_up((96, 100), [102, 103, 104, 105, 106]) == "gap"
    assert pose_window_catch_up((90, 98), [100, 102, 104]) is None  # frame step 2: 100 == 98 + 2, still adjacent
    assert pose_window_catch_up((90, 98), [102, 104, 106]) == "gap"


def test_a3_ref_stream_slides_without_resetting_and_catches_up_on_a_gap(fk):
    pose, _ = synthetic_clip("wave", 30.0, seconds=6.0)
    stream = A3RefStream(A3RealtimeRetargeter())
    assert len(stream.process(_window_msg(pose, fk, 10), 1_000_000_000)) == 5  # first window: five new frames
    assert len(stream.process(_window_msg(pose, fk, 11), 1_020_000_000)) == 1  # window slid by one frame
    assert len(stream.process(_window_msg(pose, fk, 12), 1_040_000_000)) == 1
    assert stream.catch_ups == 0
    frames = stream.process(_window_msg(pose, fk, 25), 1_300_000_000)  # 7 frames lost -> G1 would catch up
    assert len(frames) == 5 and stream.catch_ups == 1
    assert any("gap" in e for e in stream.events)


def test_a3_ref_stream_keeps_streaming_after_the_sender_restarts_its_frame_counter(fk):
    pose, _ = synthetic_clip("wave", 30.0, seconds=6.0)
    stream = A3RefStream(A3RealtimeRetargeter())
    for i, t in enumerate(range(20, 30)):  # session 1: indices 120.. (the old node would wait for 134+ afterwards)
        stream.process(_window_msg(pose, fk, t, index_start=100 + t), 1_000_000_000 + i * 20_000_000)
    assert stream.last_index == 133 and stream.catch_ups == 0
    # the operator leaves and re-enters POSE mode: frame_index starts again from 0
    frames = stream.process(_window_msg(pose, fk, 40, index_start=0), 5_000_000_000)
    assert len(frames) == 5, "frames of the restarted session must not be skipped"
    assert stream.catch_ups == 1 and any("restart" in e for e in stream.events)
    assert stream.last_index == 4
    assert all(f.is_finite() for f in frames)
    assert len(stream.process(_window_msg(pose, fk, 41, index_start=1), 5_020_000_000)) == 1  # normal again
    assert stream.catch_ups == 1


def test_a3_ref_stream_also_restarts_after_a_very_short_session(fk):
    pose, _ = synthetic_clip("wave", 30.0, seconds=6.0)
    stream = A3RefStream(A3RealtimeRetargeter())
    assert len(stream.process(_window_msg(pose, fk, 0), 1_000_000_000)) == 5  # indices 0..4, then the operator leaves
    frames = stream.process(_window_msg(pose, fk, 30, index_start=0), 9_000_000_000)  # re-enters: 0..4 again
    assert len(frames) == 5 and stream.catch_ups == 1  # a "newest index went back" rule would have dropped all five


def test_smpl_pose_augmenter_does_not_serve_stale_wrist_angles_after_a_restart(fk):
    pose, _ = synthetic_clip("wave", 30.0, seconds=6.0)
    aug = A3SmplPoseAugmenter()
    raised = aug.augment(_window_msg(pose, fk, 58, index_start=100))  # right arm at the top of its lift, indices 100..104
    assert aug.catch_ups == 0
    # restart: a different human pose (arm down) is stamped with indices 0..4, which are NOT in the cache; the old code
    # answered "late frame -> nearest cached solution" i.e. the raised-arm wrist angles of the previous session
    low_msg = _window_msg(pose, fk, 0, index_start=0)
    after = aug.augment(low_msg)
    assert aug.catch_ups == 1
    fresh = A3SmplPoseAugmenter().augment(low_msg)
    # the new session's own solution (the warm start still holds the old arm pose, so the first frames are ~0.04 rad off)
    assert np.allclose(after["joint_pos"], fresh["joint_pos"], atol=0.1)
    # ... and clearly not the raised-arm angles of the previous session (IsaacLab right shoulder roll: slot 16)
    assert abs(after["joint_pos"][-1][16] - raised["joint_pos"][-1][16]) > 0.5


def test_smpl_pose_augmenter_restarts_after_a_very_short_session_too(fk):
    pose, _ = synthetic_clip("wave", 30.0, seconds=6.0)
    aug = A3SmplPoseAugmenter()
    first = aug.augment(_window_msg(pose, fk, 58, index_start=0))  # indices 0..4 (raised arm), then the operator leaves
    again = aug.augment(_window_msg(pose, fk, 0, index_start=0))  # re-enters: 0..4 again, arm down
    assert aug.catch_ups == 1
    assert abs(again["joint_pos"][-1][16] - first["joint_pos"][-1][16]) > 0.5  # not the cached (stale) solution
