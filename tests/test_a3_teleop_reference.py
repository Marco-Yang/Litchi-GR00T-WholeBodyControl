"""Tests for the A3 teleop reference buffer / wire format (pure numpy, no sim needed)."""

from __future__ import annotations

import ast
from pathlib import Path
import re
import sys

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from gear_sonic.utils.teleop.a3.frames import (  # noqa: E402
    A3_IL_TO_MJ29,
    DEFAULT_Q_MJ29,
    MJ29_JOINT_NAMES,
    WIRE_BODY_SIZE,
    A3TeleopFrame,
    pack_frame,
    unpack_frame,
)
from gear_sonic.utils.teleop.a3.reference_buffer import (  # noqa: E402
    TOKENIZER_FLOATS,
    A3TeleopReferenceBuffer,
    A3TeleopReferenceOptions,
    TokenizerStatus,
    build_default_stand_tokenizer_slice,
)


def _active_joint_names_il() -> list[str]:
    src = (REPO / "gear_sonic/envs/manager_env/robots/a3.py").read_text()
    match = re.search(r"A3_ACTIVE_JOINT_NAMES\s*=\s*(\[.*?\])", src, re.S)
    assert match
    return ast.literal_eval(match.group(1))


def test_il_permutation_matches_training_joint_order():
    il_names = _active_joint_names_il()
    assert len(il_names) == 29
    for i_il, i_mj in enumerate(A3_IL_TO_MJ29):
        assert MJ29_JOINT_NAMES[i_mj] == il_names[i_il]
    assert sorted(A3_IL_TO_MJ29.tolist()) == list(range(29))


def test_il_permutation_matches_cpp_header():
    hpp = (
        REPO / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_policy_parameters.hpp"
    ).read_text()
    block = re.search(r"a3_mujoco_to_isaaclab\s*=\s*\{(.*?)\};", hpp, re.S).group(1)
    nums = [int(x) for x in re.findall(r"\d+", re.sub(r"//.*", "", block))]
    assert nums == A3_IL_TO_MJ29.tolist()


def test_default_pose_matches_cpp_header():
    hpp = (
        REPO / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/a3_policy_parameters.hpp"
    ).read_text()
    block = re.search(r"a3_default_angles\s*=\s*\{(.*?)\};", hpp, re.S).group(1)
    nums = [float(x) for x in re.findall(r"-?\d+\.\d+", re.sub(r"//.*", "", block))]
    assert len(nums) == 29
    np.testing.assert_allclose(nums, DEFAULT_Q_MJ29, atol=1e-9)


def test_wire_roundtrip():
    rng = np.random.default_rng(0)
    f = A3TeleopFrame(
        stamp_ns=123456789,
        q_mj29=rng.normal(size=29),
        dq_mj29=rng.normal(size=29),
        pelvis_quat_wxyz=np.array([0.5, 0.5, -0.5, 0.5]),
        head_q=np.array([0.1, -0.2]),
        head_dq=np.array([0.0, 0.3]),
        has_head=True,
    )
    msg = pack_frame(f)
    assert len(msg) == len(b"a3_ref") + WIRE_BODY_SIZE == 6 + 545
    g = unpack_frame(msg)
    assert g.stamp_ns == f.stamp_ns and g.has_head
    for name in ("q_mj29", "dq_mj29", "pelvis_quat_wxyz", "head_q", "head_dq"):
        np.testing.assert_array_equal(getattr(f, name), getattr(g, name))
    with pytest.raises(ValueError):
        unpack_frame(msg[:-1])
    with pytest.raises(ValueError):
        unpack_frame(b"zzzzzz" + msg[6:])


def _frames_from_arrays(q_il, dq_il, quat, step_ns=20_000_000, t0=1_000_000_000):
    frames = []
    for i in range(len(q_il)):
        q_mj = np.zeros(29)
        dq_mj = np.zeros(29)
        q_mj[A3_IL_TO_MJ29] = q_il[i]
        dq_mj[A3_IL_TO_MJ29] = dq_il[i]
        frames.append(
            A3TeleopFrame(stamp_ns=t0 + i * step_ns, q_mj29=q_mj, dq_mj29=dq_mj, pelvis_quat_wxyz=quat[i])
        )
    return frames


def _random_motion(n=120, seed=1):
    rng = np.random.default_rng(seed)
    q_il = rng.normal(scale=0.3, size=(n, 29))
    dq_il = rng.normal(scale=0.5, size=(n, 29))
    quat = rng.normal(size=(n, 4))
    quat /= np.linalg.norm(quat, axis=1, keepdims=True)
    quat[quat[:, 0] < 0] *= -1
    return q_il, dq_il, quat


def test_slice_layout_on_frame_grid():
    q_il, dq_il, quat = _random_motion()
    opts = A3TeleopReferenceOptions(delay_ns=900_000_000, future_frame_skip=5)
    buf = A3TeleopReferenceBuffer(opts)
    for f in _frames_from_arrays(q_il, dq_il, quat):
        buf.push_frame(f)
    ref_frame = 10
    now_ns = 1_000_000_000 + opts.delay_ns + ref_frame * 20_000_000
    out, status = buf.build_tokenizer_slice(now_ns, np.array([1.0, 0, 0, 0]), 0.0)
    assert status == TokenizerStatus.RUNNING and out.shape == (TOKENIZER_FLOATS,)
    idx = ref_frame + 5 * np.arange(10)
    np.testing.assert_allclose(out[:290].reshape(10, 29), q_il[idx], atol=1e-6)
    np.testing.assert_allclose(out[290:580].reshape(10, 29), dq_il[idx], atol=1e-6)
    # identity robot quat -> rel rotation is the reference rotation itself
    w, x, y, z = quat[idx[0]]
    m00, m01 = 1 - 2 * (y * y + z * z), 2 * (x * y - w * z)
    m10, m11 = 2 * (x * y + w * z), 1 - 2 * (x * x + z * z)
    np.testing.assert_allclose(out[580:584], [m00, m01, m10, m11], atol=1e-6)


def test_matches_sim2sim_csv_reference_builder():
    """The live buffer must reproduce sim2sim's CSV-driven tokenizer terms exactly."""
    pytest.importorskip("mujoco")
    pytest.importorskip("torch")
    from types import SimpleNamespace

    from gear_sonic.scripts import sim2sim_a3_mujoco as s2s

    q_il, dq_il, quat = _random_motion(n=200, seed=3)
    ref = SimpleNamespace(dof_il=q_il, dof_vel_il=dq_il, anchor_quat_wxyz=quat, num_frames=len(q_il))
    opts = A3TeleopReferenceOptions(delay_ns=900_000_000, future_frame_skip=5)
    buf = A3TeleopReferenceBuffer(opts)
    for f in _frames_from_arrays(q_il, dq_il, quat):
        buf.push_frame(f)
    robot_quat = np.array([0.9238795, 0.0, 0.0, 0.3826834])
    for ref_frame in (0, 7, 30, 100):
        cmd, ori = s2s.build_tokenizer_terms(ref, ref_frame, robot_quat, "hold_last", 5)
        now_ns = 1_000_000_000 + opts.delay_ns + ref_frame * 20_000_000
        out, status = buf.build_tokenizer_slice(now_ns, robot_quat, 0.0)
        assert status == TokenizerStatus.RUNNING
        np.testing.assert_allclose(out[:580], cmd, atol=1e-5)
        np.testing.assert_allclose(out[580:], ori.reshape(-1), atol=1e-5)


def test_fast_window_and_interpolation():
    q_il, dq_il, quat = _random_motion(n=60, seed=5)
    opts = A3TeleopReferenceOptions(delay_ns=220_000_000, future_frame_skip=1)
    buf = A3TeleopReferenceBuffer(opts)
    frames = _frames_from_arrays(q_il, dq_il, quat)
    for f in frames:
        buf.push_frame(f)
    now_ns = 1_000_000_000 + 220_000_000 + 5 * 20_000_000 + 10_000_000  # half a frame off the grid
    out, status = buf.build_tokenizer_slice(now_ns, np.array([1.0, 0, 0, 0]), 0.0)
    assert status == TokenizerStatus.RUNNING
    expect = 0.5 * (q_il[5:15] + q_il[6:16])
    np.testing.assert_allclose(out[:290].reshape(10, 29), expect, atol=1e-6)


def test_status_transitions_and_out_of_order_insert():
    buf = A3TeleopReferenceBuffer(A3TeleopReferenceOptions(delay_ns=100_000_000, future_frame_skip=1))
    robot = np.array([1.0, 0, 0, 0])
    assert buf.build_tokenizer_slice(10**9, robot)[1] == TokenizerStatus.NO_DATA
    q_il, dq_il, quat = _random_motion(n=30)
    frames = _frames_from_arrays(q_il, dq_il, quat)
    buf.push_frame(frames[0])
    assert buf.build_tokenizer_slice(10**9, robot)[1] == TokenizerStatus.BUFFERING
    # out-of-order arrival is sorted in; duplicates replace
    for f in (frames[3], frames[1], frames[2], frames[2]):
        buf.push_frame(f)
    assert buf.frame_count() == 4
    stamps = [f.stamp_ns for f in buf._frames]
    assert stamps == sorted(stamps)
    # window not yet covered -> buffering (base before first stamp)
    assert buf.build_tokenizer_slice(frames[0].stamp_ns, robot)[1] == TokenizerStatus.BUFFERING
    # non-finite and non-positive stamps are rejected
    bad = A3TeleopFrame(stamp_ns=10**12, q_mj29=np.full(29, np.nan))
    buf.push_frame(bad)
    buf.push_frame(A3TeleopFrame(stamp_ns=0))
    assert buf.frame_count() == 4


def test_default_stand_slice():
    out = build_default_stand_tokenizer_slice()
    assert out.shape == (TOKENIZER_FLOATS,)
    np.testing.assert_allclose(out[:29], DEFAULT_Q_MJ29[A3_IL_TO_MJ29], atol=1e-6)
    np.testing.assert_array_equal(out[290:580], 0.0)
    np.testing.assert_allclose(out[580:586], [1, 0, 0, 1, 0, 0], atol=1e-6)
