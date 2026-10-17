"""Mirror-twin arm repair: segment detection, fade weights and the choice of the better twin (synthetic joint trajectories)."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("joblib")
pytest.importorskip("scipy")

from gear_sonic.data_process.repair_a3_arm_mirror import repair_arm, segment_weights, segments_from_mask  # noqa: E402


def test_segments_are_padded_merged_and_clipped_to_the_clip():
    mask = np.zeros(60, bool)
    mask[10:13] = True
    mask[20:22] = True  # 7 frames after the first run: merged when merge_gap >= 8
    mask[58:60] = True
    assert segments_from_mask(mask, pad=2, merge_gap=8) == [(8, 24), (56, 60)]
    assert segments_from_mask(mask, pad=2, merge_gap=3) == [(8, 15), (18, 24), (56, 60)]
    assert segments_from_mask(np.zeros(10, bool), pad=2, merge_gap=4) == []


def test_weights_are_one_on_the_core_and_fade_to_zero_inside_the_padding():
    w = segment_weights(40, [(5, 25)], [(10, 20)], ramp=4)
    assert np.all(w[10:20] == 1.0) and np.all(w[:5] == 0) and np.all(w[25:] == 0)
    assert np.all(np.diff(w[5:11]) >= 0) and np.all(np.diff(w[19:26]) <= 0)  # monotone fade in / out
    assert w[9] == pytest.approx(1 - 1 / 5) and w[6] == pytest.approx(1 - 4 / 5) and w[5] == 0.0


def _arm_case(t=80, bad=(30, 50)):
    """Clip A is right; clip B's arm is wrong in ``bad`` (joint offset 1 rad). Columns 0..3 are the A arm, 4..7 the B arm."""
    rng = np.random.default_rng(0)
    truth = np.cumsum(rng.normal(0, 0.01, size=(t, 4)), axis=0)
    qa = np.zeros((t, 8))
    qb = np.zeros((t, 8))
    qa[:, :4] = truth
    qb[:, 4:] = truth
    qb[bad[0] : bad[1], 4:] += 1.0
    cols_a, cols_b = [0, 1, 2, 3], [4, 5, 6, 7]
    ideal_a_from_b = np.zeros_like(qa)
    ideal_b_from_a = np.zeros_like(qb)
    ideal_a_from_b[:, :4] = qb[:, 4:]  # identity "mirror" for the synthetic case
    ideal_b_from_a[:, 4:] = qa[:, :4]
    return qa, qb, ideal_a_from_b, ideal_b_from_a, cols_a, cols_b, truth


def test_the_arm_that_follows_the_human_is_the_source_and_the_other_is_replaced():
    qa, qb, ia, ib, ca, cb, truth = _arm_case()
    err_a = np.full(80, 10.0)  # A follows the human
    err_b = np.full(80, 10.0)
    err_b[30:50] = 60.0
    out_a, out_b, info = repair_arm(qa, qb, ia, ib, err_a, err_b, ca, cb, threshold=0.3, pad=8, merge_gap=16, ramp=6)
    assert [x[2] for x in info] == ["A"] and len(info) == 1
    assert np.allclose(out_b[30:50, 4:], truth[30:50])  # replaced by A's arm
    assert np.allclose(out_b[:20, 4:], truth[:20]) and np.allclose(out_b[60:, 4:], truth[60:])  # untouched far from the segment
    assert np.array_equal(out_a, qa)  # A is never changed when it is the source
    assert np.abs(np.diff(out_b[:, 4:], axis=0)).max() < 0.2  # the fade removes the 1 rad jump (blend over 6 frames, offset only 0 -> 1 where d<=0.3)


def test_when_the_other_twin_is_better_the_roles_swap_and_ties_go_to_a():
    qa, qb, ia, ib, ca, cb, truth = _arm_case()
    qa[30:50, :4] += 1.0  # now A is wrong, B is right
    qb[30:50, 4:] -= 1.0
    ia[:, :4], ib[:, 4:] = qb[:, 4:], qa[:, :4]
    err_a, err_b = np.full(80, 10.0), np.full(80, 10.0)
    err_a[30:50] = 60.0
    out_a, out_b, info = repair_arm(qa, qb, ia, ib, err_a, err_b, ca, cb, threshold=0.3, pad=8, merge_gap=16, ramp=6)
    assert [x[2] for x in info] == ["B"] and np.allclose(out_a[30:50, :4], truth[30:50]) and np.array_equal(out_b, qb)
    out_a, out_b, info = repair_arm(qa, qb, ia, ib, np.full(80, 10.0), np.full(80, 10.0), ca, cb, 0.3, 8, 16, 6)
    assert [x[2] for x in info] == ["A"]  # tie: A is the source


def test_a_consistent_pair_is_not_touched():
    qa, qb, ia, ib, ca, cb, _ = _arm_case(bad=(0, 0))
    out_a, out_b, info = repair_arm(qa, qb, ia, ib, np.zeros(80), np.zeros(80), ca, cb, 0.3, 8, 16, 6)
    assert info == [] and np.array_equal(out_a, qa) and np.array_equal(out_b, qb)
