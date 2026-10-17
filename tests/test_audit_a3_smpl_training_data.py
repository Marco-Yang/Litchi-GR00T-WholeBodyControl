"""Pure helpers of the A3 SMPL training-data audit (mirror asymmetry, soft-limit occupancy, affine fit, joint layout)."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("joblib")

from gear_sonic.data_process.audit_a3_smpl_training_data import (  # noqa: E402
    A3_WRIST_COLS,
    affine_fit,
    limit_occupancy,
    mirror_asymmetry,
    mirror_pairs,
)


def test_mirror_asymmetry_reports_a_constant_left_right_offset_as_bias():
    rng = np.random.default_rng(0)
    twin = rng.normal(size=(2000, 2))
    x = np.stack([-twin[:, 1] - 0.2, twin[:, 0]], axis=1)  # joint 0 mirrors with sign -1 plus a 0.2 rad calibration error
    out = mirror_asymmetry(x, twin, [(0, 1), (1, 0)])
    assert out[0]["sign"] == -1.0
    assert out[0]["bias"] == pytest.approx(-0.2, abs=1e-6)
    assert out[0]["std"] == pytest.approx(0.0, abs=1e-9)
    assert out[0]["gt_0p1"] == 1.0
    assert out[1]["rmse"] == pytest.approx(0.0, abs=1e-9)  # a perfect mirror: no error at all


def test_limit_occupancy_soft_range_is_the_inner_ninety_percent():
    lo, hi = -0.1, 2.5  # 5 % of the 2.6 rad range = 0.13 rad is cut at each end
    q = np.array([-0.09, -0.02, 0.5, 1.0, 2.3, 2.49])
    o = limit_occupancy(q, lo, hi)
    assert o["outside_soft"] == pytest.approx(3 / 6)  # soft range is [0.03, 2.37]: -0.09, -0.02 and 2.49 are outside
    assert o["near_lo"] == pytest.approx(1 / 6)  # within 0.03 rad of the lower limit: -0.09
    assert o["near_hi"] == pytest.approx(1 / 6)  # 2.49


def test_affine_fit_recovers_gain_and_offset():
    x = np.linspace(-1.0, 1.0, 101)
    gain, off, res = affine_fit(x, 0.6 * x - 0.25)
    assert (gain, off) == (pytest.approx(0.6), pytest.approx(-0.25))
    assert res == pytest.approx(0.0, abs=1e-9)


def test_joint_layout_matches_the_a3_policy_view_and_g1_29dof():
    a3 = mirror_pairs("a3")
    assert a3["arm"][4:] == [(7, 14), (8, 15), (9, 16)]  # wrist roll / pitch / yaw, left -> right
    assert [c for pair in a3["arm"][4:] for c in pair[:1]] == A3_WRIST_COLS[:3]
    assert A3_WRIST_COLS[3:] == [14, 15, 16]
    assert mirror_pairs("g1")["arm"][0] == (15, 22)
    # every column 0..28 is used exactly once by the A3 pairs, as a joint or as its counterpart
    cols = sorted({c for grp in ("waist", "arm", "leg") for pair in a3[grp] for c in pair})
    assert cols == list(range(29))
