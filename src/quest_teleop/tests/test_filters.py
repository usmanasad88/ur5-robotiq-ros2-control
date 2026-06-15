"""Tests for the one-pole VR pose filter (Phase 2)."""

import numpy as np
import pytest

from quest_teleop.filters import VRPoseFilter
from quest_teleop.transforms import quat_to_rotvec, rotvec_to_quat


IDENT = np.array([1.0, 0.0, 0.0, 0.0])


def test_alpha_zero_is_passthrough():
    f = VRPoseFilter(alpha=0.0)
    p, q = f([1.0, 2.0, 3.0], rotvec_to_quat([0, 0, 0.5]))
    np.testing.assert_allclose(p, [1.0, 2.0, 3.0])
    np.testing.assert_allclose(q, rotvec_to_quat([0, 0, 0.5]))


def test_first_sample_passes_through():
    f = VRPoseFilter(alpha=0.8)
    p, q = f([1.0, 0.0, 0.0], IDENT)
    np.testing.assert_allclose(p, [1.0, 0.0, 0.0])
    np.testing.assert_allclose(q, IDENT)


def test_position_ema_step():
    f = VRPoseFilter(alpha=0.8)
    f([0.0, 0.0, 0.0], IDENT)            # prime
    p, _ = f([1.0, 0.0, 0.0], IDENT)     # 0.8*0 + 0.2*1
    np.testing.assert_allclose(p, [0.2, 0.0, 0.0])


def test_position_converges_to_constant_input():
    f = VRPoseFilter(alpha=0.8)
    f([0.0, 0.0, 0.0], IDENT)
    p = None
    for _ in range(200):
        p, _ = f([5.0, -2.0, 1.0], IDENT)
    np.testing.assert_allclose(p, [5.0, -2.0, 1.0], atol=1e-3)


def test_orientation_lags_toward_target():
    f = VRPoseFilter(alpha=0.8)
    f([0, 0, 0], IDENT)                          # prime at identity
    target = rotvec_to_quat([0, 0, 1.0])         # 1 rad about z
    _, q = f([0, 0, 0], target)
    ang = np.linalg.norm(quat_to_rotvec(q))
    # one step of slerp by (1-alpha)=0.2 → about 0.2 rad, well short of 1.0
    assert 0.0 < ang < 0.5


def test_orientation_converges():
    f = VRPoseFilter(alpha=0.8)
    f([0, 0, 0], IDENT)
    target = rotvec_to_quat([0, 0, 1.0])
    q = None
    for _ in range(300):
        _, q = f([0, 0, 0], target)
    assert np.linalg.norm(quat_to_rotvec(q) - [0, 0, 1.0]) < 1e-3


def test_reset_reprimes():
    f = VRPoseFilter(alpha=0.8)
    f([0, 0, 0], IDENT)
    f([1, 0, 0], IDENT)
    f.reset()
    p, _ = f([9.0, 0.0, 0.0], IDENT)     # first after reset → passthrough
    np.testing.assert_allclose(p, [9.0, 0.0, 0.0])


def test_invalid_alpha_rejected():
    with pytest.raises(ValueError):
        VRPoseFilter(alpha=1.0)
    with pytest.raises(ValueError):
        VRPoseFilter(alpha=-0.1)
