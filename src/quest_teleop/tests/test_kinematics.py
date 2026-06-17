"""Tests for the UR5 FK / Jacobian / damped IK used by the ROS backend."""

import numpy as np
import pytest

from quest_teleop.kinematics import (
    cartesian_to_joint_vel,
    ur5_fk_and_jacobian,
)
from quest_teleop.transforms import quat_to_rotvec, rmat_to_quat

RNG = np.random.default_rng(7)


def random_q(n=20):
    # keep away from the kinematic singularities at exactly 0 elbow/wrist
    return RNG.uniform(-2.0, 2.0, size=(n, 6))


def test_fk_returns_orthonormal_rotation():
    for q in random_q(10):
        _pos, R, quat, _J = ur5_fk_and_jacobian(q)
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-9)
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-9)
        # quat is consistent with R
        np.testing.assert_allclose(quat, rmat_to_quat(R), atol=1e-9)


def test_fk_is_deterministic():
    q = np.array([0.1, -0.5, 0.7, -1.2, 0.3, 0.9])
    a = ur5_fk_and_jacobian(q)
    b = ur5_fk_and_jacobian(q)
    np.testing.assert_array_equal(a[0], b[0])


@pytest.mark.parametrize("q", random_q(12))
def test_jacobian_matches_finite_difference(q):
    eps = 1e-6
    pos0, R0, _q0, J = ur5_fk_and_jacobian(q)
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        pos1, R1, _q1, _J1 = ur5_fk_and_jacobian(q + dq)
        # linear part: d(pos)/d(qi)
        lin_fd = (pos1 - pos0) / eps
        np.testing.assert_allclose(J[:3, i], lin_fd, atol=1e-4)
        # angular part: omega ≈ rotvec(R1 R0^T)/eps
        ang_fd = quat_to_rotvec(rmat_to_quat(R1 @ R0.T)) / eps
        np.testing.assert_allclose(J[3:, i], ang_fd, atol=1e-4)


@pytest.mark.parametrize("q", random_q(12))
def test_ik_reproduces_cartesian_velocity(q):
    # Away from singularities, damped IK with small λ should reproduce the
    # commanded twist: J q_dot ≈ x_dot.
    _pos, _R, _quat, J = ur5_fk_and_jacobian(q)
    # Damped LS only tracks well away from singularities; gate on the smallest
    # singular value so this asserts accuracy only where it is expected.
    if np.linalg.svd(J, compute_uv=False)[-1] < 0.1:
        pytest.skip("near-singular config")
    x_dot = np.array([0.02, -0.01, 0.03, 0.0, 0.05, 0.0])
    q_dot = cartesian_to_joint_vel(J, x_dot, max_joint_vel=10.0, damping=1e-4)
    np.testing.assert_allclose(J @ q_dot, x_dot, atol=1e-3)


def test_ik_respects_joint_velocity_limit():
    q = np.array([0.3, -0.8, 1.0, -1.5, 0.7, 0.2])
    _pos, _R, _quat, J = ur5_fk_and_jacobian(q)
    x_dot = np.array([5.0, 5.0, 5.0, 0.0, 0.0, 0.0])   # absurdly fast → clamp
    q_dot = cartesian_to_joint_vel(J, x_dot, max_joint_vel=1.5, damping=0.05)
    assert np.max(np.abs(q_dot)) <= 1.5 + 1e-9


def test_ik_direction_preserved_when_clamped():
    q = np.array([0.3, -0.8, 1.0, -1.5, 0.7, 0.2])
    _pos, _R, _quat, J = ur5_fk_and_jacobian(q)
    x_dot = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    slow = cartesian_to_joint_vel(J, x_dot, max_joint_vel=10.0, damping=0.05)
    fast = cartesian_to_joint_vel(J, 100 * x_dot, max_joint_vel=1.5, damping=0.05)
    # clamped vector is a positive scalar multiple of the unclamped direction
    k = np.linalg.norm(fast) / np.linalg.norm(slow)
    np.testing.assert_allclose(fast, slow * k, atol=1e-9)
