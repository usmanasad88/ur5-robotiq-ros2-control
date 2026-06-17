"""Known-answer and round-trip tests for quest_teleop.transforms.

scipy is a test-only dependency, used as the reference implementation.
scipy quaternions are [x, y, z, w]; ours are [w, x, y, z].
"""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R

from quest_teleop.transforms import (
    limit_velocity,
    quat_conj,
    quat_diff,
    quat_mult,
    quat_to_rmat,
    quat_to_rotvec,
    rmat_to_quat,
    rotvec_to_quat,
    scale_to_max_norm,
    slerp,
    twist_about_axis,
    vec_to_reorder_mat,
)

RNG = np.random.default_rng(42)


def to_scipy(q_wxyz):
    w, x, y, z = q_wxyz
    return np.array([x, y, z, w])


def from_scipy(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([w, x, y, z])


def assert_quat_close(q1, q2, atol=1e-9):
    """q and -q are the same rotation."""
    if np.dot(q1, q2) < 0:
        q2 = -q2
    np.testing.assert_allclose(q1, q2, atol=atol)


def random_rotvecs(n=50):
    axes = RNG.normal(size=(n, 3))
    axes /= np.linalg.norm(axes, axis=1, keepdims=True)
    angles = RNG.uniform(1e-6, np.pi - 1e-6, size=(n, 1))
    return axes * angles


# ---------------------------------------------------------------------------
# rotvec <-> quat
# ---------------------------------------------------------------------------

def test_rotvec_to_quat_identity():
    np.testing.assert_allclose(rotvec_to_quat([0, 0, 0]), [1, 0, 0, 0])


def test_rotvec_to_quat_known():
    # 90 deg about z
    q = rotvec_to_quat([0, 0, np.pi / 2])
    s = np.sqrt(0.5)
    np.testing.assert_allclose(q, [s, 0, 0, s], atol=1e-12)


@pytest.mark.parametrize("rv", random_rotvecs(25))
def test_rotvec_to_quat_vs_scipy(rv):
    ours = rotvec_to_quat(rv)
    ref = from_scipy(R.from_rotvec(rv).as_quat())
    assert_quat_close(ours, ref)


@pytest.mark.parametrize("rv", random_rotvecs(25))
def test_rotvec_quat_roundtrip(rv):
    np.testing.assert_allclose(quat_to_rotvec(rotvec_to_quat(rv)), rv, atol=1e-9)


def test_quat_to_rotvec_handles_negative_w():
    rv = np.array([0.3, -0.2, 0.5])
    q = rotvec_to_quat(rv)
    np.testing.assert_allclose(quat_to_rotvec(-q), rv, atol=1e-9)


def test_quat_to_rotvec_identity():
    np.testing.assert_allclose(quat_to_rotvec([1, 0, 0, 0]), [0, 0, 0])


# ---------------------------------------------------------------------------
# rmat <-> quat
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("rv", random_rotvecs(25))
def test_rmat_to_quat_vs_scipy(rv):
    M = R.from_rotvec(rv).as_matrix()
    assert_quat_close(rmat_to_quat(M), from_scipy(R.from_matrix(M).as_quat()))


def test_rmat_to_quat_branch_coverage():
    # Hit the three non-trace branches: 180-deg rotations about x, y, z
    for axis in np.eye(3):
        M = R.from_rotvec(axis * np.pi).as_matrix()
        assert_quat_close(rmat_to_quat(M), from_scipy(R.from_matrix(M).as_quat()),
                          atol=1e-9)


@pytest.mark.parametrize("rv", random_rotvecs(10))
def test_quat_to_rmat_roundtrip(rv):
    q = rotvec_to_quat(rv)
    assert_quat_close(rmat_to_quat(quat_to_rmat(q)), q)


# ---------------------------------------------------------------------------
# quat algebra
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("rv1,rv2", list(zip(random_rotvecs(15), random_rotvecs(15))))
def test_quat_mult_vs_scipy(rv1, rv2):
    q1, q2 = rotvec_to_quat(rv1), rotvec_to_quat(rv2)
    ours = quat_mult(q1, q2)
    ref = from_scipy((R.from_rotvec(rv1) * R.from_rotvec(rv2)).as_quat())
    assert_quat_close(ours, ref)


def test_quat_conj_inverts():
    q = rotvec_to_quat([0.1, 0.7, -0.4])
    np.testing.assert_allclose(quat_mult(q, quat_conj(q)), [1, 0, 0, 0], atol=1e-12)


def test_quat_diff_recovers_displacement():
    # quat_diff(q1, q2) is the rotation that takes q2 to q1: d * q2 == q1
    q1 = rotvec_to_quat([0.2, -0.1, 0.9])
    q2 = rotvec_to_quat([-0.5, 0.3, 0.1])
    d = quat_diff(q1, q2)
    assert_quat_close(quat_mult(d, q2), q1)


# ---------------------------------------------------------------------------
# reorder matrix
# ---------------------------------------------------------------------------

def test_reorder_identity():
    np.testing.assert_allclose(vec_to_reorder_mat([1, 2, 3, 4]), np.eye(4))


def test_reorder_droid_default():
    # [-2, -1, -3, 4]: out_x = -in_y, out_y = -in_x, out_z = -in_z
    M = vec_to_reorder_mat([-2, -1, -3, 4])
    v = np.array([1.0, 2.0, 3.0, 1.0])
    np.testing.assert_allclose(M @ v, [-2.0, -1.0, -3.0, 1.0])


def test_reorder_is_orthonormal():
    M = vec_to_reorder_mat([-2, -1, -3, 4])[:3, :3]
    np.testing.assert_allclose(M @ M.T, np.eye(3), atol=1e-12)


# ---------------------------------------------------------------------------
# slerp  (Phase 2)
# ---------------------------------------------------------------------------

def test_slerp_endpoints():
    q0 = rotvec_to_quat([0.3, -0.2, 0.5])
    q1 = rotvec_to_quat([-0.5, 0.4, 0.1])
    assert_quat_close(slerp(q0, q1, 0.0), q0)
    assert_quat_close(slerp(q0, q1, 1.0), q1)


@pytest.mark.parametrize("rv0,rv1", list(zip(random_rotvecs(15), random_rotvecs(15))))
@pytest.mark.parametrize("t", [0.1, 0.25, 0.5, 0.8])
def test_slerp_vs_scipy(rv0, rv1, t):
    from scipy.spatial.transform import Slerp
    q0, q1 = rotvec_to_quat(rv0), rotvec_to_quat(rv1)
    key = R.from_quat(np.stack([to_scipy(q0), to_scipy(q1)]))
    ref = from_scipy(Slerp([0, 1], key)([t])[0].as_quat())
    assert_quat_close(slerp(q0, q1, t), ref, atol=1e-9)


def test_slerp_takes_shortest_path():
    q0 = rotvec_to_quat([0, 0, 0.1])
    q1 = -rotvec_to_quat([0, 0, 0.2])   # same rotation family, opposite sign
    mid = slerp(q0, q1, 0.5)
    # halfway between 0.1 and 0.2 rad about z is ~0.15 rad — small angle
    assert np.linalg.norm(quat_to_rotvec(mid)) < 0.25


def test_slerp_nearly_aligned():
    q0 = rotvec_to_quat([0.0, 0.0, 1e-7])
    q1 = rotvec_to_quat([0.0, 0.0, 2e-7])
    out = slerp(q0, q1, 0.5)
    np.testing.assert_allclose(np.linalg.norm(out), 1.0, atol=1e-9)


# ---------------------------------------------------------------------------
# limit_velocity  (uniform saturation, Phase 2)
# ---------------------------------------------------------------------------

def test_limit_velocity_no_saturation_passthrough():
    lin = np.array([0.05, 0.0, 0.0])
    ang = np.array([0.1, 0.0, 0.0])
    out_l, out_a = limit_velocity(lin, ang, 0.15, 0.75)
    np.testing.assert_allclose(out_l, lin)
    np.testing.assert_allclose(out_a, ang)


def test_limit_velocity_scales_both_uniformly():
    # lin over limit, ang under: BOTH scale by the lin ratio (screw preserved)
    lin = np.array([0.30, 0.0, 0.0])    # norm 0.30, limit 0.15 → ratio 0.5
    ang = np.array([0.0, 0.20, 0.0])    # norm 0.20, limit 0.75 → fine alone
    out_l, out_a = limit_velocity(lin, ang, 0.15, 0.75)
    np.testing.assert_allclose(out_l, lin * 0.5)
    np.testing.assert_allclose(out_a, ang * 0.5)


def test_limit_velocity_preserves_direction():
    lin = np.array([0.3, -0.4, 0.0])    # norm 0.5
    ang = np.array([1.0, 0.0, 1.0])     # norm sqrt(2)
    out_l, out_a = limit_velocity(lin, ang, 0.15, 0.75)
    # directions unchanged
    np.testing.assert_allclose(out_l / np.linalg.norm(out_l),
                               lin / np.linalg.norm(lin), atol=1e-12)
    np.testing.assert_allclose(out_a / np.linalg.norm(out_a),
                               ang / np.linalg.norm(ang), atol=1e-12)
    # tightest constraint binds: both within limits
    assert np.linalg.norm(out_l) <= 0.15 + 1e-12
    assert np.linalg.norm(out_a) <= 0.75 + 1e-12


def test_scale_to_max_norm_passes_small_through():
    v = np.array([0.001, 0.002, 0.0])
    np.testing.assert_allclose(scale_to_max_norm(v, 0.01), v)


def test_scale_to_max_norm_clamps_and_keeps_direction():
    v = np.array([0.3, -0.4, 0.0])     # norm 0.5
    out = scale_to_max_norm(v, 0.1)
    np.testing.assert_allclose(np.linalg.norm(out), 0.1, atol=1e-12)
    np.testing.assert_allclose(out / np.linalg.norm(out), v / np.linalg.norm(v))


def test_scale_to_max_norm_disabled_when_nonpositive():
    v = np.array([5.0, 0.0, 0.0])
    np.testing.assert_allclose(scale_to_max_norm(v, 0.0), v)
    np.testing.assert_allclose(scale_to_max_norm(v, -1.0), v)


def test_limit_velocity_uses_min_ratio():
    # both over: ang is the tighter constraint
    lin = np.array([0.30, 0.0, 0.0])    # ratio 0.5
    ang = np.array([3.0, 0.0, 0.0])     # norm 3.0, limit 0.75 → ratio 0.25
    out_l, out_a = limit_velocity(lin, ang, 0.15, 0.75)
    np.testing.assert_allclose(out_l, lin * 0.25)
    np.testing.assert_allclose(out_a, ang * 0.25)


# ---------------------------------------------------------------------------
# twist_about_axis  (yaw-only forward reset, Phase 2)
# ---------------------------------------------------------------------------

def test_twist_of_pure_yaw_is_identity_op():
    # A pure rotation about z, decomposed about z, returns itself
    q = rotvec_to_quat([0, 0, 0.7])
    assert_quat_close(twist_about_axis(q, [0, 0, 1]), q)


def test_twist_of_pure_tilt_is_identity():
    # A pure rotation about x has no z-component → twist about z is identity
    q = rotvec_to_quat([0.6, 0, 0])
    assert_quat_close(twist_about_axis(q, [0, 0, 1]), [1, 0, 0, 0])


def test_twist_axis_sign_invariant():
    q = rotvec_to_quat([0.2, -0.1, 0.5])
    assert_quat_close(twist_about_axis(q, [0, 0, 1]),
                      twist_about_axis(q, [0, 0, -1]))


@pytest.mark.parametrize("rv", random_rotvecs(15))
def test_twist_is_rotation_about_axis(rv):
    # twist's rotation axis is parallel to the decomposition axis
    axis = np.array([0.0, 0.0, 1.0])
    twist = twist_about_axis(rotvec_to_quat(rv), axis)
    rv_twist = quat_to_rotvec(twist)
    if np.linalg.norm(rv_twist) > 1e-6:
        cosang = np.dot(rv_twist, axis) / np.linalg.norm(rv_twist)
        assert abs(abs(cosang) - 1.0) < 1e-9


@pytest.mark.parametrize("rv", random_rotvecs(15))
def test_swing_twist_reconstructs_and_swing_perp(rv):
    # Defining property: q = swing * twist, twist about `axis`, swing ⊥ axis.
    from quest_teleop.transforms import quat_mult, quat_conj
    axis = np.array([0.0, 0.0, 1.0])
    q = rotvec_to_quat(rv)
    twist = twist_about_axis(q, axis)
    swing = quat_mult(q, quat_conj(twist))      # q = swing * twist
    assert_quat_close(quat_mult(swing, twist), q)
    rv_swing = quat_to_rotvec(swing)
    if np.linalg.norm(rv_swing) > 1e-6:
        # swing axis is perpendicular to the twist axis
        assert abs(np.dot(rv_swing, axis)) < 1e-9
