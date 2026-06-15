"""Pure rotation / transform math (no scipy, no ROS).

Quaternion convention: [w, x, y, z].
"""

import numpy as np


def rotvec_to_quat(rv):
    """Rotation vector (axis×angle) → quaternion [w, x, y, z]."""
    rv = np.asarray(rv, dtype=float)
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rv / angle
    s = np.sin(angle / 2.0)
    return np.array([np.cos(angle / 2.0), axis[0]*s, axis[1]*s, axis[2]*s])


def quat_to_rotvec(q):
    """Quaternion [w, x, y, z] → rotation vector (axis×angle)."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    w, x, y, z = q
    angle = 2.0 * np.arccos(np.clip(w, -1.0, 1.0))
    s = np.sin(angle / 2.0)
    if s < 1e-10:
        return np.zeros(3)
    return np.array([x, y, z]) / s * angle


def rmat_to_quat(R):
    """3×3 rotation matrix → quaternion [w, x, y, z]."""
    R = np.asarray(R, dtype=float)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return np.array([0.25/s,
                         (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s])
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return np.array([(R[2,1]-R[1,2])/s, 0.25*s,
                         (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s])
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return np.array([(R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s,
                         0.25*s, (R[1,2]+R[2,1])/s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return np.array([(R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s,
                         (R[1,2]+R[2,1])/s, 0.25*s])


def quat_to_rmat(q):
    """Quaternion [w, x, y, z] → 3×3 rotation matrix."""
    q = np.asarray(q, dtype=float)
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_diff(q1, q2):
    """Rotation from q2 to q1: q1 * conj(q2)  — same as DROID."""
    return quat_mult(q1, quat_conj(q2))


def vec_to_reorder_mat(vec):
    """Signed-index → 4×4 axis permutation matrix (DROID convention)."""
    n = len(vec)
    M = np.zeros((n, n))
    for i, v in enumerate(vec):
        M[i, int(abs(v))-1] = np.sign(v)
    return M


def slerp(q0, q1, t):
    """Spherical linear interpolation between quats [w,x,y,z] at fraction t.

    t=0 → q0, t=1 → q1. Takes the shortest path. Used by the one-pole
    orientation filter (Open-Teach Filter uses scipy Slerp; this is the
    scipy-free equivalent).
    """
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:          # shortest path
        q1 = -q1
        dot = -dot
    if dot > 0.9995:       # nearly aligned → linear interp + renormalize
        q = q0 + t * (q1 - q0)
        return q / np.linalg.norm(q)
    theta0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta = theta0 * t
    q2 = q1 - q0 * dot
    q2 = q2 / np.linalg.norm(q2)
    return q0 * np.cos(theta) + q2 * np.sin(theta)


def limit_velocity(lin_vel, ang_vel, max_lin, max_ang):
    """Uniform saturation (DROID _limit_velocity).

    If either the linear or angular velocity exceeds its limit, scale BOTH by
    the same factor so the screw axis (motion direction) is preserved.
    """
    lin_vel = np.asarray(lin_vel, dtype=float)
    ang_vel = np.asarray(ang_vel, dtype=float)
    ln = np.linalg.norm(lin_vel)
    an = np.linalg.norm(ang_vel)
    scale = 1.0
    if max_lin > 0 and ln > max_lin:
        scale = min(scale, max_lin / ln)
    if max_ang > 0 and an > max_ang:
        scale = min(scale, max_ang / an)
    return lin_vel * scale, ang_vel * scale


def twist_about_axis(q, axis):
    """Swing-twist decomposition: the component of rotation q about `axis`.

    Returns the twist quaternion (rotation purely about `axis`). The swing
    (tilt off the axis) is discarded. Used for yaw-only forward reset: keep
    only the rotation about the vertical so controller tilt doesn't tilt the
    mapped workspace.
    """
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    proj = np.dot(q[1:], axis) * axis
    twist = np.array([q[0], proj[0], proj[1], proj[2]])
    n = np.linalg.norm(twist)
    if n < 1e-10:           # 180° about a perpendicular axis → no twist
        return np.array([1.0, 0.0, 0.0, 0.0])
    twist = twist / n
    if twist[0] < 0:        # canonical hemisphere
        twist = -twist
    return twist
