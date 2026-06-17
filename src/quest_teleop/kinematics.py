"""UR5 analytical forward kinematics, geometric Jacobian, and damped IK.

Pure math (numpy only) so it is unit-testable without ROS or a robot.
Ported from quest_servo_teleop.py:126-208 (the path that already drives the
launch_all.sh fake-hardware sim through /forward_velocity_controller).
"""

import numpy as np

from .transforms import rmat_to_quat

# Standard UR5 DH parameters (no tool offset).
_UR5_D     = np.array([0.089159, 0.0,      0.0,      0.10915, 0.09465, 0.0823])
_UR5_A     = np.array([0.0,     -0.425,   -0.39225,  0.0,     0.0,     0.0  ])
_UR5_ALPHA = np.array([np.pi/2,  0.0,      0.0,      np.pi/2,-np.pi/2, 0.0  ])

# UR joint order as reported by joint_state_broadcaster.
UR_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


def _dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1],
    ])


def ur5_fk_and_jacobian(q):
    """UR5 TCP pose and 6×6 geometric Jacobian in the base frame.

    Parameters
    ----------
    q : array-like, shape (6,)
        Joint angles [rad] in UR order (see UR_JOINT_NAMES).

    Returns
    -------
    pos  : (3,)   TCP position in base frame
    rot  : (3,3)  TCP rotation matrix
    quat : (4,)   TCP quaternion [w, x, y, z]
    J    : (6,6)  geometric Jacobian [linear; angular]
    """
    q = np.asarray(q, dtype=float)
    T = [np.eye(4)]
    for i in range(6):
        T.append(T[-1] @ _dh_transform(q[i], _UR5_D[i], _UR5_A[i], _UR5_ALPHA[i]))

    p_ee = T[6][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z_i = T[i][:3, 2]           # z-axis of frame i
        p_i = T[i][:3, 3]           # origin of frame i
        J[:3, i] = np.cross(z_i, p_ee - p_i)   # linear part
        J[3:, i] = z_i                          # angular part

    return p_ee, T[6][:3, :3], rmat_to_quat(T[6][:3, :3]), J


def cartesian_to_joint_vel(J, cart_vel, max_joint_vel=1.5, damping=0.05):
    """Damped least-squares IK: q_dot = J^T (J J^T + λ²I)^{-1} x_dot.

    The Tikhonov damping keeps q_dot finite near singularities. If any joint
    exceeds max_joint_vel the whole vector is scaled down (direction kept).
    """
    J = np.asarray(J, dtype=float)
    cart_vel = np.asarray(cart_vel, dtype=float)
    lam2 = damping ** 2
    A = J @ J.T + lam2 * np.eye(6)
    q_dot = J.T @ np.linalg.solve(A, cart_vel)
    max_v = np.max(np.abs(q_dot)) if q_dot.size else 0.0
    if max_v > max_joint_vel:
        q_dot = q_dot * (max_joint_vel / max_v)
    return q_dot
