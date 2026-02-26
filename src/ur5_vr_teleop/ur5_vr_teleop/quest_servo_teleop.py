#!/usr/bin/env python3
"""
Quest 3S VR teleop for simulated UR5 via direct joint velocity control.

Bypasses MoveIt/cuRobo entirely:
  Quest → DROID P-controller → Cartesian vel → UR5 Jacobian IK → joint velocities
                                              → /forward_velocity_controller/commands

Same control law as quest_rtde_teleop.py (real robot), but instead of
ur_rtde.speedL() it publishes Float64MultiArray to the ROS 2 velocity controller.

The UR5 Jacobian is computed analytically from /joint_states — no MoveGroup needed.

Usage:
    ros2 run ur5_vr_teleop quest_servo_teleop
    # or via script:
    ./run_quest_servo_teleop.sh

Prerequisites:
    launch_all.sh must be running (for the UR5 driver + joint_states).
    forward_velocity_controller must be active (the script activates it if needed).

Controls:
    Joystick click  = reset forward direction (do FIRST)
    Hold GRIP       = robot moves
    Trigger         = gripper /ur5/gripper_cmd
    Ctrl-C          = quit (sends zero velocities first)
"""

import threading
import time
import sys
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Float64MultiArray, Float64
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

try:
    from oculus_reader.reader import OculusReader
except ImportError:
    print("ERROR: oculus_reader not found.")
    print("  sudo pip3 install git+https://github.com/rail-berkeley/oculus_reader.git")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Rotation helpers (no scipy / tf2 dependency)
# ---------------------------------------------------------------------------

def rotvec_to_quat(rv):
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rv / angle
    s = np.sin(angle / 2.0)
    return np.array([np.cos(angle / 2.0), axis[0]*s, axis[1]*s, axis[2]*s])


def quat_to_rotvec(q):
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


def quat_diff(target, source):
    """Relative rotation: target * source^{-1}  (matches DROID quat_diff).
    All quats are [w, x, y, z] scalar-first."""
    return quat_mult(target, quat_conj(source))


def quat_mult(q1, q2):
    w1,x1,y1,z1 = q1;  w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def vec_to_reorder_mat(vec):
    n = len(vec)
    M = np.zeros((n, n))
    for i, v in enumerate(vec):
        M[i, int(abs(v))-1] = np.sign(v)
    return M


# ---------------------------------------------------------------------------
# UR5 analytical forward kinematics + Jacobian
# ---------------------------------------------------------------------------
# DH parameters for UR5 (standard, no prefix)
_UR5_D     = np.array([0.089159, 0.0,      0.0,      0.10915, 0.09465, 0.0823])
_UR5_A     = np.array([0.0,     -0.425,   -0.39225,  0.0,     0.0,     0.0  ])
_UR5_ALPHA = np.array([np.pi/2,  0.0,      0.0,      np.pi/2,-np.pi/2, 0.0  ])


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
    """
    Compute UR5 TCP pose and 6×6 geometric Jacobian in base frame.

    Parameters
    ----------
    q : array-like, shape (6,)
        Joint angles [rad] in UR order: shoulder_pan, shoulder_lift, elbow,
        wrist_1, wrist_2, wrist_3.

    Returns
    -------
    pos   : ndarray (3,)  — TCP position in base frame
    rot   : ndarray (3,3) — TCP rotation matrix
    quat  : ndarray (4,)  — TCP quaternion [w,x,y,z]
    J     : ndarray (6,6) — geometric Jacobian [linear; angular]
    """
    T = [np.eye(4)]
    for i in range(6):
        Ti = _dh_transform(q[i], _UR5_D[i], _UR5_A[i], _UR5_ALPHA[i])
        T.append(T[-1] @ Ti)

    p_ee = T[6][:3, 3]

    J = np.zeros((6, 6))
    for i in range(6):
        z_i = T[i][:3, 2]           # z-axis of frame i
        p_i = T[i][:3, 3]           # origin of frame i
        J[:3, i] = np.cross(z_i, p_ee - p_i)   # linear part
        J[3:, i] = z_i                           # angular part

    return p_ee, T[6][:3, :3], rmat_to_quat(T[6][:3, :3]), J


def cartesian_to_joint_vel(J, cart_vel, max_joint_vel=1.5, damping=0.05):
    """
    Damped least-squares pseudo-inverse: q_dot = J†  * x_dot.

    damping  — Tikhonov regularisation (prevents blow-up near singularities).
    """
    # Damped pseudo-inverse: J^T (J J^T + λ²I)^{-1}
    lam2 = damping ** 2
    A = J @ J.T + lam2 * np.eye(6)
    J_pinv = J.T @ np.linalg.inv(A)
    q_dot = J_pinv @ cart_vel           # shape (6,)

    # Scale down if any joint exceeds limit
    max_v = np.max(np.abs(q_dot))
    if max_v > max_joint_vel:
        q_dot *= max_joint_vel / max_v

    return q_dot


# UR joint name order as reported by joint_state_broadcaster
_UR_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class QuestServoTeleop(Node):
    """
    Quest 3S → Cartesian velocity → UR5 Jacobian IK → joint velocity controller.

    Two threads:
      _reader_loop  — reads Quest at 50 Hz (DROID _update_internal_state)
      ROS 2 timer   — 50 Hz control loop (P-controller + Jacobian IK)
    """

    def __init__(self):
        super().__init__('quest_servo_teleop')

        # ---- parameters ----
        self.declare_parameter('controller_side',  'right')
        self.declare_parameter('pos_gain',          3.0)
        self.declare_parameter('rot_gain',          2.0)
        self.declare_parameter('max_lin_vel',       0.15)   # m/s
        self.declare_parameter('max_rot_vel',       0.75)   # rad/s
        self.declare_parameter('max_joint_vel',     1.5)    # rad/s
        self.declare_parameter('control_hz',       50.0)
        self.declare_parameter('rmat_reorder',     [-2, -1, -3, 4])
        self.declare_parameter('spatial_coeff',     1.0)
        self.declare_parameter('damping',           0.05)

        side         = self.get_parameter('controller_side').value
        self.pos_gain    = self.get_parameter('pos_gain').value
        self.rot_gain    = self.get_parameter('rot_gain').value
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_rot_vel = self.get_parameter('max_rot_vel').value
        self.max_jnt_vel = self.get_parameter('max_joint_vel').value
        hz           = self.get_parameter('control_hz').value
        reorder           = self.get_parameter('rmat_reorder').value
        self.spatial_coeff = self.get_parameter('spatial_coeff').value
        self.damping      = self.get_parameter('damping').value
        self.dt      = 1.0 / hz

        self.cid      = 'r' if side == 'right' else 'l'
        self.grip_key = self.cid.upper() + 'G'
        self.joy_key  = self.cid.upper() + 'J'
        self.trig_key = 'rightTrig' if self.cid == 'r' else 'leftTrig'
        # A/B are always on the right controller regardless of teleop side
        self.btn_a_key = 'A'
        self.btn_b_key = 'B'
        self.global_to_env = vec_to_reorder_mat(reorder)

        # ---- publishers ----
        self._vel_pub = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self._grip_pub = self.create_publisher(Float64, '/ur5/gripper_cmd', 10)

        # ---- joint state subscriber ----
        self._js_lock    = threading.Lock()
        self._joint_pos  = None        # ndarray (6,) in UR order
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # ---- save-position service clients ----
        self._executor_node = '/ur5_program_executor'
        self._set_param_cli = self.create_client(
            SetParameters, f'{self._executor_node}/set_parameters')
        self._save_pos_cli  = self.create_client(
            Trigger, f'{self._executor_node}/save_position')

        # ---- Quest shared state ----
        self._lock            = threading.Lock()
        self._poses           = {}
        self._buttons         = {}
        self._enabled         = False
        self._vr_to_global    = np.eye(4)
        self._reset_orient    = True

        self._robot_origin_pos  = None
        self._robot_origin_quat = None
        self._vr_origin_pos     = None
        self._vr_origin_quat    = None

        self._prev_enabled = False
        self._log_count    = 0

        # ---- button edge detection (A / B) ----
        self._prev_btn_a = False
        self._prev_btn_b = False

        # ---- start Quest reader thread ----
        self.get_logger().info('Connecting to Quest 3S via ADB ...')
        self._reader = OculusReader()
        self.get_logger().info('Quest connected.')
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name='quest_reader')
        self._reader_thread.start()

        # ---- control timer ----
        self.create_timer(self.dt, self._control_cb)

        self.get_logger().info(
            f'Quest Servo Teleop ready  (side={side}, pos_gain={self.pos_gain}, '
            f'max_lin={self.max_lin_vel} m/s, hz={hz})')
        self.get_logger().info('  Joystick click → reset forward direction')
        self.get_logger().info('  Hold GRIP      → robot moves')
        self.get_logger().info('  Trigger        → gripper')
        self.get_logger().info('  A button       → save position (auto name)')
        self.get_logger().info('  B button       → save position (type name in terminal)')

    # ------------------------------------------------------------------
    # Joint state callback
    # ------------------------------------------------------------------

    def _js_cb(self, msg: JointState):
        try:
            q = [msg.position[msg.name.index(n)] for n in _UR_JOINT_NAMES]
        except (ValueError, IndexError):
            return
        with self._js_lock:
            self._joint_pos = np.array(q)

    # ------------------------------------------------------------------
    # Quest reader thread  (DROID _update_internal_state)
    # ------------------------------------------------------------------

    def _reader_loop(self, hz: float = 50.0):
        while rclpy.ok():
            time.sleep(1.0 / hz)
            try:
                poses, buttons = self._reader.get_transformations_and_buttons()
            except Exception:
                continue
            if not poses:
                continue

            with self._lock:
                prev_en = self._enabled
                now_en  = bool(buttons.get(self.grip_key, False))
                toggled = prev_en != now_en

                if self._reset_orient or buttons.get(self.joy_key, False):
                    raw  = np.asarray(poses.get(self.cid, np.eye(4)))
                    stop = buttons.get(self.joy_key, False) or now_en
                    if stop:
                        self._reset_orient = False
                    try:
                        self._vr_to_global = np.linalg.inv(raw)
                    except np.linalg.LinAlgError:
                        self._vr_to_global = np.eye(4)
                        self._reset_orient = True

                if toggled:
                    self._robot_origin_pos  = None
                    self._robot_origin_quat = None
                    self._vr_origin_pos     = None
                    self._vr_origin_quat    = None

                self._poses   = poses
                self._buttons = buttons
                self._enabled = now_en

            # ---- A/B button edge detection (outside main lock, no blocking) ----
            now_a = bool(buttons.get(self.btn_a_key, False))
            now_b = bool(buttons.get(self.btn_b_key, False))

            if now_a and not self._prev_btn_a:
                # A pressed: save with auto timestamp name
                auto_name = 'pos_' + datetime.now().strftime('%H%M%S')
                threading.Thread(
                    target=self._save_position, args=(auto_name,),
                    daemon=True, name='save_pos_a').start()

            if now_b and not self._prev_btn_b:
                # B pressed: prompt for name in terminal (non-blocking thread)
                threading.Thread(
                    target=self._prompt_and_save,
                    daemon=True, name='save_pos_b').start()

            self._prev_btn_a = now_a
            self._prev_btn_b = now_b

    # ------------------------------------------------------------------
    # Save named position helpers
    # ------------------------------------------------------------------

    def _save_position(self, name: str):
        """Set save_position_name param on executor node, then call save_position."""
        # Check services available (don't block forever)
        if not self._set_param_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'Save skipped: {self._executor_node}/set_parameters not available '
                '(is program_executor_node running?)')
            return
        if not self._save_pos_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f'Save skipped: {self._executor_node}/save_position not available')
            return

        # Set save_position_name parameter
        pv = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=name)
        param = Parameter(name='save_position_name', value=pv)
        set_req = SetParameters.Request(parameters=[param])
        set_future = self._set_param_cli.call_async(set_req)
        rclpy.spin_until_future_complete(self, set_future, timeout_sec=3.0)
        if not set_future.done() or not set_future.result():
            self.get_logger().error(f"Save '{name}': failed to set parameter")
            return

        # Call save_position
        save_future = self._save_pos_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, save_future, timeout_sec=5.0)
        if save_future.done() and save_future.result():
            res = save_future.result()
            if res.success:
                self.get_logger().info(f"[SAVED] '{name}' — {res.message}")
                print(f"\n  [SAVED] position '{name}'\n")
            else:
                self.get_logger().error(f"Save failed: {res.message}")
        else:
            self.get_logger().error(f"Save '{name}': service call timed out")

    def _prompt_and_save(self):
        """Prompt for a position name on stdin, then save."""
        print('\n  [B] Enter position name (Enter to confirm): ', end='', flush=True)
        try:
            name = input().strip()
        except (EOFError, KeyboardInterrupt):
            print('  (cancelled)')
            return
        if not name:
            print('  (empty name — cancelled)')
            return
        self._save_position(name)

    # ------------------------------------------------------------------
    # Control timer  (50 Hz)
    # ------------------------------------------------------------------

    def _control_cb(self):
        with self._lock:
            poses        = dict(self._poses)
            buttons      = dict(self._buttons)
            enabled      = self._enabled
            vr_to_global = self._vr_to_global.copy()
            origin_ready = self._robot_origin_pos is not None

        # ---- gripper (always active) ----
        trig = float(buttons.get(self.trig_key, [0.0])[0])
        grip_msg = Float64()
        grip_msg.data = float(np.clip(trig, 0.0, 1.0))
        self._grip_pub.publish(grip_msg)

        # ---- not enabled: send zero velocities ----
        if not enabled:
            if self._prev_enabled:
                self._send_zero_vel()
                self.get_logger().info('Grip released — robot stopped.')
            self._prev_enabled = False
            return

        if not self._prev_enabled:
            self.get_logger().info('Grip pressed — setting origins ...')
        self._prev_enabled = True

        # ---- current robot state from FK ----
        with self._js_lock:
            q = self._joint_pos.copy() if self._joint_pos is not None else None

        if q is None:
            self.get_logger().warn('No joint state yet', throttle_duration_sec=2.0)
            return

        current_pos, current_rot, current_quat, J = ur5_fk_and_jacobian(q)

        # ---- VR pose in env frame (DROID _process_reading) ----
        if self.cid not in poses:
            return
        raw     = np.asarray(poses[self.cid])
        mat     = self.global_to_env @ vr_to_global @ raw
        vr_pos  = self.spatial_coeff * mat[:3, 3]
        vr_quat = rmat_to_quat(mat[:3, :3])

        # ---- set origins on first tick after grip press ----
        if not origin_ready:
            with self._lock:
                self._robot_origin_pos  = current_pos.copy()
                self._robot_origin_quat = current_quat.copy()
                self._vr_origin_pos     = vr_pos.copy()
                self._vr_origin_quat    = vr_quat.copy()
            self.get_logger().info(
                f'  Origin set: TCP {np.round(current_pos, 4)}')
            return

        with self._lock:
            ro_pos  = self._robot_origin_pos.copy()
            ro_quat = self._robot_origin_quat.copy()
            vo_pos  = self._vr_origin_pos.copy()
            vo_quat = self._vr_origin_quat.copy()

        # ---- DROID P-controller (Cartesian error → velocity) ----
        # Position: same as DROID — offset tracking
        robot_pos_offset = current_pos - ro_pos
        target_pos_offset = vr_pos - vo_pos
        pos_error = target_pos_offset - robot_pos_offset
        lin_vel   = pos_error * self.pos_gain

        # Rotation: match DROID _calculate_action exactly
        #   robot_quat_offset  = quat_diff(current, robot_origin)   = current * robot_origin^{-1}
        #   target_quat_offset = quat_diff(vr,      vr_origin)      = vr      * vr_origin^{-1}
        #   quat_action        = quat_diff(target_offset, robot_offset)
        robot_quat_offset  = quat_diff(current_quat, ro_quat)
        target_quat_offset = quat_diff(vr_quat,      vo_quat)
        quat_error         = quat_diff(target_quat_offset, robot_quat_offset)
        if quat_error[0] < 0:
            quat_error = -quat_error
        ang_vel = quat_to_rotvec(quat_error) * self.rot_gain

        # ---- velocity limits ----
        ln = np.linalg.norm(lin_vel)
        rn = np.linalg.norm(ang_vel)
        if ln > self.max_lin_vel:
            lin_vel = lin_vel * self.max_lin_vel / ln
        if rn > self.max_rot_vel:
            ang_vel = ang_vel * self.max_rot_vel / rn

        # ---- Cartesian → joint velocities via Jacobian ----
        cart_vel = np.concatenate([lin_vel, ang_vel])
        q_dot    = cartesian_to_joint_vel(J, cart_vel, self.max_jnt_vel, self.damping)

        # ---- publish ----
        msg = Float64MultiArray()
        msg.data = q_dot.tolist()
        self._vel_pub.publish(msg)

        # ---- periodic log ----
        self._log_count += 1
        if self._log_count % 50 == 0:
            self.get_logger().info(
                f'lin=[{lin_vel[0]:+.3f},{lin_vel[1]:+.3f},{lin_vel[2]:+.3f}] '
                f'ang=[{ang_vel[0]:+.3f},{ang_vel[1]:+.3f},{ang_vel[2]:+.3f}]  '
                f'|q_dot|={np.linalg.norm(q_dot):.3f}'
            )

    def _send_zero_vel(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self._vel_pub.publish(msg)

    def destroy_node(self):
        self._send_zero_vel()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QuestServoTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
