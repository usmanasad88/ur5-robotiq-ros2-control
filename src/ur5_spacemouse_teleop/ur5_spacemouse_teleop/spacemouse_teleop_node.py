#!/usr/bin/env python3
"""
SpaceMouse teleoperation node for UR5 robot via direct joint velocity control.

Bypasses MoveIt/cuRobo entirely:
  SpaceMouse axes → scale to Cartesian velocity → UR5 Jacobian IK → joint velocities
                                                 → /forward_velocity_controller/commands

Same control architecture as quest_servo_teleop.py but with SpaceMouse input
instead of Quest 3S VR controllers.

Usage:
    ros2 run ur5_spacemouse_teleop spacemouse_teleop_node
    # or via script:
    ./run_spacemouse_teleop.sh

Prerequisites:
    launch_all.sh must be running (for the UR5 driver + joint_states).
    forward_velocity_controller must be active.
"""

import subprocess
import threading
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
    import pyspacemouse
except ImportError:
    print("Error: pyspacemouse library not found!")
    print("Please install it using: pip install pyspacemouse")
    sys.exit(1)


# ---------------------------------------------------------------------------
# UR5 analytical forward kinematics + Jacobian
# ---------------------------------------------------------------------------
# DH parameters for UR5
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
    Compute UR5 TCP pose and 6x6 geometric Jacobian in base frame.

    Returns
    -------
    pos : ndarray (3,)  — TCP position in base frame
    J   : ndarray (6,6) — geometric Jacobian [linear; angular]
    """
    T = [np.eye(4)]
    for i in range(6):
        Ti = _dh_transform(q[i], _UR5_D[i], _UR5_A[i], _UR5_ALPHA[i])
        T.append(T[-1] @ Ti)

    p_ee = T[6][:3, 3]

    J = np.zeros((6, 6))
    for i in range(6):
        z_i = T[i][:3, 2]
        p_i = T[i][:3, 3]
        J[:3, i] = np.cross(z_i, p_ee - p_i)
        J[3:, i] = z_i

    return p_ee, J


def cartesian_to_joint_vel(J, cart_vel, max_joint_vel=1.5, damping=0.05):
    """Damped least-squares pseudo-inverse: q_dot = J† * x_dot."""
    lam2 = damping ** 2
    A = J @ J.T + lam2 * np.eye(6)
    J_pinv = J.T @ np.linalg.inv(A)
    q_dot = J_pinv @ cart_vel

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

class SpaceMouseTeleopNode(Node):
    """
    SpaceMouse → Cartesian velocity → UR5 Jacobian IK → joint velocity controller.

    SpaceMouse axes are normalized [-1, 1] and act as velocity commands:
      linear  = axes * max_lin_vel
      angular = axes * max_rot_vel
    These are converted to joint velocities via the damped Jacobian pseudo-inverse
    and published to /forward_velocity_controller/commands.

    Gripper:
      BTN_1 → close gripper
      BTN_2 → open gripper
    Real hardware: ros2 action send_goal to /robotiq_2f_urcap_adapter/gripper_command
    Sim/fake:      publishes Float64 to /gripper_position_command (RViz viz)
    """

    # SpaceMouse Pro button indices (from pyspacemouse devices.toml)
    _BTN_1 = 5   # close gripper
    _BTN_2 = 6   # open gripper
    _BTN_3 = 7   # save position

    def __init__(self):
        super().__init__('ur5_spacemouse_teleop')

        # ---- parameters ----
        self.declare_parameter('device_number', 0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('max_lin_vel', 0.15)     # m/s at full deflection
        self.declare_parameter('max_rot_vel', 0.75)     # rad/s at full deflection
        self.declare_parameter('max_joint_vel', 1.5)    # rad/s per joint
        self.declare_parameter('control_hz', 50.0)
        self.declare_parameter('damping', 0.05)
        self.declare_parameter('btn_close', self._BTN_1)  # button index for close
        self.declare_parameter('btn_open', self._BTN_2)   # button index for open
        self.declare_parameter('btn_save', self._BTN_3)   # button index for save position

        self.device_number = self.get_parameter('device_number').value
        self.deadzone      = self.get_parameter('deadzone').value
        self.max_lin_vel   = self.get_parameter('max_lin_vel').value
        self.max_rot_vel   = self.get_parameter('max_rot_vel').value
        self.max_jnt_vel   = self.get_parameter('max_joint_vel').value
        hz                 = self.get_parameter('control_hz').value
        self.damping       = self.get_parameter('damping').value
        self.dt            = 1.0 / hz
        self._btn_close    = self.get_parameter('btn_close').value
        self._btn_open     = self.get_parameter('btn_open').value
        self._btn_save     = self.get_parameter('btn_save').value

        # ---- publishers ----
        self._vel_pub = self.create_publisher(
            Float64MultiArray, '/forward_velocity_controller/commands', 10)
        # Gripper visualization (sim/fake hardware — animates gripper in RViz)
        self._grip_viz_pub = self.create_publisher(
            Float64, '/gripper_position_command', 10)

        # ---- save-position service clients ----
        self._executor_node = '/ur5_program_executor'
        self._set_param_cli = self.create_client(
            SetParameters, f'{self._executor_node}/set_parameters')
        self._save_pos_cli = self.create_client(
            Trigger, f'{self._executor_node}/save_position')

        # ---- joint state subscriber ----
        self._js_lock   = threading.Lock()
        self._joint_pos = None  # ndarray (6,) in UR order
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # ---- SpaceMouse input state (written by reader thread) ----
        self._sm_lock = threading.Lock()
        self._sm_lin  = np.zeros(3)  # dx, dy, dz  normalized [-1, 1]
        self._sm_ang  = np.zeros(3)  # droll, dpitch, dyaw

        # ---- gripper state ----
        self._gripper_val      = 0.0   # 0.0 = open, 1.0 = closed
        self._gripper_busy     = False  # True while action goal is in flight
        self._prev_btn_close   = False
        self._prev_btn_open    = False
        self._prev_btn_save    = False

        # ---- open SpaceMouse device ----
        try:
            self.device = pyspacemouse.open(
                nonblocking=True,
                device_index=self.device_number,
            )
        except RuntimeError as e:
            self.get_logger().error(f'Could not open SpaceMouse device: {e}')
            self.get_logger().error('Troubleshooting:')
            self.get_logger().error('1. Make sure the SpaceMouse is connected via USB')
            self.get_logger().error('2. Check if detected: lsusb | grep 3Dconnexion')
            self.get_logger().error('3. You might need udev rules for non-root access')
            self.get_logger().error('   Create /etc/udev/rules.d/90-spacemouse.rules with:')
            self.get_logger().error('   SUBSYSTEM=="usb", ATTRS{idVendor}=="256f", MODE="0666"')
            self.get_logger().error('   SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", MODE="0666"')
            self.get_logger().error('4. Reload udev: sudo udevadm control --reload-rules')
            sys.exit(1)

        self.get_logger().info('SpaceMouse device opened successfully!')

        # ---- start SpaceMouse reader thread ----
        self._last_t = -1.0
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name='spacemouse_reader')
        self._reader_thread.start()

        # ---- control timer ----
        self._log_count = 0
        self._prev_active = False
        self.create_timer(self.dt, self._control_cb)

        self.get_logger().info(
            f'SpaceMouse Servo Teleop ready  (max_lin={self.max_lin_vel} m/s, '
            f'max_rot={self.max_rot_vel} rad/s, hz={hz})')
        self.get_logger().info('  Move SpaceMouse → robot moves')
        self.get_logger().info('  Release          → robot stops')
        self.get_logger().info(f'  BTN_1 (idx {self._btn_close})    → close gripper')
        self.get_logger().info(f'  BTN_2 (idx {self._btn_open})    → open gripper')
        self.get_logger().info(f'  BTN_3 (idx {self._btn_save})    → save position')

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
    # SpaceMouse reader thread
    # ------------------------------------------------------------------

    def _reader_loop(self):
        """Drain HID queue at ~200 Hz, keep only latest event."""
        import time
        while rclpy.ok():
            time.sleep(0.005)  # 200 Hz poll
            latest = None
            while True:
                s = self.device.read()
                if s is None or s.t == self._last_t:
                    break
                self._last_t = s.t
                latest = s

            if latest is None:
                # No new event — set axes to zero (device at rest)
                with self._sm_lock:
                    self._sm_lin[:] = 0.0
                    self._sm_ang[:] = 0.0
                continue

            # Apply deadzone
            dx    = 0.0 if abs(latest.x)     < self.deadzone else latest.x
            dy    = 0.0 if abs(latest.y)     < self.deadzone else latest.y
            dz    = 0.0 if abs(latest.z)     < self.deadzone else latest.z
            droll = 0.0 if abs(latest.roll)  < self.deadzone else latest.roll
            dpitch= 0.0 if abs(latest.pitch) < self.deadzone else latest.pitch
            dyaw  = 0.0 if abs(latest.yaw)   < self.deadzone else latest.yaw

            # ---- gripper button edge detection ----
            buttons = latest.buttons if hasattr(latest, 'buttons') else []
            n_btns = len(buttons)

            now_close = bool(buttons[self._btn_close]) if self._btn_close < n_btns else False
            now_open  = bool(buttons[self._btn_open])  if self._btn_open  < n_btns else False

            if now_close and not self._prev_btn_close and not self._gripper_busy:
                self.get_logger().info('Gripper → CLOSE')
                threading.Thread(
                    target=self._send_gripper_cmd, args=(1.0,),
                    daemon=True, name='gripper_close').start()
            if now_open and not self._prev_btn_open and not self._gripper_busy:
                self.get_logger().info('Gripper → OPEN')
                threading.Thread(
                    target=self._send_gripper_cmd, args=(0.0,),
                    daemon=True, name='gripper_open').start()

            # ---- save-position button edge detection ----
            now_save = bool(buttons[self._btn_save]) if self._btn_save < n_btns else False
            if now_save and not self._prev_btn_save:
                auto_name = 'pos_' + datetime.now().strftime('%H%M%S')
                self.get_logger().info(f'Save position → {auto_name}')
                threading.Thread(
                    target=self._save_position, args=(auto_name,),
                    daemon=True, name='save_pos').start()

            self._prev_btn_close = now_close
            self._prev_btn_open  = now_open
            self._prev_btn_save  = now_save

            with self._sm_lock:
                self._sm_lin[:] = [dx, dy, dz]
                self._sm_ang[:] = [droll, dpitch, dyaw]

    # ------------------------------------------------------------------
    # Control timer  (50 Hz)
    # ------------------------------------------------------------------

    def _control_cb(self):
        # ---- read SpaceMouse state ----
        with self._sm_lock:
            sm_lin = self._sm_lin.copy()
            sm_ang = self._sm_ang.copy()

        is_active = np.any(sm_lin != 0.0) or np.any(sm_ang != 0.0)

        # ---- not active: send zero velocities ----
        if not is_active:
            if self._prev_active:
                self._send_zero_vel()
                self.get_logger().info('SpaceMouse released — robot stopped.')
            self._prev_active = False
            return

        if not self._prev_active:
            self.get_logger().info('SpaceMouse active — moving robot.')
        self._prev_active = True

        # ---- get current joint state ----
        with self._js_lock:
            q = self._joint_pos.copy() if self._joint_pos is not None else None

        if q is None:
            self.get_logger().warn('No joint state yet', throttle_duration_sec=2.0)
            return

        # ---- FK + Jacobian ----
        _pos, J = ur5_fk_and_jacobian(q)

        # ---- scale SpaceMouse axes to Cartesian velocity ----
        lin_vel = sm_lin * self.max_lin_vel   # [-max_lin, +max_lin] m/s
        ang_vel = sm_ang * self.max_rot_vel   # [-max_rot, +max_rot] rad/s

        # ---- Cartesian → joint velocities via Jacobian ----
        cart_vel = np.concatenate([lin_vel, ang_vel])
        q_dot = cartesian_to_joint_vel(J, cart_vel, self.max_jnt_vel, self.damping)

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

    # ------------------------------------------------------------------
    # Save position (runs in background thread)
    # ------------------------------------------------------------------

    def _save_position(self, name: str):
        """Set save_position_name param on executor node, then call save_position."""
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

    # ------------------------------------------------------------------
    # Gripper command (runs in background thread)
    # ------------------------------------------------------------------

    def _send_gripper_cmd(self, position: float):
        """Send gripper command.

        position: 0.0 = fully open, 1.0 = fully closed.

        Real hardware:  ros2 action send_goal → /robotiq_2f_urcap_adapter/gripper_command
        Sim/fake:       publishes Float64 to /gripper_position_command (RViz viz)

        Both paths are tried — the action will simply fail if the adapter node
        is not running (fake hardware), and the viz topic is always published.
        """
        self._gripper_busy = True
        self._gripper_val = position

        # Robotiq 2F-85: 0.085m = fully open, 0.0m = fully closed
        robotiq_pos = (1.0 - position) * 0.085

        # Publish visualization (works for sim/fake)
        viz_msg = Float64()
        viz_msg.data = position  # 0.0 open, 1.0 closed
        self._grip_viz_pub.publish(viz_msg)

        # Send action goal for real hardware (non-blocking subprocess)
        cmd = (
            'unset LD_PRELOAD && '
            'source /opt/ros/humble/setup.bash && '
            'source /home/rml/ur5-robotiq-ros2-control/install/setup.bash && '
            'ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command '
            'robotiq_2f_urcap_adapter/action/GripperCommand '
            f"'{{command: {{position: {robotiq_pos}, max_effort: 100.0, max_speed: 0.1}}}}'"
        )
        try:
            result = subprocess.run(
                cmd, shell=True, capture_output=True, text=True,
                timeout=15, executable='/bin/bash')
            if result.returncode == 0:
                self.get_logger().info(
                    f'Gripper {"closed" if position >= 0.5 else "opened"} successfully')
            else:
                self.get_logger().warn(
                    f'Gripper action returned {result.returncode}: {result.stderr.strip()}')
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Gripper action timed out')
        except Exception as e:
            self.get_logger().warn(f'Gripper action error: {e}')
        finally:
            self._gripper_busy = False

    def _send_zero_vel(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self._vel_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Stopping — sending zero velocities ...')
        self._send_zero_vel()
        self.device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpaceMouseTeleopNode()
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
