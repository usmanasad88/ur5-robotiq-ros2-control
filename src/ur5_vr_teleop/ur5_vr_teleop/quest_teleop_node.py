#!/usr/bin/env python3
"""
Meta Quest 3S VR teleoperation node for UR5 robot.

Uses the rail-berkeley/oculus_reader library (ADB-based) to read controller
poses and buttons from a Quest 3S, then publishes PoseDelta messages to
/ur5/teleop_delta — the same topic consumed by the existing UR5 controller.

Inspired by DROID project's VRPolicy (droid/controllers/oculus_controller.py).

Requirements:
    pip install git+https://github.com/rail-berkeley/oculus_reader.git
    sudo apt install android-tools-adb

Hardware setup:
    1. Enable Developer Mode on Quest 3S (Meta Quest app → Devices → Developer Mode)
    2. Connect Quest 3S to PC via USB-C
    3. Put on headset, accept the ADB authorization prompt
    4. Verify: `adb devices` shows the Quest listed

Usage:
    Grip button (side button) = deadman switch — hold to enable robot motion
    Joystick click           = reset "forward" direction to current controller facing
    Trigger (front)          = gripper open/close (published on /ur5/gripper_cmd)

Coordinate mapping:
    The rmat_reorder parameter remaps controller axes to the robot base frame.
    Default [-2, -1, -3, 4] is DROID's mapping (Franka, facing the robot).
    Tune this if directions feel wrong — or use joystick click to redefine forward.
"""

import threading
import time
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from ur5_teleop_msgs.msg import PoseDelta

try:
    from oculus_reader.reader import OculusReader
except ImportError:
    print("ERROR: oculus_reader not found.")
    print("  pip install git+https://github.com/rail-berkeley/oculus_reader.git")
    print("  sudo apt install android-tools-adb")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Rotation math helpers (no extra dependencies)
# ---------------------------------------------------------------------------

def rmat_to_quat(R):
    """3×3 rotation matrix → quaternion [w, x, y, z]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return np.array([0.25 / s,
                         (R[2, 1] - R[1, 2]) * s,
                         (R[0, 2] - R[2, 0]) * s,
                         (R[1, 0] - R[0, 1]) * s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                         (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                         0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                         (R[1, 2] + R[2, 1]) / s, 0.25 * s])


def quat_mult(q1, q2):
    """Quaternion multiplication [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_conj(q):
    """Quaternion conjugate (= inverse for unit quaternions)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_euler(q):
    """Quaternion [w, x, y, z] → Euler angles [roll, pitch, yaw] (rad)."""
    w, x, y, z = q
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def vec_to_reorder_mat(vec):
    """
    Build a 4×4 axis-permutation matrix from a signed-index vector.
    E.g. [-2, -1, -3, 4] means:
        output row 0 ← -input col 1   (Y axis, negated)
        output row 1 ← -input col 0   (X axis, negated)
        output row 2 ← -input col 2   (Z axis, negated)
        output row 3 ←  input col 3   (W, unchanged)
    Same convention as DROID's VRPolicy.
    """
    n = len(vec)
    M = np.zeros((n, n))
    for i, v in enumerate(vec):
        M[i, int(abs(v)) - 1] = np.sign(v)
    return M


# ---------------------------------------------------------------------------
# ROS 2 node
# ---------------------------------------------------------------------------

class QuestTeleopNode(Node):
    """
    Reads Meta Quest 3S controller state via oculus_reader and publishes
    incremental PoseDelta messages compatible with the existing UR5 controller.
    """

    def __init__(self):
        super().__init__('ur5_quest_teleop')

        # --- Publishers ---
        self.delta_pub = self.create_publisher(PoseDelta, '/ur5/teleop_delta', 10)
        self.gripper_pub = self.create_publisher(Float64, '/ur5/gripper_cmd', 10)

        # --- Parameters ---
        self.declare_parameter('controller_side', 'right')
        self.declare_parameter('scale_translation', 5.0)
        self.declare_parameter('scale_rotation', 0.5)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('trigger_threshold', 0.1)
        # Axis reorder: maps Quest controller frame → robot env frame.
        # DROID default: [-2, -1, -3, 4].  Adjust signs/order if axes feel wrong.
        self.declare_parameter('rmat_reorder', [-2, -1, -3, 4])
        # Max velocity clipping (same units as scaled deltas)
        self.declare_parameter('max_lin_delta', 0.5)
        self.declare_parameter('max_rot_delta', 0.5)
        # auto_enable_teleop: call ~/set_teleop_mode True on the program_executor
        # node when this node starts. Set to True when running against the sim
        # (launch_all.sh fake hardware) — saves the manual service call step.
        self.declare_parameter('auto_enable_teleop', False)
        # Service name for the program executor teleop mode toggle.
        self.declare_parameter(
            'teleop_mode_service',
            '/program_executor_node/set_teleop_mode'
        )

        side = self.get_parameter('controller_side').value          # 'right' or 'left'
        self.cid = 'r' if side == 'right' else 'l'                 # 'r' or 'l'
        self.scale_t = self.get_parameter('scale_translation').value
        self.scale_r = self.get_parameter('scale_rotation').value
        rate = self.get_parameter('publish_rate').value
        self.trig_thresh = self.get_parameter('trigger_threshold').value
        reorder = self.get_parameter('rmat_reorder').value
        self.max_lin = self.get_parameter('max_lin_delta').value
        self.max_rot = self.get_parameter('max_rot_delta').value
        auto_enable  = self.get_parameter('auto_enable_teleop').value
        svc_name     = self.get_parameter('teleop_mode_service').value

        # Coordinate frame transform (applied after vr_to_global; matches DROID)
        self.global_to_env = vec_to_reorder_mat(reorder)

        # --- Internal state (shared with reader thread) ---
        self._lock = threading.Lock()
        self._poses = {}
        self._buttons = {}
        self._enabled = False          # grip deadman switch
        self._ctrl_on = False          # controller connected

        # Orientation alignment matrix (updated while user holds joystick or before first move)
        self._vr_to_global = np.eye(4)
        self._reset_orientation = True  # capture orientation on startup

        # Per-frame delta tracking
        self._last_pos = None          # position in env frame at previous tick
        self._last_quat = None         # quaternion in env frame at previous tick
        self._was_enabled = False      # previous enable state

        self._msg_count = 0

        # --- Start background reader thread ---
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name='quest_reader')
        self._reader_thread.start()

        # --- Publish timer ---
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)

        self.get_logger().info('Quest 3S Teleop Node ready.')
        self.get_logger().info(f'  Controller : {side} (id={self.cid})')
        self.get_logger().info(f'  Scale      : translation={self.scale_t}, rotation={self.scale_r}')
        self.get_logger().info(f'  Rate       : {rate} Hz')
        self.get_logger().info('Hold GRIP button to move robot.')
        self.get_logger().info('Click JOYSTICK to reset forward direction.')

        # --- Auto-enable cuRobo teleop mode (sim) ---
        if auto_enable:
            self._enable_teleop_mode(svc_name)

    # ------------------------------------------------------------------
    # cuRobo teleop mode auto-enable (sim only)
    # ------------------------------------------------------------------

    def _enable_teleop_mode(self, svc_name: str):
        """Call set_teleop_mode(True) on the program_executor_node."""
        self.get_logger().info(f'Calling {svc_name} (auto_enable_teleop=True) ...')
        client = self.create_client(SetBool, svc_name)
        # Wait up to 8 s for the service to come up (executor may still be starting)
        if not client.wait_for_service(timeout_sec=8.0):
            self.get_logger().warning(
                f'Service {svc_name} not available after 8 s. '
                'Start launch_all.sh first, or enable teleop mode manually:\n'
                f'  ros2 service call {svc_name} std_srvs/srv/SetBool \'{{data: true}}\''
            )
            return
        req = SetBool.Request()
        req.data = True
        future = client.call_async(req)
        future.add_done_callback(self._teleop_enable_cb)

    def _teleop_enable_cb(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('cuRobo teleop mode enabled successfully.')
            else:
                self.get_logger().warning(
                    f'set_teleop_mode returned success=False: {result.message}'
                )
        except Exception as exc:
            self.get_logger().warning(f'set_teleop_mode call failed: {exc}')

    # ------------------------------------------------------------------
    # Background reader thread
    # ------------------------------------------------------------------

    def _reader_loop(self, hz: int = 50):
        """Read Quest state at ~50 Hz — same as DROID's _update_internal_state."""
        self.get_logger().info('Connecting to Quest 3S via ADB ...')
        try:
            reader = OculusReader()
        except Exception as exc:
            self.get_logger().fatal(f'OculusReader init failed: {exc}')
            return

        self.get_logger().info('OculusReader connected. Put on headset and authorize ADB if prompted.')

        grip_key = self.cid.upper() + 'G'      # 'RG' or 'LG'
        joy_key = self.cid.upper() + 'J'       # 'RJ' or 'LJ'

        while True:
            time.sleep(1.0 / hz)
            try:
                poses, buttons = reader.get_transformations_and_buttons()
            except Exception:
                continue

            if not poses:
                continue

            with self._lock:
                prev_enabled = self._enabled
                now_enabled = bool(buttons.get(grip_key, False))
                toggled = prev_enabled != now_enabled

                # Reset orientation while joystick held OR before first movement
                if self._reset_orientation or buttons.get(joy_key, False):
                    raw = np.asarray(poses.get(self.cid, np.eye(4)))
                    # Stop updating once the user confirms (joystick click or grip press)
                    stop = buttons.get(joy_key, False) or now_enabled
                    if stop:
                        self._reset_orientation = False
                    try:
                        self._vr_to_global = np.linalg.inv(raw)
                    except np.linalg.LinAlgError:
                        self._vr_to_global = np.eye(4)
                        self._reset_orientation = True

                # When grip is released/pressed, force a re-baseline on next enable
                if toggled and not now_enabled:
                    self._last_pos = None
                    self._last_quat = None

                self._poses = poses
                self._buttons = buttons
                self._enabled = now_enabled
                self._ctrl_on = True

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_env_frame_state(self):
        """Return current controller pose in the environment frame."""
        with self._lock:
            if not self._poses or self.cid not in self._poses:
                return None, None, None, None, False
            raw = np.asarray(self._poses[self.cid])
            vr_to_global = self._vr_to_global.copy()
            enabled = self._enabled
            trig_key = 'rightTrig' if self.cid == 'r' else 'leftTrig'
            gripper_raw = float(self._buttons.get(trig_key, [0.0])[0])

        # Apply coordinate frame chain (same as DROID's _process_reading)
        mat = self.global_to_env @ vr_to_global @ raw
        pos = mat[:3, 3].copy()
        quat = rmat_to_quat(mat[:3, :3])
        return pos, quat, gripper_raw, mat, enabled

    def _clip_delta(self, lin, rot):
        """Clip linear and rotational deltas to configured max."""
        ln = np.linalg.norm(lin)
        rn = np.linalg.norm(rot)
        if ln > self.max_lin:
            lin = lin * self.max_lin / ln
        if rn > self.max_rot:
            rot = rot * self.max_rot / rn
        return lin, rot

    # ------------------------------------------------------------------
    # Main timer callback
    # ------------------------------------------------------------------

    def _timer_cb(self):
        pos, quat, gripper_raw, _, enabled = self._get_env_frame_state()

        if pos is None:
            return

        if not enabled:
            # Clear baseline so next enable starts fresh
            self._last_pos = None
            self._last_quat = None
            if self._was_enabled:
                self.get_logger().info('Grip released — robot motion paused.')
            self._was_enabled = False
            return

        if not self._was_enabled:
            self.get_logger().info('Grip pressed — robot motion enabled.')

        self._was_enabled = True

        # First frame after (re-)enable: just store baseline, don't move
        if self._last_pos is None or self._last_quat is None:
            self._last_pos = pos.copy()
            self._last_quat = quat.copy()
            return

        # --- Frame-to-frame position delta ---
        pos_delta = pos - self._last_pos

        # --- Frame-to-frame rotation delta (quaternion difference) ---
        #   q_delta = q_current * conj(q_last)   →  rotation since last tick
        quat_delta = quat_mult(quat, quat_conj(self._last_quat))
        euler_delta = quat_to_euler(quat_delta)

        # Store for next tick
        self._last_pos = pos.copy()
        self._last_quat = quat.copy()

        # --- Scale ---
        lin = pos_delta * self.scale_t
        rot = euler_delta * self.scale_r

        # --- Clip ---
        lin, rot = self._clip_delta(lin, rot)

        # --- Threshold filter (suppress noise) ---
        if (np.all(np.abs(lin) < 1e-4) and np.all(np.abs(rot) < 1e-3)):
            return

        # --- Publish PoseDelta ---
        msg = PoseDelta()
        msg.dx = float(lin[0])
        msg.dy = float(lin[1])
        msg.dz = float(lin[2])
        msg.droll = float(rot[0])
        msg.dpitch = float(rot[1])
        msg.dyaw = float(rot[2])
        self.delta_pub.publish(msg)

        # --- Publish gripper command ---
        gripper_msg = Float64()
        gripper_msg.data = float(np.clip(gripper_raw, 0.0, 1.0))
        self.gripper_pub.publish(gripper_msg)

        # --- Periodic log ---
        self._msg_count += 1
        if self._msg_count % 50 == 0:
            self.get_logger().info(
                f'Δpos=[{lin[0]:+.4f}, {lin[1]:+.4f}, {lin[2]:+.4f}]  '
                f'Δrot=[{rot[0]:+.3f}, {rot[1]:+.3f}, {rot[2]:+.3f}]  '
                f'grip={gripper_raw:.2f}'
            )

    def destroy_node(self):
        self.get_logger().info('Quest teleop node shutting down.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = QuestTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
