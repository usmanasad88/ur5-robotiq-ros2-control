#!/usr/bin/env python3
"""
SpaceMouse → simulated human hand teleop.

Integrates the SpaceMouse linear axes into an absolute hand position and
publishes it for Isaac Sim to consume. A button toggles the grab state.

Topics published
----------------
/hand/cmd_pose  geometry_msgs/PoseStamped  Absolute hand position in the
                                           robot/env frame (world frame in
                                           Isaac Sim). Orientation is identity.
/hand/grab_held std_msgs/Bool              True while the hand is grabbing.

Buttons (SpaceMouse Pro indices, overridable via params)
--------------------------------------------------------
btn_grab_toggle  toggles the grab state on each press (default: BTN_1 = 5)
btn_reset        resets the hand position to the anchor (default: BTN_3 = 7)

The grab semantics are *toggle on press* (single click to grab, single click
to release) — the SpaceMouse buttons are not ergonomic to hold down.
"""

import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

try:
    import pyspacemouse
except ImportError:
    print("Error: pyspacemouse library not found!")
    print("Please install it using: pip install pyspacemouse")
    sys.exit(1)


class SpaceMouseHandTeleopNode(Node):
    """Streams an absolute hand pose (world frame) from SpaceMouse axes."""

    # SpaceMouse Pro defaults — same as spacemouse_teleop_node.py
    _BTN_GRAB_TOGGLE = 5
    _BTN_RESET       = 7

    def __init__(self):
        super().__init__('ur5_spacemouse_hand_teleop')

        # ---- parameters ----
        self.declare_parameter('device_number', 0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('max_lin_vel', 0.30)        # m/s at full deflection
        self.declare_parameter('control_hz', 60.0)
        self.declare_parameter('btn_grab_toggle', self._BTN_GRAB_TOGGLE)
        self.declare_parameter('btn_reset', self._BTN_RESET)
        # Anchor / workspace bounds (xmin, xmax, ymin, ymax, zmin, zmax)
        self.declare_parameter('anchor', [0.60, 0.0, 0.20])
        self.declare_parameter('bounds_x', [0.20, 1.20])
        self.declare_parameter('bounds_y', [-0.60, 0.60])
        self.declare_parameter('bounds_z', [-0.10, 0.80])
        self.declare_parameter('frame_id', 'world')

        self.device_number = self.get_parameter('device_number').value
        self.deadzone      = self.get_parameter('deadzone').value
        self.max_lin_vel   = self.get_parameter('max_lin_vel').value
        hz                 = self.get_parameter('control_hz').value
        self._btn_grab     = self.get_parameter('btn_grab_toggle').value
        self._btn_reset    = self.get_parameter('btn_reset').value
        self._anchor       = np.array(self.get_parameter('anchor').value, dtype=np.float64)
        self._bx           = self.get_parameter('bounds_x').value
        self._by           = self.get_parameter('bounds_y').value
        self._bz           = self.get_parameter('bounds_z').value
        self._frame_id     = self.get_parameter('frame_id').value
        self.dt            = 1.0 / hz

        # ---- state ----
        self._sm_lock   = threading.Lock()
        self._sm_lin    = np.zeros(3)         # latest deadzoned axes [-1, 1]
        self._pos       = self._anchor.copy()  # absolute hand position
        self._grab_held = False
        self._last_t    = -1.0
        self._prev_btn_grab  = False
        self._prev_btn_reset = False

        # ---- publishers ----
        self._pose_pub = self.create_publisher(PoseStamped, '/hand/cmd_pose', 10)
        self._grab_pub = self.create_publisher(Bool, '/hand/grab_held', 10)

        # ---- open SpaceMouse ----
        try:
            self.device = pyspacemouse.open(
                nonblocking=True, device_index=self.device_number)
        except RuntimeError as e:
            self.get_logger().error(f'Could not open SpaceMouse device: {e}')
            sys.exit(1)
        self.get_logger().info('SpaceMouse device opened.')

        # ---- reader + control timers ----
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name='spacemouse_hand_reader')
        self._reader_thread.start()
        self.create_timer(self.dt, self._control_cb)

        # Publish initial state once so subscribers see something even if the
        # SpaceMouse stays at rest.
        self._publish_pose()
        self._publish_grab()

        self.get_logger().info(
            f'SpaceMouse hand teleop ready. anchor={self._anchor.tolist()}, '
            f'max_lin_vel={self.max_lin_vel} m/s, hz={hz}')
        self.get_logger().info(
            f'  BTN {self._btn_grab}   → toggle grab')
        self.get_logger().info(
            f'  BTN {self._btn_reset}  → reset hand to anchor')

    # ------------------------------------------------------------------
    # SpaceMouse reader thread
    # ------------------------------------------------------------------

    def _reader_loop(self):
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
                with self._sm_lock:
                    self._sm_lin[:] = 0.0
                continue

            dx = 0.0 if abs(latest.x) < self.deadzone else latest.x
            dy = 0.0 if abs(latest.y) < self.deadzone else latest.y
            dz = 0.0 if abs(latest.z) < self.deadzone else latest.z

            buttons = latest.buttons if hasattr(latest, 'buttons') else []
            n_btns = len(buttons)
            now_grab  = bool(buttons[self._btn_grab])  if self._btn_grab  < n_btns else False
            now_reset = bool(buttons[self._btn_reset]) if self._btn_reset < n_btns else False

            if now_grab and not self._prev_btn_grab:
                self._grab_held = not self._grab_held
                self._publish_grab()
                self.get_logger().info(
                    f'Hand grab → {"GRAB" if self._grab_held else "RELEASE"}')

            if now_reset and not self._prev_btn_reset:
                with self._sm_lock:
                    self._pos[:] = self._anchor
                self.get_logger().info(f'Hand reset to anchor {self._anchor.tolist()}')

            self._prev_btn_grab  = now_grab
            self._prev_btn_reset = now_reset

            with self._sm_lock:
                self._sm_lin[:] = [dx, dy, dz]

    # ------------------------------------------------------------------
    # Control timer
    # ------------------------------------------------------------------

    def _control_cb(self):
        with self._sm_lock:
            sm_lin = self._sm_lin.copy()
            self._pos += sm_lin * self.max_lin_vel * self.dt
            # Clamp to workspace bounds
            self._pos[0] = float(np.clip(self._pos[0], self._bx[0], self._bx[1]))
            self._pos[1] = float(np.clip(self._pos[1], self._by[0], self._by[1]))
            self._pos[2] = float(np.clip(self._pos[2], self._bz[0], self._bz[1]))

        self._publish_pose()

    def _publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = self._frame_id
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = float(self._pos[0])
        msg.pose.position.y = float(self._pos[1])
        msg.pose.position.z = float(self._pos[2])
        msg.pose.orientation.w = 1.0
        self._pose_pub.publish(msg)

    def _publish_grab(self):
        self._grab_pub.publish(Bool(data=bool(self._grab_held)))

    def destroy_node(self):
        try:
            self.device.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpaceMouseHandTeleopNode()
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
