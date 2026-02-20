#!/usr/bin/env python3
"""
SpaceMouse teleoperation node for UR5 robot.
Reads input from 3Dconnexion SpaceMouse and publishes PoseDelta messages
with raw normalized axis values [-1, 1] after deadzone filtering.
The executor node uses these values to jog the robot at a fixed speed.
"""

import rclpy
from rclpy.node import Node
from ur5_teleop_msgs.msg import PoseDelta
import sys

try:
    import pyspacemouse
except ImportError:
    print("Error: pyspacemouse library not found!")
    print("Please install it using: pip install pyspacemouse")
    sys.exit(1)


class SpaceMouseTeleopNode(Node):
    """ROS 2 node for SpaceMouse teleoperation."""

    def __init__(self):
        super().__init__('ur5_spacemouse_teleop')

        # Publisher
        self.publisher_ = self.create_publisher(PoseDelta, '/ur5/teleop_delta', 10)

        # Declare parameters
        self.declare_parameter('device_number', 0)
        self.declare_parameter('deadzone', 0.1)   # Fraction of full range to ignore as noise
        self.declare_parameter('publish_rate', 20.0)  # Hz — only needs to be fast enough to catch transitions

        # Get parameters
        self.device_number = self.get_parameter('device_number').value
        self.deadzone = self.get_parameter('deadzone').value
        publish_rate = self.get_parameter('publish_rate').value

        # Open SpaceMouse device
        try:
            self.device = pyspacemouse.open(
                nonblocking=True,
                device_index=self.device_number
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
        self.get_logger().info(f'  - Deadzone: {self.deadzone}')
        self.get_logger().info(f'  - Publish rate: {publish_rate} Hz')
        self.get_logger().info('Publishing normalized axis values [-1, 1] on /ur5/teleop_delta')

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        self.last_was_zero = True
        self._last_t = -1.0  # Track last seen timestamp to detect stale state

    def _deadzone(self, value):
        """Return 0 if |value| < deadzone, otherwise return value unchanged."""
        return 0.0 if abs(value) < self.deadzone else value

    def timer_callback(self):
        # Drain the full HID queue, keeping only the latest event.
        # Without this, a burst of events accumulates faster than the timer
        # fires and the node keeps publishing stale motion for seconds after release.
        latest = None
        while True:
            s = self.device.read()
            if s is None or s.t == self._last_t:
                break
            self._last_t = s.t
            latest = s
        state = latest

        # No new HID packet — device is at rest
        if state is None:
            if not self.last_was_zero:
                self.get_logger().info('SpaceMouse released — publishing zero')
                self.publisher_.publish(PoseDelta())
                self.last_was_zero = True
            return

        # Log raw values every time we get a fresh packet
        self.get_logger().info(
            f'RAW  x={state.x:+.3f} y={state.y:+.3f} z={state.z:+.3f} '
            f'roll={state.roll:+.3f} pitch={state.pitch:+.3f} yaw={state.yaw:+.3f}'
        )

        # Apply deadzone; raw spacemouse values are already in [-1, 1]
        dx    = self._deadzone(state.x)
        dy    = self._deadzone(state.y)
        dz    = self._deadzone(state.z)
        droll  = self._deadzone(state.roll)
        dpitch = self._deadzone(state.pitch)
        dyaw   = self._deadzone(state.yaw)

        is_zero = (dx == 0.0 and dy == 0.0 and dz == 0.0 and
                   droll == 0.0 and dpitch == 0.0 and dyaw == 0.0)

        if is_zero:
            self.get_logger().info(
                f'POST-DEADZONE all zero (deadzone={self.deadzone}) — not publishing'
            )
        else:
            self.get_logger().info(
                f'POST-DEADZONE dx={dx:+.3f} dy={dy:+.3f} dz={dz:+.3f} '
                f'dr={droll:+.3f} dp={dpitch:+.3f} dyw={dyaw:+.3f} — publishing'
            )

        # Publish on every tick while active; publish one final zero on release
        if not is_zero or not self.last_was_zero:
            msg = PoseDelta()
            msg.dx    = dx
            msg.dy    = dy
            msg.dz    = dz
            msg.droll  = droll
            msg.dpitch = dpitch
            msg.dyaw   = dyaw
            self.publisher_.publish(msg)

        self.last_was_zero = is_zero

    def destroy_node(self):
        self.get_logger().info('Closing SpaceMouse device...')
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
