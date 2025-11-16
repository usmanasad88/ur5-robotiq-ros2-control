#!/usr/bin/env python3
"""
SpaceMouse teleoperation node for UR5 robot.
Reads input from 3Dconnexion SpaceMouse and publishes PoseDelta messages.
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
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('scale_translation', 0.0001)  # Scale raw values to meters
        self.declare_parameter('scale_rotation', 0.0005)     # Scale raw values to radians
        self.declare_parameter('publish_rate', 50.0)         # Hz
        
        # Get parameters
        self.device_number = self.get_parameter('device_number').value
        self.deadzone = self.get_parameter('deadzone').value
        self.scale_translation = self.get_parameter('scale_translation').value
        self.scale_rotation = self.get_parameter('scale_rotation').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Open SpaceMouse device
        success = pyspacemouse.open(
            set_nonblocking_loop=False,
            DeviceNumber=self.device_number
        )
        
        if not success:
            self.get_logger().error('Could not open SpaceMouse device!')
            self.get_logger().error('Troubleshooting:')
            self.get_logger().error('1. Make sure the SpaceMouse is connected via USB')
            self.get_logger().error('2. Check if detected: lsusb | grep 3Dconnexion')
            self.get_logger().error('3. You might need udev rules for non-root access')
            self.get_logger().error('   Create /etc/udev/rules.d/90-spacemouse.rules with:')
            self.get_logger().error('   SUBSYSTEM=="usb", ATTRS{idVendor}=="256f", MODE="0666"')
            self.get_logger().error('4. Reload udev: sudo udevadm control --reload-rules')
            sys.exit(1)
        
        self.get_logger().info('SpaceMouse device opened successfully!')
        self.get_logger().info(f'Configuration:')
        self.get_logger().info(f'  - Deadzone: {self.deadzone}')
        self.get_logger().info(f'  - Translation scale: {self.scale_translation}')
        self.get_logger().info(f'  - Rotation scale: {self.scale_rotation}')
        self.get_logger().info(f'  - Publish rate: {publish_rate} Hz')
        
        # Create timer for reading SpaceMouse
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        # Track last non-zero state to avoid spamming zero messages
        self.last_was_zero = True

    def apply_deadzone(self, value):
        """Apply deadzone to filter out small noise."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def timer_callback(self):
        """Read SpaceMouse and publish PoseDelta message."""
        state = pyspacemouse.read()
        
        if state is None:
            return
        
        # Apply deadzone and scaling
        dx = self.apply_deadzone(state.x) * self.scale_translation
        dy = self.apply_deadzone(state.y) * self.scale_translation
        dz = self.apply_deadzone(state.z) * self.scale_translation
        droll = self.apply_deadzone(state.roll) * self.scale_rotation
        dpitch = self.apply_deadzone(state.pitch) * self.scale_rotation
        dyaw = self.apply_deadzone(state.yaw) * self.scale_rotation
        
        # Check if all values are zero
        is_zero = (dx == 0.0 and dy == 0.0 and dz == 0.0 and 
                   droll == 0.0 and dpitch == 0.0 and dyaw == 0.0)
        
        # Only publish if non-zero or transitioning from non-zero to zero
        if not is_zero or not self.last_was_zero:
            msg = PoseDelta()
            msg.dx = dx
            msg.dy = dy
            msg.dz = dz
            msg.droll = droll
            msg.dpitch = dpitch
            msg.dyaw = dyaw
            
            self.publisher_.publish(msg)
            
            if not is_zero:
                self.get_logger().debug(
                    f'Published: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}, '
                    f'dr={droll:.4f}, dp={dpitch:.4f}, dyw={dyaw:.4f}'
                )
        
        self.last_was_zero = is_zero

    def destroy_node(self):
        """Clean up SpaceMouse device on shutdown."""
        self.get_logger().info('Closing SpaceMouse device...')
        pyspacemouse.close()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = SpaceMouseTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
