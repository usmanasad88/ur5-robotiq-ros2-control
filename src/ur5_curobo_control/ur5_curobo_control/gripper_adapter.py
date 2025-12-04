#!/home/mani/miniconda3/envs/ur5_python/bin/python
"""
Simple Gripper Command Adapter

Bridges simple string commands from the program executor to the Robotiq gripper.
Accepts commands on /gripper_command_simple topic like:
  - "open" or "position:0.0"
  - "close" or "position:1.0"
  - "position:0.5" (partial position)

And translates them to Robotiq gripper action goals.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
import subprocess
import re


class GripperAdapter(Node):
    def __init__(self):
        super().__init__('gripper_adapter')
        
        self.subscription = self.create_subscription(
            String,
            '/gripper_command_simple',
            self.command_callback,
            10
        )
        
        # Robotiq gripper position range: 0.0 (open) to 0.085 (closed) in meters
        self.MAX_GRIPPER_POS = 0.085
        
        self.get_logger().info("Gripper Adapter started. Listening on /gripper_command_simple")
    
    def command_callback(self, msg: String):
        """Handle incoming gripper commands."""
        command = msg.data.strip().lower()
        
        # Parse command
        position = None
        
        if command == 'open':
            position = 0.0
        elif command == 'close':
            position = self.MAX_GRIPPER_POS
        elif command.startswith('position:'):
            try:
                # Extract position value (0.0 to 1.0 normalized)
                value = float(command.split(':')[1])
                # Convert to gripper position (0.0 = open, 0.085 = closed)
                position = value * self.MAX_GRIPPER_POS
            except ValueError:
                self.get_logger().error(f"Invalid position value: {command}")
                return
        else:
            self.get_logger().warn(f"Unknown gripper command: {command}")
            return
        
        # Execute gripper command
        self.send_gripper_command(position)
    
    def send_gripper_command(self, position: float):
        """Send command to Robotiq gripper via action."""
        self.get_logger().info(f"Sending gripper command: position={position:.4f}")
        
        # Use ros2 action send_goal command
        # This is a simple approach; could be replaced with proper action client
        cmd = (
            f"ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command "
            f"robotiq_2f_urcap_adapter/GripperCommand "
            f"'{{ command: {{ position: {position}, max_effort: 70, max_speed: 0.05 }}}}'"
        )
        
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                self.get_logger().info("Gripper command sent successfully")
            else:
                self.get_logger().warn(f"Gripper command may have failed: {result.stderr}")
        except subprocess.TimeoutExpired:
            self.get_logger().error("Gripper command timed out")
        except Exception as e:
            self.get_logger().error(f"Error sending gripper command: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GripperAdapter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
