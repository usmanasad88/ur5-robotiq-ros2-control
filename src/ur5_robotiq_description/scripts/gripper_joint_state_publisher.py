#!/usr/bin/env python3.10
"""
Dynamic joint state publisher for Robotiq gripper joints
Subscribes to gripper commands and animates the gripper in RViz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class GripperJointStatePublisher(Node):
    def __init__(self):
        super().__init__('gripper_joint_state_publisher')
        
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribe to gripper position commands (0.0 = open, 1.0 = closed)
        self.command_sub = self.create_subscription(
            Float64,
            '/gripper_position_command',
            self.gripper_command_callback,
            10
        )
        
        # Publish at 50Hz for smooth animation
        self.timer = self.create_timer(0.02, self.publish_joint_states)
        
        # Gripper joint position (0.0 = open, 0.8 = fully closed)
        # Start open
        self.current_position = 0.0
        self.target_position = 0.0
        
        # Animation speed (radians per second)
        self.speed = 2.0  # Smooth animation
        
        self.get_logger().info('Gripper joint state publisher started')
        self.get_logger().info('  Subscribe to /gripper_position_command (Float64: 0.0=open, 1.0=closed)')
        self.get_logger().info(f'  Current position: {self.current_position}')
    
    def gripper_command_callback(self, msg: Float64):
        """Handle gripper position command (0.0 = open, 1.0 = closed)"""
        # Clamp to valid range and map to joint angle
        # Input: 0.0 (open) to 1.0 (closed)
        # Output: 0.0 (open) to 0.8 (closed) for the joint
        command = max(0.0, min(1.0, msg.data))
        self.target_position = command * 0.8
        self.get_logger().info(f'Gripper command received: {msg.data:.2f} -> target joint: {self.target_position:.3f}')
    
    def publish_joint_states(self):
        # Animate towards target position
        dt = 0.02  # 50Hz
        diff = self.target_position - self.current_position
        
        if abs(diff) > 0.001:
            # Move towards target
            step = self.speed * dt
            if abs(diff) < step:
                self.current_position = self.target_position
            elif diff > 0:
                self.current_position += step
            else:
                self.current_position -= step
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish state for the left knuckle joint (right mimics it via URDF)
        msg.name = ['robotiq_85_left_knuckle_joint']
        msg.position = [self.current_position]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GripperJointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
