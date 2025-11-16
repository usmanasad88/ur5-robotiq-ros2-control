#!/usr/bin/env python3.10
"""
Static joint state publisher for Robotiq gripper joints
Publishes a fixed open position for visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperJointStatePublisher(Node):
    def __init__(self):
        super().__init__('gripper_joint_state_publisher')
        
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Publish at 10Hz
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Open position for gripper (0.0 = closed, 0.8 = fully open)
        self.gripper_position = 0.7  # Nearly fully open for clear visibility
        
        self.get_logger().info(f'Gripper joint state publisher started - position: {self.gripper_position}')
    
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish state for the left knuckle joint (right mimics it)
        msg.name = ['robotiq_85_left_knuckle_joint']
        msg.position = [self.gripper_position]
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
