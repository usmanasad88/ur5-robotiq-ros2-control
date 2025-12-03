#!/home/mani/miniconda3/envs/ur5_python/bin/python
"""
Isaac Sim Follower Node
Subscribes to /isaac_joint_commands and mirrors those positions to the real robot
via /scaled_joint_trajectory_controller/joint_trajectory
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class IsaacFollowerNode(Node):
    def __init__(self):
        super().__init__('isaac_follower_node')
        
        # Subscribe to Isaac Sim joint commands
        self.isaac_sub = self.create_subscription(
            JointState,
            '/isaac_joint_commands',
            self.isaac_callback,
            10
        )
        
        # Publish to real robot trajectory controller
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Rate limiting: only send commands every N messages to avoid flooding
        self.msg_count = 0
        self.skip_rate = 3  # Send every 3rd message (~20Hz if Isaac publishes at 60Hz)
        
        self.get_logger().info("Isaac Follower Node started - subscribing to /isaac_joint_commands")
    
    def isaac_callback(self, msg: JointState):
        """Receive joint positions from Isaac Sim and send to real robot"""
        self.msg_count += 1
        
        # Rate limiting
        if self.msg_count % self.skip_rate != 0:
            return
        
        # Verify we have all 6 joints
        if len(msg.position) != 6:
            self.get_logger().warn(f"Expected 6 joints, got {len(msg.position)}")
            return
        
        # Create trajectory message with single waypoint
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = list(msg.position)
        point.velocities = [0.0] * 6  # Zero velocities for position control
        
        # Set time from start - how long the robot should take to reach this position
        # Adjust this based on your desired responsiveness vs smoothness tradeoff
        point.time_from_start = Duration(sec=0, nanosec=200000000)  # 0.2 seconds
        
        traj_msg.points.append(point)
        
        self.traj_pub.publish(traj_msg)
        
        if self.msg_count % 60 == 0:
            self.get_logger().info(f"Following Isaac Sim - sent {self.msg_count // self.skip_rate} commands")


def main(args=None):
    rclpy.init(args=args)
    node = IsaacFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
