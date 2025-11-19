#!/home/mani/miniconda3/envs/ur5_python/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Bool

import torch
import numpy as np
import time
import os
import sys

# Add curobo to path if not installed
sys.path.append('/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/curobo/src')

from curobo.geom.types import WorldConfig, Cuboid
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose as CuroboPose
from curobo.types.robot import JointState as CuroboJointState
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.util_file import load_yaml, join_path

class CuroboControlNode(Node):
    def __init__(self):
        super().__init__('curobo_control_node')
        
        # Parameters
        self.declare_parameter('robot_config_file', '')
        self.declare_parameter('world_config_file', '')
        
        robot_config_file = self.get_parameter('robot_config_file').get_parameter_value().string_value
        world_config_file = self.get_parameter('world_config_file').get_parameter_value().string_value
        
        if not robot_config_file:
            self.get_logger().error("robot_config_file parameter is required")
            return

        self.get_logger().info(f"Loading robot config: {robot_config_file}")
        self.get_logger().info(f"Loading world config: {world_config_file}")

        # Initialize Curobo
        self.tensor_args = TensorDeviceType()
        
        self.dt = 0.05 # Increased from 0.01 to reduce speed
        self.motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_config_file,
            world_config_file,
            self.tensor_args,
            trajopt_tsteps=32,
            use_cuda_graph=False,
            interpolation_dt=self.dt,
        )
        self.motion_gen = MotionGen(self.motion_gen_config)
        
        # ROS Interfaces
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Safety Subscriber
        self.safety_triggered = False
        self.safety_sub = self.create_subscription(
            Bool,
            '/human_safety',
            self.safety_callback,
            10
        )
        
        self.current_joint_state = None
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Define a sequence of target poses (position x,y,z, quaternion w,x,y,z)
        # These are in base_link frame
        # Note: UR5 reach is about 0.85m.
        self.target_poses = [
            ([0.4, 0.3, 0.4], [0.0, 0.707, 0.707, 0.0]), 
            ([0.4, -0.3, 0.4], [0.0, 0.707, 0.707, 0.0]),
            ([0.5, 0.0, 0.5], [0.0, 1.0, 0.0, 0.0]),
        ]
        self.current_target_idx = 0
        
        # Timer to execute sequence
        self.timer = self.create_timer(5.0, self.execute_next_move)
        
    def safety_callback(self, msg):
        prev_state = self.safety_triggered
        self.safety_triggered = msg.data
        
        if self.safety_triggered and not prev_state:
            self.get_logger().warn("SAFETY TRIGGERED: Human detected! Stopping robot.")
            self.stop_robot()
        elif not self.safety_triggered and prev_state:
            self.get_logger().info("SAFETY CLEARED: Resuming operation.")

    def stop_robot(self):
        if self.current_joint_state is None:
            return
            
        # Create a stop trajectory (hold current position)
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        # Get current positions from tensor
        current_pos = self.current_joint_state.cpu().numpy().tolist()
        point.positions = current_pos
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000 # 0.5 seconds to stop smoothly
        
        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)

    def joint_state_callback(self, msg):
        # Extract joint positions for UR5 joints
        positions = []
        try:
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    positions.append(msg.position[idx])
            
            if len(positions) == len(self.joint_names):
                self.current_joint_state = torch.tensor(positions, device=self.tensor_args.device, dtype=self.tensor_args.dtype)
        except ValueError:
            pass

    def execute_next_move(self):
        if self.safety_triggered:
            self.get_logger().warn("Skipping move: Safety trigger is active.")
            return

        if self.current_joint_state is None:
            self.get_logger().warn("Waiting for joint states...")
            return
            
        # Cycle through targets
        if self.current_target_idx >= len(self.target_poses):
            self.current_target_idx = 0

        target_pos, target_quat = self.target_poses[self.current_target_idx]
        self.get_logger().info(f"Planning to target {self.current_target_idx}: {target_pos}")
        
        # Create Curobo Pose
        # Curobo expects quaternion as [w, x, y, z]
        target_pose = CuroboPose(
            position=torch.tensor(target_pos, device=self.tensor_args.device, dtype=self.tensor_args.dtype),
            quaternion=torch.tensor(target_quat, device=self.tensor_args.device, dtype=self.tensor_args.dtype)
        )
        
        # Create Start State
        start_state = CuroboJointState.from_position(self.current_joint_state.view(1, -1))
        
        # Plan
        result = self.motion_gen.plan_single(start_state, target_pose, MotionGenPlanConfig(enable_graph=False, timeout=1.0))
        
        if result.success.item():
            self.get_logger().info("Plan successful! Executing...")
            traj = result.interpolated_plan # [1, steps, dof]
            self.publish_trajectory(traj)
            self.current_target_idx += 1
        else:
            self.get_logger().error(f"Planning failed! Status: {result.status}")
            self.get_logger().info(f"Start state: {start_state.position}")
            self.get_logger().info(f"Target pose: {target_pose.position}")

    def publish_trajectory(self, traj_input):
        # Convert tensor to JointTrajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        # Check if input is a JointState object and extract position
        if hasattr(traj_input, 'position'):
             traj_tensor = traj_input.position
        else:
             traj_tensor = traj_input
        
        # traj_tensor is [1, steps, dof]
        traj_np = traj_tensor.squeeze(0).cpu().numpy()
        steps = traj_np.shape[0]
        dt = self.dt # Time step
        
        # Add current time to header? No, usually relative time in points is enough.
        
        for i in range(steps):
            point = JointTrajectoryPoint()
            point.positions = traj_np[i].tolist()
            # Calculate time from start
            time_sec = i * dt
            point.time_from_start.sec = int(time_sec)
            point.time_from_start.nanosec = int((time_sec - int(time_sec)) * 1e9)
            traj_msg.points.append(point)
            
        self.traj_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CuroboControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
