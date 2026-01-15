#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_share_directory

import torch
import numpy as np
import time
import os
import sys

# Try to import curobo, if fails, try adding common paths
try:
    import curobo
except ImportError:
    # Add curobo to path if not installed
    # Check for common locations or environment variables
    possible_paths = [
        '/home/rml/Repos/curobo/src',
        '/home/mani/Repos/curobo/src',
        os.path.join(os.path.expanduser('~'), 'Repos/curobo/src'),
        # Add Isaac Sim curobo path if available
        '/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/curobo/src'
    ]
    for path in possible_paths:
        if os.path.exists(path):
            sys.path.append(path)
            break

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
        
        # Load YAML manually to fix paths dynamically
        robot_cfg_dict = load_yaml(robot_config_file)
        
        # Fix URDF path
        if 'robot_cfg' in robot_cfg_dict:
            kinematics = robot_cfg_dict['robot_cfg'].get('kinematics', {})
            if 'urdf_path' in kinematics:
                # Construct correct path dynamically
                try:
                    pkg_share = get_package_share_directory('ur5_curobo_control')
                    correct_urdf_path = os.path.join(pkg_share, 'config', 'ur5.urdf')
                    
                    # Update if different
                    current_path = kinematics['urdf_path']
                    if current_path != correct_urdf_path:
                        self.get_logger().warn(f"Updating URDF path from {current_path} to {correct_urdf_path}")
                        kinematics['urdf_path'] = correct_urdf_path
                except Exception as e:
                    self.get_logger().error(f"Failed to resolve URDF path dynamically: {e}")
        
        self.dt = 0.05 # Increased from 0.01 to reduce speed
        self.motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_cfg_dict,
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

        # Command Subscriber
        self.cmd_sub = self.create_subscription(
            Pose,
            '/curobo/cmd_pose',
            self.cmd_pose_callback,
            10
        )
        
        # Status Publisher
        self.status_pub = self.create_publisher(
            String,
            '/curobo/status',
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
        
        # Timer to execute sequence (REMOVED for external control)
        # self.timer = self.create_timer(5.0, self.execute_next_move)
        
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

    def cmd_pose_callback(self, msg: Pose):
        if self.safety_triggered:
            self.get_logger().warn("Skipping move: Safety trigger is active.")
            self.status_pub.publish(String(data="FAILED: Safety Triggered"))
            return

        if self.current_joint_state is None:
            self.get_logger().warn("Waiting for joint states...")
            self.status_pub.publish(String(data="FAILED: No Joint States"))
            return
            
        self.get_logger().info(f"Received target pose: {msg.position}")
        self.status_pub.publish(String(data="MOVING"))
        
        # Create Curobo Pose
        # Curobo expects quaternion as [w, x, y, z]
        # ROS msg is [x, y, z, w]
        target_pos = [msg.position.x, msg.position.y, msg.position.z]
        target_quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        
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
            self.status_pub.publish(String(data="SUCCEEDED"))
        else:
            self.get_logger().error(f"Planning failed! Status: {result.status}")
            self.status_pub.publish(String(data="FAILED: Planning Error"))

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
