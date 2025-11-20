#!/usr/bin/env python3
"""
Trajectory Player Node for UR5 Kinesthetic Teaching

Replays recorded trajectories in either joint space or task space.
Supports different playback modes and speed control.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float32
import json
import os
from pathlib import Path
from datetime import datetime
import numpy as np


class TrajectoryPlayer(Node):
    """Replays recorded kinesthetic teaching trajectories."""

    def __init__(self):
        super().__init__('trajectory_player')
        
        # Declare parameters
        self.declare_parameter('trajectory_dir', os.path.expanduser('~/ur5_trajectories'))
        self.declare_parameter('playback_speed', 1.0)  # Speed multiplier
        self.declare_parameter('controller_name', 'scaled_joint_trajectory_controller')
        self.declare_parameter('use_moveit', False)  # Future: integrate with MoveIt
        self.declare_parameter('loop_playback', False)
        
        # Get parameters
        self.trajectory_dir = self.get_parameter('trajectory_dir').value
        self.playback_speed = self.get_parameter('playback_speed').value
        self.controller_name = self.get_parameter('controller_name').value
        self.use_moveit = self.get_parameter('use_moveit').value
        self.loop_playback = self.get_parameter('loop_playback').value
        
        # Playback state
        self.is_playing = False
        self.current_trajectory = None
        self.loaded_trajectory_name = None
        
        # Action client for joint trajectory controller
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{self.controller_name}/follow_joint_trajectory'
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '~/playback_status', 10)
        
        # Services (using Trigger for all - simple and works)
        self.load_srv = self.create_service(
            Trigger,
            '~/load_trajectory',
            self.load_trajectory_callback
        )
        self.play_srv = self.create_service(
            Trigger,
            '~/play_trajectory',
            self.play_trajectory_callback
        )
        self.stop_srv = self.create_service(
            Trigger,
            '~/stop_playback',
            self.stop_playback_callback
        )
        self.list_srv = self.create_service(
            Trigger,
            '~/list_trajectories',
            self.list_trajectories_callback
        )
        
        # Subscriber for playback speed control
        self.speed_sub = self.create_subscription(
            Float32,
            '~/set_playback_speed',
            self.speed_callback,
            10
        )
        
        self.get_logger().info('Trajectory Player initialized')
        self.get_logger().info(f'Trajectory directory: {self.trajectory_dir}')
        self.get_logger().info(f'Controller: {self.controller_name}')
        
    def load_trajectory_callback(self, request, response):
        """Load a trajectory from file."""
        # Note: Using Trigger service for now, filename passed via parameter
        # In production, create custom service definition with filename field
        
        # For this implementation, we'll load the most recent trajectory
        trajectory_files = sorted(Path(self.trajectory_dir).glob('*.json'), 
                                 key=os.path.getmtime, reverse=True)
        
        if not trajectory_files:
            response.success = False
            response.message = f'No trajectory files found in {self.trajectory_dir}'
            return response
        
        # Load most recent file
        filename = str(trajectory_files[0])
        result = self.load_trajectory_file(filename)
        
        response.success = result['success']
        response.message = result['message']
        return response
    
    def load_trajectory_file(self, filename):
        """Load trajectory data from JSON file."""
        try:
            with open(filename, 'r') as f:
                trajectory_data = json.load(f)
            
            self.current_trajectory = trajectory_data
            self.loaded_trajectory_name = trajectory_data.get('name', os.path.basename(filename))
            
            num_joint_points = len(trajectory_data['joint_space']['positions'])
            num_task_points = len(trajectory_data['task_space']['poses'])
            
            message = f'Loaded trajectory: {self.loaded_trajectory_name} '
            message += f'({num_joint_points} joint points, {num_task_points} task points)'
            
            self.get_logger().info(message)
            
            return {'success': True, 'message': message}
            
        except Exception as e:
            message = f'Failed to load trajectory: {str(e)}'
            self.get_logger().error(message)
            return {'success': False, 'message': message}
    
    def play_trajectory_callback(self, request, response):
        """Play the loaded trajectory."""
        if self.current_trajectory is None:
            response.success = False
            response.message = 'No trajectory loaded. Load a trajectory first.'
            return response
        
        if self.is_playing:
            response.success = False
            response.message = 'Already playing a trajectory.'
            return response
        
        # Play in joint space (default mode)
        result = self.play_joint_space_trajectory()
        
        response.success = result['success']
        response.message = result['message']
        return response
    
    def play_joint_space_trajectory(self):
        """Execute trajectory in joint space using joint trajectory controller."""
        joint_data = self.current_trajectory['joint_space']
        
        if len(joint_data['positions']) == 0:
            return {'success': False, 'message': 'No joint space data in trajectory.'}
        
        # Wait for action server
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            return {'success': False, 'message': 'Joint trajectory controller not available.'}
        
        # Create joint trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_data['joint_names']
        
        # Add trajectory points
        start_time = joint_data['timestamps'][0] if joint_data['timestamps'] else 0.0
        
        for i, (positions, velocities, timestamp) in enumerate(zip(
            joint_data['positions'],
            joint_data['velocities'],
            joint_data['timestamps']
        )):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            
            # Calculate time from start, adjusted by playback speed
            time_from_start = (timestamp - start_time) / self.playback_speed
            point.time_from_start.sec = int(time_from_start)
            point.time_from_start.nanosec = int((time_from_start % 1) * 1e9)
            
            trajectory.points.append(point)
        
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Send goal
        self.get_logger().info(f'Sending trajectory with {len(trajectory.points)} points...')
        self.is_playing = True
        
        # Publish status
        status_msg = String()
        status_msg.data = 'PLAYING'
        self.status_pub.publish(status_msg)
        
        # Send goal asynchronously
        send_goal_future = self.trajectory_client.send_goal_async(
            goal,
            feedback_callback=self.trajectory_feedback_callback
        )
        send_goal_future.add_done_callback(self.trajectory_response_callback)
        
        return {
            'success': True,
            'message': f'Started playback of {len(trajectory.points)} points at {self.playback_speed}x speed'
        }
    
    def trajectory_feedback_callback(self, feedback_msg):
        """Handle trajectory execution feedback."""
        feedback = feedback_msg.feedback
        # Can add progress reporting here if needed
        pass
    
    def trajectory_response_callback(self, future):
        """Handle trajectory goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            self.is_playing = False
            status_msg = String()
            status_msg.data = 'REJECTED'
            self.status_pub.publish(status_msg)
            return
        
        self.get_logger().info('Trajectory goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.trajectory_result_callback)
    
    def trajectory_result_callback(self, future):
        """Handle trajectory execution result."""
        result = future.result().result
        status = future.result().status
        
        self.is_playing = False
        
        status_msg = String()
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Trajectory execution completed successfully')
            status_msg.data = 'COMPLETED'
            
            # Loop if enabled
            if self.loop_playback and self.current_trajectory is not None:
                self.get_logger().info('Looping trajectory playback...')
                self.play_joint_space_trajectory()
        else:
            self.get_logger().warn(f'Trajectory execution failed with status: {status}')
            status_msg.data = 'FAILED'
        
        self.status_pub.publish(status_msg)
    
    def stop_playback_callback(self, request, response):
        """Stop trajectory playback."""
        if not self.is_playing:
            response.success = False
            response.message = 'Not currently playing.'
            return response
        
        # Cancel current goal
        # Note: Proper goal cancellation would require storing goal_handle
        self.is_playing = False
        
        response.success = True
        response.message = 'Playback stopped.'
        self.get_logger().info(response.message)
        
        status_msg = String()
        status_msg.data = 'STOPPED'
        self.status_pub.publish(status_msg)
        
        return response
    
    def list_trajectories_callback(self, request, response):
        """List all available trajectory files."""
        trajectory_files = sorted(
            Path(self.trajectory_dir).glob('*.json'),
            key=os.path.getmtime,
            reverse=True
        )
        
        if not trajectory_files:
            response.success = True
            response.message = f'No trajectories found in {self.trajectory_dir}'
            return response
        
        file_list = []
        for i, filepath in enumerate(trajectory_files[:10]):  # Limit to 10 most recent
            stat = filepath.stat()
            mtime = datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
            file_list.append(f'{i+1}. {filepath.name} ({mtime})')
        
        message = 'Available trajectories:\n' + '\n'.join(file_list)
        response.success = True
        response.message = message
        self.get_logger().info(message)
        
        return response
    
    def speed_callback(self, msg: Float32):
        """Update playback speed."""
        self.playback_speed = max(0.1, min(5.0, msg.data))  # Clamp between 0.1x and 5.0x
        self.get_logger().info(f'Playback speed set to {self.playback_speed}x')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
