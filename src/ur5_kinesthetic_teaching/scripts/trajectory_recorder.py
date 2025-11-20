#!/usr/bin/env python3
"""
Trajectory Recorder Node for UR5 Kinesthetic Teaching

Records robot trajectories in both joint space and task space during manual manipulation.
Provides services to start/stop recording and save trajectories to file.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException
import numpy as np
import json
import os
from datetime import datetime
from pathlib import Path


class TrajectoryRecorder(Node):
    """Records robot trajectories during kinesthetic teaching."""

    def __init__(self):
        super().__init__('trajectory_recorder')
        
        # Declare parameters
        self.declare_parameter('recording_rate', 30.0)  # Hz
        self.declare_parameter('end_effector_frame', 'tool0')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('trajectory_dir', os.path.expanduser('~/ur5_trajectories'))
        self.declare_parameter('min_position_change', 0.001)  # meters or radians
        self.declare_parameter('auto_save', True)
        
        # Get parameters
        self.recording_rate = self.get_parameter('recording_rate').value
        self.ee_frame = self.get_parameter('end_effector_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.trajectory_dir = self.get_parameter('trajectory_dir').value
        self.min_change = self.get_parameter('min_position_change').value
        self.auto_save = self.get_parameter('auto_save').value
        
        # Create trajectory directory if it doesn't exist
        Path(self.trajectory_dir).mkdir(parents=True, exist_ok=True)
        
        # Recording state
        self.is_recording = False
        self.current_trajectory_name = None
        
        # Joint space data
        self.joint_names = []
        self.joint_positions_list = []
        self.joint_velocities_list = []
        self.joint_timestamps = []
        self.last_joint_position = None
        
        # Task space data
        self.ee_poses_list = []
        self.ee_timestamps = []
        self.last_ee_position = None
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS profile for reliable data
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '~/recording_status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '~/trajectory_markers', 10)
        
        # Services
        self.start_srv = self.create_service(
            Trigger,
            '~/start_recording',
            self.start_recording_callback
        )
        self.stop_srv = self.create_service(
            Trigger,
            '~/stop_recording',
            self.stop_recording_callback
        )
        self.save_srv = self.create_service(
            Trigger,
            '~/save_trajectory',
            self.save_trajectory_callback
        )
        self.clear_srv = self.create_service(
            Trigger,
            '~/clear_trajectory',
            self.clear_trajectory_callback
        )
        
        # Timer for task space recording (TF lookup)
        self.recording_timer = self.create_timer(
            1.0 / self.recording_rate,
            self.recording_timer_callback
        )
        
        # Visualization timer
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)
        
        self.get_logger().info('Trajectory Recorder initialized')
        self.get_logger().info(f'Trajectory directory: {self.trajectory_dir}')
        self.get_logger().info(f'Recording rate: {self.recording_rate} Hz')
        
    def joint_state_callback(self, msg: JointState):
        """Record joint states when recording is active."""
        if not self.is_recording:
            return
            
        # Initialize joint names on first message
        if not self.joint_names:
            self.joint_names = list(msg.name)
            self.get_logger().info(f'Joint names: {self.joint_names}')
        
        # Check if position changed significantly
        if self.last_joint_position is not None:
            position_change = np.linalg.norm(
                np.array(msg.position) - np.array(self.last_joint_position)
            )
            if position_change < self.min_change:
                return  # Skip if robot hasn't moved enough
        
        # Record joint data
        self.joint_positions_list.append(list(msg.position))
        self.joint_velocities_list.append(list(msg.velocity) if msg.velocity else [0.0] * len(msg.position))
        self.joint_timestamps.append(self.get_clock().now().nanoseconds / 1e9)
        self.last_joint_position = list(msg.position)
        
    def recording_timer_callback(self):
        """Record task space pose via TF lookup."""
        if not self.is_recording:
            return
            
        try:
            # Look up transform from base to end effector
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Extract position
            position = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Check if position changed significantly
            if self.last_ee_position is not None:
                position_change = np.linalg.norm(position - self.last_ee_position)
                if position_change < self.min_change:
                    return  # Skip if end effector hasn't moved enough
            
            # Create pose stamped message
            pose_stamped = PoseStamped()
            pose_stamped.header = transform.header
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation
            
            # Record task space data
            self.ee_poses_list.append(pose_stamped)
            self.ee_timestamps.append(self.get_clock().now().nanoseconds / 1e9)
            self.last_ee_position = position
            
        except TransformException as ex:
            if self.is_recording:
                self.get_logger().warn(f'Could not transform: {ex}', throttle_duration_sec=5.0)
    
    def start_recording_callback(self, request, response):
        """Start recording a new trajectory."""
        if self.is_recording:
            response.success = False
            response.message = 'Already recording. Stop current recording first.'
            return response
        
        # Clear previous data
        self.clear_trajectory()
        
        # Generate trajectory name
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.current_trajectory_name = f'trajectory_{timestamp}'
        
        # Start recording
        self.is_recording = True
        
        response.success = True
        response.message = f'Started recording: {self.current_trajectory_name}'
        self.get_logger().info(response.message)
        
        # Publish status
        status_msg = String()
        status_msg.data = 'RECORDING'
        self.status_pub.publish(status_msg)
        
        return response
    
    def stop_recording_callback(self, request, response):
        """Stop recording and optionally save."""
        if not self.is_recording:
            response.success = False
            response.message = 'Not currently recording.'
            return response
        
        # Stop recording
        self.is_recording = False
        
        num_joint_points = len(self.joint_positions_list)
        num_ee_points = len(self.ee_poses_list)
        
        response.success = True
        response.message = f'Stopped recording. Captured {num_joint_points} joint space points and {num_ee_points} task space points.'
        self.get_logger().info(response.message)
        
        # Publish status
        status_msg = String()
        status_msg.data = 'STOPPED'
        self.status_pub.publish(status_msg)
        
        # Auto-save if enabled
        if self.auto_save and (num_joint_points > 0 or num_ee_points > 0):
            save_result = self.save_trajectory_internal()
            response.message += f' Auto-saved to: {save_result["filename"]}'
        
        return response
    
    def save_trajectory_callback(self, request, response):
        """Save the recorded trajectory to file."""
        result = self.save_trajectory_internal()
        response.success = result['success']
        response.message = result['message']
        return response
    
    def save_trajectory_internal(self):
        """Internal method to save trajectory."""
        if not self.current_trajectory_name:
            return {'success': False, 'message': 'No trajectory name set.'}
        
        if len(self.joint_positions_list) == 0 and len(self.ee_poses_list) == 0:
            return {'success': False, 'message': 'No trajectory data to save.'}
        
        # Prepare trajectory data
        trajectory_data = {
            'name': self.current_trajectory_name,
            'timestamp': datetime.now().isoformat(),
            'recording_rate': self.recording_rate,
            'frames': {
                'base_frame': self.base_frame,
                'end_effector_frame': self.ee_frame
            },
            'joint_space': {
                'joint_names': self.joint_names,
                'positions': self.joint_positions_list,
                'velocities': self.joint_velocities_list,
                'timestamps': self.joint_timestamps
            },
            'task_space': {
                'poses': [],
                'timestamps': self.ee_timestamps
            }
        }
        
        # Convert poses to serializable format
        for pose_stamped in self.ee_poses_list:
            pose_data = {
                'position': {
                    'x': pose_stamped.pose.position.x,
                    'y': pose_stamped.pose.position.y,
                    'z': pose_stamped.pose.position.z
                },
                'orientation': {
                    'x': pose_stamped.pose.orientation.x,
                    'y': pose_stamped.pose.orientation.y,
                    'z': pose_stamped.pose.orientation.z,
                    'w': pose_stamped.pose.orientation.w
                }
            }
            trajectory_data['task_space']['poses'].append(pose_data)
        
        # Save to JSON file
        filename = os.path.join(self.trajectory_dir, f'{self.current_trajectory_name}.json')
        try:
            with open(filename, 'w') as f:
                json.dump(trajectory_data, f, indent=2)
            
            message = f'Trajectory saved to: {filename}'
            self.get_logger().info(message)
            return {'success': True, 'message': message, 'filename': filename}
        
        except Exception as e:
            message = f'Failed to save trajectory: {str(e)}'
            self.get_logger().error(message)
            return {'success': False, 'message': message}
    
    def clear_trajectory_callback(self, request, response):
        """Clear the current trajectory data."""
        self.clear_trajectory()
        response.success = True
        response.message = 'Trajectory data cleared.'
        self.get_logger().info(response.message)
        return response
    
    def clear_trajectory(self):
        """Clear all recorded data."""
        self.joint_positions_list.clear()
        self.joint_velocities_list.clear()
        self.joint_timestamps.clear()
        self.ee_poses_list.clear()
        self.ee_timestamps.clear()
        self.last_joint_position = None
        self.last_ee_position = None
    
    def publish_visualization(self):
        """Publish visualization markers for the recorded trajectory."""
        if len(self.ee_poses_list) < 2:
            return
        
        marker_array = MarkerArray()
        
        # Line strip showing the path
        path_marker = Marker()
        path_marker.header.frame_id = self.base_frame
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = 'trajectory_path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.005  # Line width
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0 if self.is_recording else 0.5
        
        for pose_stamped in self.ee_poses_list:
            path_marker.points.append(pose_stamped.pose.position)
        
        marker_array.markers.append(path_marker)
        
        # Spheres at key points
        for i, pose_stamped in enumerate(self.ee_poses_list[::10]):  # Every 10th point
            sphere_marker = Marker()
            sphere_marker.header.frame_id = self.base_frame
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = 'trajectory_points'
            sphere_marker.id = i + 1
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose = pose_stamped.pose
            sphere_marker.scale.x = 0.01
            sphere_marker.scale.y = 0.01
            sphere_marker.scale.z = 0.01
            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.0
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 0.8
            
            marker_array.markers.append(sphere_marker)
        
        # Start point (green sphere)
        if len(self.ee_poses_list) > 0:
            start_marker = Marker()
            start_marker.header.frame_id = self.base_frame
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = 'start_point'
            start_marker.id = 999
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose = self.ee_poses_list[0].pose
            start_marker.scale.x = 0.02
            start_marker.scale.y = 0.02
            start_marker.scale.z = 0.02
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            
            marker_array.markers.append(start_marker)
        
        # End point (red sphere)
        if len(self.ee_poses_list) > 0:
            end_marker = Marker()
            end_marker.header.frame_id = self.base_frame
            end_marker.header.stamp = self.get_clock().now().to_msg()
            end_marker.ns = 'end_point'
            end_marker.id = 998
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose = self.ee_poses_list[-1].pose
            end_marker.scale.x = 0.02
            end_marker.scale.y = 0.02
            end_marker.scale.z = 0.02
            end_marker.color.r = 1.0
            end_marker.color.g = 0.0
            end_marker.color.b = 0.0
            end_marker.color.a = 1.0
            
            marker_array.markers.append(end_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
