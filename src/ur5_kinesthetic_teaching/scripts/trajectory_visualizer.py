#!/usr/bin/env python3
"""
Trajectory Visualizer for UR5 Kinesthetic Teaching

Loads and visualizes recorded trajectories in RViz with detailed markers.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from std_msgs.msg import ColorRGBA
import json
import os
from pathlib import Path
import numpy as np


class TrajectoryVisualizer(Node):
    """Visualizes recorded trajectories in RViz."""

    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        # Declare parameters
        self.declare_parameter('trajectory_dir', os.path.expanduser('~/ur5_trajectories'))
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('visualization_rate', 1.0)  # Hz
        
        # Get parameters
        self.trajectory_dir = self.get_parameter('trajectory_dir').value
        self.base_frame = self.get_parameter('base_frame').value
        self.viz_rate = self.get_parameter('visualization_rate').value
        
        # State
        self.loaded_trajectories = []
        self.current_trajectory = None
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '~/trajectory_visualization',
            10
        )
        
        # Services
        self.load_srv = self.create_service(
            Trigger,
            '~/load_latest',
            self.load_latest_callback
        )
        self.clear_srv = self.create_service(
            Trigger,
            '~/clear_visualization',
            self.clear_visualization_callback
        )
        
        # Timer for continuous visualization
        self.viz_timer = self.create_timer(
            1.0 / self.viz_rate,
            self.publish_visualization
        )
        
        self.get_logger().info('Trajectory Visualizer initialized')
        
    def load_latest_callback(self, request, response):
        """Load and visualize the most recent trajectory."""
        trajectory_files = sorted(
            Path(self.trajectory_dir).glob('*.json'),
            key=os.path.getmtime,
            reverse=True
        )
        
        if not trajectory_files:
            response.success = False
            response.message = f'No trajectory files found in {self.trajectory_dir}'
            return response
        
        # Load most recent file
        filename = str(trajectory_files[0])
        
        try:
            with open(filename, 'r') as f:
                self.current_trajectory = json.load(f)
            
            response.success = True
            response.message = f'Loaded and visualizing: {self.current_trajectory["name"]}'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to load trajectory: {str(e)}'
            self.get_logger().error(response.message)
        
        return response
    
    def clear_visualization_callback(self, request, response):
        """Clear all visualizations."""
        self.current_trajectory = None
        
        # Publish empty marker array to clear
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_pub.publish(marker_array)
        
        response.success = True
        response.message = 'Visualization cleared'
        self.get_logger().info(response.message)
        
        return response
    
    def publish_visualization(self):
        """Publish visualization markers."""
        if self.current_trajectory is None:
            return
        
        marker_array = MarkerArray()
        marker_id = 0
        
        # Visualize task space trajectory
        task_poses = self.current_trajectory['task_space']['poses']
        
        if len(task_poses) > 1:
            # Path as line strip
            path_marker = Marker()
            path_marker.header.frame_id = self.base_frame
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = 'task_space_path'
            path_marker.id = marker_id
            marker_id += 1
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.003
            
            # Color gradient from blue to red
            for i, pose_data in enumerate(task_poses):
                point = Point()
                point.x = pose_data['position']['x']
                point.y = pose_data['position']['y']
                point.z = pose_data['position']['z']
                path_marker.points.append(point)
                
                # Gradient color
                color = ColorRGBA()
                ratio = i / max(len(task_poses) - 1, 1)
                color.r = ratio
                color.g = 0.0
                color.b = 1.0 - ratio
                color.a = 0.8
                path_marker.colors.append(color)
            
            marker_array.markers.append(path_marker)
            
            # Waypoint spheres
            for i in range(0, len(task_poses), max(1, len(task_poses) // 20)):  # Max 20 spheres
                pose_data = task_poses[i]
                
                sphere = Marker()
                sphere.header.frame_id = self.base_frame
                sphere.header.stamp = self.get_clock().now().to_msg()
                sphere.ns = 'waypoints'
                sphere.id = marker_id
                marker_id += 1
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                
                sphere.pose.position.x = pose_data['position']['x']
                sphere.pose.position.y = pose_data['position']['y']
                sphere.pose.position.z = pose_data['position']['z']
                sphere.pose.orientation.x = pose_data['orientation']['x']
                sphere.pose.orientation.y = pose_data['orientation']['y']
                sphere.pose.orientation.z = pose_data['orientation']['z']
                sphere.pose.orientation.w = pose_data['orientation']['w']
                
                sphere.scale.x = 0.015
                sphere.scale.y = 0.015
                sphere.scale.z = 0.015
                
                ratio = i / max(len(task_poses) - 1, 1)
                sphere.color.r = ratio
                sphere.color.g = 0.5
                sphere.color.b = 1.0 - ratio
                sphere.color.a = 0.6
                
                marker_array.markers.append(sphere)
                
                # Orientation arrows
                arrow = Marker()
                arrow.header.frame_id = self.base_frame
                arrow.header.stamp = self.get_clock().now().to_msg()
                arrow.ns = 'orientations'
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                
                arrow.pose.position.x = pose_data['position']['x']
                arrow.pose.position.y = pose_data['position']['y']
                arrow.pose.position.z = pose_data['position']['z']
                arrow.pose.orientation.x = pose_data['orientation']['x']
                arrow.pose.orientation.y = pose_data['orientation']['y']
                arrow.pose.orientation.z = pose_data['orientation']['z']
                arrow.pose.orientation.w = pose_data['orientation']['w']
                
                arrow.scale.x = 0.03  # Length
                arrow.scale.y = 0.005  # Width
                arrow.scale.z = 0.005  # Height
                
                arrow.color.r = 1.0
                arrow.color.g = 1.0
                arrow.color.b = 0.0
                arrow.color.a = 0.7
                
                marker_array.markers.append(arrow)
            
            # Start marker (large green sphere)
            start_marker = Marker()
            start_marker.header.frame_id = self.base_frame
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = 'start'
            start_marker.id = marker_id
            marker_id += 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            
            start_pose = task_poses[0]
            start_marker.pose.position.x = start_pose['position']['x']
            start_marker.pose.position.y = start_pose['position']['y']
            start_marker.pose.position.z = start_pose['position']['z']
            
            start_marker.scale.x = 0.03
            start_marker.scale.y = 0.03
            start_marker.scale.z = 0.03
            
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            
            marker_array.markers.append(start_marker)
            
            # End marker (large red sphere)
            end_marker = Marker()
            end_marker.header.frame_id = self.base_frame
            end_marker.header.stamp = self.get_clock().now().to_msg()
            end_marker.ns = 'end'
            end_marker.id = marker_id
            marker_id += 1
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            
            end_pose = task_poses[-1]
            end_marker.pose.position.x = end_pose['position']['x']
            end_marker.pose.position.y = end_pose['position']['y']
            end_marker.pose.position.z = end_pose['position']['z']
            
            end_marker.scale.x = 0.03
            end_marker.scale.y = 0.03
            end_marker.scale.z = 0.03
            
            end_marker.color.r = 1.0
            end_marker.color.g = 0.0
            end_marker.color.b = 0.0
            end_marker.color.a = 1.0
            
            marker_array.markers.append(end_marker)
            
            # Text label with trajectory info
            text_marker = Marker()
            text_marker.header.frame_id = self.base_frame
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'info'
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = start_pose['position']['x']
            text_marker.pose.position.y = start_pose['position']['y']
            text_marker.pose.position.z = start_pose['position']['z'] + 0.1
            
            text_marker.scale.z = 0.02
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"{self.current_trajectory['name']}\n{len(task_poses)} points"
            
            marker_array.markers.append(text_marker)
        
        # Publish markers
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
