#!/usr/bin/env python3
"""
Launch file for workspace visualization markers (Option 4)

This launch file starts the workspace_markers node which publishes
visual markers to RViz. These are for visualization only and do not
affect collision planning.

Usage:
    ros2 launch ur5_workspace_description workspace_markers.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for workspace markers."""
    
    workspace_markers_node = Node(
        package='ur5_workspace_description',
        executable='workspace_markers',
        name='workspace_markers_publisher',
        output='screen',
        parameters=[],
    )
    
    return LaunchDescription([
        workspace_markers_node,
    ])
