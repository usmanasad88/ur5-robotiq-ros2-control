#!/usr/bin/env python3
"""
Launch file for workspace collision objects (Option 5)

This launch file starts the workspace_collision_objects node which publishes
collision objects to the MoveIt planning scene. These objects will be used
for collision avoidance during motion planning.

Requires MoveIt to be running.

Usage:
    ros2 launch ur5_workspace_description workspace_collision_objects.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for workspace collision objects."""
    
    workspace_collision_node = Node(
        package='ur5_workspace_description',
        executable='workspace_collision_objects',
        name='workspace_collision_objects_publisher',
        output='screen',
        parameters=[],
    )
    
    return LaunchDescription([
        workspace_collision_node,
    ])
