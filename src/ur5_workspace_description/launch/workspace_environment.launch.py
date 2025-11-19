#!/usr/bin/env python3
"""
Complete workspace environment launch file

This launch file starts both visualization markers and collision objects,
providing a complete tabletop environment for the UR5 robot.

Usage:
    # For visualization only (no collision checking):
    ros2 launch ur5_workspace_description workspace_environment.launch.py use_collision_objects:=false
    
    # For both visualization and collision checking (requires MoveIt):
    ros2 launch ur5_workspace_description workspace_environment.launch.py use_collision_objects:=true
    
    # Default is both visualization and collision objects:
    ros2 launch ur5_workspace_description workspace_environment.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete workspace environment."""
    
    # Declare launch arguments
    use_collision_objects_arg = DeclareLaunchArgument(
        'use_collision_objects',
        default_value='true',
        description='Whether to publish collision objects to MoveIt planning scene'
    )
    
    use_markers_arg = DeclareLaunchArgument(
        'use_markers',
        default_value='true',
        description='Whether to publish visualization markers to RViz'
    )
    
    # Get launch configurations
    use_collision_objects = LaunchConfiguration('use_collision_objects')
    use_markers = LaunchConfiguration('use_markers')
    
    # Workspace markers node (visualization only)
    workspace_markers_node = Node(
        package='ur5_workspace_description',
        executable='workspace_markers',
        name='workspace_markers_publisher',
        output='screen',
        condition=IfCondition(use_markers)
    )
    
    # Workspace collision objects node (MoveIt integration)
    workspace_collision_node = Node(
        package='ur5_workspace_description',
        executable='workspace_collision_objects',
        name='workspace_collision_objects_publisher',
        output='screen',
        condition=IfCondition(use_collision_objects)
    )
    
    return LaunchDescription([
        use_collision_objects_arg,
        use_markers_arg,
        workspace_markers_node,
        workspace_collision_node,
    ])
