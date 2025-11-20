"""Launch file for kinesthetic teaching trajectory player."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch description for trajectory player."""
    
    # Declare arguments
    trajectory_dir_arg = DeclareLaunchArgument(
        'trajectory_dir',
        default_value=os.path.expanduser('~/ur5_trajectories'),
        description='Directory containing recorded trajectories'
    )
    
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='1.0',
        description='Playback speed multiplier (0.1 to 5.0)'
    )
    
    controller_name_arg = DeclareLaunchArgument(
        'controller_name',
        default_value='scaled_joint_trajectory_controller',
        description='Joint trajectory controller name'
    )
    
    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='false',
        description='Use MoveIt for trajectory execution (future feature)'
    )
    
    loop_playback_arg = DeclareLaunchArgument(
        'loop_playback',
        default_value='false',
        description='Loop trajectory playback continuously'
    )
    
    # Trajectory player node
    player_node = Node(
        package='ur5_kinesthetic_teaching',
        executable='trajectory_player.py',
        name='trajectory_player',
        output='screen',
        parameters=[{
            'trajectory_dir': LaunchConfiguration('trajectory_dir'),
            'playback_speed': LaunchConfiguration('playback_speed'),
            'controller_name': LaunchConfiguration('controller_name'),
            'use_moveit': LaunchConfiguration('use_moveit'),
            'loop_playback': LaunchConfiguration('loop_playback'),
        }]
    )
    
    return LaunchDescription([
        trajectory_dir_arg,
        playback_speed_arg,
        controller_name_arg,
        use_moveit_arg,
        loop_playback_arg,
        player_node,
    ])
