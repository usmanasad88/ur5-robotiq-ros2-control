"""Launch file for complete kinesthetic teaching system."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():
    """Generate launch description for complete kinesthetic teaching."""
    
    # Declare arguments
    trajectory_dir_arg = DeclareLaunchArgument(
        'trajectory_dir',
        default_value=os.path.expanduser('~/ur5_trajectories'),
        description='Directory for trajectories'
    )
    
    recording_rate_arg = DeclareLaunchArgument(
        'recording_rate',
        default_value='30.0',
        description='Recording rate in Hz'
    )
    
    end_effector_frame_arg = DeclareLaunchArgument(
        'end_effector_frame',
        default_value='tool0',
        description='End effector frame name'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame name'
    )
    
    # Trajectory recorder node
    recorder_node = Node(
        package='ur5_kinesthetic_teaching',
        executable='trajectory_recorder.py',
        name='trajectory_recorder',
        output='screen',
        parameters=[{
            'recording_rate': LaunchConfiguration('recording_rate'),
            'end_effector_frame': LaunchConfiguration('end_effector_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'trajectory_dir': LaunchConfiguration('trajectory_dir'),
            'min_position_change': 0.001,
            'auto_save': True,
        }]
    )
    
    # Trajectory player node
    player_node = Node(
        package='ur5_kinesthetic_teaching',
        executable='trajectory_player.py',
        name='trajectory_player',
        output='screen',
        parameters=[{
            'trajectory_dir': LaunchConfiguration('trajectory_dir'),
            'playback_speed': 1.0,
            'controller_name': 'scaled_joint_trajectory_controller',
            'use_moveit': False,
            'loop_playback': False,
        }]
    )
    
    # Trajectory visualizer node
    visualizer_node = Node(
        package='ur5_kinesthetic_teaching',
        executable='trajectory_visualizer.py',
        name='trajectory_visualizer',
        output='screen',
        parameters=[{
            'trajectory_dir': LaunchConfiguration('trajectory_dir'),
            'base_frame': LaunchConfiguration('base_frame'),
            'visualization_rate': 1.0,
        }]
    )
    
    return LaunchDescription([
        trajectory_dir_arg,
        recording_rate_arg,
        end_effector_frame_arg,
        base_frame_arg,
        recorder_node,
        player_node,
        visualizer_node,
    ])
