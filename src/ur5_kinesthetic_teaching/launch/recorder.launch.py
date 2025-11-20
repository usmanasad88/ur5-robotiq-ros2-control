"""Launch file for kinesthetic teaching trajectory recorder."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for trajectory recorder."""
    
    # Declare arguments
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
    
    trajectory_dir_arg = DeclareLaunchArgument(
        'trajectory_dir',
        default_value=os.path.expanduser('~/ur5_trajectories'),
        description='Directory to save recorded trajectories'
    )
    
    min_position_change_arg = DeclareLaunchArgument(
        'min_position_change',
        default_value='0.001',
        description='Minimum position change to record (meters or radians)'
    )
    
    auto_save_arg = DeclareLaunchArgument(
        'auto_save',
        default_value='true',
        description='Automatically save trajectory when recording stops'
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
            'min_position_change': LaunchConfiguration('min_position_change'),
            'auto_save': LaunchConfiguration('auto_save'),
        }],
        remappings=[
            ('/joint_states', '/joint_states'),
        ]
    )
    
    return LaunchDescription([
        recording_rate_arg,
        end_effector_frame_arg,
        base_frame_arg,
        trajectory_dir_arg,
        min_position_change_arg,
        auto_save_arg,
        recorder_node,
    ])
