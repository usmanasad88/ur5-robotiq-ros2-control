from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('ur5_curobo_control')
    
    robot_config_file = LaunchConfiguration('robot_config_file')
    world_config_file = LaunchConfiguration('world_config_file')
    
    declare_robot_config = DeclareLaunchArgument(
        'robot_config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'ur5_existing.yml']),
        description='Path to robot config file'
    )
    
    declare_world_config = DeclareLaunchArgument(
        'world_config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'collision_base.yml']),
        description='Path to world config file'
    )
    
    curobo_node = Node(
        package='ur5_curobo_control',
        executable='curobo_control_node',
        name='curobo_control_node',
        output='screen',
        parameters=[{
            'robot_config_file': robot_config_file,
            'world_config_file': world_config_file
        }]
    )
    
    return LaunchDescription([
        declare_robot_config,
        declare_world_config,
        curobo_node
    ])
