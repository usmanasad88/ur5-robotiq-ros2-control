from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('ur5_curobo_control')
    
    robot_config_file = LaunchConfiguration('robot_config_file')
    world_config_file = LaunchConfiguration('world_config_file')
    programs_directory = LaunchConfiguration('programs_directory')
    program_file = LaunchConfiguration('program_file')
    default_speed = LaunchConfiguration('default_speed')
    trajectory_dt = LaunchConfiguration('trajectory_dt')
    auto_execute = LaunchConfiguration('auto_execute')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    presenter_control = LaunchConfiguration('presenter_control')
    robot_ip = LaunchConfiguration('robot_ip')
    
    declare_robot_config = DeclareLaunchArgument(
        'robot_config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'ur5_existing.yml']),
        description='Path to cuRobo robot config YAML'
    )
    
    declare_world_config = DeclareLaunchArgument(
        'world_config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'collision_base.yml']),
        description='Path to cuRobo world config YAML'
    )
    
    declare_programs_dir = DeclareLaunchArgument(
        'programs_directory',
        default_value=PathJoinSubstitution([pkg_share, 'programs']),
        description='Directory containing robot program files'
    )
    
    declare_program_file = DeclareLaunchArgument(
        'program_file',
        default_value='',
        description='Program file to load (relative to programs_directory or absolute path)'
    )
    
    declare_default_speed = DeclareLaunchArgument(
        'default_speed',
        default_value='0.3',
        description='Default speed factor (0.0-1.0)'
    )
    
    declare_robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.17.66.105',
        description='Robot IP address for gripper communication'
    )
    
    declare_trajectory_dt = DeclareLaunchArgument(
        'trajectory_dt',
        default_value='0.05',
        description='Trajectory interpolation timestep in seconds'
    )
    
    declare_auto_execute = DeclareLaunchArgument(
        'auto_execute',
        default_value='false',
        description='Auto-execute program after loading'
    )
    
    declare_use_fake_hardware = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware (sim) or real robot for gripper'
    )
    
    declare_presenter_control = DeclareLaunchArgument(
        'presenter_control',
        default_value='true',
        description='Enable Logitech presenter control (Next=start/restart, Prev=pause)'
    )
    
    # Robotiq gripper adapter - only launched when NOT using fake hardware
    robotiq_adapter_node = Node(
        package='robotiq_2f_urcap_adapter',
        executable='robotiq_2f_adapter_node.py',
        name='robotiq_2f_urcap_adapter',
        output='screen',
        parameters=[{
            'robot_ip': robot_ip,
        }],
        condition=UnlessCondition(use_fake_hardware)
    )
    
    program_executor_node = Node(
        package='ur5_curobo_control',
        executable='program_executor_node',
        name='ur5_program_executor',
        output='screen',
        parameters=[{
            'robot_config_file': robot_config_file,
            'world_config_file': world_config_file,
            'programs_directory': programs_directory,
            'program_file': program_file,
            'default_speed': default_speed,
            'trajectory_dt': trajectory_dt,
            'auto_execute': auto_execute,
            'use_fake_hardware': use_fake_hardware,
            'presenter_control': presenter_control,
        }]
    )
    
    return LaunchDescription([
        declare_robot_config,
        declare_world_config,
        declare_programs_dir,
        declare_program_file,
        declare_default_speed,
        declare_robot_ip,
        declare_trajectory_dt,
        declare_auto_execute,
        declare_use_fake_hardware,
        declare_presenter_control,
        robotiq_adapter_node,
        program_executor_node
    ])
