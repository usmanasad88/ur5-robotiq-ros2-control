#!/usr/bin/env python3
"""
Launch file for UR5 with Robotiq 2F-85 gripper
Compatible with ROS2 Humble and Universal Robots ROS2 Driver
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare arguments that can be passed to this launch file
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.102",
            description="IP address of the UR5 robot",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware (simulation mode)",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz for visualization",
        )
    )
    
    # Path to our custom RViz config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur5_robotiq_description"),
        "rviz",
        "view_robot.rviz"
    ])
    
    # Gripper joint state publisher (publishes gripper finger positions)
    gripper_joint_publisher = Node(
        package='ur5_robotiq_description',
        executable='gripper_joint_state_publisher.py',
        name='gripper_joint_state_publisher',
        output='screen'
    )
    
    # Include the standard UR control launch file with our custom URDF
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": "ur5",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            # Use our custom description package with gripper
            "description_package": "ur5_robotiq_description",
            "description_file": "ur5_robotiq.urdf.xacro",
            # Use our custom RViz config
            "rviz_config_file": rviz_config_file,
        }.items(),
    )
    
    return LaunchDescription(declared_arguments + [
        ur_control_launch,
        gripper_joint_publisher,
    ])
