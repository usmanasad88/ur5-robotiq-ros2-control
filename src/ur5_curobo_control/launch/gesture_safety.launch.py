"""
Launch file for Gesture Safety Monitor.

This launch file starts the gesture-based safety monitor that uses MediaPipe
to recognize hand gestures for robot safety control.

Gestures:
- STOP (safety triggered): Open Palm, Pointing Up
- RESUME (safety cleared): Thumbs Up, Victory
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    camera_id = LaunchConfiguration('camera_id')
    model_path = LaunchConfiguration('model_path')
    use_realsense = LaunchConfiguration('use_realsense')
    min_detection_confidence = LaunchConfiguration('min_detection_confidence')
    gesture_hold_frames = LaunchConfiguration('gesture_hold_frames')
    
    declare_camera_id = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID (0 for default webcam)'
    )
    
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to gesture_recognizer.task model (auto-downloads if empty)'
    )
    
    declare_use_realsense = DeclareLaunchArgument(
        'use_realsense',
        default_value='false',
        description='Use Intel RealSense camera instead of webcam'
    )
    
    declare_min_detection_confidence = DeclareLaunchArgument(
        'min_detection_confidence',
        default_value='0.5',
        description='Minimum confidence for hand detection (0.0-1.0)'
    )
    
    declare_gesture_hold_frames = DeclareLaunchArgument(
        'gesture_hold_frames',
        default_value='3',
        description='Number of frames to hold gesture before triggering action'
    )
    
    gesture_safety_node = Node(
        package='ur5_curobo_control',
        executable='gesture_safety_monitor',
        name='gesture_safety_monitor',
        output='screen',
        parameters=[{
            'camera_id': camera_id,
            'model_path': model_path,
            'use_realsense': use_realsense,
            'min_detection_confidence': min_detection_confidence,
            'gesture_hold_frames': gesture_hold_frames,
        }]
    )
    
    return LaunchDescription([
        declare_camera_id,
        declare_model_path,
        declare_use_realsense,
        declare_min_detection_confidence,
        declare_gesture_hold_frames,
        gesture_safety_node
    ])
