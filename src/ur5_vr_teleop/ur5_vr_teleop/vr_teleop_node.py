#!/usr/bin/env python3
"""
HTC Vive VR teleoperation node for UR5 robot.
Reads input from HTC Vive controller and publishes PoseDelta messages.
"""

import rclpy
from rclpy.node import Node
from ur5_teleop_msgs.msg import PoseDelta
import sys
import numpy as np

try:
    import openvr
except ImportError:
    print("Error: openvr library not found!")
    print("Please install it using: pip install openvr")
    sys.exit(1)


class VRTeleopNode(Node):
    """ROS 2 node for HTC Vive VR teleoperation."""

    def __init__(self):
        super().__init__('ur5_vr_teleop')
        
        # Publisher
        self.publisher_ = self.create_publisher(PoseDelta, '/ur5/teleop_delta', 10)
        
        # Declare parameters
        self.declare_parameter('controller_side', 'any')  # 'left', 'right', or 'any'
        self.declare_parameter('scale_translation', 50.0)    # Scale for position movement (high for relative mode)
        self.declare_parameter('scale_rotation', 50.0)       # Scale for rotation (high for relative mode)
        self.declare_parameter('publish_rate', 30.0)         # Hz (30Hz for smooth tracking)
        self.declare_parameter('trigger_threshold', 0.1)     # Trigger activation threshold
        self.declare_parameter('use_relative_mode', True)    # Use relative position deltas
        
        # Get parameters
        self.controller_side = self.get_parameter('controller_side').value
        self.scale_translation = self.get_parameter('scale_translation').value
        self.scale_rotation = self.get_parameter('scale_rotation').value
        publish_rate = self.get_parameter('publish_rate').value
        self.trigger_threshold = self.get_parameter('trigger_threshold').value
        self.use_relative_mode = self.get_parameter('use_relative_mode').value
        
        # Initialize OpenVR
        try:
            self.vr_system = openvr.init(openvr.VRApplication_Other)
        except Exception as e:
            self.get_logger().error(f'Failed to initialize OpenVR: {e}')
            self.get_logger().error('Make sure SteamVR is running!')
            sys.exit(1)
        
        self.get_logger().info('OpenVR/SteamVR initialized successfully!')
        self.get_logger().info(f'Configuration:')
        self.get_logger().info(f'  - Controller side: {self.controller_side}')
        self.get_logger().info(f'  - Translation scale: {self.scale_translation}')
        self.get_logger().info(f'  - Rotation scale: {self.scale_rotation}')
        self.get_logger().info(f'  - Publish rate: {publish_rate} Hz')
        self.get_logger().info(f'  - Relative mode: {self.use_relative_mode}')
        
        # Find controller device index
        self.controller_index = self.find_controller()
        if self.controller_index is None:
            self.get_logger().error(f'Could not find {self.controller_side} controller!')
            sys.exit(1)
        
        self.get_logger().info(f'Found {self.controller_side} controller at index {self.controller_index}')
        
        # Store previous pose for relative mode
        self.last_pose = None
        self.last_trigger_pressed = False
        self.message_count = 0  # Counter for logging
        
        # Create timer for reading VR controller
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

    def find_controller(self):
        """Find the controller device index based on side preference."""
        controllers_found = []
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self.vr_system.getTrackedDeviceClass(i)
            if device_class == openvr.TrackedDeviceClass_Controller:
                try:
                    # Get controller role
                    role = self.vr_system.getInt32TrackedDeviceProperty(
                        i, openvr.Prop_ControllerRoleHint_Int32
                    )
                    
                    role_name = 'unknown'
                    if role == openvr.TrackedControllerRole_RightHand:
                        role_name = 'right'
                    elif role == openvr.TrackedControllerRole_LeftHand:
                        role_name = 'left'
                    
                    controllers_found.append((i, role_name))
                    self.get_logger().info(f'Found controller at index {i}: {role_name}')
                    
                    # Match based on controller_side preference
                    if self.controller_side == 'any':
                        # Use first controller found
                        self.get_logger().info(f'Using controller {i} ({role_name}) - side set to "any"')
                        return i
                    elif self.controller_side == 'right' and role == openvr.TrackedControllerRole_RightHand:
                        return i
                    elif self.controller_side == 'left' and role == openvr.TrackedControllerRole_LeftHand:
                        return i
                except Exception as e:
                    self.get_logger().warn(f'Error reading controller {i}: {e}')
        
        if controllers_found:
            if self.controller_side == 'any':
                self.get_logger().error('Found controllers but none were accessible')
            else:
                self.get_logger().error(f'Found controllers: {controllers_found}, but none matched "{self.controller_side}"')
                self.get_logger().info(f'Hint: Try -p controller_side:=any to use any available controller')
        else:
            self.get_logger().error('No controllers found. Make sure SteamVR shows controllers as tracked (green).')
        
        return None

    def get_controller_pose(self):
        """Get the current pose of the controller."""
        poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0,
            openvr.k_unMaxTrackedDeviceCount
        )
        
        pose = poses[self.controller_index]
        if not pose.bPoseIsValid:
            return None
        
        # Extract position and rotation from pose matrix
        mat = pose.mDeviceToAbsoluteTracking
        
        # Position (last column)
        position = np.array([mat[0][3], mat[1][3], mat[2][3]])
        
        # Rotation (convert to roll, pitch, yaw)
        # Using OpenVR coordinate system: X=right, Y=up, Z=back
        roll = np.arctan2(mat[2][1], mat[2][2])
        pitch = np.arctan2(-mat[2][0], np.sqrt(mat[2][1]**2 + mat[2][2]**2))
        yaw = np.arctan2(mat[1][0], mat[0][0])
        
        return {
            'position': position,
            'rotation': np.array([roll, pitch, yaw])
        }

    def get_controller_state(self):
        """Get button and trigger state of the controller."""
        state = self.vr_system.getControllerState(self.controller_index)
        if state is None:
            return None
        
        # Get trigger value (axis 1 for Vive controllers)
        trigger_value = state[1].rAxis[1].x
        
        # Check if trigger is pressed
        trigger_pressed = trigger_value > self.trigger_threshold
        
        # Get trackpad/joystick (axis 0)
        trackpad_x = state[1].rAxis[0].x
        trackpad_y = state[1].rAxis[0].y
        
        # Get button presses
        button_mask = state[1].ulButtonPressed
        
        return {
            'trigger_value': trigger_value,
            'trigger_pressed': trigger_pressed,
            'trackpad_x': trackpad_x,
            'trackpad_y': trackpad_y,
            'button_mask': button_mask
        }

    def timer_callback(self):
        """Read VR controller and publish PoseDelta message."""
        pose = self.get_controller_pose()
        controller_state = self.get_controller_state()
        
        if pose is None or controller_state is None:
            return
        
        # Only send commands when trigger is pressed
        if not controller_state['trigger_pressed']:
            # Don't send zero delta - just stop publishing
            if self.last_trigger_pressed:
                self.get_logger().info('Trigger released - stopping motion')
            
            self.last_trigger_pressed = False
            self.last_pose = pose
            return
        
        # On first trigger press, log initial pose
        if not self.last_trigger_pressed:
            self.get_logger().info('Trigger pressed - starting motion')
            self.get_logger().info(f'Initial VR controller pose:')
            self.get_logger().info(f'  Position: x={pose["position"][0]:.4f}, y={pose["position"][1]:.4f}, z={pose["position"][2]:.4f}')
            self.get_logger().info(f'  Rotation: r={pose["rotation"][0]:.4f}, p={pose["rotation"][1]:.4f}, y={pose["rotation"][2]:.4f}')
        
        # Calculate delta from last pose
        if self.last_pose is not None and self.use_relative_mode:
            # Calculate position delta
            pos_delta = pose['position'] - self.last_pose['position']
            
            # Calculate rotation delta
            rot_delta = pose['rotation'] - self.last_pose['rotation']
            
            # Only log every 10th message to reduce spam
            self.message_count += 1
            should_log = (self.message_count % 10 == 0)
            
            if should_log:
                self.get_logger().info(
                    f'Raw VR delta: pos=({pos_delta[0]:.6f}, {pos_delta[1]:.6f}, {pos_delta[2]:.6f}), '
                    f'rot=({rot_delta[0]:.6f}, {rot_delta[1]:.6f}, {rot_delta[2]:.6f})'
                )
            
            # Apply scaling
            dx = pos_delta[0] * self.scale_translation
            dy = pos_delta[1] * self.scale_translation
            dz = pos_delta[2] * self.scale_translation
            
            droll = rot_delta[0] * self.scale_rotation
            dpitch = rot_delta[1] * self.scale_rotation
            dyaw = rot_delta[2] * self.scale_rotation
            
            if should_log:
                self.get_logger().info(
                    f'Scaled delta: pos=({dx:.6f}, {dy:.6f}, {dz:.6f}), '
                    f'rot=({droll:.6f}, {dpitch:.6f}, {dyaw:.6f})'
                )
            
            # Only publish if there's meaningful motion
            if (abs(dx) > 0.0001 or abs(dy) > 0.0001 or abs(dz) > 0.0001 or
                abs(droll) > 0.001 or abs(dpitch) > 0.001 or abs(dyaw) > 0.001):
                
                msg = PoseDelta()
                msg.dx = float(dx)
                msg.dy = float(dy)
                msg.dz = float(dz)
                msg.droll = float(droll)
                msg.dpitch = float(dpitch)
                msg.dyaw = float(dyaw)
                
                self.publisher_.publish(msg)
                
                if should_log:
                    self.get_logger().info(
                        f'✓ PUBLISHED: dx={dx:.6f}, dy={dy:.6f}, dz={dz:.6f}, '
                        f'dr={droll:.6f}, dp={dpitch:.6f}, dyw={dyaw:.6f}'
                    )
            else:
                if should_log:
                    self.get_logger().warn(
                        f'✗ FILTERED (below threshold): dx={dx:.6f}, dy={dy:.6f}, dz={dz:.6f}, '
                        f'dr={droll:.6f}, dp={dpitch:.6f}, dyw={dyaw:.6f}'
                    )
        
        # Update last pose
        self.last_pose = pose
        self.last_trigger_pressed = True

    def destroy_node(self):
        """Clean up OpenVR on shutdown."""
        self.get_logger().info('Shutting down OpenVR...')
        openvr.shutdown()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = None
    
    try:
        node = VRTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
