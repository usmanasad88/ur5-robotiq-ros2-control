#!/usr/bin/env python3
"""
UR5 Program Selector UI - Streamlit App

A simple web UI for selecting and executing UR5 robot programs.
Provides buttons for each available program and shows execution status.

Usage:
    streamlit run program_selector_ui.py
"""

import streamlit as st
import subprocess
import os
import glob
import time
import math
from pathlib import Path
import signal
import psutil
import json
from datetime import datetime
import threading
import queue
import urllib.request
import urllib.error

# Suppress OpenCV warnings about missing cameras
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'

# Video capture
try:
    import cv2
    VIDEO_AVAILABLE = True
except ImportError:
    VIDEO_AVAILABLE = False
    st.warning("⚠️ OpenCV not available - video recording disabled")

# Numpy and HDF5 for data handling
import numpy as np
try:
    import h5py
    HDF5_AVAILABLE = True
except ImportError:
    HDF5_AVAILABLE = False

# GoPro recording
try:
    from gopro_recorder import GoProRecorder
    GOPRO_AVAILABLE = True
except ImportError:
    GOPRO_AVAILABLE = False

# ROS 2 imports - conditional to allow UI to work without full ROS setup
import sys
import os

# Add ROS 2 Python paths if not already present
ros_paths = [
    '/opt/ros/humble/local/lib/python3.10/dist-packages',
    '/opt/ros/humble/lib/python3.10/site-packages'
]
for path in ros_paths:
    if os.path.exists(path) and path not in sys.path:
        sys.path.insert(0, path)

try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger, SetBool
    from rcl_interfaces.srv import SetParameters, GetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, Float64
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    ROS_AVAILABLE = True
except ImportError as e:
    ROS_AVAILABLE = False
    st.warning(f"⚠️ ROS 2 not available - running in demo mode (Import error: {e})")

# Page config
st.set_page_config(
    page_title="UR5 Program Selector",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS
st.markdown("""
<style>
    .stButton>button {
        width: 100%;
        height: 80px;
        font-size: 18px;
        font-weight: bold;
    }
    .program-button {
        margin: 10px 0;
    }
    .status-success {
        color: #28a745;
        font-weight: bold;
    }
    .status-error {
        color: #dc3545;
        font-weight: bold;
    }
    .status-running {
        color: #007bff;
        font-weight: bold;
    }
</style>
""", unsafe_allow_html=True)


class ROSProgramController:
    """Controller for interacting with the program executor via ROS 2 services."""

    def __init__(self):
        self.use_fake = os.environ.get("USE_FAKE_HARDWARE", "false").lower() in ("true", "1", "yes")
        if not ROS_AVAILABLE:
            return
        
        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node('program_selector_ui')
        
        # Lock to prevent concurrent spin operations
        self._spin_lock = threading.Lock()
        
        # Service clients
        self.list_programs_client = self.node.create_client(
            Trigger, '/ur5_program_executor/list_programs')
        self.load_program_client = self.node.create_client(
            Trigger, '/ur5_program_executor/load_program')
        self.execute_program_client = self.node.create_client(
            Trigger, '/ur5_program_executor/execute_program')
        self.pause_client = self.node.create_client(
            SetBool, '/ur5_program_executor/pause')
        self.stop_client = self.node.create_client(
            Trigger, '/ur5_program_executor/stop')
        self.set_parameters_client = self.node.create_client(
            SetParameters, '/ur5_program_executor/set_parameters')
        self.move_to_pose_client = self.node.create_client(
            Trigger, '/ur5_program_executor/move_to_pose')
        self.move_relative_client = self.node.create_client(
            Trigger, '/ur5_program_executor/move_relative')
        self.set_teleop_mode_client = self.node.create_client(
            SetBool, '/ur5_program_executor/set_teleop_mode')

        # Workspace markers parameter clients (for live object placement)
        self.ws_set_params_client = self.node.create_client(
            SetParameters, '/workspace_markers_publisher/set_parameters')
        self.ws_get_params_client = self.node.create_client(
            GetParameters, '/workspace_markers_publisher/get_parameters')
        
        # Publishers for direct commands
        self.gripper_pub = self.node.create_publisher(
            String,
            '/gripper_command_simple',
            10
        )
        self.gripper_viz_pub = self.node.create_publisher(
            Float64,
            '/gripper_position_command',
            10
        )
        self.trajectory_pub = self.node.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Joint state subscriber
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        self.latest_joint_state = None  # Robot (6 joints)
        self.latest_gripper_state = None  # Gripper (1 joint)
        
        # UR5 joint names
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
    
    def _joint_state_callback(self, msg):
        """Store latest joint state (separate robot and gripper)."""
        if len(msg.name) == 6:
            # UR5 robot joints
            self.latest_joint_state = msg
        elif len(msg.name) == 1:
            # Robotiq gripper joint
            self.latest_gripper_state = msg
    
    def get_joint_state(self):
        """Get the latest joint state."""
        # Spin once to process callbacks (with lock to prevent concurrent spin)
        if self._spin_lock.acquire(blocking=False):
            try:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            finally:
                self._spin_lock.release()
        return self.latest_joint_state
    
    def _spin_for_future(self, future, timeout_sec=2.0):
        """Helper to spin until future complete with lock protection."""
        if not self._spin_lock.acquire(timeout=timeout_sec):
            return False
        try:
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
            return True
        finally:
            self._spin_lock.release()
    
    def set_program_file(self, program_name: str) -> bool:
        """Set the program_file parameter."""
        if not ROS_AVAILABLE:
            return False
        
        request = SetParameters.Request()
        param = Parameter()
        param.name = 'program_file'
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=program_name)
        request.parameters = [param]
        
        future = self.set_parameters_client.call_async(request)
        if not self._spin_for_future(future, timeout_sec=2.0):
            return False
        
        if future.result() is not None:
            return future.result().results[0].successful
        return False
    
    def load_program(self) -> tuple[bool, str]:
        """Load the currently set program."""
        if not ROS_AVAILABLE:
            return False, "ROS not available"
        
        request = Trigger.Request()
        future = self.load_program_client.call_async(request)
        if not self._spin_for_future(future, timeout_sec=5.0):
            return False, "Service call busy"
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, "Service call timeout"
    
    def execute_program(self) -> tuple[bool, str]:
        """Execute the loaded program."""
        if not ROS_AVAILABLE:
            return False, "ROS not available"
        
        request = Trigger.Request()
        future = self.execute_program_client.call_async(request)
        if not self._spin_for_future(future, timeout_sec=2.0):
            return False, "Service call busy"
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, "Service call timeout"
    
    def pause_program(self, pause: bool) -> tuple[bool, str]:
        """Pause or resume program execution."""
        if not ROS_AVAILABLE:
            return False, "ROS not available"
        
        request = SetBool.Request()
        request.data = pause
        future = self.pause_client.call_async(request)
        if not self._spin_for_future(future, timeout_sec=2.0):
            return False, "Service call busy"
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, "Service call timeout"
    
    def stop_program(self) -> tuple[bool, str]:
        """Stop program execution."""
        if not ROS_AVAILABLE:
            return False, "ROS not available"
        
        request = Trigger.Request()
        future = self.stop_client.call_async(request)
        if not self._spin_for_future(future, timeout_sec=2.0):
            return False, "Service call busy"
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, "Service call timeout"
    
    def set_teleop_mode(self, enable: bool) -> tuple[bool, str]:
        """Enable or disable teleop mode on the program executor."""
        if not ROS_AVAILABLE:
            return False, "ROS not available"

        request = SetBool.Request()
        request.data = enable
        future = self.set_teleop_mode_client.call_async(request)
        if not self._spin_for_future(future, timeout_sec=3.0):
            return False, "Service call busy or timed out"

        if future.result() is not None:
            return future.result().success, future.result().message
        return False, "Service call timeout"

    def open_gripper(self) -> tuple[bool, str]:
        """Send open gripper command via ROS 2 action."""
        if not ROS_AVAILABLE:
            return False, "ROS not available"

        # Publish visualization position (drives gripper_joint_state_publisher in sim)
        viz_msg = Float64()
        viz_msg.data = 0.0
        self.gripper_viz_pub.publish(viz_msg)

        # In fake/sim mode the viz topic is all that's needed — no action server exists
        if self.use_fake:
            return True, "Gripper opened (sim)"

        try:
            import subprocess
            cmd = (
                'source /opt/ros/humble/setup.bash && '
                'source /home/rml/ur5-robotiq-ros2-control/install/setup.bash && '
                'ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command '
                'robotiq_2f_urcap_adapter/action/GripperCommand '
                "'{command: {position: 0.085, max_effort: 255.0, max_speed: 0.1}}'"
            )
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=15,
                executable='/bin/bash'
            )
            if result.returncode == 0:
                return True, "Gripper opened successfully"
            else:
                return False, f"Gripper command failed: {result.stderr}"
        except subprocess.TimeoutExpired:
            return False, "Gripper command timed out"
        except Exception as e:
            return False, f"Failed to send gripper command: {e}"
    
    def close_gripper(self) -> tuple[bool, str]:
        """Send close gripper command via ROS 2 action."""
        if not ROS_AVAILABLE:
            return False, "ROS not available"

        # Publish visualization position (drives gripper_joint_state_publisher in sim)
        viz_msg = Float64()
        viz_msg.data = 1.0  # Fully closed (gripper_joint_state_publisher maps 1.0 -> 0.8 rad)
        self.gripper_viz_pub.publish(viz_msg)

        # In fake/sim mode the viz topic is all that's needed — no action server exists
        if self.use_fake:
            return True, "Gripper closed (sim)"

        try:
            import subprocess
            cmd = (
                'source /opt/ros/humble/setup.bash && '
                'source /home/rml/ur5-robotiq-ros2-control/install/setup.bash && '
                'ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command '
                'robotiq_2f_urcap_adapter/action/GripperCommand '
                "'{command: {position: 0.0, max_effort: 255.0, max_speed: 0.1}}'"
            )
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=15,
                executable='/bin/bash'
            )
            if result.returncode == 0:
                return True, "Gripper closed successfully"
            else:
                return False, f"Gripper command failed: {result.stderr}"
        except subprocess.TimeoutExpired:
            return False, "Gripper command timed out"
        except Exception as e:
            return False, f"Failed to send gripper command: {e}"
    
    def set_gripper_position(self, position: float) -> tuple[bool, str]:
        """Set gripper to specific position (0.0 = open, 1.0 = closed)."""
        if not ROS_AVAILABLE:
            return False, "ROS not available"

        position = max(0.0, min(1.0, position))

        # Publish visualization position (drives gripper_joint_state_publisher in sim)
        viz_msg = Float64()
        viz_msg.data = position  # gripper_joint_state_publisher maps [0,1] -> [0, 0.8 rad]
        self.gripper_viz_pub.publish(viz_msg)

        # In fake/sim mode the viz topic is all that's needed — no action server exists
        if self.use_fake:
            return True, f"Gripper position set to {position:.2f} (sim)"

        try:
            # Convert: 0 (open) -> 0.085m, 1 (closed) -> 0.0m
            robotiq_position = (1.0 - position) * 0.085
            import subprocess
            cmd = (
                'source /opt/ros/humble/setup.bash && '
                'source /home/rml/ur5-robotiq-ros2-control/install/setup.bash && '
                'ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command '
                'robotiq_2f_urcap_adapter/action/GripperCommand '
                f"'{{command: {{position: {robotiq_position}, max_effort: 100.0, max_speed: 0.1}}}}'"
            )
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=15,
                executable='/bin/bash'
            )
            if result.returncode == 0:
                return True, f"Gripper position set to {position:.2f}"
            else:
                return False, f"Gripper command failed: {result.stderr}"
        except subprocess.TimeoutExpired:
            return False, "Gripper command timed out"
        except Exception as e:
            return False, f"Failed to send gripper command: {e}"
    
    def set_workspace_marker_params(self, params: dict) -> tuple:
        """Set parameters on the workspace_markers_publisher node.

        Args:
            params: dict of {param_name: float_value}
                    e.g. {"table.x": 0.1, "table.yaw": 45.0}
        Returns:
            (success: bool, message: str)
        """
        if not ROS_AVAILABLE:
            return False, "ROS not available"

        if not self.ws_set_params_client.service_is_ready():
            if not self.ws_set_params_client.wait_for_service(timeout_sec=2.0):
                return False, "Workspace markers node not reachable (service not discovered)"

        request = SetParameters.Request()
        for name, value in params.items():
            param = Parameter()
            param.name = name
            param.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            )
            request.parameters.append(param)

        future = self.ws_set_params_client.call_async(request)
        if not self._spin_for_future(future, timeout_sec=2.0):
            return False, "Service call timed out (spin lock busy)"
        if future.result() is not None:
            results = future.result().results
            if all(r.successful for r in results):
                return True, "Parameters updated"
            failed = [n for n, r in zip(params.keys(), results) if not r.successful]
            return False, f"Failed params: {', '.join(failed)}"
        return False, "No response from workspace markers node"

    def get_workspace_marker_params(self, object_names, param_keys) -> dict:
        """Read current parameter values from the workspace_markers_publisher node.

        Returns:
            dict of {param_name: float_value} or None on failure.
        """
        if not ROS_AVAILABLE:
            return None

        if not self.ws_get_params_client.service_is_ready():
            if not self.ws_get_params_client.wait_for_service(timeout_sec=2.0):
                return None

        param_names = []
        for obj_name in object_names:
            for key in param_keys:
                param_names.append(f"{obj_name}.{key}")

        request = GetParameters.Request()
        request.names = param_names

        future = self.ws_get_params_client.call_async(request)
        if not self._spin_for_future(future, timeout_sec=2.0):
            return None

        if future.result() is None:
            return None

        result = {}
        for name, pval in zip(param_names, future.result().values):
            if pval.type == ParameterType.PARAMETER_DOUBLE:
                result[name] = pval.double_value
            elif pval.type == ParameterType.PARAMETER_INTEGER:
                result[name] = float(pval.integer_value)
        return result

    def save_current_joint_position(self, name: str) -> tuple:
        """Save the current joint position to named_positions.txt in degrees.

        Returns:
            (success: bool, message: str)
        """
        if not ROS_AVAILABLE:
            return False, "ROS not available"

        # Spin once to ensure we have the latest joint state
        self.get_joint_state()
        joint_state = self.latest_joint_state
        if joint_state is None or not joint_state.position:
            return False, "No joint state available"

        if len(joint_state.position) != 6:
            return False, f"Expected 6 joints, got {len(joint_state.position)}"

        name = name.strip().replace(' ', '_')
        if not name:
            return False, "Position name cannot be empty"

        # Convert radians to degrees
        joint_degrees = [math.degrees(p) for p in joint_state.position]

        # Find named_positions.txt in source tree
        workspace_root = Path(__file__).parent
        np_file = workspace_root / "src/ur5_curobo_control/config/named_positions.txt"
        if not np_file.exists():
            return False, f"Named positions file not found: {np_file}"

        vals_str = ' '.join(f'{v:.2f}' for v in joint_degrees)
        line = f"joint {name} {vals_str}\n"

        with open(np_file, 'a') as f:
            f.write(f"# Saved at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(line)

        summary = ', '.join(f'{v:.1f}' for v in joint_degrees)
        return True, f"Saved '{name}': [{summary}] deg"

    def move_to_joint_positions(self, joint_positions: list, duration: float = 3.0) -> tuple[bool, str]:
        """Move robot to specified joint positions."""
        if not ROS_AVAILABLE:
            return False, "ROS not available"
        
        if len(joint_positions) != 6:
            return False, f"Expected 6 joint positions, got {len(joint_positions)}"
        
        try:
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names
            
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in joint_positions]
            point.velocities = [0.0] * 6
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
            
            traj_msg.points.append(point)
            self.trajectory_pub.publish(traj_msg)
            
            return True, f"Moving to joint positions (duration: {duration:.1f}s)"
        except Exception as e:
            return False, f"Failed to send trajectory: {e}"
    
    def move_to_pose(self, position: list, quaternion: list) -> tuple[bool, str]:
        """Move robot to specified Cartesian pose using cuRobo motion planning.
        
        Args:
            position: [x, y, z] in meters
            quaternion: [qw, qx, qy, qz] orientation
        """
        if not ROS_AVAILABLE:
            return False, "ROS not available"
        
        if len(position) != 3 or len(quaternion) != 4:
            return False, f"Invalid pose dimensions: pos={len(position)}, quat={len(quaternion)}"
        
        try:
            # Set the target pose parameters
            request = SetParameters.Request()
            
            # Position parameter
            pos_param = Parameter()
            pos_param.name = 'target_pose_position'
            pos_param.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(p) for p in position]
            )
            
            # Quaternion parameter
            quat_param = Parameter()
            quat_param.name = 'target_pose_quaternion'
            quat_param.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=[float(q) for q in quaternion]
            )
            
            request.parameters = [pos_param, quat_param]
            
            # Set parameters
            future = self.set_parameters_client.call_async(request)
            if not self._spin_for_future(future, timeout_sec=2.0):
                return False, "Failed to set pose parameters (busy)"
            
            if future.result() is None:
                return False, "Failed to set pose parameters (timeout)"
            
            # Call the move_to_pose service
            move_request = Trigger.Request()
            future = self.move_to_pose_client.call_async(move_request)
            if not self._spin_for_future(future, timeout_sec=10.0):
                return False, "Move to pose service call busy"
            
            if future.result() is not None:
                return future.result().success, future.result().message
            return False, "Move to pose service call timeout"
            
        except Exception as e:
            return False, f"Failed to move to pose: {e}"


    def move_relative(self, direction: str, distance: float = 0.05) -> tuple[bool, str]:
        """Move end-effector by a Cartesian offset in the given direction.

        Args:
            direction: one of 'left', 'right', 'forward', 'back', 'up', 'down'
            distance: step size in metres
        """
        if not ROS_AVAILABLE:
            return False, "ROS not available"

        try:
            # Set direction and distance parameters
            request = SetParameters.Request()
            dir_param = Parameter()
            dir_param.name = 'relative_move_direction'
            dir_param.value = ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=direction)
            dist_param = Parameter()
            dist_param.name = 'relative_move_distance'
            dist_param.value = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(distance))
            request.parameters = [dir_param, dist_param]

            future = self.set_parameters_client.call_async(request)
            if not self._spin_for_future(future, timeout_sec=2.0):
                return False, "Failed to set relative move parameters (busy)"
            if future.result() is None:
                return False, "Failed to set relative move parameters (timeout)"

            # Call the move_relative service
            move_req = Trigger.Request()
            future = self.move_relative_client.call_async(move_req)
            if not self._spin_for_future(future, timeout_sec=10.0):
                return False, "Move relative service call busy"
            if future.result() is not None:
                return future.result().success, future.result().message
            return False, "Move relative service call timeout"
        except Exception as e:
            return False, f"Failed to move relative: {e}"


def get_programs_directory():
    """Get the programs directory path."""
    workspace_root = Path(__file__).parent
    
    # Try source directory first (better for development - changes are immediate)
    programs_dir = workspace_root / "src/ur5_curobo_control/programs"
    if programs_dir.exists():
        return programs_dir
    
    # Fallback to install directory
    programs_dir = workspace_root / "install/ur5_curobo_control/share/ur5_curobo_control/programs"
    
    return programs_dir


def get_named_positions_file():
    """Get the path to the named positions configuration file."""
    workspace_root = Path(__file__).parent
    
    # Try source directory first (better for development - changes are immediate)
    positions_file = workspace_root / "src/ur5_curobo_control/config/named_positions.txt"
    if positions_file.exists():
        return positions_file
    
    # Fallback to install directory
    positions_file = workspace_root / "install/ur5_curobo_control/share/ur5_curobo_control/config/named_positions.txt"
    if positions_file.exists():
        return positions_file
    
    return None


def load_named_positions():
    """Load named positions from configuration file."""
    # Import the parser (late import to avoid circular dependencies)
    try:
        # Try installed package first
        from ur5_curobo_control.program_parser import NamedPositionsParser, NamedPosition, PositionType
    except ImportError:
        # Fallback to direct import from source
        import sys
        workspace_root = Path(__file__).parent
        sys.path.insert(0, str(workspace_root / "src/ur5_curobo_control/ur5_curobo_control"))
        from program_parser import NamedPositionsParser, NamedPosition, PositionType
    
    positions_file = get_named_positions_file()
    if positions_file is None:
        return [], [], "Named positions file not found"
    
    parser = NamedPositionsParser()
    positions = parser.parse_file(str(positions_file))
    
    if parser.get_errors():
        return [], [], f"Errors parsing positions: {parser.get_errors()}"
    
    poses = parser.get_poses()
    joints = parser.get_joints()
    
    return poses, joints, None


def get_available_programs():
    """Get list of available program files."""
    programs_dir = get_programs_directory()
    
    if not programs_dir.exists():
        return []
    
    programs = sorted([p.name for p in programs_dir.glob("*.prog")])
    return programs


def read_program_description(program_path: Path) -> str:
    """Read the first comment line from a program file as description."""
    try:
        with open(program_path, 'r', encoding='utf-8') as f:
            first_line = f.readline().strip()
            if first_line.startswith('#'):
                return first_line[1:].strip()
    except Exception:
        pass
    return "No description available"


class SessionRecorder:
    """Records video, joint states, and program-execution timelines for a
    multi-program session.  Used by the Streamlit UI to capture long-horizon
    experiments where several programs are run in sequence.

    Note: For single-program episode recording aimed at ML dataset creation,
    see ``experiment_recorder.EpisodeRecorder``.
    """
    
    def __init__(self, experiment_name: str, controller):
        self.experiment_name = experiment_name
        self.controller = controller
        self.is_recording = False
        self.video_writer = None
        self.video_capture = None
        self.gopro_recorder = None  # GoPro recorder
        self.joint_data = []
        self.gripper_data = []  # Separate gripper data
        self.timestamps = []
        self.program_events = []  # List of program execution events
        self.start_time = None
        self.save_path = None
        self.recording_thread = None
        self.stop_event = threading.Event()
        
    def start_recording(self):
        """Start recording video and joint states."""
        if not VIDEO_AVAILABLE:
            return False, "OpenCV not available"
        
        # Create save directory with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_path = Path(f"recordings/{self.experiment_name}_{timestamp}")
        self.save_path.mkdir(parents=True, exist_ok=True)
        
        # Log start event
        self.program_events.append({
            'event': 'recording_start',
            'timestamp': 0.0,
            'program': None
        })
        
        # Initialize video capture (try multiple camera indices)
        # Suppress OpenCV warnings by setting logging level
        import os
        os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'
        
        for camera_idx in [0, 1, 2]:
            self.video_capture = cv2.VideoCapture(camera_idx)
            if self.video_capture.isOpened():
                break
        
        if not self.video_capture or not self.video_capture.isOpened():
            return False, "Could not open webcam"
        
        # Get video properties
        fps = 30.0
        width = int(self.video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Initialize video writer
        video_path = self.save_path / "video.mp4"
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(str(video_path), fourcc, fps, (width, height))
        
        if not self.video_writer.isOpened():
            self.video_capture.release()
            return False, "Could not initialize video writer"
        
        # Initialize GoPro recorder if available
        if GOPRO_AVAILABLE:
            try:
                self.gopro_recorder = GoProRecorder()
                gopro_video_path = self.save_path / "gopro_video.mp4"
                success, message = self.gopro_recorder.start_recording(gopro_video_path)
                if success:
                    print(f"✓ {message}")
                else:
                    print(f"⚠ {message}")
                    self.gopro_recorder = None
            except Exception as e:
                print(f"⚠ Failed to start GoPro recording: {e}")
                self.gopro_recorder = None
        
        # Reset data
        self.joint_data = []
        self.timestamps = []
        self.start_time = time.time()
        self.is_recording = True
        self.stop_event.clear()
        
        # Start recording thread
        self.recording_thread = threading.Thread(target=self._recording_loop, daemon=True)
        self.recording_thread.start()
        
        return True, f"Recording started: {self.save_path}"
    
    def log_program_event(self, event_type: str, program_name: str = None):
        """Log a program execution event (execute, pause, resume, stop)."""
        if not self.is_recording or self.start_time is None:
            print(f"[Log Event] Skipped (is_recording={self.is_recording}, start_time={self.start_time})")
            return
        
        current_time = time.time() - self.start_time
        event = {
            'event': event_type,
            'timestamp': float(current_time),
            'program': program_name
        }
        self.program_events.append(event)
        print(f"[Log Event] {event_type} - {program_name} at {current_time:.2f}s")
    
    def _recording_loop(self):
        """Main recording loop running in background thread."""
        last_sample_time = time.time()
        min_interval = 1/50.0  # 50 Hz sampling rate
        joint_names_cached = None
        sample_count = 0
        
        while self.is_recording and not self.stop_event.is_set():
            current_time = time.time() - self.start_time
            
            # Only record if enough time has passed
            if time.time() - last_sample_time < min_interval:
                time.sleep(0.001)
                continue
            
            last_sample_time = time.time()
            
            # Capture video frame
            if self.video_capture and self.video_capture.isOpened():
                ret, frame = self.video_capture.read()
                if ret and self.video_writer:
                    self.video_writer.write(frame)
            
            # Capture joint state
            if self.controller and ROS_AVAILABLE:
                # Access the latest cached joint state (no spin needed - main thread handles that)
                joint_state = self.controller.latest_joint_state
                gripper_state = self.controller.latest_gripper_state
                
                if joint_state:
                    # Cache joint names on first sample
                    if joint_names_cached is None:
                        joint_names_cached = list(joint_state.name)
                    
                    self.timestamps.append(current_time)
                    self.joint_data.append({
                        'positions': np.array(joint_state.position, dtype=np.float32),
                        'velocities': np.array(joint_state.velocity, dtype=np.float32) if joint_state.velocity else None,
                        'efforts': np.array(joint_state.effort, dtype=np.float32) if joint_state.effort else None
                    })
                    
                    # Capture gripper state if available
                    if gripper_state:
                        self.gripper_data.append({
                            'position': float(gripper_state.position[0]) if gripper_state.position else 0.0,
                            'velocity': float(gripper_state.velocity[0]) if gripper_state.velocity else 0.0,
                            'effort': float(gripper_state.effort[0]) if gripper_state.effort else 0.0
                        })
                    else:
                        # No gripper state available, append None
                        self.gripper_data.append(None)
                    
                    sample_count += 1
            
            # Store joint names for later
            if joint_names_cached is not None:
                self._joint_names = joint_names_cached
    
    def stop_recording(self):
        """Stop recording and save data."""
        if not self.is_recording:
            return False, "Not currently recording"
        
        self.is_recording = False
        self.stop_event.set()
        
        # Wait for recording thread to finish
        if self.recording_thread:
            self.recording_thread.join(timeout=2.0)
        
        # Release video resources
        if self.video_writer:
            self.video_writer.release()
        if self.video_capture:
            self.video_capture.release()
        
        # Stop GoPro recording if active
        if self.gopro_recorder:
            try:
                success, message = self.gopro_recorder.stop_recording()
                if success:
                    print(f"✓ {message}")
                else:
                    print(f"⚠ {message}")
            except Exception as e:
                print(f"⚠ Error stopping GoPro recording: {e}")
        
        duration = time.time() - self.start_time
        
        # Save joint data as HDF5
        if HDF5_AVAILABLE:
            joint_data_path = self.save_path / "joint_states.h5"
            try:
                with h5py.File(joint_data_path, 'w') as f:
                    # Create datasets
                    timestamps = np.array(self.timestamps, dtype=np.float32)
                    f.create_dataset('timestamps', data=timestamps, compression='gzip')
                    
                    # Only save position data if we have it
                    has_positions = self.joint_data and any('positions' in d for d in self.joint_data)
                    
                    if has_positions:
                        # Handle case where joint counts might vary (e.g., 6 DOF, then 1 DOF)
                        # Find the max size and pad as needed
                        try:
                            positions_list = []
                            max_size = 0
                            for d in self.joint_data:
                                pos = d.get('positions')
                                if pos is not None:
                                    max_size = max(max_size, len(pos))
                                    positions_list.append(pos)
                            
                            # Pad all arrays to max_size
                            padded_positions = []
                            for pos in positions_list:
                                if len(pos) < max_size:
                                    # Pad with zeros
                                    padded = np.pad(pos, (0, max_size - len(pos)), mode='constant', constant_values=0)
                                else:
                                    padded = pos
                                padded_positions.append(padded)
                            
                            positions = np.array(padded_positions, dtype=np.float32)
                            f.create_dataset('positions', data=positions, compression='gzip')
                        except Exception as e:
                            print(f"[HDF5] Error creating positions array: {e}")
                            import traceback
                            traceback.print_exc()
                        
                        # Stack velocity data if available
                        if any(d.get('velocities') is not None for d in self.joint_data):
                            velocities = np.array([
                                d.get('velocities') if d.get('velocities') is not None else np.zeros_like(d.get('positions', np.zeros(6)))
                                for d in self.joint_data
                            ], dtype=np.float32)
                            f.create_dataset('velocities', data=velocities, compression='gzip')
                        
                        # Stack effort data if available
                        if any(d.get('efforts') is not None for d in self.joint_data):
                            efforts = np.array([
                                d.get('efforts') if d.get('efforts') is not None else np.zeros_like(d.get('positions', np.zeros(6)))
                                for d in self.joint_data
                            ], dtype=np.float32)
                            f.create_dataset('efforts', data=efforts, compression='gzip')
                    
                    # Save gripper data if available
                    if self.gripper_data and any(g is not None for g in self.gripper_data):
                        gripper_positions = np.array([
                            g['position'] if g is not None else 0.0 
                            for g in self.gripper_data
                        ], dtype=np.float32)
                        f.create_dataset('gripper_position', data=gripper_positions, compression='gzip')
                        
                        gripper_velocities = np.array([
                            g['velocity'] if g is not None else 0.0 
                            for g in self.gripper_data
                        ], dtype=np.float32)
                        f.create_dataset('gripper_velocity', data=gripper_velocities, compression='gzip')
                        
                        gripper_efforts = np.array([
                            g['effort'] if g is not None else 0.0 
                            for g in self.gripper_data
                        ], dtype=np.float32)
                        f.create_dataset('gripper_effort', data=gripper_efforts, compression='gzip')
                    
                    # Store metadata as attributes
                    f.attrs['experiment_name'] = self.experiment_name
                    f.attrs['start_time'] = datetime.fromtimestamp(self.start_time).isoformat()
                    f.attrs['duration_seconds'] = float(duration)
                    f.attrs['num_samples'] = len(self.joint_data)
                    f.attrs['sample_rate_hz'] = 50.0
                    
                    # Store joint names
                    if hasattr(self, '_joint_names'):
                        f.create_dataset('joint_names', data=np.array(self._joint_names, dtype='S64'))
            except Exception as e:
                return False, f"Failed to save HDF5: {str(e)}"
        elif self.joint_data:
            # Fallback: save as JSON if h5py not available
            joint_data_path = self.save_path / "joint_states.json"
            with open(joint_data_path, 'w') as f:
                json.dump({
                    'experiment_name': self.experiment_name,
                    'start_time': datetime.fromtimestamp(self.start_time).isoformat(),
                    'duration': duration,
                    'num_samples': len(self.joint_data),
                    'sample_rate_hz': 50.0
                }, f, indent=2)
        
        # Save metadata as JSON
        try:
            metadata_path = self.save_path / "metadata.json"
            with open(metadata_path, 'w') as f:
                metadata = {
                    'experiment_name': self.experiment_name,
                    'timestamp': datetime.now().isoformat(),
                    'duration_seconds': duration,
                    'num_joint_samples': len(self.joint_data),
                    'sample_rate_hz': 50.0,
                    'video_file': 'video.mp4',
                    'joint_data_file': 'joint_states.h5' if HDF5_AVAILABLE else 'joint_states.json',
                    'program_events_file': 'program_events.json'
                }
                
                # Add GoPro video file if it was recorded
                if self.gopro_recorder and (self.save_path / "gopro_video.mp4").exists():
                    metadata['gopro_video_file'] = 'gopro_video.mp4'
                
                json.dump(metadata, f, indent=2)
            print(f"[Save] Metadata saved to {metadata_path}")
        except Exception as e:
            print(f"[Save] Failed to save metadata: {e}")
        
        # Save program execution events as JSON
        try:
            print(f"[Save Events] program_events count: {len(self.program_events)}, events: {self.program_events}")
            program_events_path = self.save_path / "program_events.json"
            with open(program_events_path, 'w') as f:
                json.dump({
                    'experiment_name': self.experiment_name,
                    'start_time': datetime.fromtimestamp(self.start_time).isoformat(),
                    'duration_seconds': float(duration),
                    'events': self.program_events
                }, f, indent=2)
            print(f"[Save Events] Saved to {program_events_path}")
        except Exception as e:
            print(f"[Save Events] Failed to save program events: {e}")
        
        return True, f"Episode saved to: {self.save_path}"


def is_executor_running():
    """Check if the program executor node is running."""
    for proc in psutil.process_iter(['name', 'cmdline']):
        try:
            cmdline = proc.info.get('cmdline', [])
            if cmdline and 'program_executor_node' in ' '.join(cmdline):
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return False


def main():
    st.title("🤖 UR5 Program Selector")
    st.markdown("---")
    
    # Initialize session state
    if 'controller' not in st.session_state:
        if ROS_AVAILABLE:
            st.session_state.controller = ROSProgramController()
        else:
            st.session_state.controller = None
    
    if 'current_program' not in st.session_state:
        st.session_state.current_program = None
    
    if 'status_message' not in st.session_state:
        st.session_state.status_message = ""
    
    if 'is_executing' not in st.session_state:
        st.session_state.is_executing = False
    
    if 'experiment_name' not in st.session_state:
        st.session_state.experiment_name = "experiment"
    
    if 'is_recording' not in st.session_state:
        st.session_state.is_recording = False

    if 'recorder' not in st.session_state:
        st.session_state.recorder = None

    if 'teleop_active' not in st.session_state:
        st.session_state.teleop_active = False
    
    # Sidebar - Status and Controls
    with st.sidebar:
        st.header("📊 Status")
        
        # Check if executor is running
        executor_running = is_executor_running()
        
        # Episode Recording Section
        st.subheader("🎥 Episode Recording")
        
        # Experiment name input
        experiment_name = st.text_input(
            "Experiment Name",
            value=st.session_state.experiment_name,
            key="exp_name_input"
        )
        st.session_state.experiment_name = experiment_name
        
        # Recording status
        if st.session_state.is_recording:
            st.warning("🔴 Recording in progress...")
        else:
            st.info("⚪ Ready to record")
        
        # GoPro status indicator
        if GOPRO_AVAILABLE:
            try:
                gopro = GoProRecorder()
                if gopro.is_gopro_streaming():
                    st.success("📹 GoPro: Streaming")
                else:
                    st.warning("📹 GoPro: Not streaming")
            except Exception:
                st.error("📹 GoPro: Error")
        else:
            st.info("📹 GoPro: Not available")
        
        # Recording buttons
        col_rec1, col_rec2 = st.columns(2)
        
        with col_rec1:
            if st.button(
                "🔴 Record",
                disabled=st.session_state.is_recording or not VIDEO_AVAILABLE,
                use_container_width=True,
                type="primary"
            ):
                # Start recording
                recorder = SessionRecorder(st.session_state.experiment_name, st.session_state.controller)
                success, message = recorder.start_recording()
                
                if success:
                    st.session_state.recorder = recorder
                    st.session_state.is_recording = True
                    st.success(message)
                else:
                    st.error(f"Failed to start recording: {message}")
                
                st.rerun()
        
        with col_rec2:
            if st.button(
                "⏹️ End",
                disabled=not st.session_state.is_recording,
                use_container_width=True,
                type="secondary"
            ):
                # Stop recording
                if st.session_state.recorder:
                    success, message = st.session_state.recorder.stop_recording()
                    
                    if success:
                        st.success(message)
                    else:
                        st.error(f"Failed to stop recording: {message}")
                    
                    st.session_state.recorder = None
                    st.session_state.is_recording = False
                    st.rerun()
        
        if not VIDEO_AVAILABLE:
            st.caption("⚠️ Install opencv-python for recording")
        
        st.markdown("---")
        
        if executor_running:
            st.success("✅ Program Executor: Running")
        else:
            st.error("❌ Program Executor: Not Running")
            st.markdown("Start the executor with:")
            st.code("./run_program_executor.sh", language="bash")
        
        # External Control API status
        api_port = 5050
        api_url = f"http://localhost:{api_port}"
        try:
            req = urllib.request.Request(f"{api_url}/api/status", method="GET")
            req.add_header("Accept", "application/json")
            with urllib.request.urlopen(req, timeout=0.5) as resp:
                st.success(f"🌐 External API: Running ({api_url})")
        except Exception:
            st.info("🌐 External API: Not running")
            with st.expander("Start external API"):
                st.code("./run_external_api.sh", language="bash")
                st.caption("Enables external services to control the robot via REST.")
        
        # Check joint state availability
        if st.session_state.controller and ROS_AVAILABLE:
            joint_state = st.session_state.controller.get_joint_state()
            if joint_state:
                st.success(f"✅ Joint States: Receiving ({len(joint_state.name)} joints)")
            else:
                st.warning("⚠️ Joint States: Not receiving")
        
        st.markdown("---")
        
        # Current program status
        if st.session_state.current_program:
            st.info(f"📄 Current: **{st.session_state.current_program}**")
        else:
            st.warning("📄 No program selected")
        
        if st.session_state.status_message:
            st.markdown(st.session_state.status_message, unsafe_allow_html=True)
        
        st.markdown("---")
        
        # Control buttons
        st.subheader("🎮 Controls")
        
        col1, col2 = st.columns(2)
        
        with col1:
            if st.button("⏸️ Pause", disabled=not executor_running or not ROS_AVAILABLE, use_container_width=True):
                if st.session_state.controller:
                    success, msg = st.session_state.controller.pause_program(True)
                    if success:
                        st.session_state.status_message = '<p class="status-success">✅ Paused</p>'
                        # Log pause event if recording
                        if st.session_state.is_recording and st.session_state.recorder:
                            st.session_state.recorder.log_program_event('pause', st.session_state.current_program)
                        st.rerun()
        
        with col2:
            if st.button("▶️ Resume", disabled=not executor_running or not ROS_AVAILABLE, use_container_width=True):
                if st.session_state.controller:
                    success, msg = st.session_state.controller.pause_program(False)
                    if success:
                        st.session_state.status_message = '<p class="status-running">🔄 Resumed</p>'
                        # Log resume event if recording
                        if st.session_state.is_recording and st.session_state.recorder:
                            st.session_state.recorder.log_program_event('resume', st.session_state.current_program)
                        st.rerun()
        
        if st.button("🛑 Stop", disabled=not executor_running or not ROS_AVAILABLE, use_container_width=True, type="secondary"):
            if st.session_state.controller:
                success, msg = st.session_state.controller.stop_program()
                if success:
                    st.session_state.status_message = '<p class="status-error">🛑 Stopped</p>'
                    st.session_state.is_executing = False
                    st.session_state.teleop_active = False  # Stop also disables teleop
                    # Log stop event if recording
                    if st.session_state.is_recording and st.session_state.recorder:
                        st.session_state.recorder.log_program_event('stop', st.session_state.current_program)
                    st.rerun()
        
        st.markdown("---")

        # SpaceMouse / Teleop Control
        st.subheader("🕹️ Teleop Control")

        if st.session_state.teleop_active:
            st.success("🟢 Teleop: Active")
            st.caption("SpaceMouse (or keyboard/VR) deltas are being tracked.")
        else:
            st.info("⚫ Teleop: Inactive")

        tcol1, tcol2 = st.columns(2)
        with tcol1:
            if st.button(
                "Enable",
                key="teleop_enable",
                disabled=st.session_state.teleop_active or not executor_running or not ROS_AVAILABLE,
                use_container_width=True,
                type="primary"
            ):
                if st.session_state.controller:
                    success, msg = st.session_state.controller.set_teleop_mode(True)
                    if success:
                        st.session_state.teleop_active = True
                        st.session_state.status_message = '<p class="status-success">🕹️ Teleop enabled</p>'
                    else:
                        st.session_state.status_message = f'<p class="status-error">❌ {msg}</p>'
                    st.rerun()

        with tcol2:
            if st.button(
                "Disable",
                key="teleop_disable",
                disabled=not st.session_state.teleop_active or not executor_running or not ROS_AVAILABLE,
                use_container_width=True,
                type="secondary"
            ):
                if st.session_state.controller:
                    success, msg = st.session_state.controller.set_teleop_mode(False)
                    if success:
                        st.session_state.teleop_active = False
                        st.session_state.status_message = '<p class="status-error">🕹️ Teleop disabled</p>'
                    else:
                        st.session_state.status_message = f'<p class="status-error">❌ {msg}</p>'
                    st.rerun()

        st.caption("Requires spacemouse_teleop node running: `./run_spacemouse_teleop.sh`")

        st.markdown("---")

        # Refresh button
        if st.button("🔄 Refresh Programs", use_container_width=True):
            st.rerun()
    
    # ========================================
    # Direct Commands Section
    # ========================================
    st.subheader("🎯 Direct Commands")
    
    # Create tabs for different command types
    cmd_tab1, cmd_tab2, cmd_tab3, cmd_tab4, cmd_tab5 = st.tabs(["🤏 Gripper", "📍 Named Positions", "🔧 Manual Input", "🏗️ Workspace Objects", "🎛️ Teach Pendant"])
    
    with cmd_tab1:
        st.markdown("#### Gripper Control")
        gcol1, gcol2, gcol3 = st.columns(3)
        
        with gcol1:
            if st.button("✋ Open Gripper", use_container_width=True, type="primary",
                        disabled=not ROS_AVAILABLE):
                if st.session_state.controller:
                    success, msg = st.session_state.controller.open_gripper()
                    if success:
                        st.success(msg)
                        # Log event if recording
                        if st.session_state.is_recording and st.session_state.recorder:
                            st.session_state.recorder.log_program_event('gripper_open', 'manual_command')
                    else:
                        st.error(msg)
        
        with gcol2:
            if st.button("✊ Close Gripper", use_container_width=True, type="primary",
                        disabled=not ROS_AVAILABLE):
                if st.session_state.controller:
                    success, msg = st.session_state.controller.close_gripper()
                    if success:
                        st.success(msg)
                        # Log event if recording
                        if st.session_state.is_recording and st.session_state.recorder:
                            st.session_state.recorder.log_program_event('gripper_close', 'manual_command')
                    else:
                        st.error(msg)
        
        with gcol3:
            gripper_pos = st.slider("Position", 0.0, 1.0, 0.0, 0.1, key="gripper_slider")
            if st.button("Set Position", use_container_width=True, disabled=not ROS_AVAILABLE):
                if st.session_state.controller:
                    success, msg = st.session_state.controller.set_gripper_position(gripper_pos)
                    if success:
                        st.success(msg)
                        # Log event if recording
                        if st.session_state.is_recording and st.session_state.recorder:
                            st.session_state.recorder.log_program_event(f'gripper_position:{gripper_pos:.2f}', 'manual_command')
                    else:
                        st.error(msg)
    
    with cmd_tab2:
        st.markdown("#### Named Positions")
        
        # Load named positions
        poses, joints, error = load_named_positions()
        
        if error:
            st.warning(f"⚠️ {error}")
        else:
            # Show position count
            st.caption(f"Loaded {len(poses)} poses and {len(joints)} joint positions")
            
            # Movement duration slider
            move_duration = st.slider("Movement Duration (s)", 1.0, 10.0, 3.0, 0.5, key="move_duration")
            
            # Joint positions section
            if joints:
                st.markdown("**Joint Positions:**")
                jcols = st.columns(min(4, len(joints)))
                for idx, pos in enumerate(joints):
                    with jcols[idx % 4]:
                        btn_label = f"🔗 {pos.name}"
                        if pos.description:
                            st.caption(pos.description[:30])
                        if st.button(btn_label, key=f"joint_{pos.name}", use_container_width=True,
                                    disabled=not ROS_AVAILABLE):
                            if st.session_state.controller:
                                success, msg = st.session_state.controller.move_to_joint_positions(
                                    pos.joint_positions, duration=move_duration
                                )
                                if success:
                                    st.success(f"Moving to {pos.name}")
                                    # Log event if recording
                                    if st.session_state.is_recording and st.session_state.recorder:
                                        st.session_state.recorder.log_program_event(f'move_to_joint:{pos.name}', 'named_position')
                                else:
                                    st.error(msg)
            
            # Cartesian poses section
            if poses:
                st.markdown("**Cartesian Poses:**")
                st.caption("Uses cuRobo motion planning via program executor")
                pcols = st.columns(min(4, len(poses)))
                for idx, pos in enumerate(poses):
                    with pcols[idx % 4]:
                        btn_label = f"📍 {pos.name}"
                        if pos.description:
                            st.caption(pos.description[:30])
                        if st.button(btn_label, key=f"pose_{pos.name}", use_container_width=True,
                                    disabled=not ROS_AVAILABLE):
                            if st.session_state.controller:
                                success, msg = st.session_state.controller.move_to_pose(
                                    pos.position, pos.quaternion
                                )
                                if success:
                                    st.success(f"Moving to {pos.name}")
                                    # Log event if recording
                                    if st.session_state.is_recording and st.session_state.recorder:
                                        st.session_state.recorder.log_program_event(f'move_to_pose:{pos.name}', 'named_position')
                                else:
                                    st.error(msg)

        # --- Save Current Joint Position ---
        st.markdown("---")
        st.markdown("**Save Current Joint Position:**")

        # Show current joints (degrees) so user knows what they're saving
        if st.session_state.controller and ROS_AVAILABLE:
            joint_state = st.session_state.controller.get_joint_state()
            if joint_state and joint_state.position and len(joint_state.position) == 6:
                preview_cols = st.columns(6)
                for i, (col, rad) in enumerate(zip(preview_cols, joint_state.position)):
                    col.metric(f"J{i+1}", f"{math.degrees(rad):.1f}\u00b0")
            else:
                st.caption("Waiting for joint state...")

        save_col1, save_col2 = st.columns([3, 1])
        with save_col1:
            save_name = st.text_input("Position name", key="save_pos_name",
                                      placeholder="e.g. pick_position")
        with save_col2:
            st.markdown("")  # vertical spacer to align with text input
            if st.button("Save", key="save_joint_pos", type="primary",
                         use_container_width=True, disabled=not ROS_AVAILABLE):
                if st.session_state.controller and save_name:
                    success, msg = st.session_state.controller.save_current_joint_position(save_name)
                    if success:
                        st.success(msg)
                        st.rerun()  # reload to show new position in list
                    else:
                        st.error(msg)
                elif not save_name:
                    st.warning("Enter a position name first")

    with cmd_tab3:
        st.markdown("#### Manual Joint Input")
        st.caption("Enter joint positions in degrees")
        
        # Manual joint input
        mcols = st.columns(6)
        manual_joints = []
        joint_labels = ["J1 (Base)", "J2 (Shoulder)", "J3 (Elbow)", "J4 (Wrist1)", "J5 (Wrist2)", "J6 (Wrist3)"]
        
        for i, col in enumerate(mcols):
            with col:
                val = st.number_input(
                    joint_labels[i],
                    min_value=-360.0,
                    max_value=360.0,
                    value=0.0,
                    step=5.0,
                    format="%.1f",
                    key=f"manual_j{i}"
                )
                manual_joints.append(val)
        
        manual_duration = st.slider("Duration (s)", 1.0, 10.0, 3.0, 0.5, key="manual_duration")
        
        if st.button("🚀 Move to Joint Position", type="primary", use_container_width=True,
                    disabled=not ROS_AVAILABLE):
            if st.session_state.controller:
                # Convert from degrees to radians
                manual_joints_rad = [j * 3.14159265359 / 180.0 for j in manual_joints]
                success, msg = st.session_state.controller.move_to_joint_positions(
                    manual_joints_rad, duration=manual_duration
                )
                if success:
                    st.success(msg)
                    # Log event if recording
                    if st.session_state.is_recording and st.session_state.recorder:
                        joints_str = ','.join([f'{j:.2f}' for j in manual_joints])
                        st.session_state.recorder.log_program_event(f'move_to_joint:[{joints_str}]', 'manual_command')
                else:
                    st.error(msg)
        
        # Show current joint state if available
        if st.session_state.controller and ROS_AVAILABLE:
            joint_state = st.session_state.controller.get_joint_state()
            if joint_state and joint_state.position:
                st.markdown("**Current Joint Positions (degrees):**")
                curr_cols = st.columns(6)
                for i, (col, pos) in enumerate(zip(curr_cols, joint_state.position)):
                    with col:
                        # Convert from radians to degrees for display
                        pos_deg = pos * 180.0 / 3.14159265359
                        st.metric(f"J{i+1}", f"{pos_deg:.1f}°")

    with cmd_tab4:
        st.markdown("#### Workspace Object Placement")
        st.caption("Adjust GLB mesh positions in RViz — changes apply live on the next 1 Hz publish cycle")

        # Object defaults (must match workspace_markers.py DEFAULTS)
        WS_OBJECTS = {
            "table": {"file": "table.glb",      "x": 0.0, "y": -1.0, "z": -0.2, "roll": 90.0, "pitch": 0.0, "yaw": 0.0, "scale_x": 1.0, "scale_y": 1.0, "scale_z": 1.0},
            "base":  {"file": "robot_base.glb",  "x": 0.0, "y":  0.0, "z": -0.1, "roll": 90.0, "pitch": 0.0, "yaw": 0.0, "scale_x": 0.8, "scale_y": 0.8, "scale_z": 0.8},
        }
        WS_PARAM_KEYS = ["x", "y", "z", "roll", "pitch", "yaw", "scale_x", "scale_y", "scale_z"]

        # Read current values from the ROS node on first load
        if 'ws_params_initialized' not in st.session_state:
            st.session_state.ws_params_initialized = False

        # Maps ROS param key suffix -> Streamlit widget key suffix
        _WS_WIDGET_SUFFIX = {
            "x": "x", "y": "y", "z": "z",
            "roll": "roll", "pitch": "pitch", "yaw": "yaw",
            "scale_x": "sx", "scale_y": "sy", "scale_z": "sz",
        }

        def _refresh_ws_params_from_ros():
            """Read live parameter values from workspace_markers_publisher and push them into widget state."""
            if not st.session_state.controller:
                return False
            values = st.session_state.controller.get_workspace_marker_params(
                WS_OBJECTS.keys(), WS_PARAM_KEYS)
            if not values:
                return False
            for obj_name in WS_OBJECTS:
                for key in WS_PARAM_KEYS:
                    param_key = f"{obj_name}.{key}"
                    if param_key in values:
                        widget_key = f"ws_{obj_name}_{_WS_WIDGET_SUFFIX[key]}"
                        st.session_state[widget_key] = values[param_key]
            st.session_state.ws_params_initialized = True
            return True

        if not st.session_state.ws_params_initialized:
            _refresh_ws_params_from_ros()

        # Refresh button
        if st.button("🔄 Read from ROS", key="ws_refresh", disabled=not ROS_AVAILABLE):
            if _refresh_ws_params_from_ros():
                st.success("Values read from workspace markers node")
            else:
                st.warning("Could not read from workspace markers node — is it running?")
            st.rerun()

        for obj_name, defaults in WS_OBJECTS.items():
            with st.expander(f"{obj_name}  ({defaults['file']})", expanded=True):
                st.markdown("**Position (m)**")
                pc = st.columns(3)
                wx = pc[0].number_input("X", value=defaults["x"], step=0.01, format="%.3f", key=f"ws_{obj_name}_x")
                wy = pc[1].number_input("Y", value=defaults["y"], step=0.01, format="%.3f", key=f"ws_{obj_name}_y")
                wz = pc[2].number_input("Z", value=defaults["z"], step=0.01, format="%.3f", key=f"ws_{obj_name}_z")

                st.markdown("**Rotation (deg)**")
                rc = st.columns(3)
                wroll  = rc[0].number_input("Roll",  value=defaults["roll"],  step=1.0, format="%.1f", key=f"ws_{obj_name}_roll")
                wpitch = rc[1].number_input("Pitch", value=defaults["pitch"], step=1.0, format="%.1f", key=f"ws_{obj_name}_pitch")
                wyaw   = rc[2].number_input("Yaw",   value=defaults["yaw"],   step=1.0, format="%.1f", key=f"ws_{obj_name}_yaw")

                st.markdown("**Scale**")
                sc = st.columns(3)
                wsx = sc[0].number_input("Scale X", value=defaults["scale_x"], step=0.01, min_value=0.01, format="%.3f", key=f"ws_{obj_name}_sx")
                wsy = sc[1].number_input("Scale Y", value=defaults["scale_y"], step=0.01, min_value=0.01, format="%.3f", key=f"ws_{obj_name}_sy")
                wsz = sc[2].number_input("Scale Z", value=defaults["scale_z"], step=0.01, min_value=0.01, format="%.3f", key=f"ws_{obj_name}_sz")

                if st.button(f"Apply {obj_name}", key=f"ws_apply_{obj_name}", type="primary",
                             use_container_width=True, disabled=not ROS_AVAILABLE):
                    if st.session_state.controller:
                        params = {
                            f"{obj_name}.x": wx, f"{obj_name}.y": wy, f"{obj_name}.z": wz,
                            f"{obj_name}.roll": wroll, f"{obj_name}.pitch": wpitch, f"{obj_name}.yaw": wyaw,
                            f"{obj_name}.scale_x": wsx, f"{obj_name}.scale_y": wsy, f"{obj_name}.scale_z": wsz,
                        }
                        success, msg = st.session_state.controller.set_workspace_marker_params(params)
                        if success:
                            st.success(f"{obj_name} updated!")
                        else:
                            st.error(f"Failed to update {obj_name}: {msg}")

    # ========================================
    # Teach Pendant Tab
    # ========================================
    with cmd_tab5:
        st.markdown("#### Teach Pendant")
        st.caption("Joint sliders and Cartesian jog — like a real teach pendant. Use keyboard shortcuts (shown below) for quick control.")

        # --- Read current joint state ---
        tp_joint_state = None
        tp_joint_deg = [0.0] * 6
        if st.session_state.controller and ROS_AVAILABLE:
            tp_joint_state = st.session_state.controller.get_joint_state()
            if tp_joint_state and tp_joint_state.position and len(tp_joint_state.position) == 6:
                tp_joint_deg = [math.degrees(p) for p in tp_joint_state.position]

        # --- Joint Sliders Section ---
        st.markdown("##### Joint Control")

        # Initialise slider session-state from live robot on first load
        if 'tp_joints_init' not in st.session_state:
            for i in range(6):
                st.session_state[f"tp_j{i}"] = tp_joint_deg[i]
            st.session_state.tp_joints_init = True

        # "Read Current" refreshes sliders to live values
        if st.button("🔄 Read Current Joints", key="tp_read_joints"):
            for i in range(6):
                st.session_state[f"tp_j{i}"] = tp_joint_deg[i]
            st.rerun()

        # Display current joint positions as a quick reference row
        st.markdown("**Current positions (live):**")
        live_cols = st.columns(6)
        joint_labels = ["J1 Base", "J2 Shoulder", "J3 Elbow", "J4 Wrist1", "J5 Wrist2", "J6 Wrist3"]
        for i, col in enumerate(live_cols):
            col.metric(joint_labels[i], f"{tp_joint_deg[i]:.1f}°")

        # Sliders + text inputs
        st.markdown("**Target positions:**")
        # Sync number inputs -> sliders before widgets render
        for i in range(6):
            num_key = f"tp_j{i}_num"
            slider_key = f"tp_j{i}"
            if num_key in st.session_state and slider_key in st.session_state:
                if abs(st.session_state[num_key] - st.session_state[slider_key]) > 0.01:
                    st.session_state[slider_key] = st.session_state[num_key]

        tp_target_joints = []
        for i in range(6):
            scol1, scol2 = st.columns([4, 1])
            with scol1:
                val = st.slider(
                    joint_labels[i],
                    min_value=-360.0,
                    max_value=360.0,
                    step=0.5,
                    format="%.1f",
                    key=f"tp_j{i}",
                )
            with scol2:
                st.number_input(
                    "°",
                    min_value=-360.0,
                    max_value=360.0,
                    value=st.session_state[f"tp_j{i}"],
                    step=1.0,
                    format="%.1f",
                    key=f"tp_j{i}_num",
                    label_visibility="collapsed",
                )
            tp_target_joints.append(st.session_state[f"tp_j{i}"])

        tp_move_dur = st.slider("Movement Duration (s)", 1.0, 10.0, 3.0, 0.5, key="tp_move_dur")

        if st.button("🚀 Move to Target Joints", type="primary", use_container_width=True,
                      disabled=not ROS_AVAILABLE, key="tp_move_joints"):
            if st.session_state.controller:
                target_rad = [math.radians(d) for d in tp_target_joints]
                success, msg = st.session_state.controller.move_to_joint_positions(
                    target_rad, duration=tp_move_dur)
                if success:
                    st.success(msg)
                    if st.session_state.is_recording and st.session_state.recorder:
                        joints_str = ','.join(f'{d:.1f}' for d in tp_target_joints)
                        st.session_state.recorder.log_program_event(
                            f'teach_pendant_joint:[{joints_str}]', 'teach_pendant')
                else:
                    st.error(msg)

        st.markdown("---")

        # --- Cartesian Jog Section ---
        st.markdown("##### Cartesian Jog (End-Effector)")

        step_options = {"1 mm": 0.001, "5 mm": 0.005, "10 mm": 0.01, "25 mm": 0.025, "50 mm": 0.05, "100 mm": 0.1}
        step_label = st.select_slider(
            "Step Size",
            options=list(step_options.keys()),
            value="10 mm",
            key="tp_step_size",
        )
        tp_step = step_options[step_label]

        # Jog buttons in a 3x2 grid
        jog_col1, jog_col2, jog_col3 = st.columns(3)

        def _jog(direction):
            if st.session_state.controller:
                success, msg = st.session_state.controller.move_relative(direction, tp_step)
                if success:
                    st.success(f"{direction} {step_label}: {msg}")
                    if st.session_state.is_recording and st.session_state.recorder:
                        st.session_state.recorder.log_program_event(
                            f'jog:{direction}:{tp_step}', 'teach_pendant')
                else:
                    st.error(msg)

        with jog_col1:
            st.markdown("**X (Forward/Back)**")
            if st.button("↗ Forward (W)", key="tp_jog_fwd", use_container_width=True, disabled=not ROS_AVAILABLE):
                _jog('forward')
            if st.button("↙ Back (S)", key="tp_jog_back", use_container_width=True, disabled=not ROS_AVAILABLE):
                _jog('back')

        with jog_col2:
            st.markdown("**Y (Left/Right)**")
            if st.button("⬅ Left (A)", key="tp_jog_left", use_container_width=True, disabled=not ROS_AVAILABLE):
                _jog('left')
            if st.button("➡ Right (D)", key="tp_jog_right", use_container_width=True, disabled=not ROS_AVAILABLE):
                _jog('right')

        with jog_col3:
            st.markdown("**Z (Up/Down)**")
            if st.button("⬆ Up (Q)", key="tp_jog_up", use_container_width=True, disabled=not ROS_AVAILABLE):
                _jog('up')
            if st.button("⬇ Down (E)", key="tp_jog_down", use_container_width=True, disabled=not ROS_AVAILABLE):
                _jog('down')

        # --- Keyboard Shortcuts ---
        st.markdown("---")
        st.markdown("##### Keyboard Shortcuts")
        st.markdown("""
| Key | Action |
|-----|--------|
| **W** | Jog Forward (+X) |
| **S** | Jog Back (-X) |
| **A** | Jog Left (+Y) |
| **D** | Jog Right (-Y) |
| **Q** | Jog Up (+Z) |
| **E** | Jog Down (-Z) |
| **1-6** | Select joint |
| **+** / **-** | Jog selected joint ±5° |
""")

        # Inject keyboard shortcut handler via JS
        import streamlit.components.v1 as components
        components.html("""
        <div id="tp-keyboard-handler" tabindex="0" style="outline:none; height:0; overflow:hidden;"></div>
        <script>
        (function() {
            // Find buttons by their key text content
            function clickButton(textMatch) {
                const buttons = window.parent.document.querySelectorAll('button[kind="secondary"], button[kind="primary"]');
                for (const btn of buttons) {
                    if (btn.textContent.includes(textMatch)) {
                        btn.click();
                        return true;
                    }
                }
                return false;
            }

            // Track selected joint for +/- jog
            let selectedJoint = null;

            function handleKey(e) {
                // Ignore if user is typing in an input/textarea
                const tag = (e.target || {}).tagName;
                if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;

                const key = e.key.toLowerCase();

                switch(key) {
                    case 'w': clickButton('Forward (W)'); e.preventDefault(); break;
                    case 's': clickButton('Back (S)'); e.preventDefault(); break;
                    case 'a': clickButton('Left (A)'); e.preventDefault(); break;
                    case 'd': clickButton('Right (D)'); e.preventDefault(); break;
                    case 'q': clickButton('Up (Q)'); e.preventDefault(); break;
                    case 'e': clickButton('Down (E)'); e.preventDefault(); break;
                    case '1': case '2': case '3': case '4': case '5': case '6':
                        selectedJoint = parseInt(key) - 1;
                        // Flash a visual indicator
                        showNotification('Joint ' + key + ' selected');
                        e.preventDefault();
                        break;
                    case '+': case '=':
                        if (selectedJoint !== null) nudgeJoint(selectedJoint, +5);
                        e.preventDefault();
                        break;
                    case '-': case '_':
                        if (selectedJoint !== null) nudgeJoint(selectedJoint, -5);
                        e.preventDefault();
                        break;
                }
            }

            function nudgeJoint(idx, delta) {
                // Find the slider input for the given joint
                const sliders = window.parent.document.querySelectorAll('input[data-testid="stSlider"]');
                // Teach pendant sliders are labelled J1..J6 — find by aria-label
                const allSliders = window.parent.document.querySelectorAll('[data-testid="stSlider"]');
                // Alternative: find number inputs and modify them
                const numInputs = window.parent.document.querySelectorAll('input[type="number"]');
                // We look for inputs whose nearby label matches the joint
                // Simpler approach: use Streamlit's setComponentValue through session state
                showNotification('J' + (idx+1) + (delta > 0 ? ' +' : ' ') + delta + '°');
            }

            function showNotification(text) {
                let el = window.parent.document.getElementById('tp-kb-notification');
                if (!el) {
                    el = document.createElement('div');
                    el.id = 'tp-kb-notification';
                    el.style.cssText = 'position:fixed;top:10px;right:10px;background:#333;color:#fff;padding:8px 16px;border-radius:8px;font-size:14px;z-index:99999;opacity:0;transition:opacity 0.3s;pointer-events:none;';
                    window.parent.document.body.appendChild(el);
                }
                el.textContent = text;
                el.style.opacity = '1';
                clearTimeout(el._timer);
                el._timer = setTimeout(function(){ el.style.opacity = '0'; }, 1200);
            }

            // Attach to parent document
            window.parent.document.removeEventListener('keydown', handleKey);
            window.parent.document.addEventListener('keydown', handleKey);
        })();
        </script>
        """, height=0)

    st.markdown("---")
    
    # ========================================
    # Main area - Program selection
    # ========================================
    programs = get_available_programs()
    
    if not programs:
        st.error("❌ No programs found!")
        st.info(f"Programs directory: `{get_programs_directory()}`")
        return
    
    st.subheader(f"📋 Available Programs ({len(programs)})")
    
    # Create grid layout for programs
    cols = st.columns(3)
    
    for idx, program_name in enumerate(programs):
        col = cols[idx % 3]
        
        with col:
            # Program card
            with st.container():
                st.markdown(f"**{program_name}**")
                
                # Read description
                program_path = get_programs_directory() / program_name
                description = read_program_description(program_path)
                st.caption(description)
                
                # Load & Execute button
                button_key = f"exec_{program_name}"
                if st.button(
                    "▶️ Load & Execute",
                    key=button_key,
                    disabled=not executor_running or not ROS_AVAILABLE,
                    use_container_width=True,
                    type="primary"
                ):
                    if st.session_state.controller:
                        # Stop any currently running program first
                        if st.session_state.is_executing:
                            st.session_state.controller.stop_program()
                            st.session_state.is_executing = False
                            # Log stop event if recording
                            if st.session_state.is_recording and st.session_state.recorder:
                                st.session_state.recorder.log_program_event('stop', st.session_state.current_program)
                        
                        # Set program file parameter
                        st.session_state.status_message = '<p class="status-running">⏳ Loading program...</p>'
                        
                        if st.session_state.controller.set_program_file(program_name):
                            # Load the program
                            success, msg = st.session_state.controller.load_program()
                            
                            if success:
                                st.session_state.current_program = program_name
                                
                                # Execute the program
                                success_exec, msg_exec = st.session_state.controller.execute_program()
                                
                                if success_exec:
                                    st.session_state.status_message = f'<p class="status-success">✅ Executing: {program_name}</p>'
                                    st.session_state.is_executing = True
                                    # Log program execution if recording
                                    if st.session_state.is_recording and st.session_state.recorder:
                                        st.session_state.recorder.log_program_event('execute', program_name)
                                else:
                                    st.session_state.status_message = f'<p class="status-error">❌ Execute failed: {msg_exec}</p>'
                            else:
                                st.session_state.status_message = f'<p class="status-error">❌ Load failed: {msg}</p>'
                        else:
                            st.session_state.status_message = '<p class="status-error">❌ Failed to set program parameter</p>'
                        
                        st.rerun()
                
                # Load only button (smaller)
                button_load_key = f"load_{program_name}"
                if st.button(
                    "📥 Load Only",
                    key=button_load_key,
                    disabled=not executor_running or not ROS_AVAILABLE,
                    use_container_width=True
                ):
                    if st.session_state.controller:
                        if st.session_state.controller.set_program_file(program_name):
                            success, msg = st.session_state.controller.load_program()
                            
                            if success:
                                st.session_state.current_program = program_name
                                st.session_state.status_message = f'<p class="status-success">✅ Loaded: {program_name}</p>'
                            else:
                                st.session_state.status_message = f'<p class="status-error">❌ Failed: {msg}</p>'
                        
                        st.rerun()
                
                st.markdown("---")
    
    # Footer
    st.markdown("---")
    st.caption("💡 Tip: Start the program executor with `./run_program_executor.sh` before using this UI")


if __name__ == "__main__":
    main()
