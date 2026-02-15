#!/home/rml/miniconda3/envs/ur5_python/bin/python
"""
UR5 Program Executor Node

A ROS 2 node that executes robot programs from files. Programs contain
instructions like movetopose, wait, opengripper, etc.

Features:
- Load and execute programs from files
- ROS 2 service interface for AI agents to call
- Safety monitoring integration
- Gripper control via Robotiq action server
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, String, Float64
from std_srvs.srv import Trigger, SetBool
from rcl_interfaces.msg import ParameterDescriptor

import torch
import numpy as np
import time
import os
import sys
import threading
import select
import selectors
import termios
import tty
from datetime import datetime
from enum import Enum, auto
from typing import Optional, List

# Add curobo to path if not installed
sys.path.append('/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/curobo/src')

from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose as CuroboPose
from curobo.types.robot import JointState as CuroboJointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.util_file import load_yaml

# Import the program parser
from ur5_curobo_control.program_parser import (
    ProgramParser, RobotInstruction, InstructionType,
    NamedPositionsParser, NamedPosition, PositionType
)

# Import Robotiq gripper action
try:
    from robotiq_2f_urcap_adapter.action import GripperCommand as GripperCommandAction
    GRIPPER_ACTION_AVAILABLE = True
except ImportError:
    GRIPPER_ACTION_AVAILABLE = False

# ROS 2 imports for path resolution
from ament_index_python.packages import get_package_share_directory


class ExecutorState(Enum):
    IDLE = auto()
    LOADING = auto()
    EXECUTING = auto()
    PAUSED = auto()
    ERROR = auto()


class UR5ProgramExecutorNode(Node):
    """ROS 2 Node for executing UR5 robot programs from files."""
    
    def __init__(self):
        super().__init__('ur5_program_executor')
        
        # Callback group for concurrent service calls
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('robot_config_file', '',
            ParameterDescriptor(description='Path to cuRobo robot config YAML'))
        self.declare_parameter('world_config_file', '',
            ParameterDescriptor(description='Path to cuRobo world config YAML'))
        self.declare_parameter('programs_directory', '',
            ParameterDescriptor(description='Directory containing program files'))
        self.declare_parameter('default_speed', 0.5,
            ParameterDescriptor(description='Default speed factor (0.0-1.0)'))
        self.declare_parameter('trajectory_dt', 0.05,
            ParameterDescriptor(description='Trajectory interpolation timestep'))
        self.declare_parameter('program_file', '',
            ParameterDescriptor(description='Program file to load'))
        self.declare_parameter('auto_execute', False,
            ParameterDescriptor(description='Auto-execute program after loading'))
        self.declare_parameter('use_fake_hardware', True,
            ParameterDescriptor(description='Use fake hardware (sim) or real robot'))
        self.declare_parameter('presenter_control', True,
            ParameterDescriptor(description='Enable Logitech presenter control'))
        
        # Parameters for direct pose commands (set via UI)
        self.declare_parameter('target_pose_position', [0.0, 0.0, 0.0],
            ParameterDescriptor(description='Target pose position [x, y, z]'))
        self.declare_parameter('target_pose_quaternion', [1.0, 0.0, 0.0, 0.0],
            ParameterDescriptor(description='Target pose quaternion [w, x, y, z]'))
        
        # Parameters for save position command
        self.declare_parameter('save_position_name', '',
            ParameterDescriptor(description='Name for saving current position'))
        self.declare_parameter('save_position_type', 'joint',
            ParameterDescriptor(description='Type for saving position: joint or pose'))
        
        # Parameters for relative motion commands
        self.declare_parameter('relative_move_direction', '',
            ParameterDescriptor(description='Direction for relative move: left/right/forward/back/up/down'))
        self.declare_parameter('relative_move_distance', 0.05,
            ParameterDescriptor(description='Distance in meters for relative move (default 5cm)'))
        
        robot_config_file = self.get_parameter('robot_config_file').value
        self.use_fake_hardware = self.get_parameter('use_fake_hardware').value
        world_config_file = self.get_parameter('world_config_file').value
        self.programs_dir = self.get_parameter('programs_directory').value
        self.speed_factor = self.get_parameter('default_speed').value
        self.dt = self.get_parameter('trajectory_dt').value
        
        if not robot_config_file:
            self.get_logger().error("robot_config_file parameter is required")
            return
        
        self.get_logger().info(f"Loading robot config: {robot_config_file}")
        self.get_logger().info(f"Loading world config: {world_config_file}")
        self.get_logger().info(f"Programs directory: {self.programs_dir}")
        
        # Initialize state
        self.state = ExecutorState.IDLE
        self.current_program: List[RobotInstruction] = []
        self.current_instruction_idx = 0
        self.current_program_name = ""
        self.safety_triggered = False
        self.parser = ProgramParser()
        self._stop_event = threading.Event()  # Used to interrupt blocking sleeps on stop/pause
        
        # Initialize cuRobo
        self.tensor_args = TensorDeviceType()
        
        try:
            # Load YAML manually to fix paths dynamically (portable for different users/workspaces)
            robot_cfg_dict = load_yaml(robot_config_file)
            
            # Fix URDF path to be workspace-relative
            if 'robot_cfg' in robot_cfg_dict:
                kinematics = robot_cfg_dict['robot_cfg'].get('kinematics', {})
                if 'urdf_path' in kinematics:
                    try:
                        pkg_share = get_package_share_directory('ur5_curobo_control')
                        correct_urdf_path = os.path.join(pkg_share, 'config', 'ur5.urdf')
                        
                        current_path = kinematics['urdf_path']
                        if current_path != correct_urdf_path:
                            self.get_logger().warn(f"Updating URDF path from {current_path} to {correct_urdf_path}")
                            kinematics['urdf_path'] = correct_urdf_path
                            
                        # Also fix asset_root_path if present
                        if 'asset_root_path' in kinematics:
                            correct_asset_path = os.path.join(pkg_share, 'config')
                            if kinematics['asset_root_path'] != correct_asset_path:
                                self.get_logger().warn(f"Updating asset_root_path to {correct_asset_path}")
                                kinematics['asset_root_path'] = correct_asset_path
                    except Exception as e:
                        self.get_logger().error(f"Failed to resolve URDF path dynamically: {e}")
            
            self.motion_gen_config = MotionGenConfig.load_from_robot_config(
                robot_cfg_dict,  # Pass the modified dict instead of file path
                world_config_file,
                self.tensor_args,
                trajopt_tsteps=32,
                use_cuda_graph=False,
                interpolation_dt=self.dt,
            )
            self.motion_gen = MotionGen(self.motion_gen_config)
            self.get_logger().info("cuRobo MotionGen initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize cuRobo: {e}")
            self.state = ExecutorState.ERROR
            return
        
        # Joint names for UR5
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.current_joint_state: Optional[torch.Tensor] = None
        
        # Gripper state tracking (0.0 = open, 1.0 = closed)
        self.gripper_state: float = 0.0  # Assume open at startup
        
        # Named positions (loaded from config)
        self.named_positions: dict = {}  # name -> NamedPosition
        self._load_named_positions()
        
        # Sub-program recursion guard (max depth)
        self.max_sub_program_depth = 10
        self.current_sub_program_depth = 0
        
        # ROS Interfaces
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        
        # Parameter change callback so external speed changes take effect
        self.add_on_set_parameters_callback(self._on_parameter_change)
        
        # Execution timer (disabled by default)
        self.execution_timer = None
        
        # Auto-load program if specified
        self.auto_execute = self.get_parameter('auto_execute').value
        program_file = self.get_parameter('program_file').value
        if program_file:
            self._auto_load_program(program_file)
        
        # Presenter/keyboard control
        self.presenter_enabled = self.get_parameter('presenter_control').value
        self.keyboard_thread = None
        self.keyboard_running = False
        self.program_ready_to_start = False  # Flag to indicate program is loaded and ready
        
        # Recording mode
        self.recording_file = None
        self.recording_filename = None
        
        if self.presenter_enabled:
            self._start_keyboard_listener()
            self.get_logger().info("Presenter control enabled - press [Next/PageDown/Right/Space] to start/restart program")
        
        self.get_logger().info("UR5 Program Executor Node initialized")
    
    def _on_parameter_change(self, params):
        """Callback when parameters are changed externally (e.g. via API)."""
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'default_speed':
                new_speed = max(0.01, min(1.0, param.value))
                self.speed_factor = new_speed
                self.get_logger().info(f"Speed updated externally to {new_speed:.2f}")
        return SetParametersResult(successful=True)
    
    def _auto_load_program(self, program_file: str):
        """Auto-load a program file at startup."""
        # Build full path
        if not os.path.isabs(program_file):
            if self.programs_dir:
                program_file = os.path.join(self.programs_dir, program_file)
        
        if not os.path.exists(program_file):
            self.get_logger().error(f"Program file not found: {program_file}")
            return
        
        try:
            self.current_program = self.parser.parse_file(program_file)
            errors = self.parser.get_errors()
            
            if errors:
                self.get_logger().warn(f"Parse warnings: {errors}")
            
            self.current_program_name = os.path.basename(program_file)
            self.current_instruction_idx = 0
            
            self.get_logger().info(f"Auto-loaded program: {self.current_program_name} ({len(self.current_program)} instructions)")
            
            # If auto_execute is enabled, start a timer to execute after we get joint states
            if self.auto_execute:
                self.get_logger().info("Auto-execute enabled, will start execution when joint states are available...")
                self.startup_timer = self.create_timer(1.0, self._check_and_start_execution)
            else:
                # Mark program as ready to start via presenter
                self.program_ready_to_start = True
                self.get_logger().info("Program loaded. Press presenter [Next] button to start execution.")
                
        except Exception as e:
            self.get_logger().error(f"Failed to auto-load program: {e}")
    
    def _check_and_start_execution(self):
        """Check if we have joint states and start execution."""
        if self.current_joint_state is not None:
            self.get_logger().info("Joint states received, starting auto-execution...")
            self.startup_timer.cancel()
            self.startup_timer = None
            
            # Start execution
            self.state = ExecutorState.EXECUTING
            self.current_instruction_idx = 0
            self.execution_timer = self.create_timer(0.1, self.execution_step)
            self.publish_status(f"Auto-executing: {self.current_program_name}")
        else:
            self.get_logger().info("Waiting for joint states...")
    
    def _start_keyboard_listener(self):
        """Start background thread to listen for keyboard/presenter input."""
        self.keyboard_running = True
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener_loop, daemon=True)
        self.keyboard_thread.start()
    
    def _stop_keyboard_listener(self):
        """Stop the keyboard listener thread."""
        self.keyboard_running = False
        if self.keyboard_thread:
            self.keyboard_thread.join(timeout=1.0)
    
    def _keyboard_listener_loop(self):
        """Background thread that listens for keyboard/presenter input using evdev."""
        # Try to use evdev for direct input device reading (works better with ROS launch)
        try:
            import evdev
            self._evdev_listener_loop(evdev)
            return
        except ImportError:
            self.get_logger().warn("evdev not available, falling back to stdin listener")
        except Exception as e:
            self.get_logger().warn(f"evdev failed: {e}, falling back to stdin listener")
        
        # Fallback to stdin-based listener
        self._stdin_listener_loop()
    
    def _evdev_listener_loop(self, evdev):
        """Listen for input events using evdev (works with presenters)."""
        # Find keyboard/presenter devices
        devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
        keyboard_devices = []
        
        for device in devices:
            # Look for keyboard-like devices (presenters appear as keyboards)
            caps = device.capabilities()
            if evdev.ecodes.EV_KEY in caps:
                key_caps = caps[evdev.ecodes.EV_KEY]
                # Check if it has typical keyboard/presenter keys
                if evdev.ecodes.KEY_PAGEDOWN in key_caps or evdev.ecodes.KEY_RIGHT in key_caps:
                    keyboard_devices.append(device)
                    self.get_logger().info(f"Found input device: {device.name}")
        
        if not keyboard_devices:
            self.get_logger().warn("No keyboard/presenter devices found via evdev")
            self._stdin_listener_loop()
            return
        
        self.get_logger().info(f"Listening on {len(keyboard_devices)} input device(s)")
        
        # Key codes for presenter buttons
        NEXT_KEYS = {
            evdev.ecodes.KEY_PAGEDOWN,
            evdev.ecodes.KEY_RIGHT,
            evdev.ecodes.KEY_SPACE,
            evdev.ecodes.KEY_N,
            evdev.ecodes.KEY_F5,  # Some presenters use F5
        }
        PREV_KEYS = {
            evdev.ecodes.KEY_PAGEUP,
            evdev.ecodes.KEY_LEFT,
            evdev.ecodes.KEY_P,
            evdev.ecodes.KEY_BACKSPACE,
        }
        STOP_KEYS = {
            evdev.ecodes.KEY_S,
            evdev.ecodes.KEY_ESC,
        }
        
        sel = selectors.DefaultSelector()
        for device in keyboard_devices:
            sel.register(device, selectors.EVENT_READ)
        
        try:
            while self.keyboard_running and rclpy.ok():
                # Non-blocking select with timeout
                events = sel.select(timeout=0.1)
                for key, mask in events:
                    device = key.fileobj
                    try:
                        for event in device.read():
                            if event.type == evdev.ecodes.EV_KEY:
                                # Only handle key press events (value=1), not release (value=0)
                                if event.value == 1:
                                    if event.code in NEXT_KEYS:
                                        self._handle_next_button()
                                    elif event.code in PREV_KEYS:
                                        self._handle_prev_button()
                                    elif event.code in STOP_KEYS:
                                        self._handle_stop_button()
                    except BlockingIOError:
                        pass
        finally:
            sel.close()
            for device in keyboard_devices:
                device.close()
    
    def _stdin_listener_loop(self):
        """Fallback stdin-based keyboard listener."""
        # Logitech presenter sends these keys:
        # - Next slide: Page Down (^[[6~), Right Arrow, or 'n'
        # - Previous slide: Page Up (^[[5~), Left Arrow, or 'p'
        
        self.get_logger().info("Using stdin keyboard listener (may not work well with ros2 launch)")
        self.get_logger().info("Consider installing evdev: pip install evdev")
        
        try:
            # Save terminal settings
            old_settings = termios.tcgetattr(sys.stdin)
            
            try:
                # Set terminal to raw mode to capture key presses
                tty.setraw(sys.stdin.fileno())
                
                while self.keyboard_running and rclpy.ok():
                    # Check if there's input available (non-blocking)
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        
                        # Handle escape sequences (arrow keys, page up/down)
                        if key == '\x1b':  # Escape sequence
                            if select.select([sys.stdin], [], [], 0.1)[0]:
                                key2 = sys.stdin.read(1)
                                if key2 == '[':
                                    if select.select([sys.stdin], [], [], 0.1)[0]:
                                        key3 = sys.stdin.read(1)
                                        if key3 == 'C':  # Right arrow
                                            self._handle_next_button()
                                        elif key3 == 'D':  # Left arrow
                                            self._handle_prev_button()
                                        elif key3 == '5':  # Page Up
                                            sys.stdin.read(1)  # Read trailing ~
                                            self._handle_prev_button()
                                        elif key3 == '6':  # Page Down
                                            sys.stdin.read(1)  # Read trailing ~
                                            self._handle_next_button()
                        elif key == ' ':  # Space bar
                            self._handle_next_button()
                        elif key == 'n':  # 'n' key
                            self._handle_next_button()
                        elif key == 'p':  # 'p' key
                            self._handle_prev_button()
                        elif key == 's':  # 's' key - stop
                            self._handle_stop_button()
                        elif key == 'q':  # 'q' key - quit
                            self.get_logger().info("Quit requested via keyboard")
                            break
                        elif key == '\x03':  # Ctrl+C
                            break
                            
            finally:
                # Restore terminal settings
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                
        except Exception as e:
            self.get_logger().warn(f"Keyboard listener error: {e}")
    
    def _handle_next_button(self):
        """Handle presenter 'next' button press - start/resume or pause."""
        self.get_logger().info(">>> PRESENTER: Next button pressed")
        
        if self.current_joint_state is None:
            self.get_logger().warn("Cannot start - waiting for joint states...")
            return
        
        if self.state == ExecutorState.EXECUTING:
            # Pause execution when running
            self.get_logger().info("Pausing execution...")
            self.state = ExecutorState.PAUSED
            self.stop_robot()
            self.publish_status("Paused - Press [Next] to resume")
            return
        
        if not self.current_program:
            self.get_logger().warn("No program loaded")
            return
        
        if self.state == ExecutorState.IDLE:
            # Resume from current index, or restart if program is complete
            if self.current_instruction_idx >= len(self.current_program):
                self.current_instruction_idx = 0
                self.get_logger().info(f"Restarting program: {self.current_program_name}")
            else:
                self.get_logger().info(f"Resuming program: {self.current_program_name} from step {self.current_instruction_idx + 1}/{len(self.current_program)}")
            
            self.state = ExecutorState.EXECUTING
            self._stop_event.clear()
            
            # Create execution timer if not exists
            if self.execution_timer is None:
                self.execution_timer = self.create_timer(0.1, self.execution_step)
            
            self.publish_status(f"Executing: {self.current_program_name}")
            
        elif self.state == ExecutorState.PAUSED:
            # Resume execution
            self.get_logger().info("Resuming execution...")
            self._stop_event.clear()
            self.state = ExecutorState.EXECUTING
            self.publish_status("Resumed")
    
    def _handle_prev_button(self):
        """Handle presenter 'previous' button press - always records current pose."""
        self.get_logger().info(">>> PRESENTER: Previous/Record button pressed")
        # Always record pose regardless of state
        self._record_current_pose()
    
    def _record_current_pose(self):
        """Record the current robot pose to a .prog file.
        
        Uses actual joint positions from /joint_states topic to compute
        end-effector pose via forward kinematics.
        """
        if self.current_joint_state is None:
            self.get_logger().warn("Cannot record - no joint state available")
            return
        
        # Get current end-effector pose using forward kinematics
        try:
            # Log actual joint values being used for recording
            joint_values = self.current_joint_state.cpu().numpy()
            self.get_logger().info(f"Recording - Joint values from /joint_states: {joint_values}")
            
            # Use cuRobo to compute forward kinematics
            joint_state = CuroboJointState.from_position(
                self.current_joint_state.view(1, -1)
            )
            
            # Get the robot model's kinematics
            kin_state = self.motion_gen.kinematics.get_state(
                self.current_joint_state.view(1, -1)
            )
            
            # Extract end-effector pose
            ee_pos = kin_state.ee_position.squeeze().cpu().numpy()
            ee_quat = kin_state.ee_quaternion.squeeze().cpu().numpy()  # [w, x, y, z]
            self.get_logger().info(f"Recording - FK result: pos={ee_pos}, quat={ee_quat}")
            
            # Create recording file if not exists
            if self.recording_file is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.recording_filename = f"recorded_{timestamp}.prog"
                filepath = os.path.join(self.programs_dir, self.recording_filename)
                self.recording_file = open(filepath, 'w')
                self.recording_file.write(f"# Recorded program - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                self.recording_file.write(f"# Press [Prev] to record poses, [Next] to finish\n")
                self.recording_file.write(f"\n# Initial settings\n")
                self.recording_file.write(f"set_speed(0.5)\n\n")
                self.get_logger().info(f"Started recording to: {self.recording_filename}")
            
            # Format pose as movetopose instruction
            # Position: [x, y, z], Quaternion: [w, x, y, z]
            pos_str = f"[{ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}]"
            quat_str = f"[{ee_quat[0]:.4f}, {ee_quat[1]:.4f}, {ee_quat[2]:.4f}, {ee_quat[3]:.4f}]"
            
            # Write to file with timestamp comment
            time_str = datetime.now().strftime("%H:%M:%S")
            self.recording_file.write(f"# Recorded at {time_str}\n")
            self.recording_file.write(f"movetopose({pos_str}, {quat_str})\n")
            self.recording_file.write(f"wait(0.5)\n\n")
            self.recording_file.flush()
            
            self.get_logger().info(f"Recorded pose: pos={pos_str}, quat={quat_str}")
            self.get_logger().info(f"Saved to: {self.recording_filename}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to record pose: {e}")
    
    def _finish_recording(self):
        """Close the recording file."""
        if self.recording_file is not None:
            self.recording_file.write(f"\n# End of recorded program\n")
            self.recording_file.close()
            self.get_logger().info(f"Recording saved: {self.recording_filename}")
            self.recording_file = None
            self.recording_filename = None
    
    def _handle_stop_button(self):
        """Handle stop/save request - saves recording (does not stop execution)."""
        self.get_logger().info(">>> PRESENTER: Save/Stop button pressed")
        
        # Save recording if active
        if self.recording_file is not None:
            self._finish_recording()
        else:
            self.get_logger().info("No active recording to save")

    def _setup_subscribers(self):
        """Set up ROS subscribers."""
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.safety_sub = self.create_subscription(
            Bool,
            '/human_safety',
            self.safety_callback,
            10
        )
    
    def _setup_publishers(self):
        """Set up ROS publishers."""
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '~/status',
            10
        )
        
        self.gripper_pub = self.create_publisher(
            String,
            '/gripper_command_simple',
            10
        )
        
        # Publisher for gripper visualization in RViz (for fake hardware)
        self.gripper_viz_pub = self.create_publisher(
            Float64,
            '/gripper_position_command',
            10
        )
        
        # Gripper action client for real hardware
        self.gripper_action_client = None
        if GRIPPER_ACTION_AVAILABLE and not self.use_fake_hardware:
            self.gripper_action_client = ActionClient(
                self,
                GripperCommandAction,
                '/robotiq_2f_urcap_adapter/gripper_command'
            )
            self.get_logger().info("Gripper action client created for /robotiq_2f_urcap_adapter/gripper_command")
    
    def _setup_services(self):
        """Set up ROS services."""
        # Service to execute a program by name
        self.execute_srv = self.create_service(
            Trigger,
            '~/execute_program',
            self.execute_program_callback,
            callback_group=self.callback_group
        )
        
        # Service to load a program
        self.load_srv = self.create_service(
            Trigger,
            '~/load_program',
            self.load_program_callback,
            callback_group=self.callback_group
        )
        
        # Service to pause execution
        self.pause_srv = self.create_service(
            SetBool,
            '~/pause',
            self.pause_callback,
            callback_group=self.callback_group
        )
        
        # Service to stop execution
        self.stop_srv = self.create_service(
            Trigger,
            '~/stop',
            self.stop_callback,
            callback_group=self.callback_group
        )
        
        # Service to get list of available programs
        self.list_programs_srv = self.create_service(
            Trigger,
            '~/list_programs',
            self.list_programs_callback,
            callback_group=self.callback_group
        )
        
        # Service to move to a pose directly (reads target_pose_* parameters)
        self.move_to_pose_srv = self.create_service(
            Trigger,
            '~/move_to_pose',
            self.move_to_pose_callback,
            callback_group=self.callback_group
        )
        
        # Service to save current position as a named position
        self.save_position_srv = self.create_service(
            Trigger,
            '~/save_position',
            self.save_position_callback,
            callback_group=self.callback_group
        )
        
        # Service to move relative in task space (Cartesian offset)
        self.move_relative_srv = self.create_service(
            Trigger,
            '~/move_relative',
            self.move_relative_callback,
            callback_group=self.callback_group
        )
    
    def joint_state_callback(self, msg: JointState):
        """Handle incoming joint states."""
        positions = []
        try:
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    positions.append(msg.position[idx])
            
            if len(positions) == len(self.joint_names):
                self.current_joint_state = torch.tensor(
                    positions,
                    device=self.tensor_args.device,
                    dtype=self.tensor_args.dtype
                )
        except ValueError:
            pass
    
    def safety_callback(self, msg: Bool):
        """Handle safety trigger messages."""
        prev_state = self.safety_triggered
        self.safety_triggered = msg.data
        
        if self.safety_triggered and not prev_state:
            self.get_logger().warn("SAFETY TRIGGERED: Human detected! Pausing execution.")
            self.pause_execution(True)
            self.stop_robot()
        elif not self.safety_triggered and prev_state:
            self.get_logger().info("SAFETY CLEARED: Resuming execution.")
            self.pause_execution(False)
    
    def execute_program_callback(self, request, response):
        """Service callback to execute a loaded program."""
        if self.state == ExecutorState.ERROR:
            response.success = False
            response.message = "Node is in error state"
            return response
        
        if not self.current_program:
            response.success = False
            response.message = "No program loaded. Use ~/load_program service first."
            return response
        
        if self.state == ExecutorState.EXECUTING:
            response.success = False
            response.message = "Already executing a program"
            return response
        
        # Resume from current index; restart only if program is complete
        if self.current_instruction_idx >= len(self.current_program):
            self.current_instruction_idx = 0
        
        self.state = ExecutorState.EXECUTING
        self._stop_event.clear()
        
        # Start execution timer
        self.execution_timer = self.create_timer(0.1, self.execution_step)
        
        remaining = len(self.current_program) - self.current_instruction_idx
        response.success = True
        response.message = f"Executing {self.current_program_name} from step {self.current_instruction_idx + 1} ({remaining} remaining)"
        self.publish_status(f"Executing: {self.current_program_name}")
        return response
    
    def load_program_callback(self, request, response):
        """Service callback to load a program file."""
        # Get the program_file parameter (set via CLI or launch)
        program_file = self.get_parameter('program_file').value
        
        self.get_logger().info(f"Load request - program_file param: '{program_file}'")
        
        if not program_file:
            response.success = False
            response.message = "Set 'program_file' parameter before calling load_program"
            return response
        
        # Build full path
        if not os.path.isabs(program_file):
            if self.programs_dir:
                program_file = os.path.join(self.programs_dir, program_file)
        
        if not os.path.exists(program_file):
            response.success = False
            response.message = f"Program file not found: {program_file}"
            return response
        
        try:
            self.current_program = self.parser.parse_file(program_file)
            errors = self.parser.get_errors()
            
            if errors:
                self.get_logger().warn(f"Parse warnings: {errors}")
            
            self.current_program_name = os.path.basename(program_file)
            self.current_instruction_idx = 0
            self.state = ExecutorState.IDLE
            
            response.success = True
            response.message = f"Loaded program: {self.current_program_name} ({len(self.current_program)} instructions)"
            self.publish_status(f"Loaded: {self.current_program_name}")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to load program: {e}"
            self.state = ExecutorState.ERROR
        
        return response
    
    def pause_callback(self, request, response):
        """Service callback to pause/resume execution."""
        self.pause_execution(request.data)
        response.success = True
        response.message = "Paused" if request.data else "Resumed"
        return response
    
    def stop_callback(self, request, response):
        """Service callback to stop execution."""
        self.stop_execution()
        response.success = True
        response.message = "Execution stopped"
        return response
    
    def list_programs_callback(self, request, response):
        """Service callback to list available programs."""
        if not self.programs_dir or not os.path.isdir(self.programs_dir):
            response.success = False
            response.message = "Programs directory not configured or doesn't exist"
            return response
        
        try:
            programs = [f for f in os.listdir(self.programs_dir) 
                       if f.endswith('.prog') or f.endswith('.txt')]
            response.success = True
            response.message = f"Available programs: {', '.join(programs)}"
        except Exception as e:
            response.success = False
            response.message = f"Error listing programs: {e}"
        
        return response
    
    def move_to_pose_callback(self, request, response):
        """Service callback to move to a Cartesian pose directly.
        
        Reads target_pose_position and target_pose_quaternion parameters
        and uses cuRobo to plan and execute motion.
        """
        if self.state == ExecutorState.EXECUTING:
            response.success = False
            response.message = "Cannot move: program is executing"
            return response
        
        if self.current_joint_state is None:
            response.success = False
            response.message = "Cannot move: no joint state received"
            return response
        
        # Get target pose from parameters
        target_pos = self.get_parameter('target_pose_position').value
        target_quat = self.get_parameter('target_pose_quaternion').value
        
        if len(target_pos) != 3 or len(target_quat) != 4:
            response.success = False
            response.message = f"Invalid pose: position={target_pos}, quaternion={target_quat}"
            return response
        
        self.get_logger().info(f"Moving to pose: pos={target_pos}, quat={target_quat}")
        
        try:
            # Create cuRobo Pose (expects quaternion as [w, x, y, z])
            target_pose = CuroboPose(
                position=torch.tensor(target_pos, device=self.tensor_args.device, dtype=self.tensor_args.dtype),
                quaternion=torch.tensor(target_quat, device=self.tensor_args.device, dtype=self.tensor_args.dtype)
            )
            
            # Create Start State
            start_state = CuroboJointState.from_position(self.current_joint_state.view(1, -1))
            
            # Plan motion
            result = self.motion_gen.plan_single(
                start_state, target_pose,
                MotionGenPlanConfig(enable_graph=False, timeout=5.0)
            )
            
            if result.success.item():
                traj = result.get_interpolated_plan()
                self.publish_trajectory(traj)
                duration = self.estimate_trajectory_duration(traj)
                response.success = True
                response.message = f"Moving to pose (duration: {duration:.1f}s)"
                self.get_logger().info(f"Motion planned successfully, executing trajectory")
            else:
                response.success = False
                response.message = f"Motion planning failed: {result.status}"
                self.get_logger().error(f"Motion planning failed: {result.status}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error planning motion: {e}"
            self.get_logger().error(f"Error planning motion: {e}")
        
        return response
    
    def save_position_callback(self, request, response):
        """Service callback to save current position as a named position.
        
        Reads save_position_name and save_position_type parameters,
        then appends the current position to named_positions.txt.
        """
        if self.current_joint_state is None:
            response.success = False
            response.message = "Cannot save: no joint state received"
            return response
        
        name = self.get_parameter('save_position_name').value
        pos_type = self.get_parameter('save_position_type').value  # "joint" or "pose"
        
        if not name:
            response.success = False
            response.message = "Set 'save_position_name' parameter before calling save_position"
            return response
        
        # Sanitise name (no spaces, replace with underscore)
        name = name.strip().replace(' ', '_')
        
        # Find the named_positions.txt file
        config_dir = None
        if self.programs_dir:
            config_dir = os.path.join(os.path.dirname(self.programs_dir), 'config')
        if not config_dir or not os.path.isdir(config_dir):
            # Try source tree
            workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            config_dir = os.path.join(workspace_root, 'src', 'ur5_curobo_control', 'config')
        
        np_file = os.path.join(config_dir, 'named_positions.txt') if config_dir else None
        if not np_file or not os.path.exists(np_file):
            response.success = False
            response.message = f"Named positions file not found (looked in {config_dir})"
            return response
        
        try:
            joint_values = self.current_joint_state.cpu().numpy().tolist()

            # Joint values from /joint_states are in radians; convert to degrees
            joint_degrees = [float(np.degrees(v)) for v in joint_values]

            if pos_type == "pose":
                # Use FK to get EE pose
                kin_state = self.motion_gen.kinematics.get_state(
                    self.current_joint_state.view(1, -1)
                )
                ee_pos = kin_state.ee_position.squeeze().cpu().numpy().tolist()
                ee_quat = kin_state.ee_quaternion.squeeze().cpu().numpy().tolist()  # [w, x, y, z]
                
                line = f"pose {name} {ee_pos[0]:.4f} {ee_pos[1]:.4f} {ee_pos[2]:.4f} {ee_quat[0]:.4f} {ee_quat[1]:.4f} {ee_quat[2]:.4f} {ee_quat[3]:.4f}\n"
                msg = f"Saved pose '{name}': pos=[{ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}], quat=[{ee_quat[0]:.4f}, {ee_quat[1]:.4f}, {ee_quat[2]:.4f}, {ee_quat[3]:.4f}]"
            else:
                # Save as joint position (store degrees)
                vals_str = ' '.join(f'{v:.4f}' for v in joint_degrees)
                line = f"joint {name} {vals_str}\n"
                msg = f"Saved joint position '{name}': [{', '.join(f'{v:.4f}' for v in joint_degrees)}]"
            
            # Append to file
            with open(np_file, 'a') as f:
                f.write(f"# Saved at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(line)
            
            # Also update installed copy if it exists (for symlink-install, this is the same file)
            installed_np = os.path.join(
                os.path.dirname(self.programs_dir) if self.programs_dir else '',
                'config', 'named_positions.txt'
            )
            if installed_np != np_file and os.path.exists(installed_np):
                with open(installed_np, 'a') as f:
                    f.write(f"# Saved at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write(line)
            
            self.get_logger().info(msg)
            response.success = True
            response.message = msg
            
        except Exception as e:
            response.success = False
            response.message = f"Error saving position: {e}"
            self.get_logger().error(f"Error saving position: {e}")
        
        return response
    
    def move_relative_callback(self, request, response):
        """Service callback to move the end-effector by a Cartesian offset.
        
        Reads relative_move_direction and relative_move_distance parameters.
        Directions are relative to the robot base frame:
          - left/right  → Y axis (left = +Y, right = -Y)
          - forward/back → X axis (forward = +X, back = -X)
          - up/down      → Z axis (global gravity; up = +Z, down = -Z)
        
        The end-effector orientation is preserved.
        """
        if self.state == ExecutorState.EXECUTING:
            response.success = False
            response.message = "Cannot move: program is executing"
            return response
        
        if self.current_joint_state is None:
            response.success = False
            response.message = "Cannot move: no joint state received"
            return response
        
        direction = self.get_parameter('relative_move_direction').value.strip().lower()
        distance = self.get_parameter('relative_move_distance').value
        
        if not direction:
            response.success = False
            response.message = "Set 'relative_move_direction' parameter (left/right/forward/back/up/down)"
            return response
        
        # Map direction to offset in base frame
        direction_map = {
            'left':    [0.0, +1.0, 0.0],
            'right':   [0.0, -1.0, 0.0],
            'forward': [+1.0, 0.0, 0.0],
            'back':    [-1.0, 0.0, 0.0],
            'up':      [0.0, 0.0, +1.0],
            'down':    [0.0, 0.0, -1.0],
        }
        
        if direction not in direction_map:
            response.success = False
            response.message = f"Unknown direction '{direction}'. Use: left, right, forward, back, up, down"
            return response
        
        offset = [d * distance for d in direction_map[direction]]
        
        try:
            # Get current EE pose via FK
            kin_state = self.motion_gen.kinematics.get_state(
                self.current_joint_state.view(1, -1)
            )
            ee_pos = kin_state.ee_position.squeeze().cpu().numpy().tolist()
            ee_quat = kin_state.ee_quaternion.squeeze().cpu().numpy().tolist()
            
            # Apply offset to position (keep orientation unchanged)
            target_pos = [ee_pos[i] + offset[i] for i in range(3)]
            
            self.get_logger().info(
                f"Relative move: {direction} {distance:.3f}m  "
                f"from [{ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}] "
                f"to [{target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}]"
            )
            
            # Plan motion with cuRobo
            target_pose = CuroboPose(
                position=torch.tensor(target_pos, device=self.tensor_args.device, dtype=self.tensor_args.dtype),
                quaternion=torch.tensor(ee_quat, device=self.tensor_args.device, dtype=self.tensor_args.dtype)
            )
            start_state = CuroboJointState.from_position(self.current_joint_state.view(1, -1))
            
            result = self.motion_gen.plan_single(
                start_state, target_pose,
                MotionGenPlanConfig(enable_graph=False, timeout=5.0)
            )
            
            if result.success.item():
                traj = result.get_interpolated_plan()
                self.publish_trajectory(traj)
                duration = self.estimate_trajectory_duration(traj)
                response.success = True
                response.message = f"Moving {direction} {distance:.3f}m (duration: {duration:.1f}s)"
                self.get_logger().info(f"Relative move planned, executing trajectory")
            else:
                response.success = False
                response.message = f"Motion planning failed for {direction} move: {result.status}"
                self.get_logger().error(f"Relative move planning failed: {result.status}")
        
        except Exception as e:
            response.success = False
            response.message = f"Error planning relative move: {e}"
            self.get_logger().error(f"Error in move_relative: {e}")
        
        return response
    
    def execution_step(self):
        """Execute the next instruction in the program.
        
        Handles conditional blocks (if/else/endif) by evaluating conditions
        and skipping instructions in false branches.
        """
        if self.state != ExecutorState.EXECUTING:
            return
        
        if self.safety_triggered:
            self.get_logger().warn("Waiting for safety clear...")
            return
        
        if self.current_joint_state is None:
            self.get_logger().warn("Waiting for joint states...")
            return
        
        if self.current_instruction_idx >= len(self.current_program):
            self.get_logger().info("=" * 50)
            self.get_logger().info("Program execution complete!")
            if self.presenter_enabled:
                self.get_logger().info("Press presenter [Next] button to restart program.")
            self.get_logger().info("=" * 50)
            self.stop_execution()
            self.publish_status("Complete - Press [Next] to restart")
            return
        
        instruction = self.current_program[self.current_instruction_idx]
        
        # Handle conditional blocks
        if instruction.type == InstructionType.IF:
            condition_result = self._evaluate_condition(instruction)
            self.get_logger().info(f"Evaluating [{self.current_instruction_idx + 1}/{len(self.current_program)}]: {instruction.raw_line} => {condition_result}")
            if not condition_result:
                # Skip to matching else or endif
                self._skip_to_else_or_endif()
            else:
                self.current_instruction_idx += 1
            return
        
        if instruction.type == InstructionType.ELSE:
            # If we reach ELSE during normal execution, the IF was true,
            # so skip to the matching ENDIF
            self._skip_to_endif()
            return
        
        if instruction.type == InstructionType.ENDIF:
            # Just advance past it
            self.current_instruction_idx += 1
            return
        
        self.get_logger().info(f"Executing [{self.current_instruction_idx + 1}/{len(self.current_program)}]: {instruction.raw_line}")
        
        success = self.execute_instruction(instruction)
        
        # If stop/pause was triggered during execution, don't advance the index
        if self.state != ExecutorState.EXECUTING:
            return
        
        if success:
            self.current_instruction_idx += 1
        else:
            self.get_logger().error(f"Failed to execute instruction at line {instruction.line_number}")
            # Continue to next instruction on failure (could also stop here)
            self.current_instruction_idx += 1
    
    def execute_instruction(self, instruction: RobotInstruction) -> bool:
        """Execute a single robot instruction."""
        try:
            if instruction.type == InstructionType.MOVE_TO_POSE:
                return self.execute_move_to_pose(instruction)
            elif instruction.type == InstructionType.MOVE_TO_JOINT:
                return self.execute_move_to_joint(instruction)
            elif instruction.type == InstructionType.MOVE_TO_NAMED:
                return self.execute_move_to_named(instruction)
            elif instruction.type == InstructionType.MOVE_RELATIVE:
                return self.execute_move_relative_instruction(instruction)
            elif instruction.type == InstructionType.RUN_PROGRAM:
                return self.execute_run_program(instruction)
            elif instruction.type == InstructionType.WAIT:
                return self.execute_wait(instruction)
            elif instruction.type == InstructionType.OPEN_GRIPPER:
                return self.execute_gripper(0.0)
            elif instruction.type == InstructionType.CLOSE_GRIPPER:
                return self.execute_gripper(1.0)
            elif instruction.type == InstructionType.GRIPPER:
                return self.execute_gripper(instruction.gripper_position)
            elif instruction.type == InstructionType.SET_SPEED:
                self.speed_factor = instruction.speed_factor
                self.get_logger().info(f"Speed set to {self.speed_factor}")
                return True
            elif instruction.type in (InstructionType.IF, InstructionType.ELSE, InstructionType.ENDIF):
                # Conditionals are handled by execution_step, not here
                return True
            else:
                self.get_logger().warn(f"Skipping unknown instruction: {instruction.raw_line}")
                return True
        except Exception as e:
            self.get_logger().error(f"Error executing instruction: {e}")
            return False
    
    def execute_move_to_pose(self, instruction: RobotInstruction) -> bool:
        """Execute a move to pose instruction using cuRobo."""
        if instruction.pose is None:
            return False
        
        target_pos, target_quat = instruction.pose
        
        # Create cuRobo Pose (expects quaternion as [w, x, y, z])
        target_pose = CuroboPose(
            position=torch.tensor(target_pos, device=self.tensor_args.device, dtype=self.tensor_args.dtype),
            quaternion=torch.tensor(target_quat, device=self.tensor_args.device, dtype=self.tensor_args.dtype)
        )
        
        # Create Start State
        start_state = CuroboJointState.from_position(self.current_joint_state.view(1, -1))
        
        # Plan motion
        result = self.motion_gen.plan_single(
            start_state, target_pose,
            MotionGenPlanConfig(enable_graph=False, timeout=2.0)
        )
        
        if result.success.item():
            self.get_logger().info("Motion plan successful, executing...")
            traj = result.interpolated_plan
            self.publish_trajectory(traj)
            
            # Wait for trajectory to complete, but allow interruption by stop/pause
            traj_duration = self.estimate_trajectory_duration(traj)
            if self._stop_event.wait(timeout=traj_duration + 0.5):
                self.get_logger().info("Motion interrupted by stop/pause")
                return False
            return True
        else:
            self.get_logger().error(f"Motion planning failed: {result.status}")
            return False
    
    def execute_move_to_joint(self, instruction: RobotInstruction) -> bool:
        """Execute a move to joint position."""
        if instruction.joint_positions is None:
            return False
        
        # Create trajectory message directly to joint position
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = instruction.joint_positions
        point.velocities = [0.0] * 6
        
        # Calculate duration based on max joint movement
        current_pos = self.current_joint_state.cpu().numpy()
        max_diff = np.max(np.abs(np.array(instruction.joint_positions) - current_pos))
        duration = max(max_diff / (0.5 * self.speed_factor), 1.0)  # At least 1 second
        
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)
        
        # Wait for trajectory to complete, but allow interruption
        if self._stop_event.wait(timeout=duration + 0.5):
            self.get_logger().info("Joint motion interrupted by stop/pause")
            return False
        return True
    
    def execute_wait(self, instruction: RobotInstruction) -> bool:
        """Execute a wait instruction."""
        if instruction.wait_duration is None:
            return False
        
        self.get_logger().info(f"Waiting {instruction.wait_duration} seconds...")
        if self._stop_event.wait(timeout=instruction.wait_duration):
            self.get_logger().info("Wait interrupted by stop/pause")
            return False
        return True
    
    def execute_gripper(self, position: float) -> bool:
        """Execute gripper command.
        
        For real hardware: Uses ros2 action send_goal subprocess command.
        For fake hardware: Just publishes to simple topic and logs.
        
        Args:
            position: Gripper position (0.0 = fully open, 1.0 = fully closed)
        """
        # Convert normalized position (0-1) to Robotiq position
        # Robotiq 2F-85: 0.085m = fully open, 0.0m = fully closed
        # So we invert: position 0 (open) -> 0.085, position 1 (closed) -> 0.0
        robotiq_position = (1.0 - position) * 0.085
        
        # Track gripper state for conditional logic
        self.gripper_state = position
        
        if self.use_fake_hardware:
            # Fake hardware - publish to visualization topic for RViz animation
            viz_msg = Float64()
            viz_msg.data = position  # 0.0 = open, 1.0 = closed
            self.gripper_viz_pub.publish(viz_msg)
            
            # Also publish to simple topic for logging
            msg = String()
            msg.data = f"position:{position}"
            self.gripper_pub.publish(msg)
            self.get_logger().info(f"[FAKE] Gripper command: position={position} (robotiq_pos={robotiq_position:.4f}m)")
            time.sleep(0.5)
        else:
            # Real hardware - use ros2 action send_goal command via subprocess
            self.get_logger().info(f"[REAL] Sending gripper command: position={robotiq_position:.4f}m (normalized={position})")
            
            import subprocess
            
            # Build the command as a single shell string with proper quoting
            # The goal message MUST be wrapped in single quotes for the shell to pass it correctly
            cmd = (
                'ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command '
                'robotiq_2f_urcap_adapter/action/GripperCommand '
                f"'{{command: {{position: {robotiq_position}, max_effort: 100.0, max_speed: 0.1}}}}'"
            )
            
            self.get_logger().info(f"Running command: {cmd}")
            
            try:
                # Need to source ROS environment in subprocess
                # Unset LD_PRELOAD to avoid conflicts with the parent process's libstdc++ preload
                env_cmd = 'unset LD_PRELOAD && source /opt/ros/humble/setup.bash && source /home/rml/ur5-robotiq-ros2-control/install/setup.bash && ' + cmd
                result = subprocess.run(
                    env_cmd,
                    shell=True,
                    capture_output=True,
                    text=True,
                    timeout=15,
                    executable='/bin/bash'
                )
                
                self.get_logger().info(f"Subprocess stdout: {result.stdout}")
                if result.stderr:
                    self.get_logger().warn(f"Subprocess stderr: {result.stderr}")
                    
                if result.returncode == 0:
                    self.get_logger().info("Gripper command executed successfully")
                else:
                    self.get_logger().warn(f"Gripper command returned code {result.returncode}")
                    
            except subprocess.TimeoutExpired:
                self.get_logger().error("Gripper command timed out")
                return False
            except Exception as e:
                self.get_logger().error(f"Error executing gripper command: {e}")
                return False
        
        # Wait for gripper to complete movement, but allow interruption
        if self._stop_event.wait(timeout=1.0):
            self.get_logger().info("Gripper wait interrupted by stop/pause")
            return False
        return True
    
    # =================================================================
    # New instruction execution methods
    # =================================================================
    
    def _load_named_positions(self):
        """Load named positions from config file."""
        config_dir = None
        if self.programs_dir:
            config_dir = os.path.join(os.path.dirname(self.programs_dir), 'config')
        if not config_dir or not os.path.isdir(config_dir):
            workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            config_dir = os.path.join(workspace_root, 'src', 'ur5_curobo_control', 'config')
        
        np_file = os.path.join(config_dir, 'named_positions.txt') if config_dir else None
        if not np_file or not os.path.exists(np_file):
            self.get_logger().warn(f"Named positions file not found (looked in {config_dir})")
            return
        
        parser = NamedPositionsParser()
        positions = parser.parse_file(np_file)
        self.named_positions = {p.name.lower(): p for p in positions}
        self.get_logger().info(f"Loaded {len(self.named_positions)} named positions: {', '.join(self.named_positions.keys())}")
    
    def execute_move_to_named(self, instruction: RobotInstruction) -> bool:
        """Execute a move to a named position from named_positions.txt."""
        name = instruction.named_position
        if not name:
            self.get_logger().error("movetonamed: no position name specified")
            return False
        
        pos = self.named_positions.get(name.lower())
        if pos is None:
            # Reload in case positions were added at runtime
            self._load_named_positions()
            pos = self.named_positions.get(name.lower())
        
        if pos is None:
            self.get_logger().error(f"movetonamed: named position '{name}' not found")
            return False
        
        self.get_logger().info(f"Moving to named position: {pos.name} ({pos.position_type.name})")
        
        if pos.position_type == PositionType.JOINT:
            # Create a synthetic MOVE_TO_JOINT instruction
            synth = RobotInstruction(
                type=InstructionType.MOVE_TO_JOINT,
                line_number=instruction.line_number,
                raw_line=f"movetonamed({name}) -> movetojoint",
                joint_positions=list(pos.joint_positions)
            )
            return self.execute_move_to_joint(synth)
        
        elif pos.position_type == PositionType.POSE:
            # Create a synthetic MOVE_TO_POSE instruction
            synth = RobotInstruction(
                type=InstructionType.MOVE_TO_POSE,
                line_number=instruction.line_number,
                raw_line=f"movetonamed({name}) -> movetopose",
                pose=(list(pos.position), list(pos.quaternion))
            )
            return self.execute_move_to_pose(synth)
        
        self.get_logger().error(f"movetonamed: unknown position type for '{name}'")
        return False
    
    def execute_move_relative_instruction(self, instruction: RobotInstruction) -> bool:
        """Execute a relative Cartesian move from a program instruction."""
        if instruction.relative_move is None:
            return False
        
        direction, distance = instruction.relative_move
        
        direction_map = {
            'left':    [0.0, +1.0, 0.0],
            'right':   [0.0, -1.0, 0.0],
            'forward': [+1.0, 0.0, 0.0],
            'back':    [-1.0, 0.0, 0.0],
            'up':      [0.0, 0.0, +1.0],
            'down':    [0.0, 0.0, -1.0],
        }
        
        if direction not in direction_map:
            self.get_logger().error(f"moverelative: unknown direction '{direction}'")
            return False
        
        offset = [d * distance for d in direction_map[direction]]
        
        try:
            # Get current EE pose via FK
            kin_state = self.motion_gen.kinematics.get_state(
                self.current_joint_state.view(1, -1)
            )
            ee_pos = kin_state.ee_position.squeeze().cpu().numpy().tolist()
            ee_quat = kin_state.ee_quaternion.squeeze().cpu().numpy().tolist()
            
            target_pos = [ee_pos[i] + offset[i] for i in range(3)]
            
            self.get_logger().info(
                f"Relative move: {direction} {distance:.3f}m  "
                f"from [{ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}] "
                f"to [{target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f}]"
            )
            
            target_pose = CuroboPose(
                position=torch.tensor(target_pos, device=self.tensor_args.device, dtype=self.tensor_args.dtype),
                quaternion=torch.tensor(ee_quat, device=self.tensor_args.device, dtype=self.tensor_args.dtype)
            )
            start_state = CuroboJointState.from_position(self.current_joint_state.view(1, -1))
            
            result = self.motion_gen.plan_single(
                start_state, target_pose,
                MotionGenPlanConfig(enable_graph=False, timeout=5.0)
            )
            
            if result.success.item():
                traj = result.get_interpolated_plan()
                self.publish_trajectory(traj)
                traj_duration = self.estimate_trajectory_duration(traj)
                if self._stop_event.wait(timeout=traj_duration + 0.5):
                    self.get_logger().info("Relative move interrupted by stop/pause")
                    return False
                return True
            else:
                self.get_logger().error(f"Relative move planning failed: {result.status}")
                return False
        except Exception as e:
            self.get_logger().error(f"Error in moverelative: {e}")
            return False
    
    def execute_run_program(self, instruction: RobotInstruction) -> bool:
        """Execute a sub-program from a .prog file.
        
        Parses the sub-program and inserts its instructions into the
        current program at the current position, replacing the runprogram
        instruction. This allows programs to call other programs.
        """
        filename = instruction.sub_program
        if not filename:
            self.get_logger().error("runprogram: no filename specified")
            return False
        
        # Guard against infinite recursion
        self.current_sub_program_depth += 1
        if self.current_sub_program_depth > self.max_sub_program_depth:
            self.get_logger().error(f"runprogram: max recursion depth ({self.max_sub_program_depth}) exceeded")
            self.current_sub_program_depth -= 1
            return False
        
        # Build full path
        if not os.path.isabs(filename):
            if self.programs_dir:
                filepath = os.path.join(self.programs_dir, filename)
            else:
                filepath = filename
        else:
            filepath = filename
        
        if not os.path.exists(filepath):
            self.get_logger().error(f"runprogram: file not found: {filepath}")
            self.current_sub_program_depth -= 1
            return False
        
        try:
            sub_parser = ProgramParser()
            sub_instructions = sub_parser.parse_file(filepath)
            errors = sub_parser.get_errors()
            if errors:
                self.get_logger().warn(f"Sub-program parse warnings: {errors}")
            
            self.get_logger().info(f"Running sub-program: {filename} ({len(sub_instructions)} instructions)")
            
            # Insert sub-program instructions right after the current runprogram instruction
            insert_idx = self.current_instruction_idx + 1
            for i, sub_inst in enumerate(sub_instructions):
                self.current_program.insert(insert_idx + i, sub_inst)
            
            self.get_logger().info(f"Inserted {len(sub_instructions)} instructions from {filename}")
            self.current_sub_program_depth -= 1
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error loading sub-program {filename}: {e}")
            self.current_sub_program_depth -= 1
            return False
    
    # =================================================================
    # Conditional logic helpers
    # =================================================================
    
    def _evaluate_condition(self, instruction: RobotInstruction) -> bool:
        """Evaluate a conditional instruction and return True/False.
        
        Supports:
          - gripper_open: True if gripper is open (state < 0.5)
          - gripper_closed: True if gripper is closed (state >= 0.5)
          - near(PositionName, tolerance): True if robot is near the named position
          - not_near(PositionName, tolerance): True if robot is NOT near the named position
        """
        cond_type = instruction.condition_type
        
        if cond_type == 'gripper_open':
            result = self.gripper_state < 0.5
            self.get_logger().info(f"Condition gripper_open: gripper_state={self.gripper_state:.2f} => {result}")
            return result
        
        elif cond_type == 'gripper_closed':
            result = self.gripper_state >= 0.5
            self.get_logger().info(f"Condition gripper_closed: gripper_state={self.gripper_state:.2f} => {result}")
            return result
        
        elif cond_type in ('near', 'not_near'):
            target_name = instruction.condition_target
            tolerance = instruction.condition_tolerance if instruction.condition_tolerance is not None else 0.1  # default 0.1 rad (~5.7 deg)
            
            if not target_name:
                self.get_logger().error(f"Condition {cond_type}: no target position name")
                return False
            
            pos = self.named_positions.get(target_name.lower())
            if pos is None:
                self._load_named_positions()
                pos = self.named_positions.get(target_name.lower())
            
            if pos is None:
                self.get_logger().error(f"Condition {cond_type}: named position '{target_name}' not found")
                return False
            
            is_near = self._is_near_position(pos, tolerance)
            result = is_near if cond_type == 'near' else not is_near
            self.get_logger().info(f"Condition {cond_type}({target_name}, {tolerance:.3f}): is_near={is_near} => {result}")
            return result
        
        self.get_logger().error(f"Unknown condition type: {cond_type}")
        return False
    
    def _is_near_position(self, pos: 'NamedPosition', tolerance: float) -> bool:
        """Check if the robot is near a named position.
        
        For joint positions: checks max absolute joint difference.
        For Cartesian poses: checks Euclidean distance of end-effector.
        """
        if self.current_joint_state is None:
            return False
        
        current = self.current_joint_state.cpu().numpy()
        
        if pos.position_type == PositionType.JOINT:
            target = np.array(pos.joint_positions)
            max_diff = np.max(np.abs(current - target))
            self.get_logger().debug(f"Near check (joint): max_diff={max_diff:.4f} tolerance={tolerance:.4f}")
            return max_diff < tolerance
        
        elif pos.position_type == PositionType.POSE:
            # Compare end-effector positions
            try:
                kin_state = self.motion_gen.kinematics.get_state(
                    self.current_joint_state.view(1, -1)
                )
                ee_pos = kin_state.ee_position.squeeze().cpu().numpy()
                target_pos = np.array(pos.position)
                dist = np.linalg.norm(ee_pos - target_pos)
                self.get_logger().debug(f"Near check (pose): dist={dist:.4f} tolerance={tolerance:.4f}")
                return dist < tolerance
            except Exception as e:
                self.get_logger().error(f"FK error in near check: {e}")
                return False
        
        return False
    
    def _skip_to_else_or_endif(self):
        """Skip instructions until a matching ELSE or ENDIF at the same nesting level.
        
        If ELSE is found, execution continues from the instruction after ELSE.
        If ENDIF is found, execution continues from the instruction after ENDIF.
        """
        depth = 1
        idx = self.current_instruction_idx + 1
        while idx < len(self.current_program):
            inst = self.current_program[idx]
            if inst.type == InstructionType.IF:
                depth += 1
            elif inst.type == InstructionType.ENDIF:
                depth -= 1
                if depth == 0:
                    # Skip past the endif
                    self.current_instruction_idx = idx + 1
                    return
            elif inst.type == InstructionType.ELSE and depth == 1:
                # Found matching else — continue executing from after else
                self.current_instruction_idx = idx + 1
                return
            idx += 1
        
        # No matching endif found — skip to end of program
        self.get_logger().warn("No matching endif found for if block")
        self.current_instruction_idx = len(self.current_program)
    
    def _skip_to_endif(self):
        """Skip instructions until a matching ENDIF at the same nesting level.
        
        Used when the IF condition was true and we hit the ELSE block —
        we need to skip all the else-branch instructions.
        """
        depth = 1
        idx = self.current_instruction_idx + 1
        while idx < len(self.current_program):
            inst = self.current_program[idx]
            if inst.type == InstructionType.IF:
                depth += 1
            elif inst.type == InstructionType.ENDIF:
                depth -= 1
                if depth == 0:
                    # Skip past the endif
                    self.current_instruction_idx = idx + 1
                    return
            idx += 1
        
        # No matching endif found
        self.get_logger().warn("No matching endif found for else block")
        self.current_instruction_idx = len(self.current_program)
    
    # =================================================================
    # Trajectory and robot control
    # =================================================================

    def publish_trajectory(self, traj_input):
        """Publish trajectory to robot controller."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        # Handle JointState object or tensor
        if hasattr(traj_input, 'position'):
            traj_tensor = traj_input.position
        else:
            traj_tensor = traj_input
        
        traj_np = traj_tensor.squeeze(0).cpu().numpy()
        steps = traj_np.shape[0]
        
        # Apply speed factor to trajectory timing
        effective_dt = self.dt / self.speed_factor
        
        for i in range(steps):
            point = JointTrajectoryPoint()
            point.positions = traj_np[i].tolist()
            time_sec = i * effective_dt
            point.time_from_start.sec = int(time_sec)
            point.time_from_start.nanosec = int((time_sec - int(time_sec)) * 1e9)
            traj_msg.points.append(point)
        
        self.traj_pub.publish(traj_msg)
    
    def estimate_trajectory_duration(self, traj_input) -> float:
        """Estimate trajectory duration in seconds."""
        if hasattr(traj_input, 'position'):
            traj_tensor = traj_input.position
        else:
            traj_tensor = traj_input
        
        steps = traj_tensor.shape[1] if len(traj_tensor.shape) > 2 else traj_tensor.shape[0]
        effective_dt = self.dt / self.speed_factor
        return steps * effective_dt
    
    def stop_robot(self):
        """Stop robot by sending current position as target with safe deceleration."""
        if self.current_joint_state is None:
            return
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.current_joint_state.cpu().numpy().tolist()
        point.velocities = [0.0] * 6
        # Use 2 seconds for safe stop motion (was 0.5 seconds which is dangerously fast)
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        
        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)
    
    def pause_execution(self, pause: bool):
        """Pause or resume execution."""
        if pause and self.state == ExecutorState.EXECUTING:
            self.state = ExecutorState.PAUSED
            self._stop_event.set()  # Interrupt any ongoing sleep
            self.stop_robot()
            self.publish_status("Paused")
        elif not pause and self.state == ExecutorState.PAUSED:
            self._stop_event.clear()
            self.state = ExecutorState.EXECUTING
            self.publish_status("Resumed")
    
    def stop_execution(self):
        """Stop program execution and hold robot at current position.
        
        Unlike the old behavior, this does NOT reset the instruction index.
        The robot holds its current position. Calling execute again (or pressing
        Next) resumes from wherever the program left off.
        """
        self._stop_event.set()  # Interrupt any ongoing sleep/wait
        
        if self.execution_timer:
            self.execution_timer.cancel()
            self.execution_timer = None
        
        self.state = ExecutorState.IDLE
        # Deliberately NOT resetting current_instruction_idx —
        # this preserves the program position so the user can resume.
        self.stop_robot()
        
        remaining = len(self.current_program) - self.current_instruction_idx if self.current_program else 0
        self.publish_status(f"Stopped at step {self.current_instruction_idx}/{len(self.current_program) if self.current_program else 0} - holding position")
        
        if self.presenter_enabled:
            self.get_logger().info(f"Program stopped at step {self.current_instruction_idx}. Press [Next] to resume, load a new program to restart.")
    
    def publish_status(self, status: str):
        """Publish executor status."""
        msg = String()
        msg.data = f"{self.state.name}: {status}"
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UR5ProgramExecutorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up keyboard listener
        if hasattr(node, 'keyboard_running'):
            node._stop_keyboard_listener()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
