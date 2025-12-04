#!/home/mani/miniconda3/envs/ur5_python/bin/python
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

# Import the program parser
from ur5_curobo_control.program_parser import (
    ProgramParser, RobotInstruction, InstructionType
)


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
        
        # Initialize cuRobo
        self.tensor_args = TensorDeviceType()
        
        try:
            self.motion_gen_config = MotionGenConfig.load_from_robot_config(
                robot_config_file,
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
        
        # ROS Interfaces
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        
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
            # Start or restart program execution
            self.get_logger().info(f"Starting program: {self.current_program_name}")
            self.state = ExecutorState.EXECUTING
            self.current_instruction_idx = 0
            
            # Create execution timer if not exists
            if self.execution_timer is None:
                self.execution_timer = self.create_timer(0.1, self.execution_step)
            
            self.publish_status(f"Started: {self.current_program_name}")
            
        elif self.state == ExecutorState.PAUSED:
            # Resume execution
            self.get_logger().info("Resuming execution...")
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
            self.get_logger().info("SAFETY CLEARED: Ready to resume.")
    
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
        
        self.state = ExecutorState.EXECUTING
        self.current_instruction_idx = 0
        
        # Start execution timer
        self.execution_timer = self.create_timer(0.1, self.execution_step)
        
        response.success = True
        response.message = f"Started executing program: {self.current_program_name}"
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
    
    def execution_step(self):
        """Execute the next instruction in the program."""
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
        self.get_logger().info(f"Executing [{self.current_instruction_idx + 1}/{len(self.current_program)}]: {instruction.raw_line}")
        
        success = self.execute_instruction(instruction)
        
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
            
            # Wait for trajectory to complete (approximate)
            traj_duration = self.estimate_trajectory_duration(traj)
            time.sleep(traj_duration + 0.5)  # Add small buffer
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
        
        time.sleep(duration + 0.5)
        return True
    
    def execute_wait(self, instruction: RobotInstruction) -> bool:
        """Execute a wait instruction."""
        if instruction.wait_duration is None:
            return False
        
        self.get_logger().info(f"Waiting {instruction.wait_duration} seconds...")
        time.sleep(instruction.wait_duration)
        return True
    
    def execute_gripper(self, position: float) -> bool:
        """Execute gripper command.
        
        For real hardware: Uses ros2 action to send command to Robotiq gripper.
        For fake hardware: Just publishes to simple topic and logs.
        
        Args:
            position: Gripper position (0.0 = fully open, 1.0 = fully closed)
        """
        # Convert normalized position (0-1) to Robotiq position (0 = open, 0.085 = closed)
        # Robotiq 2F-85 has 85mm stroke
        robotiq_position = position * 0.085
        
        if self.use_fake_hardware:
            # Fake hardware - publish to visualization topic for RViz animation
            viz_msg = Float64()
            viz_msg.data = position  # 0.0 = open, 1.0 = closed
            self.gripper_viz_pub.publish(viz_msg)
            
            # Also publish to simple topic for logging
            msg = String()
            msg.data = f"position:{position}"
            self.gripper_pub.publish(msg)
            self.get_logger().info(f"[FAKE] Gripper command: position={position} (robotiq_pos={robotiq_position:.4f})")
            time.sleep(0.5)
        else:
            # Real hardware - use ros2 action send_goal command
            self.get_logger().info(f"[REAL] Sending gripper command: position={robotiq_position:.4f}")
            
            import subprocess
            cmd = (
                f"ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command "
                f"robotiq_2f_urcap_adapter/GripperCommand "
                f"'{{ command: {{ position: {robotiq_position}, max_effort: 70, max_speed: 0.05 }}}}'"
            )
            
            try:
                result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
                if result.returncode == 0:
                    self.get_logger().info("Gripper command executed successfully")
                else:
                    self.get_logger().warn(f"Gripper command returned non-zero: {result.stderr}")
            except subprocess.TimeoutExpired:
                self.get_logger().error("Gripper command timed out")
                return False
            except Exception as e:
                self.get_logger().error(f"Error executing gripper command: {e}")
                return False
        
        # Wait for gripper to complete movement
        time.sleep(1.0)
        return True
    
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
        """Stop robot by sending current position as target."""
        if self.current_joint_state is None:
            return
        
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.current_joint_state.cpu().numpy().tolist()
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000
        
        traj_msg.points.append(point)
        self.traj_pub.publish(traj_msg)
    
    def pause_execution(self, pause: bool):
        """Pause or resume execution."""
        if pause and self.state == ExecutorState.EXECUTING:
            self.state = ExecutorState.PAUSED
            self.stop_robot()
            self.publish_status("Paused")
        elif not pause and self.state == ExecutorState.PAUSED:
            self.state = ExecutorState.EXECUTING
            self.publish_status("Resumed")
    
    def stop_execution(self):
        """Stop program execution."""
        if self.execution_timer:
            self.execution_timer.cancel()
            self.execution_timer = None
        
        self.state = ExecutorState.IDLE
        self.current_instruction_idx = 0  # Reset to beginning for restart
        self.stop_robot()
        self.publish_status("Stopped - Press [Next] to restart")
        
        if self.presenter_enabled:
            self.get_logger().info("Program stopped. Press presenter [Next] button to restart.")
    
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
