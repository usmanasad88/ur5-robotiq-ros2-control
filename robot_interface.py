import time
import math
from typing import List, Optional
import subprocess
import sys
import os
import numpy as np

# Conditional import to allow running this verification script without ROS installed
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from std_msgs.msg import Float64, String
    from sensor_msgs.msg import JointState
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# Try to import curobo for motion generation
try:
    import torch
    # Add curobo to path if not installed
    sys.path.append('/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/curobo/src')
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose as CuroboPose
    from curobo.types.robot import JointState as CuroboJointState
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
    from curobo.util_file import load_yaml
    CUROBO_AVAILABLE = True
except ImportError:
    CUROBO_AVAILABLE = False
    print("Warning: cuRobo not available. Cartesian control will be disabled.")

class RobotClient:
    """
    The abstraction layer.
    The upper-level logic calls these methods.
    These methods translate the logic into ROS 2 Actions/Services.
    """
    def __init__(self, sim_mode=True):
        self.sim_mode = sim_mode
        self.node = None
        self.motion_gen = None
        
        if not self.sim_mode:
            if not ROS_AVAILABLE:
                raise ImportError("Cannot turn off sim_mode: ROS 2 (rclpy) is not installed.")
            
            # Initialize ROS Node
            rclpy.init()
            self.node = rclpy.create_node('proactive_agent_interface')
            
            # --- SETUP ROS CLIENTS ---
            # Publisher for joint trajectory (for robot movement)
            self.traj_pub = self.node.create_publisher(
                JointTrajectory,
                '/scaled_joint_trajectory_controller/joint_trajectory',
                10
            )
            
            # Publisher for gripper visualization (fake hardware)
            self.gripper_viz_pub = self.node.create_publisher(
                Float64,
                '/gripper_position_command',
                10
            )
            
            # Publisher for simple gripper command (logging)
            self.gripper_pub = self.node.create_publisher(
                String,
                '/gripper_command_simple',
                10
            )
            
            # Subscriber for joint states
            self.joint_state_sub = self.node.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                10
            )
            
            self.current_joint_positions = None
            self.joint_names = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ]
            
            # Initialize cuRobo if available
            if CUROBO_AVAILABLE:
                self._init_curobo()
            
            print("ROS Node Initialized. Connected to drivers.")
            
            # Spin in a separate thread to handle callbacks
            import threading
            self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
            self.spin_thread.start()
            
            # Wait for joint states
            print("Waiting for joint states...")
            start_time = time.time()
            while self.current_joint_positions is None:
                if time.time() - start_time > 5.0:
                    print("Warning: No joint states received yet.")
                    break
                time.sleep(0.1)
            print("Joint states received.")
            
        else:
            print("[SIM] Robot Interface initialized in Simulation Mode.")

    def _init_curobo(self):
        """Initialize cuRobo motion generator."""
        try:
            # Hardcoded paths for now - should be configurable
            robot_config_file = "/home/mani/Repos/ur_ws/install/ur5_curobo_control/share/ur5_curobo_control/config/ur5_existing.yml"
            world_config_file = "/home/mani/Repos/ur_ws/install/ur5_curobo_control/share/ur5_curobo_control/config/collision_base.yml"
            
            if not os.path.exists(robot_config_file):
                print(f"Error: Robot config not found at {robot_config_file}")
                return

            self.tensor_args = TensorDeviceType()
            
            # Load YAML manually to fix paths dynamically (same as in node)
            robot_cfg_dict = load_yaml(robot_config_file)
            
            # Fix URDF path if needed
            if 'robot_cfg' in robot_cfg_dict:
                kinematics = robot_cfg_dict['robot_cfg'].get('kinematics', {})
                if 'urdf_path' in kinematics:
                    # Ensure it points to the correct location
                    urdf_path = "/home/mani/Repos/ur_ws/install/ur5_curobo_control/share/ur5_curobo_control/config/ur5.urdf"
                    kinematics['urdf_path'] = urdf_path
            
            self.dt = 0.05
            self.motion_gen_config = MotionGenConfig.load_from_robot_config(
                robot_cfg_dict,
                world_config_file,
                self.tensor_args,
                trajopt_tsteps=32,
                use_cuda_graph=False,
                interpolation_dt=self.dt,
            )
            self.motion_gen = MotionGen(self.motion_gen_config)
            print("cuRobo MotionGen initialized successfully.")
            
        except Exception as e:
            print(f"Failed to initialize cuRobo: {e}")
            self.motion_gen = None

    def joint_state_callback(self, msg):
        """Callback to update current joint positions."""
        positions = []
        try:
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    positions.append(msg.position[idx])
            
            if len(positions) == len(self.joint_names):
                self.current_joint_positions = positions
        except ValueError:
            pass

    def move_to_pose(self, x: float, y: float, z: float, roll: float=0, pitch: float=0, yaw: float=0) -> bool:
        """
        Commands the robot to move to a Cartesian pose.
        Returns: True if successful, False if blocked/failed.
        """
        if self.sim_mode:
            print(f"COMMAND: MOVING to X={x}, Y={y}, Z={z} (Yaw={yaw})...")
            time.sleep(1.0) # Simulate travel time
            print("STATUS:  Position Reached.")
            return True
        else:
            if not CUROBO_AVAILABLE or self.motion_gen is None:
                print("[REAL] Error: Cartesian control requires cuRobo.")
                return False
                
            if self.current_joint_positions is None:
                print("[REAL] Error: No joint state available.")
                return False

            # Convert RPY to Quaternion (simple approximation for now)
            # For full implementation, use scipy.spatial.transform.Rotation
            # Here we assume simple top-down or side approach
            # Default to pointing down [0, 1, 0, 0] if no rotation specified
            # This is a placeholder - proper RPY to Quat conversion needed
            
            # Using a fixed orientation for picking (pointing down)
            # [w, x, y, z]
            target_quat = [0.0, 1.0, 0.0, 0.0] 
            
            # Create cuRobo Pose
            target_pose = CuroboPose(
                position=torch.tensor([x, y, z], device=self.tensor_args.device, dtype=self.tensor_args.dtype),
                quaternion=torch.tensor(target_quat, device=self.tensor_args.device, dtype=self.tensor_args.dtype)
            )
            
            # Create Start State
            current_joints_tensor = torch.tensor(self.current_joint_positions, device=self.tensor_args.device, dtype=self.tensor_args.dtype)
            start_state = CuroboJointState.from_position(current_joints_tensor.view(1, -1))
            
            # Plan motion
            print(f"[REAL] Planning motion to {x}, {y}, {z}...")
            result = self.motion_gen.plan_single(
                start_state, target_pose,
                MotionGenPlanConfig(enable_graph=False, timeout=2.0)
            )
            
            if result.success.item():
                print("[REAL] Motion plan successful, executing...")
                traj = result.interpolated_plan
                self._publish_trajectory(traj)
                
                # Wait for trajectory to complete
                steps = traj.position.shape[1]
                duration = steps * self.dt
                time.sleep(duration + 0.5)
                return True
            else:
                print(f"[REAL] Motion planning failed: {result.status}")
                return False

    def _publish_trajectory(self, traj_input):
        """Publish trajectory to robot controller."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        traj_tensor = traj_input.position
        traj_np = traj_tensor.squeeze(0).cpu().numpy()
        steps = traj_np.shape[0]
        
        # Speed factor (hardcoded for now)
        speed_factor = 0.5
        effective_dt = self.dt / speed_factor
        
        for i in range(steps):
            point = JointTrajectoryPoint()
            point.positions = traj_np[i].tolist()
            time_sec = i * effective_dt
            point.time_from_start.sec = int(time_sec)
            point.time_from_start.nanosec = int((time_sec - int(time_sec)) * 1e9)
            traj_msg.points.append(point)
        
        self.traj_pub.publish(traj_msg)

    def open_gripper(self) -> bool:
        if self.sim_mode:
            print("COMMAND: OPENING Gripper...")
            time.sleep(0.5)
            print("STATUS:  Gripper Open.")
            return True
        else:
            return self._execute_gripper(0.0)

    def close_gripper(self) -> bool:
        if self.sim_mode:
            print("COMMAND: CLOSING Gripper...")
            time.sleep(0.5)
            print("STATUS:  Gripper Closed.")
            return True
        else:
            return self._execute_gripper(1.0)

    def _execute_gripper(self, position: float) -> bool:
        """Execute gripper command on real hardware or fake hardware viz."""
        # Robotiq 2F-85 has 85mm stroke. 0.0 = open, 1.0 = closed (normalized)
        robotiq_position = position * 0.085
        
        # Publish to visualization topic (for RViz/Fake Hardware)
        viz_msg = Float64()
        viz_msg.data = position
        self.gripper_viz_pub.publish(viz_msg)
        
        # Publish to simple topic for logging
        msg = String()
        msg.data = f"position:{position}"
        self.gripper_pub.publish(msg)
        
        print(f"[REAL] Gripper command: position={position} (robotiq_pos={robotiq_position:.4f})")
        
        # Try to send actual action command (if running on real robot)
        # We use subprocess to call the CLI action client as a simple way to trigger it
        cmd = (
            f"ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command "
            f"robotiq_2f_urcap_adapter/GripperCommand "
            f"'{{ command: {{ position: {robotiq_position}, max_effort: 70, max_speed: 0.05 }}}}'"
        )
        
        try:
            # We run this with a short timeout and don't block too long
            # Note: This assumes the action server is running. If using fake hardware, this might fail/timeout
            # but the visualization publish above will still work.
            print(f"[REAL] Sending action goal...")
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print("Gripper action sent successfully.")
            else:
                print(f"Gripper action warning (might be fake hardware): {result.stderr}")
        except subprocess.TimeoutExpired:
            print("Gripper action timed out (likely fake hardware or no server).")
        except Exception as e:
            print(f"Error executing gripper action: {e}")
            
        time.sleep(1.0)
        return True

    def wait(self, seconds: float):
        if self.sim_mode:
            print(f"COMMAND: WAITING for {seconds}s...")
            time.sleep(seconds)
        else:
            print(f"[REAL] Waiting for {seconds}s...")
            time.sleep(seconds)

    def shutdown(self):
        if not self.sim_mode:
            self.node.destroy_node()
            rclpy.shutdown()
        print("Interface shut down.")

if __name__ == "__main__":
    print("--- TEST: STARTING ROBOT INTERFACE MODULE ---")
    
    # 1. Instantiate in Real/ROS Mode
    robot = RobotClient(sim_mode=False)
    
    # 2. Define a Dummy Task (Pick up an object at [0.5, 0.2, 0.1])
    target_pose = [0.5, 0.2, 0.1]
    drop_zone = [0.5, -0.2, 0.2]
    
    # 3. Execute Sequence
    try:
        # Move to Pre-Grasp
        robot.move_to_pose(target_pose[0], target_pose[1], target_pose[2] + 0.1)
        
        # Open Gripper
        robot.open_gripper()
        
        # Move to Grasp
        robot.move_to_pose(target_pose[0], target_pose[1], target_pose[2])
        
        # Close Gripper
        robot.close_gripper()
        
        # Move Up
        robot.move_to_pose(target_pose[0], target_pose[1], target_pose[2] + 0.1)
        
        # Move to Drop
        robot.move_to_pose(drop_zone[0], drop_zone[1], drop_zone[2])
        
        # Release
        robot.open_gripper()
        
        print("--- TEST: SEQUENCE COMPLETED SUCCESSFULLY ---")
        
    except Exception as e:
        print(f"--- TEST FAILED: {e} ---")
    finally:
        robot.shutdown()
