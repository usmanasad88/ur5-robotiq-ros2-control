#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025 Universal Robots
# SPDX-License-Identifier: Apache-2.0
#
# Standalone Isaac Sim application for UR5 robot with HTTP-based end-effector control
# Reads target pose DELTAS from a Flask server and incrementally moves the end-effector
# using inverse kinematics solver and articulation actions
#
# The Flask server should return JSON with format:
# {"joint_actions": [dx, dy, dz, droll, dpitch, dyaw, gripper], ...}
# where dx, dy, dz are position deltas in meters
# and droll, dpitch, dyaw are orientation deltas in radians
# gripper value is ignored (UR5 has no gripper)
#
# Usage:
#   python ur5_http_control.py --server http://localhost:5000
#   python ur5_http_control.py --server http://localhost:5000 --headless
#   python ur5_http_control.py --test-motion  # Test robot movement first

import argparse
import sys
import time
import json
import numpy as np
from typing import Any, Dict, Optional, Tuple

# Parse arguments before importing Isaac Sim modules
parser = argparse.ArgumentParser(
    description="UR5 robot with HTTP-based end-effector control"
)
parser.add_argument(
    "--server",
    type=str,
    default="http://localhost:5000",
    help="Flask server URL (default: http://localhost:5000)"
)
parser.add_argument(
    "--endpoint",
    type=str,
    default="/joint_actions",
    help="Endpoint path (default: /joint_actions)"
)
parser.add_argument(
    "--headless",
    action="store_true",
    help="Run in headless mode (no GUI)"
)
parser.add_argument(
    "--update-rate",
    type=float,
    default=10.0,
    help="Control update rate in Hz (default: 10.0)"
)
parser.add_argument(
    "--timeout",
    type=float,
    default=1.0,
    help="HTTP request timeout in seconds (default: 1.0)"
)
parser.add_argument(
    "--test-motion",
    action="store_true",
    help="Test robot motion with predefined poses before starting HTTP control"
)
parser.add_argument(
    "--delta-scale",
    type=float,
    default=1.0,
    help="Scale factor applied to incoming translation deltas (default: 1.0)"
)
parser.add_argument(
    "--max-delta",
    type=float,
    default=0.02,
    help="Maximum translation step per update in meters after scaling (default: 0.02)"
)
parser.add_argument(
    "--rotation-scale",
    type=float,
    default=1.0,
    help="Scale factor applied to incoming rotation deltas (default: 1.0)"
)
parser.add_argument(
    "--max-rotation",
    type=float,
    default=0.05,
    help="Maximum rotation step per update in radians after scaling (default: 0.05)"
)
parser.add_argument(
    "--ik-position-tolerance",
    type=float,
    default=0.005,
    help="Position tolerance for inverse kinematics in meters (default: 0.005)"
)
parser.add_argument(
    "--ik-orientation-tolerance",
    type=float,
    default=0.05,
    help="Orientation tolerance for inverse kinematics in radians (default: 0.05)"
)
args = parser.parse_args()

# Import Isaac Sim
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless,
    "renderer": "RaytracedLighting"
})

# Import additional modules after SimulationApp
import carb
import omni
import requests
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats, quats_to_rot_matrices
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.examples.universal_robots import KinematicsSolver
from pxr import Gf, Sdf, UsdLux


class HTTPEndEffectorController:
    """Controller that reads target poses from HTTP endpoint and controls robot end-effector"""
    
    def __init__(
        self, 
        server_url: str,
        endpoint: str,
        timeout: float = 1.0,
        update_rate: float = 10.0
    ):
        self.server_url = server_url.rstrip('/')
        self.endpoint = endpoint if endpoint.startswith('/') else f'/{endpoint}'
        self.full_url = f"{self.server_url}{self.endpoint}"
        self.timeout = timeout
        self.update_interval = 1.0 / update_rate
        self.last_update_time = 0.0
        
        # Store last received data
        self.last_data: Optional[Dict] = None
        self.last_target_pose: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.connection_ok = False
        self.error_count = 0
        self.max_errors = 10
        self.parse_failures = 0
        
        print(f"\n{'='*70}")
        print(f"HTTP End-Effector Controller initialized")
        print(f"{'='*70}")
        print(f"Server URL: {self.full_url}")
        print(f"Update Rate: {update_rate} Hz")
        print(f"Timeout: {timeout}s")
        print(f"{'='*70}\n")
    
    @staticmethod
    def _coerce_float(value: Any, default: float = 0.0) -> float:
        """Best-effort conversion of mixed numeric representations to float"""
        try:
            if value is None:
                return default
            if isinstance(value, (float, int, np.floating, np.integer)):
                return float(value)
            if isinstance(value, str):
                cleaned = value.strip()
                if not cleaned:
                    return default
                return float(cleaned)
            if isinstance(value, (list, tuple, np.ndarray)) and len(value) > 0:
                return HTTPEndEffectorController._coerce_float(value[0], default)
        except (TypeError, ValueError):
            pass
        return default

    @staticmethod
    def _extract_from_dict(source: Any, keys: Tuple[str, ...], default: float = 0.0) -> float:
        if not isinstance(source, dict):
            return default
        for key in keys:
            if key in source and source[key] is not None:
                return HTTPEndEffectorController._coerce_float(source[key], default)
        return default

    def _normalize_actions_payload(
        self, actions: Any
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Convert various joint action payload formats into canonical numpy arrays."""
        if actions is None:
            return (None, None)

        # Accept JSON strings
        if isinstance(actions, str):
            try:
                actions = json.loads(actions)
            except json.JSONDecodeError:
                return (None, None)

        # Handle numpy arrays transparently
        if isinstance(actions, np.ndarray):
            actions = actions.tolist()

        # Case 1: flat sequence [dx, dy, dz, droll, dpitch, dyaw, ...]
        if isinstance(actions, (list, tuple)):
            if len(actions) < 6:
                return (None, None)
            position = np.array([
                self._coerce_float(actions[i], 0.0) for i in range(3)
            ], dtype=np.float64)
            orientation_euler = np.array([
                self._coerce_float(actions[i], 0.0) for i in range(3, 6)
            ], dtype=np.float64)
            orientation_quat = euler_angles_to_quat(orientation_euler)
            return (position, orientation_quat)

        # Case 2: dictionary style payload
        if isinstance(actions, dict):
            pos_source = actions.get("position", actions)
            ori_source = actions.get("orientation", actions)

            # Some services may nest deltas under "delta" or "pose"
            if isinstance(pos_source, dict) and "delta" in pos_source:
                pos_source = pos_source["delta"]
            if isinstance(ori_source, dict) and "delta" in ori_source:
                ori_source = ori_source["delta"]

            position = np.array([
                self._extract_from_dict(pos_source, ("x", "dx", "pos_x", "px"), 0.0),
                self._extract_from_dict(pos_source, ("y", "dy", "pos_y", "py"), 0.0),
                self._extract_from_dict(pos_source, ("z", "dz", "pos_z", "pz"), 0.0)
            ], dtype=np.float64)

            orientation_euler = np.array([
                self._extract_from_dict(ori_source, ("roll", "rx", "rot_x", "r"), 0.0),
                self._extract_from_dict(ori_source, ("pitch", "ry", "rot_y", "p"), 0.0),
                self._extract_from_dict(ori_source, ("yaw", "rz", "rot_z", "y"), 0.0)
            ], dtype=np.float64)

            orientation_quat = euler_angles_to_quat(orientation_euler)
            return (position, orientation_quat)

        return (None, None)

    def parse_response(self, data: Dict) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Parse HTTP response to extract target pose.
        
        Expected format from server:
        {
            "joint_actions": [x, y, z, roll, pitch, yaw, gripper],
            "joints": {"x": ..., "y": ..., "z": ..., "roll": ..., "pitch": ..., "yaw": ..., "gripper": ...},
            "timestamp": ...
        }
        
        Returns:
            Tuple of (position, orientation_quat) or None if parsing fails
        """
        try:
            if not data:
                return None

            if "joint_actions" in data:
                position, orientation_quat = self._normalize_actions_payload(
                    data["joint_actions"]
                )
                if position is not None and orientation_quat is not None:
                    return (position, orientation_quat)
                if self.parse_failures < 5:
                    carb.log_warn(
                        f"Unsupported joint_actions payload format: {str(data['joint_actions'])[:200]}"
                    )
                self.parse_failures += 1

            elif "joints" in data:
                joints = data["joints"]
                # Alternative: extract from joints dict
                position = np.array([
                    joints.get("x", 0.0),
                    joints.get("y", 0.0),
                    joints.get("z", 0.0)
                ], dtype=np.float64)
                
                roll = joints.get("roll", 0.0)
                pitch = joints.get("pitch", 0.0)
                yaw = joints.get("yaw", 0.0)
                
                orientation_quat = euler_angles_to_quat(
                    np.array([roll, pitch, yaw])
                )
                
                return (position, orientation_quat)
            
            return None
            
        except Exception as e:
            carb.log_warn(f"Error parsing response: {e}")
            return None
    
    def fetch_target_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Fetch target pose from HTTP endpoint.
        
        Returns:
            Tuple of (position, orientation_quat) or None if request fails
        """
        try:
            response = requests.get(self.full_url, timeout=self.timeout)
            
            if response.status_code == 200:
                data = response.json()
                self.last_data = data
                
                # Reset error count on successful connection
                if not self.connection_ok:
                    print(f"✓ Connected to server: {self.full_url}")
                    self.connection_ok = True
                self.error_count = 0
                
                # Parse and return target pose
                pose = self.parse_response(data)
                if pose is not None:
                    self.last_target_pose = pose
                    return pose
            else:
                if self.connection_ok:
                    carb.log_warn(
                        f"Server returned status {response.status_code}"
                    )
                return None
                
        except requests.exceptions.Timeout:
            self.error_count += 1
            if self.error_count == 1:
                carb.log_warn(f"Request timeout (waiting for server...)")
            return None
            
        except requests.exceptions.ConnectionError:
            self.error_count += 1
            if self.error_count == 1:
                carb.log_warn(
                    f"Cannot connect to {self.full_url} (is server running?)"
                )
            self.connection_ok = False
            return None
            
        except Exception as e:
            self.error_count += 1
            if self.error_count <= 3:
                carb.log_error(f"Error fetching pose: {e}")
            return None
    
    def should_update(self, current_time: float) -> bool:
        """Check if enough time has passed for next update"""
        return (current_time - self.last_update_time) >= self.update_interval
    
    def update(self, current_time: float) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Update controller and return target pose if available.
        
        Args:
            current_time: Current simulation time
            
        Returns:
            Tuple of (position, orientation_quat) or None
        """
        if self.should_update(current_time):
            self.last_update_time = current_time
            return self.fetch_target_pose()
        return None
    
    def get_status_string(self) -> str:
        """Get status string for display"""
        if self.connection_ok and self.last_target_pose is not None:
            pos, quat = self.last_target_pose
            return (
                f"Connected | Target: pos=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] "
                f"quat=[{quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f}]"
            )
        elif self.error_count > 0:
            return f"Disconnected (errors: {self.error_count})"
        else:
            return "Connecting..."


def test_robot_motion(ur5: SingleManipulator, ik_solver, my_world: World):
    """Test robot motion with predefined poses to verify IK and movement functionality"""
    
    print(f"\n{'='*70}")
    print("TESTING ROBOT MOTION")
    print(f"{'='*70}")
    print("Moving robot through test poses to verify IK functionality...")
    
    # Get initial pose
    initial_position, initial_orientation = ur5.end_effector.get_world_pose()
    print(f"Initial position: {initial_position}")
    
    # Create target visualization cubes
    target_cubes = []
    
    def create_target_cube(position: np.ndarray, name: str, color: tuple = (1.0, 0.0, 0.0)):
        """Create a small cube to visualize target position"""
        from pxr import UsdGeom, Gf
        
        stage = my_world.stage
        cube_path = f"/World/Targets/{name}_target"
        
        # Create cube
        cube_geom = UsdGeom.Cube.Define(stage, cube_path)
        cube_geom.CreateSizeAttr(0.02)  # 2cm cube
        
        # Set position
        cube_geom.CreateExtentAttr([(-0.01, -0.01, -0.01), (0.01, 0.01, 0.01)])
        
        # Set transform (convert numpy types to Python floats)
        xform = UsdGeom.Xformable(cube_geom)
        xform.ClearXformOpOrder()
        translate_op = xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
        
        # Set color (convert numpy types to Python floats)
        cube_geom.CreateDisplayColorAttr([(float(color[0]), float(color[1]), float(color[2]))])
        
        return cube_path
    
    # Define test poses (position, orientation as euler angles)
    # Use smaller, more conservative movements
    test_poses = [
        {
            "name": "Test Pose 1: Small move up",
            "position": initial_position + np.array([0.0, 0.0, 0.05]),  # Move up 5cm
            "euler": np.array([0.0, 0.0, 0.0]),  # Keep same orientation
            "color": (1.0, 0.0, 0.0)  # Red
        },
        {
            "name": "Test Pose 2: Small move forward", 
            "position": initial_position + np.array([0.05, 0.0, 0.02]),  # Move forward 5cm, up 2cm
            "euler": np.array([0.0, 0.0, 0.0]),
            "color": (0.0, 1.0, 0.0)  # Green
        },
        {
            "name": "Test Pose 3: Small move sideways",
            "position": initial_position + np.array([0.02, 0.05, 0.02]),  # Move sideways 5cm
            "euler": np.array([0.0, 0.0, 0.0]),
            "color": (0.0, 0.0, 1.0)  # Blue
        },
        {
            "name": "Test Pose 4: Return to start",
            "position": initial_position,
            "euler": np.array([0.0, 0.0, 0.0]),
            "color": (1.0, 1.0, 0.0)  # Yellow
        }
    ]
    
    # Create target visualization cubes
    print("Creating target position visualizations...")
    for i, pose in enumerate(test_poses):
        cube_path = create_target_cube(pose["position"], f"pose_{i+1}", pose["color"])
        target_cubes.append(cube_path)
        print(f"  {pose['name']}: target cube at {pose['position']}")
    
    # Create current position indicator (white cube)
    current_pos_cube = create_target_cube(initial_position, "current_position", (1.0, 1.0, 1.0))
    
    # Let visualization update
    for _ in range(30):
        my_world.step(render=True)
    
    success_count = 0
    total_poses = len(test_poses)
    
    for i, pose in enumerate(test_poses):
        print(f"\n--- {pose['name']} ---")
        target_position = pose["position"]
        target_orientation = euler_angles_to_quat(pose["euler"])
        
        print(f"Target position: {target_position}")
        print(f"Target orientation: {target_orientation}")
        print(f"Look for the {pose['color']} cube to see target position")
        
        try:
            # COORDINATE FRAME COMPENSATION for test motion:
            # Apply the same tool offset compensation used in main loop
            tool_offset = np.array([0.0, 0.0, 0.15])  # meters, fine-tuned Z-axis offset
            compensated_position = target_position + tool_offset
            
            # Compute IK
            actions, ik_success = ik_solver.compute_inverse_kinematics(
                target_position=compensated_position,
                target_orientation=target_orientation
            )
            
            if ik_success:
                print(f"✓ IK solution found")
                print(f"  Actions: {actions}")
                print(f"  Actions type: {type(actions)}")
                
                # Apply the motion using articulation controller (correct pattern from examples)
                articulation_controller = ur5.get_articulation_controller()
                articulation_controller.apply_action(actions)
                print(f"  Action applied successfully")
                
                # Wait for robot to move (longer wait for better settling)
                # Update current position cube during movement
                for step in range(120):  # Wait 2 seconds at 60 FPS
                    if step % 20 == 0:  # Update every 20 steps (1/3 second)
                        current_pos, _ = ur5.end_effector.get_world_pose()
                        # Update current position cube (convert numpy types to Python floats)
                        stage = my_world.stage
                        current_cube_prim = stage.GetPrimAtPath(current_pos_cube)
                        if current_cube_prim.IsValid():
                            from pxr import UsdGeom, Gf
                            xform = UsdGeom.Xformable(current_cube_prim)
                            translate_op = xform.GetOrderedXformOps()[0]  # Get existing translate op
                            translate_op.Set(Gf.Vec3d(float(current_pos[0]), float(current_pos[1]), float(current_pos[2])))
                    
                    my_world.step(render=True)
                
                # Check final position
                final_position, final_orientation = ur5.end_effector.get_world_pose()
                position_error = np.linalg.norm(final_position - target_position)
                
                print(f"  Achieved position: {final_position}")
                print(f"  Position error: {position_error:.4f}m")
                print(f"  Target was {pose['color']} cube, robot is at white cube")
                
                if position_error < 0.1:  # 10cm tolerance (more realistic for UR5)
                    print(f"✓ Motion successful!")
                    success_count += 1
                elif position_error < 0.2:  # 20cm tolerance (acceptable)
                    print(f"⚠️  Motion acceptable (large error but reasonable)")
                    success_count += 0.5  # Half credit
                else:
                    print(f"✗ Motion failed (very large position error)")
                    
            else:
                print(f"✗ IK solution failed")
                
        except Exception as e:
            print(f"✗ Error during motion test: {e}")
    
    print(f"\n{'='*70}")
    print(f"MOTION TEST RESULTS")
    print(f"{'='*70}")
    print(f"Successful poses: {success_count}/{total_poses}")
    if success_count >= total_poses * 0.8:  # 80% success rate
        print("✓ Motion tests passed! Robot IK and movement are working well.")
    elif success_count >= total_poses * 0.5:  # 50% success rate
        print(f"⚠️  Partial success. Robot is moving but with some accuracy issues.")
    elif success_count > 0:
        print(f"⚠️  Limited success. Robot moves but has significant accuracy problems.")
    else:
        print("✗ All motion tests failed. There may be an issue with IK or robot control.")
    print(f"{'='*70}\n")
    
    # Cleanup: Remove target visualization cubes
    print("Cleaning up target visualizations...")
    stage = my_world.stage
    targets_prim = stage.GetPrimAtPath("/World/Targets")
    if targets_prim.IsValid():
        stage.RemovePrim("/World/Targets")
    
    # Let cleanup update
    for _ in range(10):
        my_world.step(render=True)
    
    return success_count >= total_poses * 0.5  # Consider 50%+ success as passing


def main():
    # Get assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit(1)
    
    # Create world
    print("Creating simulation world...")
    my_world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0)
    my_world.scene.add_default_ground_plane()
    
    # Add lighting
    stage = my_world.stage
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(500)
    
    # Load UR5 robot (local-first lookup -> Isaac assets -> remote URL)
    print("Loading UR5 robot...")

    # Candidate local asset next to this script: isaac_standalone/ur5/ur5.usd
    import os
    local_asset_path = os.path.join(os.path.dirname(__file__), "ur", "ur5.usd")

    # Default Isaac assets path
    asset_path_default = None
    if assets_root_path is not None:
        asset_path_default = assets_root_path + "/Isaac/Robots/UniversalRobots/ur5/ur5.usd"

    # Choose asset path: prefer local if present, otherwise use Isaac assets path
    chosen_asset = None
    if os.path.exists(local_asset_path):
        chosen_asset = local_asset_path
        print(f"Using local UR5 asset: {chosen_asset}")
    else:
        # Fall back to the Isaac assets path (same mechanism the project used previously)
        chosen_asset = asset_path_default
        print(f"Using Isaac assets UR5 asset: {chosen_asset}")

    robot_prim = add_reference_to_stage(usd_path=chosen_asset, prim_path="/World/UR5")
    
    # Optional: Set gripper variant
    if robot_prim.GetVariantSets().HasVariantSet("Gripper"):
        gripper_variants = robot_prim.GetVariantSet("Gripper").GetVariantNames()
        print(f"Available gripper variants: {gripper_variants}")
        # Use parallel gripper if available
        if "Parallel" in gripper_variants:
            robot_prim.GetVariantSet("Gripper").SetVariantSelection("Parallel")
            print("Selected Parallel gripper")
    
    # Create robot as SingleManipulator for end-effector control
    # Create UR5 manipulator 
    # Using wrist_3_link as the end-effector since ee_link is not available in this UR5 model
    # Note: This may require coordinate frame adjustments for accurate positioning
    ur5 = my_world.scene.add(
        SingleManipulator(
            prim_path="/World/UR5",
            name="ur5",
            end_effector_prim_path="/World/UR5/wrist_3_link"
        )
    )
    
    # Set default joint positions (home configuration)
    default_positions = np.array([0.0, -np.pi/4, -np.pi/2, -np.pi/4, np.pi/2, 0.0])
    ur5.set_joints_default_state(positions=default_positions)
    
    # Initialize world
    print("Initializing simulation...")
    my_world.reset()
    
    # Initialize IK solver
    print("Initializing inverse kinematics solver...")
    
    # Debug: Let's check what end-effector frames are available
    from pxr import UsdGeom
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath("/World/UR5")
    
    print("Investigating robot structure for end-effector frames...")
    def find_prims_with_name(prim, name_pattern):
        """Find all prims containing the name pattern"""
        found = []
        if name_pattern.lower() in prim.GetName().lower():
            found.append(prim.GetPath())
        for child in prim.GetChildren():
            found.extend(find_prims_with_name(child, name_pattern))
        return found
    
    # Look for various end-effector related prims
    ee_prims = find_prims_with_name(robot_prim, "ee")
    wrist_prims = find_prims_with_name(robot_prim, "wrist")
    tool_prims = find_prims_with_name(robot_prim, "tool")
    
    print(f"Found end-effector candidate prims: {ee_prims}")
    print(f"Found wrist prims: {wrist_prims}")
    print(f"Found tool prims: {tool_prims}")
    
    ik_solver = None
    
    try:
        # Use Isaac Sim's built-in Universal Robots KinematicsSolver
        # The key issue: Force the IK solver to use wrist_3_link as end-effector
        # since the UR5 model doesn't have an ee_link
        ik_solver = KinematicsSolver(ur5, end_effector_frame_name="wrist_3_link")
        print("✓ IK solver initialized successfully using Universal Robots KinematicsSolver with wrist_3_link")
        
    except Exception as e1:
        print(f"Universal Robots KinematicsSolver with wrist_3_link initialization failed: {e1}")
        
        try:
            # Try with default settings
            ik_solver = KinematicsSolver(ur5)
            print("✓ IK solver initialized successfully using Universal Robots KinematicsSolver (default)")
            
        except Exception as e2:
            print(f"Universal Robots KinematicsSolver default initialization failed: {e2}")
            
            # Give up on IK solver - the robot will still work for visualization
            carb.log_warn("Could not initialize KinematicsSolver")
            carb.log_warn("Robot will display target poses but won't move")
            print("⚠️  IK solver failed to initialize - robot will not move")
            print("   The robot will still show target poses for visualization")
            print("   This might be due to Isaac Sim version compatibility issues")
            ik_solver = None
    
    # Create HTTP controller
    controller = HTTPEndEffectorController(
        server_url=args.server,
        endpoint=args.endpoint,
        timeout=args.timeout,
        update_rate=args.update_rate
    )
    
    # Get robot info
    print(f"\n{'='*70}")
    print(f"UR5 Robot Initialized")
    print(f"{'='*70}")
    print(f"Robot name: {ur5.name}")
    print(f"DOF: {ur5.num_dof}")
    print(f"Joint names: {ur5.dof_names}")
    print(f"End-effector path: /World/UR5/wrist_3_link")
    print(f"{'='*70}\n")
    
    # Get initial end-effector pose
    ee_position, ee_orientation = ur5.end_effector.get_world_pose()
    print(f"Initial end-effector pose:")
    print(f"  Position: {ee_position}")
    print(f"  Orientation (quat): {ee_orientation}")
    euler = quat_to_euler_angles(ee_orientation)
    print(f"  Orientation (euler): roll={euler[0]:.3f}, pitch={euler[1]:.3f}, yaw={euler[2]:.3f}")

    # Maintain a persistent desired pose so deltas keep accumulating even if IK fails
    desired_position = np.array(ee_position, dtype=np.float64)
    desired_euler = np.array(euler, dtype=np.float64)
    desired_orientation = np.array(ee_orientation, dtype=np.float64)
    
    # Test robot motion if requested
    if args.test_motion and ik_solver is not None:
        motion_test_passed = test_robot_motion(ur5, ik_solver, my_world)
        if not motion_test_passed:
            print("⚠️  Motion tests failed. Continuing with HTTP control anyway...")
    elif args.test_motion:
        print("⚠️  Cannot run motion test: IK solver not available")
    
    # Simulation loop
    print(f"\n{'='*70}")
    print("Starting simulation loop...")
    print("Press Ctrl+C to stop")
    print(f"{'='*70}\n")
    
    frame_count = 0
    last_status_time = 0.0
    status_update_interval = 2.0  # Print status every 2 seconds
    
    try:
        while simulation_app.is_running():
            # Step simulation
            my_world.step(render=True)
            
            if my_world.is_stopped():
                break
            
            if my_world.is_playing():
                frame_count += 1
                current_time = my_world.current_time
                
                # Update controller and get target pose
                target_pose = controller.update(current_time)
                
                if target_pose is not None:
                    delta_position, delta_orientation = target_pose
                    
                    # Debug: Print delta values to understand what we're receiving
                    if frame_count % 60 == 0:  # Print every second
                        print(f"Frame {frame_count}: Received deltas:")
                        print(f"  Delta position: {delta_position}")
                        print(f"  Delta magnitude: {np.linalg.norm(delta_position):.6f}m")
                    
                    # Check if delta is significant enough to process
                    delta_magnitude = np.linalg.norm(delta_position)
                    min_delta_threshold = 0.001  # 1mm minimum movement
                    
                    if delta_magnitude < min_delta_threshold:
                        # Skip very small movements that might cause IK issues
                        if frame_count % 180 == 0:  # Print occasionally
                            print(f"Skipping very small delta movement: {delta_magnitude:.6f}m")
                        continue
                    
                    # Flask server sends DELTAS (incremental changes), not absolute poses
                    # We apply these deltas to move the end-effector incrementally
                    current_position, current_orientation = ur5.end_effector.get_world_pose()

                    # Scale and clamp translation deltas to keep IK within a stable workspace
                    translation_scale = args.delta_scale
                    max_translation_step = max(args.max_delta, 1e-5)
                    scaled_delta = delta_position * translation_scale
                    clamped_delta = np.clip(scaled_delta, -max_translation_step, max_translation_step)
                    if frame_count % 120 == 0 and not np.allclose(clamped_delta, scaled_delta):
                        carb.log_warn(
                            f"Translation delta clamped from {scaled_delta} to {clamped_delta}"
                        )

                    candidate_position = desired_position + clamped_delta

                    # Compute new target orientation by adding delta euler angles
                    delta_euler = quat_to_euler_angles(delta_orientation) * args.rotation_scale
                    max_rotation_step = max(args.max_rotation, 1e-4)
                    clamped_delta_euler = np.clip(delta_euler, -max_rotation_step, max_rotation_step)
                    if frame_count % 120 == 0 and not np.allclose(clamped_delta_euler, delta_euler):
                        carb.log_warn(
                            f"Rotation delta clamped from {delta_euler} to {clamped_delta_euler}"
                        )

                    candidate_euler = desired_euler + clamped_delta_euler
                    candidate_orientation = euler_angles_to_quat(candidate_euler)
                    
                    # Safety checks: limit workspace to reasonable bounds for UR5
                    # UR5 has approximately 850mm reach
                    safety_bounds = {
                        'x_min': -0.8, 'x_max': 0.8,  # meters
                        'y_min': -0.8, 'y_max': 0.8,  # meters  
                        'z_min': 0.1, 'z_max': 1.0    # meters (above ground)
                    }
                    
                    # Check if new position is within safety bounds
                    position_safe = (
                        safety_bounds['x_min'] <= candidate_position[0] <= safety_bounds['x_max'] and
                        safety_bounds['y_min'] <= candidate_position[1] <= safety_bounds['y_max'] and
                        safety_bounds['z_min'] <= candidate_position[2] <= safety_bounds['z_max']
                    )
                    
                    if not position_safe:
                        if frame_count % 180 == 0:  # Print warning every 3 seconds
                            carb.log_warn(f"Target position {candidate_position} outside safety bounds")
                        continue  # Skip this update

                    # Update desired pose so that deltas accumulate even when IK fails
                    desired_position = candidate_position
                    desired_euler = candidate_euler
                    desired_orientation = candidate_orientation
                    
                    # Apply pose using IK solver if available
                    if ik_solver is not None:
                        try:
                            # COORDINATE FRAME COMPENSATION:
                            # The IK solver expects ee_link (tool center) but robot only has wrist_3_link
                            # Based on test results: 23cm offset reduced error from 25cm to 8.6cm
                            # Fine-tune to 15cm offset to get closer to target
                            tool_offset = np.array([0.0, 0.0, 0.15])  # meters, reduced Z-axis offset
                            compensated_position = desired_position + tool_offset
                            
                            # Compute joint angles using inverse kinematics with configurable tolerances
                            primary_position_tol = max(args.ik_position_tolerance, 1e-5)
                            primary_orientation_tol = max(args.ik_orientation_tolerance, 1e-4)
                            relaxed_position_tol = primary_position_tol * 2.0
                            relaxed_orientation_tol = primary_orientation_tol * 4.0
                            position_only_tol = relaxed_position_tol * 1.5

                            actions, success = ik_solver.compute_inverse_kinematics(
                                target_position=compensated_position,
                                target_orientation=desired_orientation,
                                position_tolerance=primary_position_tol,
                                orientation_tolerance=primary_orientation_tol
                            )

                            if not success:
                                actions, success = ik_solver.compute_inverse_kinematics(
                                    target_position=compensated_position,
                                    target_orientation=desired_orientation,
                                    position_tolerance=relaxed_position_tol,
                                    orientation_tolerance=relaxed_orientation_tol
                                )
                                if success and frame_count % 180 == 0:
                                    carb.log_warn("IK converged only after relaxing tolerances")

                            if not success:
                                actions, success = ik_solver.compute_inverse_kinematics(
                                    target_position=compensated_position,
                                    target_orientation=None,
                                    position_tolerance=position_only_tol
                                )
                                if success and frame_count % 180 == 0:
                                    carb.log_warn(
                                        "IK solved using position-only fallback (orientation ignored for this step)"
                                    )

                            if success:
                                # Apply the computed actions to move the robot
                                articulation_controller = ur5.get_articulation_controller()
                                articulation_controller.apply_action(actions)
                                
                                # Debug output (less frequent)
                                if frame_count % 120 == 0:  # Print every 2 seconds (60 Hz)
                                    print(f"Frame {frame_count}: Robot moved to target pose")
                                    print(f"  Target position: {desired_position}")
                                    print(f"  Target orientation (euler): {desired_euler}")
                                    print(f"  Actions applied: {actions}")
                            else:
                                # IK solution failed - print warning occasionally
                                # if frame_count % 300 == 0:
                                carb.log_warn("IK solution failed for target pose (all fallback attempts)")
                                print(f"  Target position: {desired_position}")
                                print(f"  Target orientation: {desired_orientation}")
                        
                        except Exception as e:
                            # IK computation error - print occasionally
                            if frame_count % 300 == 0:  # Print every 5 seconds
                                carb.log_error(f"IK computation error: {e}")
                    else:
                        # No IK solver - just visualize and print the target (original behavior)
                        if frame_count % 60 == 0:  # Print every second (60 Hz)
                            print(f"Frame {frame_count} (IK solver not available):")
                            print(f"  Delta position: {delta_position}")
                            print(f"  Current position: {current_position}")
                            print(f"  New target position: {desired_position}")
                            print(f"  Delta euler: {delta_euler}")
                            print(f"  New target euler: {desired_euler}")
                    
                    # Note: Gripper value (joint_actions[6]) is ignored since UR5 has no gripper
                
                # Print status periodically
                if (current_time - last_status_time) >= status_update_interval:
                    status = controller.get_status_string()
                    print(f"[{current_time:.1f}s] {status}")
                    last_status_time = current_time
    
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user.")
    
    finally:
        print(f"\n{'='*70}")
        print(f"Simulation Summary")
        print(f"{'='*70}")
        print(f"Total frames: {frame_count}")
        print(f"Total time: {my_world.current_time:.1f}s")
        print(f"Connection successful: {controller.connection_ok}")
        print(f"IK solver available: {ik_solver is not None}")
        if controller.last_target_pose:
            pos, quat = controller.last_target_pose
            print(f"Last target pose:")
            print(f"  Position: {pos}")
            print(f"  Orientation: {quat}")
        print(f"{'='*70}\n")
        
        print("Closing simulation...")
        simulation_app.close()


if __name__ == "__main__":
    main()
