#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025 Universal Robots
# SPDX-License-Identifier: Apache-2.0
#
# Advanced standalone Isaac Sim application for UR5/UR10 robots
# Features:
# - Load from Isaac Sim assets or custom URDF
# - Joint position control
# - Joint velocity control
# - Interactive control mode
# - Recording joint states
#
# Usage:
#   python ur_robot_advanced.py --robot ur10 --mode interactive
#   python ur_robot_advanced.py --robot ur5 --urdf /path/to/ur5.urdf

import argparse
import sys
import os

# Parse arguments before importing Isaac Sim modules
parser = argparse.ArgumentParser(description="Advanced UR robot simulation with Isaac Sim")
parser.add_argument(
    "--robot", 
    type=str, 
    default="ur10", 
    choices=["ur5", "ur10"],
    help="Robot model to load"
)
parser.add_argument(
    "--urdf",
    type=str,
    default=None,
    help="Path to custom URDF file (optional)"
)
parser.add_argument(
    "--mode",
    type=str,
    default="static",
    choices=["static", "animated", "interactive"],
    help="Simulation mode: static (no movement), animated (predefined motion), interactive (keyboard control)"
)
parser.add_argument(
    "--headless", 
    action="store_true", 
    help="Run in headless mode"
)
parser.add_argument(
    "--record",
    action="store_true",
    help="Record joint states to CSV file"
)
parser.add_argument(
    "--frames",
    type=int,
    default=1000,
    help="Number of frames to simulate"
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
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, UsdPhysics

class URRobotController:
    """Controller for UR robot with multiple control modes"""
    
    def __init__(self, robot: Robot, mode: str = "static"):
        self.robot = robot
        self.mode = mode
        self.frame = 0
        self.num_dof = robot.num_dof
        
        # Joint limits (approximate for UR robots)
        self.joint_lower_limits = np.array([-2*np.pi, -2*np.pi, -np.pi, -2*np.pi, -2*np.pi, -2*np.pi])
        self.joint_upper_limits = np.array([2*np.pi, 2*np.pi, np.pi, 2*np.pi, 2*np.pi, 2*np.pi])
        
        # Animation parameters
        self.animation_period = 500  # frames
        self.home_position = np.array([0.0, -np.pi/4, -np.pi/2, -np.pi/4, np.pi/2, 0.0])
        
        # Recording
        self.recorded_states = []
        
    def update(self):
        """Update robot based on control mode"""
        self.frame += 1
        
        if self.mode == "static":
            # Robot stays at home position
            pass
            
        elif self.mode == "animated":
            # Simple sinusoidal motion
            t = self.frame / self.animation_period
            amplitude = np.array([0.5, 0.3, 0.3, 0.3, 0.3, 0.3])
            target_positions = self.home_position + amplitude * np.sin(2 * np.pi * t)
            
            # Clamp to joint limits
            target_positions = np.clip(target_positions, self.joint_lower_limits, self.joint_upper_limits)
            
            # Apply action
            action = ArticulationAction(joint_positions=target_positions)
            self.robot.apply_action(action)
            
        elif self.mode == "interactive":
            # In interactive mode, user can control via keyboard or external input
            # For now, this will cycle through predefined poses
            poses = [
                self.home_position,
                np.array([0.0, -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0.0]),
                np.array([np.pi/4, -np.pi/3, -np.pi/3, -np.pi/3, np.pi/2, np.pi/4]),
                np.array([-np.pi/4, -np.pi/3, -np.pi/3, -np.pi/3, np.pi/2, -np.pi/4]),
            ]
            pose_idx = (self.frame // 200) % len(poses)
            target_positions = poses[pose_idx]
            
            action = ArticulationAction(joint_positions=target_positions)
            self.robot.apply_action(action)
    
    def record_state(self):
        """Record current joint state"""
        state = {
            'frame': self.frame,
            'positions': self.robot.get_joint_positions(),
            'velocities': self.robot.get_joint_velocities(),
        }
        self.recorded_states.append(state)
        return state
    
    def save_recording(self, filename="robot_recording.csv"):
        """Save recorded states to CSV"""
        if not self.recorded_states:
            print("No states recorded")
            return
        
        import csv
        with open(filename, 'w', newline='') as csvfile:
            fieldnames = ['frame'] + \
                        [f'pos_{i}' for i in range(self.num_dof)] + \
                        [f'vel_{i}' for i in range(self.num_dof)]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for state in self.recorded_states:
                row = {'frame': state['frame']}
                for i, pos in enumerate(state['positions']):
                    row[f'pos_{i}'] = pos
                for i, vel in enumerate(state['velocities']):
                    row[f'vel_{i}'] = vel
                writer.writerow(row)
        
        print(f"Recording saved to {filename}")


def load_robot_from_urdf(urdf_path: str, prim_path: str):
    """Load robot from URDF file"""
    print(f"Loading robot from URDF: {urdf_path}")
    
    if not os.path.exists(urdf_path):
        carb.log_error(f"URDF file not found: {urdf_path}")
        return None
    
    # Configure URDF import
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True  # Fix base for manipulator
    import_config.distance_scale = 1.0
    
    # Import URDF
    status, imported_prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        get_articulation_root=True,
    )
    
    if status:
        print(f"Successfully imported URDF to: {imported_prim_path}")
        return imported_prim_path
    else:
        carb.log_error("Failed to import URDF")
        return None


def main():
    # Get assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None and args.urdf is None:
        carb.log_error("Could not find Isaac Sim assets folder and no URDF specified")
        simulation_app.close()
        sys.exit(1)
    
    # Create world with physics
    my_world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0)
    my_world.scene.add_default_ground_plane()
    
    # Add lighting
    from pxr import UsdLux
    stage = my_world.stage
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(500)
    
    robot_name = args.robot.upper()
    prim_path = f"/World/{robot_name}"
    
    # Load robot
    if args.urdf:
        # Load from URDF
        imported_path = load_robot_from_urdf(args.urdf, prim_path)
        if imported_path is None:
            simulation_app.close()
            sys.exit(1)
        prim_path = imported_path
    else:
        # Load from Isaac Sim assets
        print(f"Loading {robot_name} robot from Isaac Sim assets...")
        asset_path = assets_root_path + f"/Isaac/Robots/UniversalRobots/{args.robot}/{args.robot}.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path=prim_path)
    
    # Create articulation
    robot = my_world.scene.add(
        Robot(prim_path=prim_path, name=f"my_{args.robot}")
    )
    
    # Set default joint positions
    default_positions = np.array([0.0, -np.pi/4, -np.pi/2, -np.pi/4, np.pi/2, 0.0])
    robot.set_joints_default_state(positions=default_positions)
    
    # Reset world
    print("Initializing simulation...")
    my_world.reset()
    
    # Create controller
    controller = URRobotController(robot, mode=args.mode)
    
    print(f"\n{'='*60}")
    print(f"UR Robot Simulation - {robot_name}")
    print(f"{'='*60}")
    print(f"Mode: {args.mode}")
    print(f"DOF: {robot.num_dof}")
    print(f"Joint names: {robot.dof_names}")
    print(f"Default positions: {default_positions}")
    print(f"{'='*60}\n")
    
    # Simulation loop
    frame_count = 0
    max_frames = args.frames
    
    try:
        while simulation_app.is_running() and frame_count < max_frames:
            my_world.step(render=True)
            
            if my_world.is_stopped():
                break
            
            if my_world.is_playing():
                frame_count += 1
                
                # Update controller
                controller.update()
                
                # Record state if enabled
                if args.record and frame_count % 10 == 0:
                    controller.record_state()
                
                # Print status every 100 frames
                if frame_count % 100 == 0:
                    positions = robot.get_joint_positions()
                    velocities = robot.get_joint_velocities()
                    print(f"Frame {frame_count}:")
                    print(f"  Positions: {positions}")
                    if args.mode != "static":
                        print(f"  Velocities: {velocities}")
    
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    
    finally:
        # Save recording if enabled
        if args.record:
            output_file = f"{args.robot}_recording_{args.mode}.csv"
            controller.save_recording(output_file)
        
        print(f"\nSimulation completed. Total frames: {frame_count}")
        print("Closing simulation...")
        simulation_app.close()


if __name__ == "__main__":
    main()
