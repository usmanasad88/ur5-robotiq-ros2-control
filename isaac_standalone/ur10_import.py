#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025 Universal Robots
# SPDX-License-Identifier: Apache-2.0
#
# Standalone Isaac Sim application to load and simulate UR5 or UR10 robot
# Usage: 
#   python ur10_import.py --robot ur10
#   python ur10_import.py --robot ur5

import argparse
import sys

# Parse arguments before importing Isaac Sim modules
parser = argparse.ArgumentParser(description="Load and simulate Universal Robots UR5 or UR10")
parser.add_argument(
    "--robot", 
    type=str, 
    default="ur10", 
    choices=["ur5", "ur10"],
    help="Robot model to load (ur5 or ur10)"
)
parser.add_argument(
    "--headless", 
    action="store_true", 
    help="Run in headless mode (no GUI)"
)
parser.add_argument(
    "--test", 
    action="store_true", 
    help="Run in test mode (shorter simulation)"
)
args = parser.parse_args()

# Import Isaac Sim and create the SimulationApp
from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless,
    "renderer": "RaytracedLighting"
})

# Import additional modules after SimulationApp is created
import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

def main():
    # Get assets root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit(1)
    
    # Create world
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()
    
    # Select robot model
    robot_name = args.robot.upper()
    print(f"Loading {robot_name} robot...")
    
    # Path to robot USD file
    # Isaac Sim provides both ur5 and ur10 models
    asset_path = assets_root_path + f"/Isaac/Robots/UniversalRobots/{args.robot}/{args.robot}.usd"
    
    # Add robot to stage
    robot_prim = add_reference_to_stage(usd_path=asset_path, prim_path=f"/World/{robot_name}")
    
    # Optional: Set gripper variant if available
    # Check available variants with robot_prim.GetVariantSets()
    variant_sets = robot_prim.GetVariantSets()
    if variant_sets.HasVariantSet("Gripper"):
        print("Available gripper variants:")
        gripper_variants = variant_sets.GetVariantSet("Gripper").GetVariantNames()
        print(f"  {gripper_variants}")
        # Example: Set a gripper variant (uncomment if needed)
        # robot_prim.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
    
    # Add robot to the scene
    robot = my_world.scene.add(
        Robot(
            prim_path=f"/World/{robot_name}",
            name=f"my_{args.robot}"
        )
    )
    
    # Set default joint positions (home position)
    # UR robots have 6 joints
    default_positions = np.array([0.0, -np.pi/4, -np.pi/2, -np.pi/4, np.pi/2, 0.0])
    robot.set_joints_default_state(positions=default_positions)
    
    # Reset the world to apply default states
    print("Resetting world...")
    my_world.reset()
    
    print(f"{robot_name} robot loaded successfully!")
    print(f"Joint names: {robot.dof_names}")
    print(f"Number of DOFs: {robot.num_dof}")
    print(f"Default joint positions: {default_positions}")
    
    # Simulation loop
    num_frames = 100 if args.test else 1000
    print(f"\nRunning simulation for {num_frames} frames...")
    print("Press Ctrl+C to stop or close the window.")
    
    frame_count = 0
    try:
        while simulation_app.is_running():
            # Step the simulation
            my_world.step(render=True)
            
            if my_world.is_stopped():
                break
            
            if my_world.is_playing():
                frame_count += 1
                
                # Print robot state every 100 frames
                if frame_count % 100 == 0:
                    joint_positions = robot.get_joint_positions()
                    print(f"Frame {frame_count}: Joint positions = {joint_positions}")
                
                # Optional: Add simple joint movement example
                # Uncomment to see the robot move
                # if frame_count == 200:
                #     target_positions = np.array([0.5, -1.0, -1.5, -1.0, 1.5, 0.5])
                #     robot.set_joint_positions(target_positions)
                #     print(f"Moving to target positions: {target_positions}")
                
                # Stop after specified frames in test mode
                if args.test and frame_count >= num_frames:
                    break
    
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    
    finally:
        print(f"\nSimulation completed. Total frames: {frame_count}")
        print("Closing simulation...")
        simulation_app.close()

if __name__ == "__main__":
    main()
