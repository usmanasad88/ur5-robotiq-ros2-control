#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2025 Universal Robots
# SPDX-License-Identifier: Apache-2.0
#
# Load UR robot from ROS 2 workspace URDF
# This script demonstrates importing the UR robot description from your ROS 2 workspace

import argparse
import sys
import os

parser = argparse.ArgumentParser(description="Load UR robot from ROS 2 workspace URDF")
parser.add_argument(
    "--robot",
    type=str,
    default="ur5",
    choices=["ur3", "ur5", "ur10", "ur16"],
    help="Robot model"
)
parser.add_argument(
    "--workspace",
    type=str,
    default="/home/mani/Repos/ur_ws",
    help="ROS 2 workspace path"
)
parser.add_argument(
    "--headless",
    action="store_true",
    help="Run headless"
)
args = parser.parse_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": args.headless,
    "renderer": "RaytracedLighting"
})

import carb
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdLux, UsdPhysics


def find_ur_urdf(workspace_path: str, robot_model: str):
    """Find UR URDF in ROS 2 workspace"""
    
    # Possible locations
    possible_paths = [
        # Install space
        f"{workspace_path}/install/ur_description/share/ur_description/urdf/{robot_model}.urdf",
        f"{workspace_path}/install/ur_description/share/ur_description/urdf/{robot_model}/{robot_model}.urdf",
        # Source space (Universal_Robots_ROS2_Description)
        f"{workspace_path}/src/Universal_Robots_ROS2_Driver/ur_description/urdf/{robot_model}.urdf",
        f"{workspace_path}/src/Universal_Robots_ROS2_Driver/ur_description/urdf/{robot_model}/{robot_model}.urdf",
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            print(f"Found URDF at: {path}")
            return path
    
    print(f"Could not find URDF for {robot_model} in workspace: {workspace_path}")
    print("Searched paths:")
    for path in possible_paths:
        print(f"  - {path}")
    return None


def main():
    # Find URDF
    urdf_path = find_ur_urdf(args.workspace, args.robot)
    if urdf_path is None:
        print("\nHint: Make sure the workspace is built with:")
        print(f"  cd {args.workspace}")
        print("  colcon build --packages-select ur_description")
        simulation_app.close()
        sys.exit(1)
    
    # Create world
    my_world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0)
    stage = my_world.stage
    
    # Configure URDF import
    print("Configuring URDF import...")
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = True  # Fix base link for manipulator
    import_config.distance_scale = 1.0
    import_config.default_drive_type = "angular"  # Use angular drives for revolute joints
    import_config.default_position_drive_damping = 1000.0
    import_config.default_position_drive_stiffness = 10000.0
    
    # Import URDF
    print(f"Importing URDF for {args.robot.upper()}...")
    status, prim_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        get_articulation_root=True,
    )
    
    if not status:
        carb.log_error("Failed to import URDF")
        simulation_app.close()
        sys.exit(1)
    
    print(f"Successfully imported robot to: {prim_path}")
    
    # Setup physics scene
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    
    # Configure PhysX
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
    physxSceneAPI.CreateEnableCCDAttr(True)
    physxSceneAPI.CreateEnableStabilizationAttr(True)
    physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
    physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
    physxSceneAPI.CreateSolverTypeAttr("TGS")
    
    # Add ground plane
    PhysicsSchemaTools.addGroundPlane(
        stage, "/groundPlane", "Z", 1500, Gf.Vec3f(0, 0, -0.05), Gf.Vec3f(0.5)
    )
    
    # Add lighting
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(500)
    
    # Update to load physics
    simulation_app.update()
    
    # Create robot wrapper - use Robot class instead of Articulation
    robot = my_world.scene.add(
        Robot(prim_path=prim_path, name=f"my_{args.robot}")
    )
    
    # Initialize the robot
    my_world.reset()
    
    if not robot.is_physics_handle_valid():
        carb.log_error(f"{prim_path} is not a valid articulation")
        simulation_app.close()
        sys.exit(1)
    
    print(f"\n{'='*60}")
    print(f"UR Robot Loaded from ROS 2 Workspace")
    print(f"{'='*60}")
    print(f"Robot: {args.robot.upper()}")
    print(f"URDF: {urdf_path}")
    print(f"Prim Path: {prim_path}")
    print(f"DOF: {robot.num_dof}")
    print(f"Joint Names: {robot.dof_names}")
    print(f"{'='*60}\n")
    
    # Set home position
    if robot.num_dof >= 6:
        home_position = np.array([0.0, -np.pi/4, -np.pi/2, -np.pi/4, np.pi/2, 0.0])
        robot.set_joint_positions(home_position)
        print(f"Set home position: {home_position}\n")
    
    # Start simulation
    print("Starting simulation...")
    import omni.timeline
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    
    # Simulation loop with simple motion
    frame_count = 0
    motion_started = False
    
    try:
        while simulation_app.is_running():
            simulation_app.update()
            
            frame_count += 1
            
            # Simple motion after 100 frames
            if frame_count == 100 and not motion_started:
                print("Starting simple motion...")
                target_positions = np.array([0.5, -1.0, -1.5, -1.0, 1.5, 0.5])
                if robot.num_dof >= 6:
                    action = ArticulationAction(joint_positions=target_positions)
                    robot.apply_action(action)
                    motion_started = True
            
            # Print status
            if frame_count % 100 == 0:
                positions = robot.get_joint_positions()
                print(f"Frame {frame_count}: Joint positions = {positions}")
            
            # Stop after 1000 frames
            if frame_count >= 1000:
                break
    
    except KeyboardInterrupt:
        print("\nSimulation interrupted")
    
    finally:
        print(f"\nSimulation complete. Total frames: {frame_count}")
        timeline.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
