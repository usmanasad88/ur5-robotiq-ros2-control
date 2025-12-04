# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
ROS 2 Follower behavior for UR10 with Robotiq gripper.
Subscribes to /joint_states and mimics the joint angles using Curobo.
"""

import sys
import time
import traceback
import numpy as np
from typing import Any, Dict, Optional, Tuple, List
from collections import OrderedDict

# Mock setuptools_scm if missing to allow curobo import from source
try:
    import setuptools_scm
except ImportError:
    import sys
    from unittest.mock import MagicMock
    mock_scm = MagicMock()
    mock_scm.get_version.return_value = "0.0.0-dev"
    sys.modules["setuptools_scm"] = mock_scm

import omni.physx
import carb
from omni.isaac.core.utils.rotations import euler_angles_to_quat

from isaacsim.cortex.framework.df import (
    DfDecider,
    DfDecision,
    DfNetwork,
    DfState,
    DfStateMachineDecider,
    DfStateSequence,
)
from isaacsim.cortex.framework.dfb import DfRobotApiContext
from isaacsim.cortex.framework.motion_commander import MotionCommand, PosePq

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState as RosJointState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    RosJointState = Any # Define as Any to prevent NameError in type hints
    print("[ROS2 Follower] ROS 2 (rclpy) not available. Please source ROS 2 environment.")

# Curobo imports
try:
    import torch
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
    from curobo.geom.types import WorldConfig, Cuboid
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.robot import JointState
    from curobo.util_file import get_robot_configs_path, join_path, load_yaml
    from curobo.util.logger import setup_curobo_logger
    from omni.isaac.core.utils.types import ArticulationAction
    CUROBO_AVAILABLE = True
except Exception as e:
    CUROBO_AVAILABLE = False
    print(f"[ROS2 Follower] Curobo not available: {e}")

class ROS2FollowerContext(DfRobotApiContext):
    def __init__(self, robot, task_description: str = "Follow ROS 2 Joint States", robot_type: str = "ur10"):
        super().__init__(robot)
        
        # Auto-detect robot type if possible
        if robot and hasattr(robot, "name") and "ur5" in robot.name.lower():
            print(f"[ROS2 Follower] Auto-detected UR5 from robot name: {robot.name}")
            robot_type = "ur5"
            
        self.task_description = task_description
        self.robot = robot
        self.robot_type = robot_type
        
        print(f"[ROS2 Follower] Initializing context for robot_type: {self.robot_type}")
        
        # ROS 2 Setup
        self.node = None
        self.subscription = None
        self.latest_joint_state = None
        self.latest_gripper_pos = None
        self.joint_names_map = [
            "shoulder_pan_joint", 
            "shoulder_lift_joint", 
            "elbow_joint", 
            "wrist_1_joint", 
            "wrist_2_joint", 
            "wrist_3_joint"
        ]
        
        if ROS2_AVAILABLE:
            print(f"[ROS2 Follower] ROS2_AVAILABLE is True. rclpy.ok() = {rclpy.ok()}")
            try:
                if not rclpy.ok():
                    print("[ROS2 Follower] Initializing rclpy...")
                    rclpy.init()
                # Use a unique node name to avoid conflicts
                node_name = "isaac_ur10_follower_" + str(int(time.time()))
                print(f"[ROS2 Follower] Creating node: {node_name}")
                self.node = rclpy.create_node(node_name)
                self.subscription = self.node.create_subscription(
                    RosJointState,
                    "/joint_states",
                    self._joint_state_callback,
                    10
                )
                print("[ROS2 Follower] ROS 2 Node initialized and subscribed to /joint_states")
            except Exception as e:
                print(f"[ROS2 Follower] Failed to init ROS 2 node: {e}")
                traceback.print_exc()
                self.node = None
        else:
            print("[ROS2 Follower] ROS2_AVAILABLE is False")

        # Curobo Setup
        self.motion_gen = None
        self.plan_config = None
        if CUROBO_AVAILABLE:
            self._init_curobo()

        self.diagnostics_message = "Initializing..."

    def reset(self):
        """Reset the context state."""
        print("[ROS2 Follower] Reset called - clearing latest joint state but keeping ROS2 node alive")
        self.latest_joint_state = None
        self.latest_gripper_pos = None
        self.diagnostics_message = "Reset complete. Waiting for ROS..."
        # We don't reset the ROS node as it persists across behavior resets usually

    def _joint_state_callback(self, msg: RosJointState):
        # Extract joint positions for the UR arm
        current_joints = {}
        for name, pos in zip(msg.name, msg.position):
            current_joints[name] = pos
        
        # Create a list of positions in the correct order
        ordered_positions = []
        for name in self.joint_names_map:
            if name in current_joints:
                ordered_positions.append(current_joints[name])
        
        if len(ordered_positions) == 6:
            self.latest_joint_state = np.array(ordered_positions, dtype=np.float32)
            # print(f"[ROS2 Follower] Callback received joint state: {self.latest_joint_state}")
            
            # Handle gripper if present
            # Robotiq 2F-85 usually has 'finger_joint'
            if "finger_joint" in current_joints:
                self.latest_gripper_pos = current_joints["finger_joint"]
            elif "robotiq_85_left_knuckle_joint" in current_joints:
                 self.latest_gripper_pos = current_joints["robotiq_85_left_knuckle_joint"]

    def _init_curobo(self):
        # Load robot config based on type
        try:
            config_filename = "ur10e.yml"
            if self.robot_type == "ur5":
                config_filename = "ur5e.yml"
                
            # Try to find config
            config_path = join_path(get_robot_configs_path(), config_filename)
            print(f"[ROS2 Follower] Loading Curobo config from: {config_path}")
            
            robot_cfg = load_yaml(config_path)
            
            # Initialize MotionGen
            # We add a dummy ground plane to satisfy Curobo's collision checker
            world_cfg = WorldConfig(cuboid=[Cuboid(name="ground", dims=[10, 10, 0.1], pose=[0, 0, -0.05, 1, 0, 0, 0])])
            
            motion_gen_config = MotionGenConfig.load_from_robot_config(
                robot_cfg,
                world_model=world_cfg,
                tensor_args=TensorDeviceType(),
                interpolation_dt=0.01,
            )
            self.motion_gen = MotionGen(motion_gen_config)
            self.motion_gen.warmup()
            
            self.plan_config = MotionGenPlanConfig(
                enable_graph=False,
                enable_opt=True,
                max_attempts=1,
                timeout=0.05, # Very fast timeout for following
            )
            print("[ROS2 Follower] Curobo initialized")
        except Exception as e:
            print(f"[ROS2 Follower] Failed to init Curobo: {e}")
            traceback.print_exc()
            self.motion_gen = None

    def update_from_ros(self):
        if self.node:
            try:
                rclpy.spin_once(self.node, timeout_sec=0.0)
            except Exception as e:
                print(f"[ROS2 Follower] Error spinning ROS node: {e}")
        else:
            print("[ROS2 Follower] Warning: ROS2 node is None, cannot spin")
            
    def follow_latest_joints(self):
        # Add counter to track how often this is called
        if not hasattr(self, '_follow_call_count'):
            self._follow_call_count = 0
        self._follow_call_count += 1
        
        # Log only every 60 steps (approx 1 sec at 60Hz)
        should_log = (self._follow_call_count % 60 == 0)
        
        if should_log:
            print(f"[ROS2 Follower] follow_latest_joints called {self._follow_call_count} times, latest_joint_state={'present' if self.latest_joint_state is not None else 'None'}")
        
        try:
            if self.latest_joint_state is None:
                if should_log:
                    self.diagnostics_message = "Waiting for ROS 2 joint states..."
                    print("[ROS2 Follower] Waiting for ROS2 joint states...")
                return

            if not self.motion_gen:
                if should_log:
                    self.diagnostics_message = "Curobo not initialized. Cannot plan."
                    print("[ROS2 Follower] Curobo not initialized")
                return
            
            if should_log:
                print(f"[ROS2 Follower] Received target joints: {self.latest_joint_state}")

            # Create Goal JointState
            # We need to get the current state from the simulation to plan FROM
            if not hasattr(self.robot, "arm"):
                if should_log:
                    print("[ROS2 Follower] Error: Robot object does not have 'arm' attribute")
                self.diagnostics_message = "Robot has no 'arm' attribute"
                return
            
            if should_log:
                print("[ROS2 Follower] Getting current robot joint state...")

            # Use the robot wrapper to query current joint state instead of the
            # MotionCommander, which does not expose get_joints_state().
            try:
                sim_js = self.robot.get_joints_state()
            except AttributeError:
                if should_log:
                    print("[ROS2 Follower] Error: robot.get_joints_state() not available")
                return

            # Build mapping from joint names to positions. If joint names are not
            # provided in sim_js, fall back to the robot arm's articulation_subset
            # joint names, which correspond to the commanded arm DOFs.
            if hasattr(sim_js, "joint_names") and sim_js.joint_names is not None:
                name_to_pos = {name: pos for name, pos in zip(sim_js.joint_names, sim_js.positions)}
            else:
                try:
                    arm_subset = self.robot.arm.articulation_subset
                    subset_names = arm_subset.joint_names
                    subset_positions = arm_subset.get_joint_positions()
                    name_to_pos = {name: pos for name, pos in zip(subset_names, subset_positions)}
                except Exception as e:
                    if should_log:
                        print(f"[ROS2 Follower] Error building name_to_pos map: {e}")
                    return

            try:
                start_positions = [name_to_pos[name] for name in self.joint_names_map]
                if should_log:
                    print(f"[ROS2 Follower] Current robot positions: {start_positions}")
            except KeyError as missing:
                if should_log:
                    print(f"[ROS2 Follower] Missing joint in sim state: {missing}")
                self.diagnostics_message = f"Missing joint: {missing}"
                return

            # Resolve cuRobo device from MotionGen's tensor_args
            device = self.motion_gen.tensor_args.device if hasattr(self.motion_gen, "tensor_args") else "cuda"

            # Start state: 1x6 tensor with UR arm joints only
            start_tensor = torch.tensor(start_positions, device=device).view(1, -1)
            start_state = JointState.from_position(
                start_tensor,
                joint_names=self.joint_names_map,
            )

            # Goal state: latest ROS joints
            # Unwrap target joints to be closest to current simulation joints to avoid limit issues
            # if the USD has stricter limits than the real robot.
            current_joints_np = np.array(start_positions)
            target_joints_np = self.latest_joint_state
            
            # Optional: Unwrap
            # We disable unwrapping here because we want to enforce [-pi, pi] range for PhysX stability
            # target_joints_np = self._unwrap_joints(current_joints_np, target_joints_np)
            
            goal_tensor = torch.tensor(target_joints_np, device=device).view(1, -1)
            goal_state = JointState.from_position(
                goal_tensor,
                joint_names=self.joint_names_map,
            )

            # Plan in joint space: use plan_single_js for JointState start/goal
            if should_log:
                print("[ROS2 Follower] Planning joint-space trajectory...")
            
            result = self.motion_gen.plan_single_js(
                start_state, goal_state, self.plan_config
            )

            if not result.success.item():
                self.diagnostics_message = f"Planning failed: status={getattr(result, 'status', 'unknown')}"
                if should_log:
                    print(f"[ROS2 Follower] Planning failed: {getattr(result, 'status', 'unknown')}")
                return
            
            if should_log:
                print("[ROS2 Follower] Planning succeeded!")

            # Choose a trajectory tensor, mirroring the real-robot node pattern.
            traj = None
            if hasattr(result, "interpolated_plan") and result.interpolated_plan is not None:
                traj = result.interpolated_plan
            elif hasattr(result, "optimized_plan") and result.optimized_plan is not None:
                traj = result.optimized_plan
            elif hasattr(result, "get_interpolated_plan"):
                try:
                    traj = result.get_interpolated_plan()
                except Exception as exc:
                    if should_log:
                        print("[ROS2 Follower] get_interpolated_plan() failed:", exc)
                    traj = None

            if traj is None:
                self.diagnostics_message = "Planning succeeded but no trajectory available."
                return

            # traj is typically [1, steps, dof]
            try:
                traj_tensor = getattr(traj, "position", traj)
                traj_np = traj_tensor.squeeze(0).detach().cpu().numpy()
            except Exception as exc:
                if should_log:
                    print("[ROS2 Follower] Failed to process trajectory tensor:", exc)
                self.diagnostics_message = "Planning succeeded but trajectory processing failed."
                return

            if traj_np.shape[-1] != len(self.joint_names_map):
                self.diagnostics_message = "Trajectory DOF mismatch; skipping command."
                return

            # Apply cuRobo trajectory directly to the robot articulation, bypassing MotionCommander.
            # This follows the VLA behavior pattern for direct cuRobo control.
            final_js = traj_np[-1]  # Shape: (6,) for 6 arm joints
            
            # Check if final_js is very different from current position, which might indicate
            # a planning jump or limit issue.
            # diff = np.linalg.norm(final_js - current_joints_np)
            # if diff > 1.0:
            #     if should_log:
            #         print(f"[ROS2 Follower] Warning: Large jump in planned trajectory: {diff:.2f}")
            
            try:
                # IMPORTANT: Disable the arm's motion commander to allow direct articulation control
                # The MotionCommander uses RMPflow which overrides direct joint commands
                if hasattr(self.robot, "arm"):
                    if hasattr(self.robot.arm, "disable"):
                        # Only log disable once
                        if not hasattr(self, "_arm_disabled_logged"):
                            print("[ROS2 Follower] Disabling arm motion commander for direct control")
                            self._arm_disabled_logged = True
                        self.robot.arm.disable()
                    else:
                        if should_log:
                            print("[ROS2 Follower] Robot arm has no 'disable' method")
                else:
                    if should_log:
                        print("[ROS2 Follower] Robot has no 'arm' attribute")
                
                # Get the robot's articulation
                articulation = None
                if hasattr(self.robot, "articulation"):
                    articulation = self.robot.articulation
                elif hasattr(self.robot, "arm") and hasattr(self.robot.arm, "articulation"):
                    articulation = self.robot.arm.articulation
                else:
                    # Fallback: robot itself might be the articulation
                    articulation = self.robot
                
                if articulation is None:
                    if should_log:
                        print("[ROS2 Follower] Could not find robot articulation")
                    self.diagnostics_message = "No articulation found."
                    return
                
                # Map cuRobo joint names to full articulation DOF indices
                full_dof_names = articulation.dof_names
                
                # Get limits for debugging
                lower_limits = articulation.dof_properties["lower"]
                upper_limits = articulation.dof_properties["upper"]
                
                indices = []
                values = []
                
                for i, name in enumerate(self.joint_names_map):
                    if name in full_dof_names:
                        idx = full_dof_names.index(name)
                        indices.append(idx)
                        val = final_js[i]
                        
                        # Check limits
                        if should_log:
                            low = lower_limits[idx]
                            high = upper_limits[idx]
                            if val < low or val > high:
                                print(f"[ROS2 Follower] WARNING: Joint {name} target {val:.3f} out of limits [{low:.3f}, {high:.3f}]")
                        
                        values.append(val)
                
                if len(indices) != len(self.joint_names_map):
                    if should_log:
                        print(f"[ROS2 Follower] Warning: Only {len(indices)}/{len(self.joint_names_map)} joints mapped")
                        print(f"[ROS2 Follower] Full DOF names: {full_dof_names}")
                        print(f"[ROS2 Follower] Target joint names: {self.joint_names_map}")
                
                # Apply action directly to articulation
                # Use set_joint_positions for immediate effect if the drive is too weak,
                # but apply_action is better for physics.
                # However, if the robot is stuck, maybe we need to force it?
                # For "following", we usually want the robot to just BE there.
                
                # Ensure values are within [-pi, pi] to avoid PhysX errors
                # PhysX setDriveTarget requires [-2pi, 2pi], but we use [-pi, pi] to be safe and consistent
                values = [((v + np.pi) % (2 * np.pi)) - np.pi for v in values]

                # Try setting joint targets first (Action)
                action = ArticulationAction(
                    joint_positions=np.array(values),
                    joint_indices=np.array(indices)
                )
                articulation.apply_action(action)
                
                # Verify the action was applied
                if should_log:
                    new_positions = articulation.get_joint_positions()
                    print(f"[ROS2 Follower] Applied action to {len(indices)} joints")
                    print(f"[ROS2 Follower]   Target: {[f'{v:.3f}' for v in values]}")
                    print(f"[ROS2 Follower]   New positions (all DOFs): {[f'{p:.3f}' for p in new_positions[:8]]}")  # Show first 8 DOFs
                self.diagnostics_message = "Applied cuRobo joint trajectory."
            except Exception as exc:
                if should_log:
                    print("[ROS2 Follower] Failed to apply articulation action:", exc)
                    traceback.print_exc()
                self.diagnostics_message = "Failed to apply articulation action."
                return

            # Gripper
            if self.latest_gripper_pos is not None:
                self.robot.gripper.send_joint_positions(np.array([self.latest_gripper_pos]))
        except Exception as e:
            if should_log:
                print(f"[ROS2 Follower] Error in follow_latest_joints: {e}")
                traceback.print_exc()
            self.diagnostics_message = f"Error: {e}"

    def shutdown(self):
        """Clean up ROS 2 resources."""
        if self.node:
            print("[ROS2 Follower] Destroying ROS 2 node...")
            try:
                self.node.destroy_node()
            except Exception as e:
                print(f"[ROS2 Follower] Error destroying node: {e}")
            self.node = None

    def _unwrap_joints(self, current_joints, target_joints):
        """
        Adjust target joints to be within +/- PI of current joints to avoid unnecessary rotations
        or hitting limits if the USD has limited range but the real robot doesn't.
        """
        # Normalize to -pi to pi
        # But UR robots are multi-turn.
        # If we assume the real robot and sim robot start roughly same place, we should just follow.
        # However, if they are offset by 2pi, we might want to unwrap.
        
        # Simple shortest path unwrapping:
        diff = target_joints - current_joints
        diff = (diff + np.pi) % (2 * np.pi) - np.pi
        return current_joints + diff

class ROS2FollowerState(DfState):
    def __init__(self):
        super().__init__()
        self.step_count = 0
    
    def enter(self):
        self.step_count = 0
        print("[ROS2 Follower State] Entering state")

    def step(self):
        self.step_count += 1
        if self.step_count % 100 == 0:
            print(f"[ROS2 Follower State] Step #{self.step_count}")
        try:
            self.context.update_from_ros()
            self.context.follow_latest_joints()
        except Exception as e:
            print(f"[ROS2 Follower] Error in step: {e}")
            traceback.print_exc()
        
        # CRITICAL: Return self to keep the state running!
        return self

    def exit(self):
        print(f"[ROS2 Follower State] Exiting state after {self.step_count} steps")


class ROS2FollowerBehavior(DfDecider):
    """DF decider wrapper that exposes the ROS2FollowerState as a state machine.

    We follow the same pattern as VLABehavior: a DfDecider with a single
    child entry whose value is a DfStateMachineDecider wrapping our
    ROS2FollowerState. The decider itself returns a DfDecision with the
    string key used when adding the child, ensuring df_descend() always
    finds a matching child and avoiding the previous KeyError from using
    the raw state instance as a key.
    """

    def __init__(self):
        super().__init__()
        self._follow_state = ROS2FollowerState()
        # Single child called "follow" that runs the state machine.
        self.add_child("follow", DfStateMachineDecider(self._follow_state))

    def decide(self):
        # Always choose the "follow" child; the internal state machine
        # keeps the behavior running by returning itself.
        return DfDecision("follow")


def make_decider_network(robot, server_url=None, task_description=None, model=None, monitor_fn=None):
    context = ROS2FollowerContext(robot)
    behavior = ROS2FollowerBehavior()
    network = DfNetwork(behavior, context=context)
    return network
