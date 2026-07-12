# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
ROS 2 Follower behavior for UR10 with Robotiq gripper.
Subscribes to /joint_states and mimics the joint angles using Curobo.
"""

import os
import sys
import time
import traceback
import numpy as np
from typing import Any, Dict, Optional, Tuple, List
from collections import OrderedDict

# Periodic per-step / per-callback chatter is gated by UR_ROS2_VERBOSE.
# One-shot setup messages (init, gripper-mapping, state enter/exit) are always
# printed. Set UR_ROS2_VERBOSE=1 to re-enable the per-step diagnostics.
_VERBOSE = os.environ.get("UR_ROS2_VERBOSE", "0").lower() in ("1", "true", "yes", "on")

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
from isaacsim.core.utils.rotations import euler_angles_to_quat

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

# ArticulationAction for direct control
try:
    from isaacsim.core.utils.types import ArticulationAction
except Exception as e:
    print(f"[ROS2 Follower] Could not import ArticulationAction: {e}")

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
        self._disabled_arm_commander = None
        self._arm_commander_disabled = False
        
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
            self._init_ros2_node()
        else:
            print("[ROS2 Follower] ROS2_AVAILABLE is False")

        self._disable_arm_commander_if_needed()

        self.diagnostics_message = "Initializing..."

    def _disable_arm_commander_if_needed(self):
        """Disable MotionCommander stepping so ROS joint targets are not overwritten each frame."""
        if self._arm_commander_disabled:
            return
        try:
            commanders = getattr(self.robot, "commanders", None)
            if commanders is None:
                print("[ROS2 Follower] No commanders dict on robot; cannot disable arm commander")
                return

            if "arm" in commanders:
                self._disabled_arm_commander = commanders.pop("arm")
                self._arm_commander_disabled = True
                print("[ROS2 Follower] Disabled robot arm commander to prevent RMPflow override")
            else:
                print("[ROS2 Follower] Arm commander not present; nothing to disable")
        except Exception as e:
            print(f"[ROS2 Follower] Failed to disable arm commander: {e}")

    def reset(self):
        """Reset the context state."""
        print("[ROS2 Follower] Reset called - clearing latest joint state but keeping ROS2 node alive")
        self.latest_joint_state = None
        self.latest_gripper_pos = None
        self.diagnostics_message = "Reset complete. Waiting for ROS..."
        # Reinitialize ROS node if it was destroyed
        if self.node is None and ROS2_AVAILABLE:
            self._init_ros2_node()

    def _init_ros2_node(self):
        """Initialize or reinitialize the ROS2 node and subscription."""
        if self.node is not None:
            print("[ROS2 Follower] ROS2 node already exists, skipping init")
            return
        
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

    def _joint_state_callback(self, msg: RosJointState):
        if not hasattr(self, '_cb_count'):
            self._cb_count = 0
        self._cb_count += 1

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
            prev = self.latest_joint_state
            self.latest_joint_state = np.array(ordered_positions, dtype=np.float32)

            # Log every 60th callback, or when joints change significantly
            changed = prev is not None and np.max(np.abs(self.latest_joint_state - prev)) > 0.01
            if _VERBOSE and (self._cb_count % 60 == 1 or changed):
                print(f"[ROS2 Follower] CB #{self._cb_count}: joints={[f'{v:.3f}' for v in self.latest_joint_state]}"
                      f"  changed={changed}")

        # Handle gripper — checked for EVERY message since the gripper
        # joint state publisher sends separate messages with only
        # ['robotiq_85_left_knuckle_joint'] (no arm joints).
        if "finger_joint" in current_joints:
            self.latest_gripper_pos = current_joints["finger_joint"]
        elif "robotiq_85_left_knuckle_joint" in current_joints:
            self.latest_gripper_pos = current_joints["robotiq_85_left_knuckle_joint"]

    def _apply_joint_targets_fast(self):
        """Fast path: apply joint targets every frame without diagnostics overhead."""
        target_joints = self.latest_joint_state
        if target_joints is None:
            return

        self._disable_arm_commander_if_needed()

        if not hasattr(self, '_fast_apply_count'):
            self._fast_apply_count = 0
        self._fast_apply_count += 1
        if _VERBOSE and self._fast_apply_count % 120 == 0:
            gripper_dbg = f", gripper={self.latest_gripper_pos:.3f}" if self.latest_gripper_pos is not None else ""
            print(f"[ROS2 Follower] fast_apply #{self._fast_apply_count}: target={[f'{v:.3f}' for v in target_joints]}{gripper_dbg}")

        # Get articulation once
        if not hasattr(self, '_articulation'):
            articulation = None
            if hasattr(self.robot, "articulation"):
                articulation = self.robot.articulation
            elif hasattr(self.robot, "arm") and hasattr(self.robot.arm, "articulation"):
                articulation = self.robot.arm.articulation
            else:
                articulation = self.robot
            self._articulation = articulation

        articulation = self._articulation
        if articulation is None:
            return

        # Get joint mapping once (arm + gripper)
        if not hasattr(self, '_joint_indices'):
            full_dof_names = articulation.dof_names
            indices = []
            for name in self.joint_names_map:
                if name in full_dof_names:
                    indices.append(full_dof_names.index(name))
            self._joint_indices = indices

            # Find gripper finger_joint index in the articulation DOFs
            if "finger_joint" in full_dof_names:
                self._gripper_dof_index = full_dof_names.index("finger_joint")
                print(f"[ROS2 Follower] Mapped gripper finger_joint to DOF index {self._gripper_dof_index}")
            else:
                self._gripper_dof_index = None
                print(f"[ROS2 Follower] WARNING: finger_joint not found in DOF names: {full_dof_names}")

        indices = self._joint_indices
        if len(indices) != len(self.joint_names_map):
            raise RuntimeError(f"Joint mapping incomplete: {len(indices)}/{len(self.joint_names_map)}")

        # Build joint targets: arm joints + gripper (if available)
        all_indices = list(indices)
        all_values = [target_joints[i] for i in range(len(indices))]

        if self._gripper_dof_index is not None and self.latest_gripper_pos is not None:
            all_indices.append(self._gripper_dof_index)
            all_values.append(self.latest_gripper_pos)

        action = ArticulationAction(
            joint_positions=np.array(all_values, dtype=np.float32),
            joint_indices=np.array(all_indices, dtype=np.int32),
        )
        articulation.apply_action(action)

    def update_from_ros(self):
        if not hasattr(self, '_update_count'):
            self._update_count = 0
        self._update_count += 1

        if self.node is None and ROS2_AVAILABLE:
            print("[ROS2 Follower] ROS2 node is None, attempting to reinitialize...")
            self._init_ros2_node()

        if self.node:
            try:
                # Spin multiple times to drain any queued messages
                for _ in range(10):
                    rclpy.spin_once(self.node, timeout_sec=0.0)
            except Exception as e:
                print(f"[ROS2 Follower] Error spinning ROS node: {e}")
                # Node may have become invalid, try to reinitialize next time
                if "already been invalidated" in str(e) or "destroyed" in str(e).lower():
                    print("[ROS2 Follower] Node appears destroyed, will reinitialize on next call")
                    self.node = None
                    self.subscription = None
        else:
            if self._update_count % 60 == 1:
                print("[ROS2 Follower] Warning: ROS2 node is None and could not be reinitialized")

        if _VERBOSE and self._update_count % 120 == 0:
            cb_count = getattr(self, '_cb_count', 0)
            print(f"[ROS2 Follower] update_from_ros #{self._update_count}, callbacks received: {cb_count}, "
                  f"latest_joint_state={'set' if self.latest_joint_state is not None else 'None'}")
            
    def follow_latest_joints(self):
        # Add counter to track how often this is called
        if not hasattr(self, '_follow_call_count'):
            self._follow_call_count = 0
        self._follow_call_count += 1

        # Log only every 60 steps (approx 1 sec at 60Hz)
        should_log = (self._follow_call_count % 60 == 0)

        # Apply action on EVERY frame without logging overhead
        # This ensures continuous command to counteract motion policy interference
        if self.latest_joint_state is not None:
            try:
                # Fast path: apply without all the diagnostics
                self._apply_joint_targets_fast()
                return
            except Exception:
                # Fall through to full logic if fast path fails
                pass

        if should_log:
            print(f"[ROS2 Follower] follow_latest_joints called {self._follow_call_count} times, latest_joint_state={'present' if self.latest_joint_state is not None else 'None'}")

        try:
            self._disable_arm_commander_if_needed()

            if self.latest_joint_state is None:
                if should_log:
                    self.diagnostics_message = "Waiting for ROS 2 joint states..."
                    print("[ROS2 Follower] Waiting for ROS2 joint states...")
                return

            if should_log:
                print(f"[ROS2 Follower] Received target joints: {self.latest_joint_state}")

            # Get the target joint values directly from ROS (no planning needed)
            target_joints_np = self.latest_joint_state

            if should_log:
                print(f"[ROS2 Follower] Target joint state: {target_joints_np}")
            
            try:
                # Get the robot's articulation directly, bypassing MotionCommander
                articulation = None
                if hasattr(self.robot, "articulation"):
                    articulation = self.robot.articulation
                elif hasattr(self.robot, "arm") and hasattr(self.robot.arm, "articulation"):
                    articulation = self.robot.arm.articulation
                else:
                    # Fallback: robot itself might be the articulation
                    articulation = self.robot

                # CRITICAL: Bypass the MotionCommander completely on first call
                # The MotionCommander's RMPflow policy will override our commands
                if not hasattr(self, "_commander_bypass_setup"):
                    print("[ROS2 Follower] Setting up to bypass MotionCommander")
                    if hasattr(self.robot, "arm"):
                        arm = self.robot.arm
                        # Try to deactivate the arm's update function
                        if hasattr(arm, "active"):
                            print(f"[ROS2 Follower] Arm active state: {arm.active}")
                            # Set active to False to stop it from running
                            try:
                                arm.active = False
                                print("[ROS2 Follower] Set arm.active = False")
                            except Exception as e:
                                print(f"[ROS2 Follower] Could not set arm.active: {e}")
                        # Try to stop the motion policy update
                        if hasattr(arm, "motion_policy") and arm.motion_policy:
                            mp = arm.motion_policy
                            if hasattr(mp, "pause"):
                                try:
                                    mp.pause()
                                    print("[ROS2 Follower] Paused motion policy")
                                except Exception as e:
                                    print(f"[ROS2 Follower] Could not pause motion policy: {e}")
                            # Try to clear the policy frame
                            if hasattr(mp, "reset"):
                                try:
                                    mp.reset()
                                    print("[ROS2 Follower] Reset motion policy")
                                except Exception as e:
                                    pass
                    self._commander_bypass_setup = True
                
                if articulation is None:
                    if should_log:
                        print("[ROS2 Follower] Could not find robot articulation")
                    self.diagnostics_message = "No articulation found."
                    return
                
                # Map cuRobo joint names to full articulation DOF indices
                full_dof_names = articulation.dof_names

                # Ensure gripper DOF index is set for slow path too
                if not hasattr(self, '_gripper_dof_index'):
                    if "finger_joint" in full_dof_names:
                        self._gripper_dof_index = full_dof_names.index("finger_joint")
                    else:
                        self._gripper_dof_index = None

                # Get limits for debugging
                lower_limits = articulation.dof_properties["lower"]
                upper_limits = articulation.dof_properties["upper"]
                
                indices = []
                values = []
                
                for i, name in enumerate(self.joint_names_map):
                    if name in full_dof_names:
                        idx = full_dof_names.index(name)
                        indices.append(idx)
                        val = target_joints_np[i]
                        
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
                
                values = np.array(values, dtype=np.float32)

                # Debug: Check drive properties before applying action
                if should_log:
                    try:
                        dof_props = articulation.dof_properties
                        print(f"[ROS2 Follower] Drive properties for arm joints:")
                        for i, idx in enumerate(indices):
                            name = full_dof_names[idx]
                            stiffness = dof_props["stiffness"][idx]
                            damping = dof_props["damping"][idx]
                            max_effort = dof_props["maxEffort"][idx]
                            print(f"[ROS2 Follower]   {name}: stiffness={stiffness:.1f}, damping={damping:.1f}, max_effort={max_effort:.1f}")
                    except Exception as e:
                        print(f"[ROS2 Follower] Could not read drive properties: {e}")

                try:
                    action = ArticulationAction(
                        joint_positions=values,
                        joint_indices=np.array(indices, dtype=np.int32),
                    )
                    articulation.apply_action(action)
                    if should_log:
                        print("[ROS2 Follower] Applied articulation action")
                except Exception as e:
                    if should_log:
                        print(f"[ROS2 Follower] Failed to apply articulation action: {e}")

                if should_log:
                    print(f"[ROS2 Follower] Applied action to {len(indices)} joints")
                    print(f"[ROS2 Follower]   Target: {[f'{v:.3f}' for v in values]}")
                    
                    # Check what the articulation controller is doing
                    if hasattr(articulation, "_articulation_controller"):
                        ctrl = articulation._articulation_controller
                        print(f"[ROS2 Follower]   Articulation controller: {type(ctrl)}")
                    
                    # Check robot.arm status
                    if hasattr(self.robot, "arm"):
                        arm = self.robot.arm
                        print(f"[ROS2 Follower]   Robot arm type: {type(arm)}")
                        if hasattr(arm, "motion_policy"):
                            print(f"[ROS2 Follower]   Motion policy: {type(arm.motion_policy)}")
                        if hasattr(arm, "_active"):
                            print(f"[ROS2 Follower]   Arm _active: {arm._active}")
                        if hasattr(arm, "active"):
                            print(f"[ROS2 Follower]   Arm active: {arm.active}")
                        # Check if commander is sending commands
                        if hasattr(arm, "_commander"):
                            print(f"[ROS2 Follower]   Commander: {type(arm._commander)}")
                    
                    # Check current positions after apply
                    new_positions = articulation.get_joint_positions()
                    print(f"[ROS2 Follower]   Positions after apply: {[f'{new_positions[i]:.3f}' for i in indices]}")
                    
                self.diagnostics_message = "Applied ROS 2 joint trajectory."
            except Exception as exc:
                if should_log:
                    print("[ROS2 Follower] Failed to apply articulation action:", exc)
                    traceback.print_exc()
                self.diagnostics_message = "Failed to apply articulation action."
                return

            # Gripper — apply via ArticulationAction (same as fast path)
            if self.latest_gripper_pos is not None and hasattr(self, '_gripper_dof_index') and self._gripper_dof_index is not None:
                gripper_action = ArticulationAction(
                    joint_positions=np.array([self.latest_gripper_pos], dtype=np.float32),
                    joint_indices=np.array([self._gripper_dof_index], dtype=np.int32),
                )
                articulation.apply_action(gripper_action)
        except Exception as e:
            if should_log:
                print(f"[ROS2 Follower] Error in follow_latest_joints: {e}")
                traceback.print_exc()
            self.diagnostics_message = f"Error: {e}"

    def shutdown(self):
        """Clean up ROS 2 resources."""
        if self._disabled_arm_commander is not None and hasattr(self.robot, "commanders"):
            if "arm" not in self.robot.commanders:
                self.robot.commanders["arm"] = self._disabled_arm_commander
                print("[ROS2 Follower] Restored robot arm commander")
            self._disabled_arm_commander = None
            self._arm_commander_disabled = False

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
        if _VERBOSE and self.step_count % 100 == 0:
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
