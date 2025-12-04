# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Simple UR10 Robotiq behavior for basic manipulation.
This behavior demonstrates:
1. Go to home position
2. Move to a target pose
3. Open and close gripper
4. Return to home

This behavior also publishes joint states to ROS2 for controlling the real robot.
"""

import numpy as np
import omni
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import (
    DfAction,
    DfDecider,
    DfDecision,
    DfNetwork,
    DfState,
    DfStateMachineDecider,
    DfStateSequence,
    DfWaitState,
)
from isaacsim.cortex.framework.dfb import make_go_home, DfBasicContext
from isaacsim.cortex.framework.motion_commander import ApproachParams, MotionCommand, PosePq
import isaacsim.cortex.framework.math_util as math_util

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    import time
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[Simple Behavior] ROS 2 (rclpy) not available. Joint state publishing disabled.")


class LoggingWaitState(DfWaitState):
    """DfWaitState with logging to show when wait completes"""
    def __init__(self, wait_time):
        super().__init__(wait_time)
        self.elapsed = 0.0
    
    def enter(self):
        self.elapsed = 0.0
        print(f"[Behavior] >>> ENTER DfWaitState (wait={self.wait_time}s)")
        super().enter()
    
    def step(self):
        self.elapsed += 1/60.0  # Assume 60Hz
        result = super().step()
        if result is None:
            print(f"[Behavior] <<< EXIT DfWaitState (waited {self.elapsed:.2f}s)")
        return result
    
    def exit(self):
        print(f"[Behavior]     DfWaitState exit")
        super().exit()


class GoToHome(DfState):
    """Move robot to home position"""
    
    def __init__(self):
        super().__init__()
        self.target_T = None
        self.step_count = 0
    
    def enter(self):
        self.step_count = 0
        print("[Behavior] >>> ENTER GoToHome")
        if not self.context or not self.context.robot:
            print("[Behavior] ERROR: No context or robot!")
            return
        
        try:
            # Get home configuration and send it as a motion command
            aji = self.context.robot.arm.aji  # Active joint indices
            home_config = self.context.robot.get_joints_default_state().positions[aji]
            self.target_T = self.context.robot.arm.get_fk_T(config=home_config)
            
            p, q = math_util.T2pq(self.target_T)
            command = MotionCommand(PosePq(p, q), posture_config=home_config)
            self.context.robot.arm.send(command)
            print("[Behavior]     GoToHome: Motion command sent")
        except Exception as e:
            print(f"[Behavior] ERROR in GoToHome.enter(): {e}")
            import traceback
            traceback.print_exc()

    def step(self):
        self.step_count += 1
        
        # Publish joint state to ROS2 if available
        if hasattr(self.context, 'publish_joint_state'):
            self.context.publish_joint_state()
        
        # Check if motion is complete
        if self.context and self.context.robot and self.target_T is not None:
            try:
                eff_T = self.context.robot.arm.get_fk_T()
                distance = np.linalg.norm(eff_T - self.target_T)
                if distance < 0.01:
                    print(f"[Behavior] <<< EXIT GoToHome (distance: {distance:.6f})")
                    return None
                if self.step_count % 200 == 0:
                    print(f"[Behavior]     GoToHome: Stepping... distance={distance:.6f}")
            except Exception as e:
                print(f"[Behavior] ERROR in GoToHome.step(): {e}")
        
        return self
    
    def exit(self):
        print(f"[Behavior]     GoToHome exit after {self.step_count} steps")


class MoveToTarget(DfState):
    """Move robot to a target pose"""
    
    def __init__(self):
        super().__init__()
        self.target_T = None
        self.step_count = 0
    
    def enter(self):
        self.step_count = 0
        print("[Behavior] >>> ENTER MoveToTarget")
        
        if not self.context or not self.context.robot:
            print("[Behavior] ERROR: No context or robot!")
            return
        
        try:
            # Define a target pose (forward reach with slight height)
            target_position = np.array([0.3, 0.2, 0.3])
            target_orientation = np.array([0.707, 0.0, 0.707, 0.0])
            
            command = MotionCommand(target_pose=PosePq(p=target_position, q=target_orientation))
            self.context.robot.arm.send(command)
            
            # Store the target for step() to check
            self.target_T = math_util.pq2T(target_position, target_orientation)
            print("[Behavior]     MoveToTarget: Command sent to [0.3, 0.2, 0.3]")
        except Exception as e:
            print(f"[Behavior] ERROR in MoveToTarget.enter(): {e}")
            import traceback
            traceback.print_exc()

    def step(self):
        self.step_count += 1
        
        # Publish joint state to ROS2 if available
        if hasattr(self.context, 'publish_joint_state'):
            self.context.publish_joint_state()
        
        if not self.context or not self.context.robot or self.target_T is None:
            return None
        
        try:
            eff_T = self.context.robot.arm.get_fk_T()
            distance = np.linalg.norm(eff_T - self.target_T)
            if distance < 0.01:
                print(f"[Behavior] <<< EXIT MoveToTarget (distance: {distance:.6f})")
                return None
            if self.step_count % 200 == 0:
                print(f"[Behavior]     MoveToTarget: Stepping... distance={distance:.6f}")
        except Exception as e:
            print(f"[Behavior] ERROR in MoveToTarget.step(): {e}")
        
        return self
    
    def exit(self):
        print(f"[Behavior]     MoveToTarget exit after {self.step_count} steps")


class OpenGripper(DfState):
    """Open the gripper"""
    
    def __init__(self):
        super().__init__()
        self.wait_count = 0
    
    def enter(self):
        self.wait_count = 0
        print("[Behavior] >>> ENTER OpenGripper")
        
        try:
            if self.context and self.context.robot:
                if hasattr(self.context.robot, 'gripper'):
                    print("[Behavior]     OpenGripper: Calling gripper.open()")
                    self.context.robot.gripper.open()
                    print("[Behavior]     OpenGripper: gripper.open() called")
                else:
                    print("[Behavior] ERROR: No 'gripper' attribute on robot!")
                    attrs = [a for a in dir(self.context.robot) if not a.startswith('_')]
                    print(f"[Behavior]     Available: {attrs[:10]}...")
            else:
                print("[Behavior] ERROR: No context or robot!")
        except Exception as e:
            print(f"[Behavior] ERROR in OpenGripper.enter(): {e}")
            import traceback
            traceback.print_exc()

    def step(self):
        self.wait_count += 1
        if self.wait_count > 30:  # ~0.5 seconds at 60Hz
            print(f"[Behavior] <<< EXIT OpenGripper (waited {self.wait_count} frames)")
            return None
        return self
    
    def exit(self):
        print(f"[Behavior]     OpenGripper exit")


class CloseGripper(DfState):
    """Close the gripper"""
    
    def __init__(self):
        super().__init__()
        self.wait_count = 0
    
    def enter(self):
        self.wait_count = 0
        print("[Behavior] >>> ENTER CloseGripper")
        
        try:
            if self.context and self.context.robot:
                if hasattr(self.context.robot, 'gripper'):
                    print("[Behavior]     CloseGripper: Calling gripper.close()")
                    self.context.robot.gripper.close()
                    print("[Behavior]     CloseGripper: gripper.close() called")
                else:
                    print("[Behavior] ERROR: No 'gripper' attribute on robot!")
            else:
                print("[Behavior] ERROR: No context or robot!")
        except Exception as e:
            print(f"[Behavior] ERROR in CloseGripper.enter(): {e}")
            import traceback
            traceback.print_exc()

    def step(self):
        self.wait_count += 1
        if self.wait_count > 30:  # ~0.5 seconds at 60Hz
            print(f"[Behavior] <<< EXIT CloseGripper (waited {self.wait_count} frames)")
            return None
        return self
    
    def exit(self):
        print(f"[Behavior]     CloseGripper exit")


class ReturnHome(DfState):
    """Return robot to home position"""
    
    def __init__(self):
        super().__init__()
        self.target_T = None
        self.step_count = 0
    
    def enter(self):
        self.step_count = 0
        print("[Behavior] >>> ENTER ReturnHome")
        if self.context and self.context.robot:
            try:
                # Get home configuration and send it as a motion command
                aji = self.context.robot.arm.aji  # Active joint indices
                home_config = self.context.robot.get_joints_default_state().positions[aji]
                self.target_T = self.context.robot.arm.get_fk_T(config=home_config)
                
                p, q = math_util.T2pq(self.target_T)
                command = MotionCommand(PosePq(p, q), posture_config=home_config)
                self.context.robot.arm.send(command)
                print("[Behavior]     ReturnHome: Motion command sent")
            except Exception as e:
                print(f"[Behavior] ERROR in ReturnHome.enter(): {e}")
        else:
            print("[Behavior] ERROR: No context or robot in ReturnHome!")

    def step(self):
        self.step_count += 1
        
        # Publish joint state to ROS2 if available
        if hasattr(self.context, 'publish_joint_state'):
            self.context.publish_joint_state()
        
        if not self.context or not self.context.robot or self.target_T is None:
            return None
        
        try:
            eff_T = self.context.robot.arm.get_fk_T()
            distance = np.linalg.norm(eff_T - self.target_T)
            if distance < 0.01:
                print(f"[Behavior] <<< EXIT ReturnHome (distance: {distance:.6f})")
                return None
            if self.step_count % 200 == 0:
                print(f"[Behavior]     ReturnHome: Stepping... distance={distance:.6f}")
        except Exception as e:
            print(f"[Behavior] ERROR in ReturnHome.step(): {e}")
        
        return self
    
    def exit(self):
        print(f"[Behavior]     ReturnHome exit after {self.step_count} steps")


class SimpleBehavior(DfStateMachineDecider):
    """Simple manipulation behavior - loops through a sequence of states"""
    
    def __init__(self):
        super().__init__(
            DfStateSequence(
                [
                    GoToHome(),
                    LoggingWaitState(wait_time=1.0),
                    MoveToTarget(),
                    LoggingWaitState(wait_time=1.0),
                    OpenGripper(),
                    LoggingWaitState(wait_time=0.5),
                    CloseGripper(),
                    LoggingWaitState(wait_time=0.5),
                    ReturnHome(),
                    LoggingWaitState(wait_time=2.0),
                ],
                loop=True
            )
        )


class ROS2PublisherContext(DfBasicContext):
    """Extended context that publishes joint states to ROS2"""
    
    def __init__(self, robot):
        super().__init__(robot)
        self.node = None
        self.publisher = None
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint", 
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        self.publish_count = 0
        
        if ROS2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.node = rclpy.create_node("isaac_ur10_publisher_" + str(int(time.time())))
                self.publisher = self.node.create_publisher(
                    JointState,
                    "/isaac_joint_commands",
                    10
                )
                print("[Simple Behavior] ROS2 publisher initialized on /isaac_joint_commands")
            except Exception as e:
                print(f"[Simple Behavior] Failed to init ROS2 publisher: {e}")
                self.node = None
    
    def publish_joint_state(self):
        """Publish current robot joint state to ROS2"""
        if not self.publisher or not self.robot:
            return
        
        try:
            # Get current joint state from the robot
            sim_js = self.robot.get_joints_state()
            
            # Build mapping from joint names to positions
            if hasattr(sim_js, "joint_names") and sim_js.joint_names is not None:
                name_to_pos = {name: pos for name, pos in zip(sim_js.joint_names, sim_js.positions)}
            else:
                # Fallback to arm articulation subset
                arm_subset = self.robot.arm.articulation_subset
                subset_names = arm_subset.joint_names
                subset_positions = arm_subset.get_joint_positions()
                name_to_pos = {name: pos for name, pos in zip(subset_names, subset_positions)}
            
            # Extract the 6 UR arm joints in order
            positions = []
            for name in self.joint_names:
                if name in name_to_pos:
                    positions.append(float(name_to_pos[name]))
            
            if len(positions) == 6:
                # Create and publish JointState message
                msg = JointState()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = positions
                
                self.publisher.publish(msg)
                self.publish_count += 1
                
                if self.publish_count % 60 == 0:  # Log every 60 publishes (~1 second at 60Hz)
                    print(f"[Simple Behavior] Published {self.publish_count} joint states")
            
        except Exception as e:
            print(f"[Simple Behavior] Error publishing joint state: {e}")


def make_decider_network(robot, monitor_fn=None):
    """Create the decider network for simple UR10 behavior"""
    
    print("[Simple Behavior] Creating behavior network with ROS2 publishing")
    
    # Create behavior - SimpleBehavior is already a DfStateMachineDecider
    behavior = SimpleBehavior()
    
    # Create and return the network with ROS2 publishing context
    network = DfNetwork(behavior, context=ROS2PublisherContext(robot))
    return network
