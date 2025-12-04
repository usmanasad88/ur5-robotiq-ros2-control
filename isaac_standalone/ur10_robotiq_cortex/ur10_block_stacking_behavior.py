# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Block stacking behavior for UR10 with Robotiq gripper.
This behavior demonstrates:
1. Detecting colored blocks in the scene
2. Picking blocks in a specific order
3. Stacking them on top of each other
4. Returning home when complete

Similar to the Franka block stacking example, but adapted for UR10.
"""

import numpy as np
import omni
from collections import OrderedDict
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
from isaacsim.cortex.framework.dfb import DfRobotApiContext, make_go_home
from isaacsim.cortex.framework.motion_commander import MotionCommand, PosePq
import isaacsim.cortex.framework.math_util as math_util


class BlockStackingContext(DfRobotApiContext):
    """Context for block stacking behavior. Tracks blocks, tower position, and gripper state."""
    
    class Block:
        """Represents a block in the scene"""
        def __init__(self, obj):
            self.obj = obj
            self.name = obj.name
            self.grasped = False
        
        def get_position(self):
            """Get block's world position"""
            p, _ = self.obj.get_world_pose()
            return p
        
        def get_transform(self):
            """Get block's world transform"""
            return self.obj.get_transform()
    
    class BlockTower:
        """Tracks the tower of stacked blocks"""
        def __init__(self, tower_position, block_size=0.05):
            self.tower_position = tower_position
            self.block_size = block_size
            self.stack = []  # List of blocks in tower, bottom to top
            self.desired_order = ["RedCube", "BlueCube", "YellowCube", "GreenCube"]
        
        @property
        def height(self):
            """Number of blocks in tower"""
            return len(self.stack)
        
        @property
        def next_placement_height(self):
            """Z position for the next block to be placed"""
            # Stack blocks on top of each other with slight margin
            return self.tower_position[2] + (self.height + 0.5) * self.block_size
        
        @property
        def next_block_pose(self):
            """Get the target pose for the next block to be placed (xy at tower, z stacked)"""
            p = np.array([
                self.tower_position[0],
                self.tower_position[1],
                self.next_placement_height
            ])
            # Neutral gripper orientation (open gripper facing down)
            q = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
            return PosePq(p, q)
        
        @property
        def is_complete(self):
            """Check if tower is fully stacked in correct order"""
            if self.height != len(self.desired_order):
                return False
            for i, block in enumerate(self.stack):
                if block.name != self.desired_order[i]:
                    return False
            return True
        
        @property
        def next_desired_block(self):
            """Get name of next block that should be placed"""
            for block_name in self.desired_order:
                if not any(b.name == block_name for b in self.stack):
                    return block_name
            return None
    
    def __init__(self, robot, tower_position=None):
        super().__init__(robot)
        self.robot = robot
        
        # Initialize tower at a fixed position in front of robot
        if tower_position is None:
            tower_position = np.array([0.4, 0.25, 0.0])  # x, y, z on table
        
        self.tower_position = tower_position
        self.tower = BlockStackingContext.BlockTower(tower_position)
        self.blocks = OrderedDict()
        self.active_block = None
        self.gripper_occupied = False
        
        # Scan registered obstacles for blocks
        if hasattr(robot, 'registered_obstacles'):
            for obj_name, obj in robot.registered_obstacles.items():
                if 'Cube' in obj_name:
                    self.blocks[obj_name] = BlockStackingContext.Block(obj)
        
        print(f"[BlockStacking] Found {len(self.blocks)} blocks: {list(self.blocks.keys())}")
    
    def reset(self):
        """Reset context state for new episode"""
        print("[BlockStacking] Resetting context")
        
        # Reset blocks dictionary from registered obstacles
        self.blocks = OrderedDict()
        if hasattr(self.robot, 'registered_obstacles'):
            for obj_name, obj in self.robot.registered_obstacles.items():
                if 'Cube' in obj_name:
                    self.blocks[obj_name] = BlockStackingContext.Block(obj)
        
        # Reset tower
        self.tower = BlockStackingContext.BlockTower(self.tower_position)
        
        # Reset gripper and block state
        self.active_block = None
        self.gripper_occupied = False
        
        print(f"[BlockStacking] Reset complete: {len(self.blocks)} blocks available")
    
    def get_blocks_on_table(self):
        """Get list of blocks still on the table (not in tower)"""
        blocks_on_table = []
        for name, block in self.blocks.items():
            if not any(tb.name == name for tb in self.tower.stack):
                blocks_on_table.append(block)
        return blocks_on_table
    
    def get_closest_block_to_gripper(self, blocks):
        """Get the closest block to the end-effector"""
        if not blocks:
            return None
        
        eff_p = self.robot.arm.get_fk_p()
        distances = []
        for block in blocks:
            block_p = block.get_position()
            dist = np.linalg.norm(block_p - eff_p)
            distances.append((dist, block))
        
        return min(distances, key=lambda x: x[0])[1]
    
    def update_tower_state(self):
        """Monitor and update which blocks are on the tower"""
        tower_xy = self.tower.tower_position[:2]
        threshold = 0.08  # Within 8cm of tower center XY
        
        # Check each block to see if it's on the tower
        for name, block in self.blocks.items():
            if block.name in [b.name for b in self.tower.stack]:
                continue  # Already in tower
            
            block_p = block.get_position()
            block_xy = block_p[:2]
            dist_to_tower = np.linalg.norm(block_xy - tower_xy)
            
            if dist_to_tower < threshold and block_p[2] > 0.1:  # On tower and above table
                # Block might be on tower - add it
                self.tower.stack.append(block)
                print(f"[BlockStacking] Block {block.name} detected on tower at height {block_p[2]:.3f}")


class GoToHomeState(DfState):
    """Move robot to home position"""
    
    def __init__(self):
        super().__init__()
        self.target_T = None
        self.step_count = 0
    
    def enter(self):
        self.step_count = 0
        print("[BlockStacking] >>> ENTER GoToHomeState")
        
        try:
            aji = self.context.robot.arm.aji
            home_config = self.context.robot.get_joints_default_state().positions[aji]
            self.target_T = self.context.robot.arm.get_fk_T(config=home_config)
            
            p, q = math_util.T2pq(self.target_T)
            command = MotionCommand(PosePq(p, q), posture_config=home_config)
            self.context.robot.arm.send(command)
            print("[BlockStacking]     GoToHome: Motion sent")
        except Exception as e:
            print(f"[BlockStacking] ERROR in GoToHomeState.enter(): {e}")

    def step(self):
        self.step_count += 1
        try:
            eff_T = self.context.robot.arm.get_fk_T()
            distance = np.linalg.norm(eff_T - self.target_T)
            if distance < 0.01:
                print(f"[BlockStacking] <<< EXIT GoToHomeState")
                return None
            if self.step_count % 200 == 0:
                print(f"[BlockStacking]     GoToHome: distance={distance:.4f}")
        except Exception as e:
            print(f"[BlockStacking] ERROR in GoToHomeState.step(): {e}")
        return self


class PickBlockState(DfState):
    """Pick up the next block: approach, close gripper, lift"""
    
    def __init__(self):
        super().__init__()
        self.phase = "approach"  # approach -> lift -> retract
        self.target_T = None
        self.step_count = 0
        self.lift_height = 0.1
    
    def enter(self):
        self.step_count = 0
        self.phase = "approach"
        print("[BlockStacking] >>> ENTER PickBlockState")
        
        ct = self.context
        
        # Get the next block to pick
        next_block_name = ct.tower.next_desired_block
        if not next_block_name or next_block_name not in ct.blocks:
            print("[BlockStacking] ERROR: No valid block to pick!")
            return
        
        ct.active_block = ct.blocks[next_block_name]
        block_p = ct.active_block.get_position()
        
        # Approach pose: above the block
        approach_p = block_p.copy()
        approach_p[2] += 0.1  # Come from above
        approach_q = np.array([1.0, 0.0, 0.0, 0.0])
        
        print(f"[BlockStacking]     Picking block: {ct.active_block.name} at {block_p}")
        
        try:
            command = MotionCommand(target_pose=PosePq(approach_p, approach_q))
            self.context.robot.arm.send(command)
            self.target_T = math_util.pq2T(approach_p, approach_q)
            print("[BlockStacking]     Approach motion sent")
        except Exception as e:
            print(f"[BlockStacking] ERROR sending approach motion: {e}")

    def step(self):
        self.step_count += 1
        ct = self.context
        
        if not ct.active_block or ct.active_block is None:
            return None
        
        try:
            eff_T = self.context.robot.arm.get_fk_T()
            
            if self.phase == "approach":
                distance = np.linalg.norm(eff_T - self.target_T)
                if distance < 0.05:
                    print("[BlockStacking]     Approach complete, moving to block")
                    # Move down to block
                    block_p = ct.active_block.get_position()
                    grasp_p = block_p.copy()
                    grasp_q = np.array([1.0, 0.0, 0.0, 0.0])
                    
                    command = MotionCommand(target_pose=PosePq(grasp_p, grasp_q))
                    self.context.robot.arm.send(command)
                    self.target_T = math_util.pq2T(grasp_p, grasp_q)
                    self.phase = "grasp"
                elif self.step_count % 100 == 0:
                    print(f"[BlockStacking]     Approaching... distance={distance:.4f}")
            
            elif self.phase == "grasp":
                distance = np.linalg.norm(eff_T - self.target_T)
                if distance < 0.02:
                    print("[BlockStacking]     At grasp position, closing gripper")
                    ct.robot.gripper.close()
                    self.phase = "lift"
                    self.step_count = 0
                elif self.step_count % 100 == 0:
                    print(f"[BlockStacking]     Approaching block... distance={distance:.4f}")
            
            elif self.phase == "lift":
                # Wait a moment for gripper to close
                if self.step_count > 20:
                    print("[BlockStacking]     Lifting block")
                    block_p = ct.active_block.get_position()
                    lift_p = block_p.copy()
                    lift_p[2] += self.lift_height
                    lift_q = np.array([1.0, 0.0, 0.0, 0.0])
                    
                    command = MotionCommand(target_pose=PosePq(lift_p, lift_q))
                    self.context.robot.arm.send(command)
                    self.target_T = math_util.pq2T(lift_p, lift_q)
                    self.phase = "verify_lift"
                    self.step_count = 0
            
            elif self.phase == "verify_lift":
                distance = np.linalg.norm(eff_T - self.target_T)
                if distance < 0.05:
                    print("[BlockStacking] <<< EXIT PickBlockState - Block picked!")
                    ct.gripper_occupied = True
                    return None
                elif self.step_count % 100 == 0:
                    print(f"[BlockStacking]     Lifting... distance={distance:.4f}")
        
        except Exception as e:
            print(f"[BlockStacking] ERROR in PickBlockState.step(): {e}")
        
        return self


class PlaceBlockState(DfState):
    """Place block on tower: move above tower, open gripper, retract"""
    
    def __init__(self):
        super().__init__()
        self.phase = "move_to_place"  # move_to_place -> lower -> open -> retract
        self.target_T = None
        self.step_count = 0
    
    def enter(self):
        self.step_count = 0
        self.phase = "move_to_place"
        print("[BlockStacking] >>> ENTER PlaceBlockState")
        
        ct = self.context
        if not ct.active_block or not ct.gripper_occupied:
            print("[BlockStacking] ERROR: No block in gripper to place!")
            return
        
        # Get position above tower
        tower_p = ct.tower.tower_position.copy()
        approach_p = tower_p.copy()
        approach_p[2] = 0.3  # Come from above
        approach_q = np.array([1.0, 0.0, 0.0, 0.0])
        
        print(f"[BlockStacking]     Moving to tower for placement")
        
        try:
            command = MotionCommand(target_pose=PosePq(approach_p, approach_q))
            self.context.robot.arm.send(command)
            self.target_T = math_util.pq2T(approach_p, approach_q)
        except Exception as e:
            print(f"[BlockStacking] ERROR sending approach motion: {e}")

    def step(self):
        self.step_count += 1
        ct = self.context
        
        try:
            eff_T = self.context.robot.arm.get_fk_T()
            
            if self.phase == "move_to_place":
                distance = np.linalg.norm(eff_T - self.target_T)
                if distance < 0.05:
                    print("[BlockStacking]     Above tower, lowering block")
                    # Move down to place height
                    tower_p = ct.tower.tower_position.copy()
                    place_p = tower_p.copy()
                    place_p[2] = ct.tower.next_placement_height
                    place_q = np.array([1.0, 0.0, 0.0, 0.0])
                    
                    command = MotionCommand(target_pose=PosePq(place_p, place_q))
                    self.context.robot.arm.send(command)
                    self.target_T = math_util.pq2T(place_p, place_q)
                    self.phase = "lower"
                elif self.step_count % 100 == 0:
                    print(f"[BlockStacking]     Moving to tower... distance={distance:.4f}")
            
            elif self.phase == "lower":
                distance = np.linalg.norm(eff_T - self.target_T)
                if distance < 0.02:
                    print("[BlockStacking]     Placement height reached, opening gripper")
                    ct.robot.gripper.open()
                    self.phase = "open_gripper"
                    self.step_count = 0
                elif self.step_count % 100 == 0:
                    print(f"[BlockStacking]     Lowering... distance={distance:.4f}")
            
            elif self.phase == "open_gripper":
                # Wait for gripper to open
                if self.step_count > 20:
                    print("[BlockStacking]     Gripper opened, retracting")
                    tower_p = ct.tower.tower_position.copy()
                    retract_p = tower_p.copy()
                    retract_p[2] = 0.3
                    retract_q = np.array([1.0, 0.0, 0.0, 0.0])
                    
                    command = MotionCommand(target_pose=PosePq(retract_p, retract_q))
                    self.context.robot.arm.send(command)
                    self.target_T = math_util.pq2T(retract_p, retract_q)
                    self.phase = "retract"
                    self.step_count = 0
            
            elif self.phase == "retract":
                distance = np.linalg.norm(eff_T - self.target_T)
                if distance < 0.05:
                    print("[BlockStacking] <<< EXIT PlaceBlockState - Block placed!")
                    # Add block to tower and update state
                    ct.tower.stack.append(ct.active_block)
                    ct.gripper_occupied = False
                    ct.active_block = None
                    return None
                elif self.step_count % 100 == 0:
                    print(f"[BlockStacking]     Retracting... distance={distance:.4f}")
        
        except Exception as e:
            print(f"[BlockStacking] ERROR in PlaceBlockState.step(): {e}")
        
        return self


class BlockStackingBehavior(DfDecider):
    """Main dispatcher: decides whether to pick, place, or go home"""
    
    def __init__(self):
        super().__init__()
        
        # Wrap each state in a state machine so dispatcher can call decide() on them
        self.pick_sm = DfStateMachineDecider(DfStateSequence([PickBlockState()]))
        self.place_sm = DfStateMachineDecider(DfStateSequence([PlaceBlockState()]))
        self.home_sm = DfStateMachineDecider(DfStateSequence([GoToHomeState()]))
        
        self.add_child("pick", self.pick_sm)
        self.add_child("place", self.place_sm)
        self.add_child("home", self.home_sm)
    
    def decide(self):
        ct = self.context
        
        print(f"[BlockStacking] Dispatcher: tower_height={ct.tower.height}, gripper_occupied={ct.gripper_occupied}")
        
        # Check if stacking is complete
        if ct.tower.is_complete:
            print("[BlockStacking] Tower complete! Going home...")
            return DfDecision("home")
        
        # If gripper has a block, place it
        if ct.gripper_occupied:
            print("[BlockStacking] Gripper has block, placing...")
            return DfDecision("place")
        
        # If gripper is empty, pick next block
        next_block_name = ct.tower.next_desired_block
        if next_block_name:
            print(f"[BlockStacking] Gripper empty, picking {next_block_name}...")
            return DfDecision("pick")
        
        # All blocks stacked, go home
        print("[BlockStacking] All blocks processed, going home...")
        return DfDecision("home")


def make_decider_network(robot, monitor_fn=None):
    """Create the decider network for block stacking behavior"""
    
    print("[BlockStacking] Creating block stacking network")
    
    # Create context with block tracking
    tower_pos = np.array([0.4, 0.25, 0.0])
    context = BlockStackingContext(robot, tower_position=tower_pos)
    
    # Create behavior dispatcher and network
    behavior = BlockStackingBehavior()
    network = DfNetwork(behavior, context=context)
    
    return network
