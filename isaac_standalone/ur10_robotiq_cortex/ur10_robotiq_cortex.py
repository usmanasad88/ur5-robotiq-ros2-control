# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import carb
import numpy as np
import omni
import sys
import traceback
import uuid
from typing import Dict, Tuple, Optional, Sequence
from pathlib import Path
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.cortex.framework.cortex_utils import load_behavior_module
from isaacsim.cortex.framework.cortex_world import Behavior, CortexWorld, LogicalStateMonitor
from isaacsim.cortex.framework.dfb import DfDiagnosticsMonitor
from isaacsim.cortex.framework.robot import CortexUr10Robotiq, add_ur10_robotiq_to_stage, MotionCommandedRobot, RobotiqGripper
from isaacsim.cortex.framework.tools import SteadyRate
from isaacsim.examples.interactive.cortex.cortex_base import CortexBase
from isaacsim.examples.interactive.ur10_robotiq_cortex.ur10_ros2_follower_behavior import ROS2FollowerContext
import isaacsim.robot_motion.motion_generation.interface_config_loader as icl
from isaacsim.core.utils.stage import add_reference_to_stage

def LOGGER(msg):
    """Log to both console and carb logger"""
    print(f"[UR10 Robotiq] {msg}")
    sys.stdout.flush()
    try:
        carb.log_info(f"[UR10 Robotiq] {msg}")
    except:
        pass


class CubeSpec:
    def __init__(self, name, color):
        self.name = name
        self.color = np.array(color)


class ContextStateMonitor(DfDiagnosticsMonitor):
    """
    State monitor to read the context and pass it to the UI.
    For these behaviors, the context has a `diagnostic_message` that contains the text to be displayed, and each
    behavior implements its own monitor to update that.

    """

    def __init__(self, print_dt, diagnostic_fn=None):
        super().__init__(print_dt=print_dt)
        self.diagnostic_fn = diagnostic_fn
        self.last_diagnostic = ""

    def print_diagnostics(self, context):
        diagnostic_msg = ""
        
        # Extract diagnostic information from context
        if hasattr(context, "diagnostics_message"):
            diagnostic_msg = f"{context.diagnostics_message}\n"
        
        # Store and pass to UI
        if diagnostic_msg != self.last_diagnostic:
            self.last_diagnostic = diagnostic_msg
        
        if self.diagnostic_fn:
            self.diagnostic_fn(context)


class CortexUr5Robotiq(MotionCommandedRobot):
    def __init__(
        self,
        name: str,
        prim_path: str,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
    ):
        # Check supported policies to avoid triggering error logs
        supported = icl.get_supported_robot_policy_pairs()
        ur5_policies = supported.get("UR5", [])
        
        policy_name = "RMPflow"
        if "RMPflowCortex" in ur5_policies:
            policy_name = "RMPflowCortex"
        else:
            print("[CortexUr5Robotiq] RMPflowCortex not available for UR5, using RMPflow")

        try:
            motion_policy_config = icl.load_supported_motion_policy_config("UR5", policy_name)
        except Exception as e:
            print(f"[CortexUr5Robotiq] Error loading {policy_name} for UR5: {e}")
            motion_policy_config = None

        if motion_policy_config is None:
             raise RuntimeError("Could not load motion policy config for UR5")
            
        super().__init__(
            name=name,
            prim_path=prim_path,
            motion_policy_config=motion_policy_config,
            position=position,
            orientation=orientation,
            settings=MotionCommandedRobot.Settings(smoothed_rmpflow=False, smoothed_commands=False),
        )

        self.gripper_commander = RobotiqGripper(self)
        self.add_commander("gripper", self.gripper_commander)

def add_ur5_robotiq_to_stage(
    name: str,
    prim_path: str,
    usd_path: Optional[str] = None,
    position: Optional[Sequence[float]] = None,
    orientation: Optional[Sequence[float]] = None,
):
    if usd_path is not None:
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    
    return CortexUr5Robotiq(name, prim_path, position, orientation)

class UR10RobotiqCortex(CortexBase):
    def __init__(self, monitor_fn=None, vla_server_url=None, task_description=None):
        super().__init__()
        self.instance_id = str(uuid.uuid4())[:8]
        LOGGER(f"UR10RobotiqCortex initialized. ID: {self.instance_id}")
        self._monitor_fn = monitor_fn
        self.behavior = None
        self.robot = None
        self.decider_network = None
        self.context_monitor = ContextStateMonitor(print_dt=0.25, diagnostic_fn=self._on_monitor_update)
        self._monitor_call_count = 0
        self._physics_step_count = 0
        # Direct ROS2 follower (no DF/Cortex) support
        self._direct_ros2_context = None
        self._use_direct_ros2 = False
        
        # Hardcoded robot selection
        self.robot_selection = "ur5"  # Options: "ur10", "ur5"

    def setup_scene(self):
        world = self.get_world()
        
        # Select robot USD based on hardcoded selection
        if self.robot_selection == "ur5":
            ur_usd_path = "/home/mani/Repos/ur_ws/isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/ur5_robotiq2f-85.usd"
            robot_name = "ur5_robotiq"
            try:
                self.robot = world.add_robot(
                    add_ur5_robotiq_to_stage(
                        name=robot_name,
                        prim_path="/World/UR10Robotiq",
                        usd_path=ur_usd_path
                    )
                )
            except Exception as e:
                LOGGER(f"Error adding UR5 robot: {e}")
                raise
        else:
            ur_usd_path = "/home/mani/Repos/ur_ws/isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/ur10e_robotiq2f-140_ROS.usd"
            robot_name = "ur10_robotiq"
            try:
                self.robot = world.add_robot(
                    add_ur10_robotiq_to_stage(
                        name=robot_name,
                        prim_path="/World/UR10Robotiq",
                        usd_path=ur_usd_path
                    )
                )
            except Exception as e:
                LOGGER(f"Error adding UR10 robot: {e}")
                raise

        # Create some cubes for manipulation tasks
        obs_specs = [
            CubeSpec("RedCube", [0.7, 0.0, 0.0]),
            CubeSpec("BlueCube", [0.0, 0.0, 0.7]),
            CubeSpec("YellowCube", [0.7, 0.7, 0.0]),
            CubeSpec("GreenCube", [0.0, 0.7, 0.0]),
        ]
        width = 0.05
        for i, (x, spec) in enumerate(zip(np.linspace(0.3, 0.6, len(obs_specs)), obs_specs)):
            try:
                obj = world.scene.add(
                    DynamicCuboid(
                        prim_path="/World/Obs/{}".format(spec.name),
                        name=spec.name,
                        size=width,
                        color=spec.color,
                        position=np.array([x, 0.3, width / 2]),
                    )
                )
                self.robot.register_obstacle(obj)
            except Exception as e:
                LOGGER(f"Error adding obstacle {spec.name}: {e}")
        
        try:
            world.scene.add_default_ground_plane()
        except Exception as e:
            LOGGER(f"Error adding ground plane: {e}")

        # Optional mixed-reality helper data and camera.
        ENABLE_MR = False

        if ENABLE_MR:
            # --- Mixed-reality helper data ------------------------------------
            # Cache the logical cube layout in world coordinates so that
            # a separate MR overlay utility can project them into a real image.
            #
            # For now we simply mirror the placement logic above. Later this
            # can be replaced with reading the actual object poses from USD.
            self._mr_cube_layout = [
                {
                    "name": spec.name,
                    "color": spec.color.tolist(),
                    "size": float(width),
                    "position": [float(x), 0.3, float(width / 2)],
                }
                for x, spec in zip(np.linspace(0.3, 0.6, len(obs_specs)), obs_specs)
            ]

            # Store a reference image path used for early MR experiments.
            self._mr_input_image = str(
                Path(__file__).with_name("input_image.png")
            )

            # --- Mixed-reality debug camera -----------------------------------
            # Create an MR camera in the world so that we can look through it in
            # the Stage and confirm it sees the robot and table. The pose is an
            # initial estimate based on the real room image and should be tuned
            # manually by adjusting the prim in the UI, then copying values back.
            try:
                from omni.isaac.sensor import Camera
                import omni.isaac.core.utils.numpy.rotations as rot_utils
                from pxr import Gf

                mr_cam_pos = np.array([1.8, -2.5, 1.6], dtype=float)
                mr_cam_euler = np.array([-20.0, 0.0, 15.0], dtype=float)  # degrees
                mr_res = (848, 478)
                mr_focal = 32.0

                self._mr_camera = Camera(
                    prim_path="/World/mr_camera",
                    position=mr_cam_pos,
                    frequency=20,
                    resolution=mr_res,
                    orientation=rot_utils.euler_angles_to_quats(
                        mr_cam_euler, degrees=True
                    ),
                )

                self._mr_camera.initialize()
                prim = self._mr_camera.prim

                prim.GetAttribute("xformOp:translate").Set(
                    Gf.Vec3d(
                        float(mr_cam_pos[0]),
                        float(mr_cam_pos[1]),
                        float(mr_cam_pos[2]),
                    )
                )

                if prim.HasAttribute("focalLength"):
                    prim.GetAttribute("focalLength").Set(float(mr_focal))

                LOGGER(
                    f"[MR] MR camera created at /World/mr_camera, "
                    f"pos={tuple(mr_cam_pos)}, euler_deg={tuple(mr_cam_euler)}, "
                    f"res={mr_res}, focal={mr_focal}"
                )
            except Exception as e:
                LOGGER(f"[MR] Error creating MR camera: {e}")

            LOGGER(
                "[MR] Cached cube layout for mixed-reality overlay using "
                f"reference image: {self._mr_input_image}"
            )

    async def load_behavior(self, behavior):
        world = self.get_world()
        self.behavior = behavior
        
        # Cleanup previous direct follower if it exists
        if self._direct_ros2_context:
            LOGGER("[UR10 Robotiq] Cleaning up previous ROS 2 follower context")
            if hasattr(self._direct_ros2_context, 'shutdown'):
                self._direct_ros2_context.shutdown()
            self._direct_ros2_context = None

        # Reset direct follower flags
        self._use_direct_ros2 = False

        try:
            # If this is the ROS2 follower behavior file, use direct mode (no DF/Cortex)
            if "ur10_ros2_follower_behavior.py" in str(self.behavior):
                LOGGER("[UR10 Robotiq] Using direct ROS 2 follower (no DF/Cortex)")
                self._use_direct_ros2 = True
                return

            behavior_module = load_behavior_module(self.behavior)

            # Check if this is VLA behavior and pass server URL and task
            if hasattr(behavior_module, "make_decider_network"):
                import inspect

                sig = inspect.signature(behavior_module.make_decider_network)

                if "server_url" in sig.parameters:
                    # VLA behavior - uses hardcoded settings from behavior file
                    LOGGER("VLA mode enabled (config in ur10_vla_behavior.py)")
                    self.decider_network = behavior_module.make_decider_network(
                        self.robot,
                        monitor_fn=self._on_monitor_update,
                    )
                else:
                    # Other behaviors
                    self.decider_network = behavior_module.make_decider_network(
                        self.robot, self._on_monitor_update
                    )

            self.decider_network.context.add_monitor(self.context_monitor.monitor)
            world.add_decider_network(self.decider_network)
        except Exception as e:
            LOGGER(f"Error loading behavior: {e}")
            LOGGER(f"Traceback: {traceback.format_exc()}")

    async def setup_post_load(self, soft=False):
        world = self.get_world()
        prim_path = "/World/UR10Robotiq"
        
        if not self.robot:
            try:
                robot_name = "ur5_robotiq" if self.robot_selection == "ur5" else "ur10_robotiq"
                self.robot = world._robots[robot_name]
            except Exception as e:
                LOGGER(f"Error retrieving robot: {e}")
                return
        # Direct ROS2 follower: build context only, no DF network
        if self._use_direct_ros2:
            try:
                LOGGER(f"[UR10 Robotiq] Initializing direct ROS 2 follower context (robot={self.robot_selection})")
                self._direct_ros2_context = ROS2FollowerContext(
                    self.robot, 
                    task_description="Follow ROS 2 Joint States",
                    robot_type=self.robot_selection
                )
                # Attach diagnostics monitor
                self.context_monitor.monitor(self._direct_ros2_context)
            except Exception as e:
                LOGGER(f"Error initializing direct ROS 2 context: {e}")
                LOGGER(f"Traceback: {traceback.format_exc()}")
                return

            await omni.kit.app.get_app().next_update_async()
            return

        try:
            behavior_module = load_behavior_module(self.behavior)

            # Check if this is VLA behavior and pass server URL and task
            if hasattr(behavior_module, "make_decider_network"):
                import inspect

                sig = inspect.signature(behavior_module.make_decider_network)

                if "server_url" in sig.parameters:
                    # VLA behavior - uses hardcoded settings from behavior file
                    LOGGER("VLA mode enabled (config in ur10_vla_behavior.py)")
                    self.decider_network = behavior_module.make_decider_network(
                        self.robot,
                        monitor_fn=self._on_monitor_update,
                    )
                else:
                    # Other behaviors
                    self.decider_network = behavior_module.make_decider_network(
                        self.robot, self._on_monitor_update
                    )

            self.decider_network.context.add_monitor(self.context_monitor.monitor)
            world.add_decider_network(self.decider_network)
        except Exception as e:
            LOGGER(f"Error in setup_post_load: {e}")
            LOGGER(f"Traceback: {traceback.format_exc()}")
            return

        await omni.kit.app.get_app().next_update_async()

    def _on_monitor_update(self, context):
        self._monitor_call_count += 1
        
        diagnostic = ""
        decision_stack = ""
        
        # Extract diagnostic information
        if hasattr(context, "diagnostics_message"):
            diagnostic = context.diagnostics_message
        
        # Check if this is VLA context and extract last action
        vla_action_info = ""
        if hasattr(context, "last_action") and context.last_action:
            action = context.last_action
            pos_delta = action.get("position_delta", None)
            new_pos = action.get("new_position", None)
            gripper = action.get("gripper", 0.0)
            
            if pos_delta is not None and new_pos is not None:
                pos_delta_str = f"[{pos_delta[0]:+.4f}, {pos_delta[1]:+.4f}, {pos_delta[2]:+.4f}]"
                new_pos_str = f"[{new_pos[0]:.4f}, {new_pos[1]:.4f}, {new_pos[2]:.4f}]"
                vla_action_info = f"\nVLA Action #{context.action_count}:\n  delta: {pos_delta_str}\n  target: {new_pos_str}\n  gripper: {gripper:.2f}"
        
        # Get the decision stack
        if self.decider_network and hasattr(self.decider_network, '_decider_state'):
            if self.decider_network._decider_state and hasattr(self.decider_network._decider_state, 'stack'):
                stack = self.decider_network._decider_state.stack
                if stack:
                    decision_stack = "\n".join(
                        [
                            "{0}{1}".format("  " * i, element)
                            for i, element in enumerate(str(i) for i in stack)
                        ]
                    )
        
        # Combine all diagnostic info
        full_diagnostic = f"{decision_stack}{vla_action_info}{diagnostic}"

        if self._monitor_fn:
            self._monitor_fn(full_diagnostic, decision_stack)

    def _on_physics_step(self, step_size):
        self._physics_step_count += 1
        if self._physics_step_count % 100 == 0:  # Log every 100 steps to avoid spam
            LOGGER(f"[UR10 Robotiq] Physics step #{self._physics_step_count} (ID: {self.instance_id})")
        
        world = self.get_world()
        # Direct ROS2 follower mode: bypass Cortex DF, call context directly
        if self._use_direct_ros2 and self._direct_ros2_context is not None:
            LOGGER(
                f"[UR10 Robotiq] Direct ROS 2 follower physics step #{self._physics_step_count} (ID: {self.instance_id})"
            )
            try:
                self._direct_ros2_context.update_from_ros()
                self._direct_ros2_context.follow_latest_joints()
            except Exception as e:
                LOGGER(f"[UR10 Robotiq] Direct ROS 2 follower error: {e}")
                traceback.print_exc()
            return

        # Default Cortex behavior stepping
        try:
            world.step(False, False)
        except Exception as e:
            # Throttle repeated errors to avoid spamming the log.
            if self._physics_step_count % 100 == 0:
                LOGGER(
                    f"[UR10 Robotiq] ERROR in physics step #{self._physics_step_count}: "
                    f"type={type(e).__name__}, msg={e}"
                )
                # Optional: keep a full traceback but only occasionally.
                traceback.print_exc()
        except BaseException as e:
            # Catch non-standard exceptions the framework might raise
            if self._physics_step_count % 100 == 0:
                LOGGER(
                    f"[UR10 Robotiq] NON-STD ERROR in physics step #{self._physics_step_count}: "
                    f"type={type(e).__name__}, repr={e!r}"
                )
                traceback.print_exc()

    async def on_event_async(self):
        LOGGER("[UR10 Robotiq] on_event_async: *** CALLED ***")
        world = self.get_world()
        LOGGER(f"[UR10 Robotiq] on_event_async: World: {world}")
        
        try:
            LOGGER("[UR10 Robotiq] on_event_async: Waiting for next update")
            await omni.kit.app.get_app().next_update_async()
            LOGGER("[UR10 Robotiq] on_event_async: Next update completed")
            
            LOGGER("[UR10 Robotiq] on_event_async: Resetting cortex world")
            world.reset_cortex()
            LOGGER("[UR10 Robotiq] on_event_async: Cortex reset complete")
            
            LOGGER("[UR10 Robotiq] on_event_async: Adding physics callback")
            world.add_physics_callback("sim_step", self._on_physics_step)
            LOGGER("[UR10 Robotiq] on_event_async: Physics callback added")
            
            LOGGER("[UR10 Robotiq] on_event_async: Starting world play")
            await world.play_async()
            LOGGER("[UR10 Robotiq] on_event_async: *** WORLD IS NOW PLAYING ***")
        except Exception as e:
            LOGGER(f"[UR10 Robotiq] on_event_async: EXCEPTION: {e}")
            import traceback
            LOGGER(f"[UR10 Robotiq] Traceback: {traceback.format_exc()}")

    async def setup_pre_reset(self):
        LOGGER("[UR10 Robotiq] setup_pre_reset: *** CALLED ***")
        world = self.get_world()
        LOGGER(f"[UR10 Robotiq] setup_pre_reset: Physics callback exists: {world.physics_callback_exists('sim_step') if world else 'World is None'}")
        if world and world.physics_callback_exists("sim_step"):
            LOGGER("[UR10 Robotiq] setup_pre_reset: Removing physics callback")
            world.remove_physics_callback("sim_step")
            LOGGER("[UR10 Robotiq] setup_pre_reset: Physics callback removed")
        LOGGER("[UR10 Robotiq] setup_pre_reset: Complete")

    def world_cleanup(self):
        if self._direct_ros2_context:
            LOGGER("[UR10 Robotiq] Cleaning up ROS 2 follower context in world_cleanup")
            if hasattr(self._direct_ros2_context, 'shutdown'):
                self._direct_ros2_context.shutdown()
            self._direct_ros2_context = None
