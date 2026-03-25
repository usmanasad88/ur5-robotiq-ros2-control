import carb
import numpy as np
import omni
import os
import sys
import traceback
import uuid
from typing import Optional, Sequence
from pathlib import Path
from pxr import Gf, Usd, Sdf

from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.cortex.framework.cortex_utils import load_behavior_module
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.dfb import DfDiagnosticsMonitor
from isaacsim.cortex.framework.robot import (
    CortexUr10Robotiq,
    add_ur10_robotiq_to_stage,
    MotionCommandedRobot,
    RobotiqGripper,
)
from isaacsim.examples.interactive.cortex.cortex_base import CortexBase
from ur_robotiq_cortex.ur_ros2_follower_behavior import ROS2FollowerContext
import isaacsim.robot_motion.motion_generation.interface_config_loader as icl
from isaacsim.core.utils.stage import add_reference_to_stage

# ---------------------------------------------------------------------------
# Path resolution
# ---------------------------------------------------------------------------
# USD assets live in ur_ws/isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/.
# Resolution order:
#   1. UR_WS_PATH environment variable (recommended for portability)
#   2. Parent directory of the extension root (works when the extension is
#      installed at <ur_ws>/isaac_sim_extension/ur_robotiq_cortex/)

def _get_ur_ws_root() -> Path:
    env = os.environ.get("UR_WS_PATH")
    if env:
        return Path(env)
    # __file__ is .../ur_robotiq_cortex/ur_robotiq_cortex/ur_robotiq_cortex.py
    # parents: [0]=ur_robotiq_cortex pkg, [1]=ur_robotiq_cortex ext, [2]=isaac_sim_extension, [3]=ur_ws
    return Path(__file__).resolve().parents[3]


UR5_USD_RELATIVE  = "isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/ur5_robotiq2f-85.usd"
UR10_USD_RELATIVE = "isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/ur10e_robotiq2f-140_ROS.usd"
ENABLE_OBJECT_COLLISIONS = False

# ---------------------------------------------------------------------------


def LOGGER(msg):
    print(f"[UR Robotiq] {msg}")
    sys.stdout.flush()
    try:
        carb.log_info(f"[UR Robotiq] {msg}")
    except Exception:
        pass


class ContextStateMonitor(DfDiagnosticsMonitor):
    def __init__(self, print_dt, diagnostic_fn=None):
        super().__init__(print_dt=print_dt)
        self.diagnostic_fn = diagnostic_fn
        self.last_diagnostic = ""

    def print_diagnostics(self, context):
        diagnostic_msg = ""
        if hasattr(context, "diagnostics_message"):
            diagnostic_msg = f"{context.diagnostics_message}\n"
        if diagnostic_msg != self.last_diagnostic:
            self.last_diagnostic = diagnostic_msg
        if self.diagnostic_fn:
            self.diagnostic_fn(context)


class CortexUr5Robotiq(MotionCommandedRobot):
    """MotionCommandedRobot wrapper for the UR5 + Robotiq 2F-85 combination."""

    def __init__(
        self,
        name: str,
        prim_path: str,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
    ):
        supported = icl.get_supported_robot_policy_pairs()
        ur5_policies = supported.get("UR5", [])

        policy_name = "RMPflowCortex" if "RMPflowCortex" in ur5_policies else "RMPflow"
        if policy_name == "RMPflow":
            print("[CortexUr5Robotiq] RMPflowCortex not available for UR5, using RMPflow")

        try:
            motion_policy_config = icl.load_supported_motion_policy_config("UR5", policy_name)
        except Exception as e:
            raise RuntimeError(f"Could not load motion policy config for UR5: {e}") from e

        super().__init__(
            name=name,
            prim_path=prim_path,
            motion_policy_config=motion_policy_config,
            position=position,
            orientation=orientation,
            settings=MotionCommandedRobot.Settings(
                smoothed_rmpflow=False, smoothed_commands=False
            ),
        )
        self.gripper_commander = RobotiqGripper(self)
        self.add_commander("gripper", self.gripper_commander)


def add_ur5_robotiq_to_stage(
    name: str,
    prim_path: str,
    usd_path: Optional[str] = None,
    position: Optional[Sequence[float]] = None,
    orientation: Optional[Sequence[float]] = None,
) -> CortexUr5Robotiq:
    if usd_path is not None:
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    return CortexUr5Robotiq(name, prim_path, position, orientation)


class URRobotiqCortex(CortexBase):
    """Isaac Sim sample for UR5/UR10 with Robotiq gripper.

    Supported behaviors (selectable from the UI dropdown):
      - ROS 2 Follower : mirrors /joint_states from a real/simulated robot
      - VLA Control    : receives actions from a Vision-Language-Action model
    """

    def __init__(self, monitor_fn=None):
        super().__init__()
        self.instance_id = str(uuid.uuid4())[:8]
        LOGGER(f"URRobotiqCortex initialized. ID: {self.instance_id}")
        self._monitor_fn = monitor_fn
        self.behavior = None
        self.robot = None
        self.decider_network = None
        self.context_monitor = ContextStateMonitor(
            print_dt=0.25, diagnostic_fn=self._on_monitor_update
        )
        self._monitor_call_count = 0
        self._physics_step_count = 0

        # Direct ROS 2 follower (bypasses Cortex DF network)
        self._direct_ros2_context = None
        self._use_direct_ros2 = False

        # Robot model: "ur5" or "ur10" — overridable via UR_ROBOT_TYPE env var
        self.robot_selection = os.environ.get("UR_ROBOT_TYPE", "ur5").lower()
        LOGGER(
            f"Robot: {self.robot_selection}  "
            f"(change with UR_ROBOT_TYPE env var; options: ur5, ur10)"
        )

    # ------------------------------------------------------------------
    # Scene setup
    # ------------------------------------------------------------------

    def setup_scene(self):
        world = self.get_world()
        ur_ws = _get_ur_ws_root()

        if self.robot_selection == "ur5":
            usd_path = str(ur_ws / UR5_USD_RELATIVE)
            robot_name = "ur5_robotiq"
            LOGGER(f"Loading UR5 from: {usd_path}")
            self.robot = world.add_robot(
                add_ur5_robotiq_to_stage(
                    name=robot_name,
                    prim_path="/World/UR10Robotiq",
                    usd_path=usd_path,
                )
            )
        else:
            usd_path = str(ur_ws / UR10_USD_RELATIVE)
            robot_name = "ur10_robotiq"
            LOGGER(f"Loading UR10 from: {usd_path}")
            self.robot = world.add_robot(
                add_ur10_robotiq_to_stage(
                    name=robot_name,
                    prim_path="/World/UR10Robotiq",
                    usd_path=usd_path,
                )
            )

        ur_ws = _get_ur_ws_root()
        stage = omni.usd.get_context().get_stage()

        def set_prim_transform(prim_path, position, scale, orientation):
            """Set position, scale, and orientation (quaternion) for a prim."""
            try:
                prim = stage.GetPrimAtPath(prim_path)
                if not prim.IsValid():
                    LOGGER(f"Prim not found: {prim_path}")
                    return False

                # Set translate
                trans_attr = prim.GetAttribute("xformOp:translate")
                trans_attr.Set(Gf.Vec3d(*position))

                # Set scale
                scale_attr = prim.GetAttribute("xformOp:scale")
                scale_attr.Set(Gf.Vec3f(*scale))

                # Set orient (quaternion: w, x, y, z) - check type and use appropriate quaternion
                orient_attr = prim.GetAttribute("xformOp:orient")
                if orient_attr.IsValid():
                    type_name = str(orient_attr.GetTypeName())
                    if "double" in type_name.lower() or "quatd" in type_name.lower():
                        orient_attr.Set(Gf.Quatd(*orientation))
                    else:
                        orient_attr.Set(Gf.Quatf(*orientation))
                else:
                    orient_attr.Set(Gf.Quatf(*orientation))
                return True
            except Exception as e:
                LOGGER(f"Error setting transform for {prim_path}: {e}")
                return False

        # Load robot_base (underneath the robot)
        try:
            robot_base_path = str(ur_ws / "isaac_standalone/Objects/robot_base.glb")
            add_reference_to_stage(usd_path=robot_base_path, prim_path="/World/robot_base")
            set_prim_transform("/World/robot_base",
                             [-0.009593696794215156, 0.0024695288591716235, -0.3590445042296741],
                             [0.6, 0.6, 0.77],
                             [0.91553444, 0, 0, 0.4022396])
            LOGGER(f"Loaded robot_base from: {robot_base_path}")
        except Exception as e:
            LOGGER(f"Error adding robot_base: {e}")

        # Object specifications from Scene_update_locations1.usda
        objects = [
            ("box.glb", "box", [0.3020465481322483, -0.7993800354369227, -0.05261620751702961], [0.3, 0.3, 0.3], [0.67167205, 0, 0, 0.7408486]),
            ("bottle.glb", "bottle", [0.62185, -0.74392, -0.03925], [0.18, 0.18, 0.18], [0.89517987, 0, 0, -0.44570506]),
            ("mold.glb", "mold", [1.0001067678586937, -0.4725607098919987, -0.1062191284941244], [0.2, 0.2, 0.2], [0.89517987, 0, 0, -0.44570506]),
            ("chair.glb", "chair", [1.4380247015837357, -0.3976784166109164, -0.4433299999999997], [0.6, 0.6, 0.6], [0.6846738, 0, 0, -0.72884965]),
            ("table.glb", "table", [0.9180239448935501, -0.10213723964001309, -0.43615574066459784], [1.12, 1.12, 1.12], [0.70710677, 0, 0, -0.70710677]),
            ("roller.glb", "roller", [0.2912099012344287, -0.6610178444917744, 0.010067407600128253], [0.2, 0.2, 0.2], [0.9907924, -0.13035351, 0.028021853, 0.023519594]),
            ("scale.usd", "scale", [0.9887047587647333, -0.21432947434265026, -0.11687290658911437], [0.2, 0.2, 0.2], [0.89517987, 0, 0, -0.44570506]),
            ("table.glb", "table_01", [0.2069019081973329, -0.9348596743844684, -0.43615999999997934], [1.12, 1.12, 1.12], [6.123234e-17, 0, 0, 1]),
            ("bottle.glb", "bottle_01", [0.48591, -0.7444, -0.03805], [0.18, 0.18, 0.18], [0.87884057, 0, 0, -0.47711557]),
            ("chair.glb", "chair_01", [1.4670980932575624, 0.05057512260420771, -0.4433302652111933], [0.6, 0.6, 0.6], [0.6846738, 0, 0, -0.72884965]),
            ("chair.glb", "chair_02", [0.4122172010525239, -1.2876057483987595, -0.44332999999999984], [0.6, 0.6, 0.6], [0.021866286, 0, 0, -0.9997609]),
            ("chair.glb", "chair_03", [-0.03659228338588363, -1.3062214460872272, -0.44332999999999984], [0.6, 0.6, 0.6], [0.021866286, 0, 0, -0.9997609]),
        ]

        def add_sdf_collision(prim_path):
            """Apply SDF collision APIs to the inner geometry_0/geometry_0 prim.

            GLB files import as: <obj>/geometry_0/geometry_0
            The USDA applies apiSchemas on the inner geometry_0:
              PhysicsRigidBodyAPI, PhysicsCollisionAPI,
              PhysicsMeshCollisionAPI, PhysxSDFMeshCollisionAPI
            plus sets physics:approximation = "sdf".
            """
            from pxr import UsdPhysics, PhysxSchema
            inner_path = f"{prim_path}/geometry_0/geometry_0"
            inner_prim = stage.GetPrimAtPath(inner_path)
            if not inner_prim.IsValid():
                LOGGER(f"Inner geometry prim not found at {inner_path}")
                return
            # Apply all four API schemas (matches USDA)
            UsdPhysics.RigidBodyAPI.Apply(inner_prim)
            UsdPhysics.CollisionAPI.Apply(inner_prim)
            UsdPhysics.MeshCollisionAPI.Apply(inner_prim)
            PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(inner_prim)
            # Set approximation token
            approx_attr = inner_prim.GetAttribute("physics:approximation")
            if not approx_attr.IsValid():
                approx_attr = inner_prim.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token)
            approx_attr.Set("sdf")
            LOGGER(f"Applied SDF collision APIs to {inner_path}")

        for obj_file, obj_name, position, scale, orientation in objects:
            try:
                obj_path = str(ur_ws / f"isaac_standalone/Objects/{obj_file}")
                if os.path.exists(obj_path):
                    prim_path = f"/World/Objects/{obj_name}"
                    add_reference_to_stage(usd_path=obj_path, prim_path=prim_path)
                    set_prim_transform(prim_path, position, scale, orientation)
                    if ENABLE_OBJECT_COLLISIONS:
                        add_sdf_collision(prim_path)
                    LOGGER(f"Loaded {obj_name}")
                else:
                    LOGGER(f"Object file not found: {obj_path}")
            except Exception as e:
                LOGGER(f"Error adding {obj_name}: {e}")

        # Add ground plane underneath with correct position
        try:
            ground_env_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Environments/Grid/default_environment.usd"
            add_reference_to_stage(usd_path=ground_env_path, prim_path="/World/defaultGroundPlane")

            ground_prim = stage.GetPrimAtPath("/World/defaultGroundPlane")
            if ground_prim.IsValid():
                # Set translate
                trans_attr = ground_prim.GetAttribute("xformOp:translate")
                if not trans_attr.IsValid():
                    trans_attr = ground_prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3)
                trans_attr.Set(Gf.Vec3d(0, 0, -0.7504923076056753))

                # Set orient (quaternion)
                orient_attr = ground_prim.GetAttribute("xformOp:orient")
                if not orient_attr.IsValid():
                    orient_attr = ground_prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatd)
                orient_attr.Set(Gf.Quatd(1, 0, 0, 0))

                # Set scale
                scale_attr = ground_prim.GetAttribute("xformOp:scale")
                if not scale_attr.IsValid():
                    scale_attr = ground_prim.CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Double3)
                scale_attr.Set(Gf.Vec3d(1, 1, 1))

                # Set xformOpOrder to enable transforms
                order_attr = ground_prim.GetAttribute("xformOpOrder")
                if not order_attr.IsValid():
                    order_attr = ground_prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray, True)
                order_attr.Set(Usd.Tokens.xformOpOrder)
                # Manually set the order list
                order_attr.Clear()
                order_attr.Set(["xformOp:translate", "xformOp:orient", "xformOp:scale"])

                LOGGER("Added ground plane at z=-0.7504923076056753")
            else:
                LOGGER("Could not create ground plane prim")
        except Exception as e:
            LOGGER(f"Error adding ground plane: {e}")

    # ------------------------------------------------------------------
    # Behavior loading
    # ------------------------------------------------------------------

    async def load_behavior(self, behavior):
        world = self.get_world()
        self.behavior = behavior

        if self._direct_ros2_context:
            LOGGER("Cleaning up previous ROS 2 follower context")
            if hasattr(self._direct_ros2_context, "shutdown"):
                self._direct_ros2_context.shutdown()
            self._direct_ros2_context = None

        self._use_direct_ros2 = False

        try:
            if "ur_ros2_follower_behavior.py" in str(self.behavior):
                LOGGER("Switching to direct ROS 2 follower mode (no DF/Cortex)")
                self._use_direct_ros2 = True
                return

            behavior_module = load_behavior_module(self.behavior)

            if hasattr(behavior_module, "make_decider_network"):
                import inspect
                sig = inspect.signature(behavior_module.make_decider_network)
                if "server_url" in sig.parameters:
                    LOGGER("VLA mode — config is inside ur_vla_behavior.py")
                    self.decider_network = behavior_module.make_decider_network(
                        self.robot, monitor_fn=self._on_monitor_update
                    )
                else:
                    self.decider_network = behavior_module.make_decider_network(
                        self.robot, self._on_monitor_update
                    )

            self.decider_network.context.add_monitor(self.context_monitor.monitor)
            world.add_decider_network(self.decider_network)
        except Exception as e:
            LOGGER(f"Error loading behavior: {e}\n{traceback.format_exc()}")

    async def setup_post_load(self, soft=False):
        world = self.get_world()

        if not self.robot:
            robot_name = "ur5_robotiq" if self.robot_selection == "ur5" else "ur10_robotiq"
            try:
                self.robot = world._robots[robot_name]
            except Exception as e:
                LOGGER(f"Error retrieving robot: {e}")
                return

        # Initialize robot from Home pose (degrees: 55, -90, 90, 210, -90, 0)
        try:
            home_joint_angles = np.array([55, -90, 90, 210, -90, 0], dtype=np.float32)
            home_joint_angles_rad = np.deg2rad(home_joint_angles)

            arm_joint_names = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]

            dof_names = list(self.robot.dof_names)
            arm_indices = [dof_names.index(name) for name in arm_joint_names if name in dof_names]
            if len(arm_indices) != 6:
                raise RuntimeError(
                    f"Could not map all arm joints for home pose. Mapped {len(arm_indices)}/6; dof_names={dof_names}"
                )

            full_joint_positions = np.array(self.robot.get_joint_positions(), dtype=np.float32)
            for arm_i, dof_i in enumerate(arm_indices):
                full_joint_positions[dof_i] = home_joint_angles_rad[arm_i]

            self.robot.set_joint_positions(full_joint_positions)
            LOGGER(
                f"Initialized robot to Home pose (arm joints only): {home_joint_angles} degrees"
            )
        except Exception as e:
            LOGGER(f"Error initializing robot to Home pose: {e}")

        if self._use_direct_ros2:
            try:
                LOGGER(f"Initializing ROS 2 follower context (robot={self.robot_selection})")
                self._direct_ros2_context = ROS2FollowerContext(
                    self.robot,
                    task_description="Follow ROS 2 Joint States",
                    robot_type=self.robot_selection,
                )
                self.context_monitor.monitor(self._direct_ros2_context)
            except Exception as e:
                LOGGER(f"Error initializing direct ROS 2 context: {e}\n{traceback.format_exc()}")
                return
            await omni.kit.app.get_app().next_update_async()
            return

        try:
            behavior_module = load_behavior_module(self.behavior)
            if hasattr(behavior_module, "make_decider_network"):
                import inspect
                sig = inspect.signature(behavior_module.make_decider_network)
                if "server_url" in sig.parameters:
                    self.decider_network = behavior_module.make_decider_network(
                        self.robot, monitor_fn=self._on_monitor_update
                    )
                else:
                    self.decider_network = behavior_module.make_decider_network(
                        self.robot, self._on_monitor_update
                    )
            self.decider_network.context.add_monitor(self.context_monitor.monitor)
            world.add_decider_network(self.decider_network)
        except Exception as e:
            LOGGER(f"Error in setup_post_load: {e}\n{traceback.format_exc()}")
            return

        await omni.kit.app.get_app().next_update_async()

    # ------------------------------------------------------------------
    # Monitor / diagnostics
    # ------------------------------------------------------------------

    def _on_monitor_update(self, context):
        self._monitor_call_count += 1
        diagnostic = getattr(context, "diagnostics_message", "")
        decision_stack = ""

        vla_action_info = ""
        if hasattr(context, "last_action") and context.last_action:
            action = context.last_action
            pos_delta = action.get("position_delta")
            new_pos = action.get("new_position")
            gripper = action.get("gripper", 0.0)
            if pos_delta is not None and new_pos is not None:
                vla_action_info = (
                    f"\nVLA Action #{context.action_count}:\n"
                    f"  delta: [{pos_delta[0]:+.4f}, {pos_delta[1]:+.4f}, {pos_delta[2]:+.4f}]\n"
                    f"  target: [{new_pos[0]:.4f}, {new_pos[1]:.4f}, {new_pos[2]:.4f}]\n"
                    f"  gripper: {gripper:.2f}"
                )

        if self.decider_network and hasattr(self.decider_network, "_decider_state"):
            ds = self.decider_network._decider_state
            if ds and hasattr(ds, "stack") and ds.stack:
                decision_stack = "\n".join(
                    f"{'  ' * i}{s}" for i, s in enumerate(str(s) for s in ds.stack)
                )

        if self._monitor_fn:
            self._monitor_fn(f"{decision_stack}{vla_action_info}{diagnostic}", decision_stack)

    # ------------------------------------------------------------------
    # Physics stepping
    # ------------------------------------------------------------------

    def _on_physics_step(self, step_size):
        self._physics_step_count += 1
        world = self.get_world()

        if self._use_direct_ros2 and self._direct_ros2_context is not None:
            try:
                self._direct_ros2_context.update_from_ros()
                self._direct_ros2_context.follow_latest_joints()
            except Exception as e:
                LOGGER(f"Direct ROS 2 follower error: {e}")
                traceback.print_exc()
            return

        try:
            world.step(False, False)
        except Exception as e:
            if self._physics_step_count % 100 == 0:
                LOGGER(f"Physics step error: {type(e).__name__}: {e}")
                traceback.print_exc()

    async def on_event_async(self):
        LOGGER("on_event_async called")
        world = self.get_world()
        try:
            await omni.kit.app.get_app().next_update_async()
            world.reset_cortex()
            world.add_physics_callback("sim_step", self._on_physics_step)
            await world.play_async()
            LOGGER("World is now playing")
        except Exception as e:
            LOGGER(f"on_event_async exception: {e}\n{traceback.format_exc()}")

    async def setup_pre_reset(self):
        world = self.get_world()
        if world and world.physics_callback_exists("sim_step"):
            world.remove_physics_callback("sim_step")

    def world_cleanup(self):
        if self._direct_ros2_context:
            LOGGER("Cleaning up ROS 2 follower context")
            if hasattr(self._direct_ros2_context, "shutdown"):
                self._direct_ros2_context.shutdown()
            self._direct_ros2_context = None
