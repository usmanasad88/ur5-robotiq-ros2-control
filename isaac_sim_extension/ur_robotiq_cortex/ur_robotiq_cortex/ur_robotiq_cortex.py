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

import time

from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid, VisualSphere
from isaacsim.cortex.framework.cortex_utils import load_behavior_module
from pxr import UsdGeom
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
ENABLE_OBJECT_COLLISIONS = True

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


class ObjectPosePublisher:
    """Publishes object poses to ROS 2 on /scene/object_poses (geometry_msgs/PoseArray).

    - Publishes initial poses once on the first call to publish().
    - Publishes dynamic poses every `publish_hz` physics steps thereafter.
    """

    TOPIC = "/scene/object_poses"
    INITIAL_TOPIC = "/scene/object_initial_poses"

    def __init__(self, prim_paths: list, publish_hz: float = 30.0, physics_hz: float = 60.0):
        self._prim_paths = prim_paths
        self._step_interval = max(1, int(physics_hz / publish_hz))
        self._step_count = 0
        self._initial_published = False

        self._node = None
        self._pub_dynamic = None
        self._pub_initial = None

        try:
            import rclpy
            from geometry_msgs.msg import PoseArray, Pose
            from std_msgs.msg import Header
            self._rclpy = rclpy
            self._PoseArray = PoseArray
            self._Pose = Pose
            self._Header = Header
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node("isaac_object_pose_publisher")
            self._pub_dynamic = self._node.create_publisher(PoseArray, self.TOPIC, 10)
            self._pub_initial = self._node.create_publisher(PoseArray, self.INITIAL_TOPIC, 10)
            LOGGER(f"ObjectPosePublisher: publishing to {self.TOPIC} and {self.INITIAL_TOPIC}")
        except Exception as e:
            LOGGER(f"ObjectPosePublisher: ROS 2 unavailable — {e}")

    def _build_pose_array(self, stage):
        # Import locally so Pyright doesn't complain about missing stubs at analysis time
        from geometry_msgs.msg import PoseArray, Pose  # noqa: F401
        msg = PoseArray()
        msg.header.frame_id = "world"
        node = self._node  # already confirmed non-None by callers
        try:
            msg.header.stamp = node.get_clock().now().to_msg()
        except Exception:
            pass

        poses = []
        for prim_path in self._prim_paths:
            prim = stage.GetPrimAtPath(prim_path)
            if not prim.IsValid():
                continue
            xformable = UsdGeom.Xformable(prim)
            world_xform = xformable.ComputeLocalToWorldTransform(0)
            translation = world_xform.ExtractTranslation()
            rotation = world_xform.ExtractRotationQuat()
            imag = rotation.GetImaginary()

            pose = Pose()
            pose.position.x = float(translation[0])
            pose.position.y = float(translation[1])
            pose.position.z = float(translation[2])
            pose.orientation.w = float(rotation.GetReal())
            pose.orientation.x = float(imag[0])
            pose.orientation.y = float(imag[1])
            pose.orientation.z = float(imag[2])
            poses.append(pose)
        msg.poses = poses
        return msg

    def publish(self, stage):
        if self._node is None or self._pub_initial is None or self._pub_dynamic is None:
            return

        self._step_count += 1

        # Publish initial poses once (on first call, after the scene is ready)
        if not self._initial_published:
            try:
                msg = self._build_pose_array(stage)
                self._pub_initial.publish(msg)
                LOGGER(f"ObjectPosePublisher: published initial poses for {len(msg.poses)} objects")
                self._initial_published = True
            except Exception as e:
                LOGGER(f"ObjectPosePublisher: error publishing initial poses — {e}")

        # Publish dynamic poses at the configured rate
        if self._step_count % self._step_interval == 0:
            try:
                msg = self._build_pose_array(stage)
                self._pub_dynamic.publish(msg)
            except Exception as e:
                LOGGER(f"ObjectPosePublisher: error publishing dynamic poses — {e}")

    def shutdown(self):
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None


class SpaceMouseHandController:
    """Drives a hand-shaped visual cluster from SpaceMouse and publishes hand state.

    Subscribes
    ----------
    /hand/cmd_pose   geometry_msgs/PoseStamped — absolute hand world pose
                                                 from spacemouse_hand_teleop_node
    /hand/grab_held  std_msgs/Bool             — true while grabbing
                                                 (toggled by BTN_1 in the node)

    Publishes
    ---------
    /hand/pose             PoseStamped — authoritative hand world pose
    /hand/closest_to       String      — graspable name with smallest
                                          euclidean distance to the hand
                                          ("none" if no graspables)
    /hand/moving_towards   String      — graspable name whose direction-from-hand
                                          best aligns with hand velocity (cosine
                                          similarity above MOVE_COS_THRESHOLD).
                                          "none" if hand is too slow or alignment
                                          is below threshold for all candidates.
    /hand/holding          String      — currently grabbed graspable, "none" if
                                          released.

    Visualization
    -------------
    Renders the hand as one flat palm cuboid + 5 finger spheres, all parented
    notionally to the hand position (offsets are world-frame because the hand
    only tracks position, not orientation).
    """

    CMD_POSE_TOPIC = "/hand/cmd_pose"
    GRAB_TOPIC     = "/hand/grab_held"
    POSE_TOPIC     = "/hand/pose"
    CLOSEST_TOPIC  = "/hand/closest_to"
    MOVING_TOPIC   = "/hand/moving_towards"
    HOLDING_TOPIC  = "/hand/holding"

    POSE_STALE_TIMEOUT  = 1.0     # s — release if cmd_pose stops arriving
    MIN_SPEED_FOR_INTENT = 0.05    # m/s — below this, "moving_towards" = none
    MOVE_COS_THRESHOLD   = 0.6     # ≈ 53° cone — required alignment
    GRAB_RADIUS          = 0.15    # m — only grab a cuboid within this radius
    PUBLISH_HZ           = 30.0
    PHYSICS_HZ           = 60.0
    VEL_FILTER_ALPHA     = 0.3     # EMA smoothing for hand velocity

    HAND_PARENT_PRIM = "/World/Hand"
    HAND_COLOR       = np.array([0.95, 0.75, 0.60])  # skin tone

    # Hand-local offsets (world-frame, since hand has no orientation).
    # Palm centred at the hand position; fingers spread along +X.
    _PALM_SIZE      = np.array([0.10, 0.07, 0.02])
    _FINGER_OFFSETS = [
        # (offset, radius, label)
        (np.array([0.030,  0.045,  0.000]), 0.014, "thumb"),
        (np.array([0.075,  0.025,  0.000]), 0.011, "index"),
        (np.array([0.080,  0.000,  0.000]), 0.011, "middle"),
        (np.array([0.075, -0.025,  0.000]), 0.011, "ring"),
        (np.array([0.065, -0.045,  0.000]), 0.010, "pinky"),
    ]

    def __init__(self, world, graspable_names, anchor_position=(0.6, 0.0, 0.20)):
        self._world = world
        self._graspable_names = list(graspable_names)
        self._anchor_pos = np.array(anchor_position, dtype=np.float64)
        self._hand_pos   = self._anchor_pos.copy()
        self._prev_pos   = self._anchor_pos.copy()
        self._velocity   = np.zeros(3, dtype=np.float64)
        self._last_pose_time   = 0.0
        self._last_update_time = time.monotonic()
        self._grab_held    = False
        self._prev_grab    = False
        self._grabbed_name = None

        self._debug = os.environ.get("UR_HAND_DEBUG", "1").lower() in ("1", "true", "yes", "on")
        self._pose_count = 0
        self._grab_msg_count = 0
        self._update_count = 0
        self._publish_interval = max(1, int(self.PHYSICS_HZ / self.PUBLISH_HZ))

        # ---- visualize a hand cluster (palm + 5 fingers) ----
        self._hand_parts = []  # list[(prim_object, np.array offset)]
        try:
            palm = world.scene.add(
                VisualCuboid(
                    prim_path=f"{self.HAND_PARENT_PRIM}/Palm",
                    name="hand_palm",
                    position=self._hand_pos,
                    scale=self._PALM_SIZE,
                    color=self.HAND_COLOR,
                )
            )
            self._hand_parts.append((palm, np.zeros(3)))
        except Exception as e:
            LOGGER(f"SpaceMouseHandController: could not spawn palm mesh: {e}")

        for offset, radius, label in self._FINGER_OFFSETS:
            try:
                tip = world.scene.add(
                    VisualSphere(
                        prim_path=f"{self.HAND_PARENT_PRIM}/Finger_{label}",
                        name=f"hand_finger_{label}",
                        position=self._hand_pos + offset,
                        radius=radius,
                        color=self.HAND_COLOR,
                    )
                )
                self._hand_parts.append((tip, offset.copy()))
            except Exception as e:
                LOGGER(f"SpaceMouseHandController: could not spawn finger {label}: {e}")

        # ---- ROS 2 sub/pub ----
        self._node = None
        self._rclpy = None
        self._pose_pub = None
        self._closest_pub = None
        self._moving_pub = None
        self._holding_pub = None
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Bool, String
            self._rclpy = rclpy
            self._PoseStamped = PoseStamped
            self._String = String
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node("isaac_spacemouse_hand")
            self._pose_sub = self._node.create_subscription(
                PoseStamped, self.CMD_POSE_TOPIC, self._on_cmd_pose, 10
            )
            self._grab_sub = self._node.create_subscription(
                Bool, self.GRAB_TOPIC, self._on_grab, 10
            )
            self._pose_pub    = self._node.create_publisher(PoseStamped, self.POSE_TOPIC, 10)
            self._closest_pub = self._node.create_publisher(String, self.CLOSEST_TOPIC, 10)
            self._moving_pub  = self._node.create_publisher(String, self.MOVING_TOPIC, 10)
            self._holding_pub = self._node.create_publisher(String, self.HOLDING_TOPIC, 10)
            LOGGER(
                f"SpaceMouseHandController: subscribed to {self.CMD_POSE_TOPIC} (PoseStamped) "
                f"and {self.GRAB_TOPIC} (Bool); publishing /hand/{{pose,closest_to,moving_towards,holding}}"
            )
        except Exception as e:
            LOGGER(f"SpaceMouseHandController: ROS 2 setup failed — {e}")

    # ---- subscribers ----

    def _on_cmd_pose(self, msg):
        # Absolute hand position in world frame.
        self._hand_pos[0] = float(msg.pose.position.x)
        self._hand_pos[1] = float(msg.pose.position.y)
        self._hand_pos[2] = float(msg.pose.position.z)
        self._last_pose_time = time.monotonic()
        self._pose_count += 1

    def _on_grab(self, msg):
        self._grab_msg_count += 1
        self._grab_held = bool(msg.data)

    # ---- per-physics-step update ----

    def update(self):
        self._update_count += 1
        if self._node is not None and self._rclpy is not None:
            for _ in range(4):
                try:
                    self._rclpy.spin_once(self._node, timeout_sec=0.0)
                except Exception as e:
                    if self._debug and self._update_count % 300 == 1:
                        LOGGER(f"SpaceMouseHandController: spin_once error: {e}")
                    break

        # Velocity from finite-difference + EMA smoothing.
        now = time.monotonic()
        dt = max(1e-3, now - self._last_update_time)
        self._last_update_time = now
        instant_vel = (self._hand_pos - self._prev_pos) / dt
        self._velocity = (
            self.VEL_FILTER_ALPHA * instant_vel
            + (1.0 - self.VEL_FILTER_ALPHA) * self._velocity
        )
        self._prev_pos = self._hand_pos.copy()

        # Stale pose → drop a held grab so a dead teleop doesn't pin objects.
        pose_fresh = (
            self._last_pose_time > 0.0
            and (now - self._last_pose_time) < self.POSE_STALE_TIMEOUT
        )

        # Grab edge detection (with stale-pose safety).
        wants_grab = self._grab_held and pose_fresh
        if wants_grab and not self._prev_grab:
            self._try_grab()
        elif not wants_grab and self._prev_grab:
            if self._debug:
                reason = "released" if not self._grab_held else "pose stale"
                LOGGER(f"Hand grab RELEASED ({reason})")
            self._release()
        self._prev_grab = wants_grab

        # Move the visual hand parts.
        for prim, offset in self._hand_parts:
            try:
                prim.set_world_pose(position=self._hand_pos + offset)
            except Exception:
                pass

        # Pin the held object to the palm centre.
        if self._grabbed_name is not None:
            obj = self._world.scene.get_object(self._grabbed_name)
            if obj is not None:
                try:
                    obj.set_world_pose(position=self._hand_pos)
                    if hasattr(obj, "set_linear_velocity"):
                        obj.set_linear_velocity(np.zeros(3))
                    if hasattr(obj, "set_angular_velocity"):
                        obj.set_angular_velocity(np.zeros(3))
                except Exception:
                    pass

        # Compute and publish hand state at PUBLISH_HZ.
        if self._update_count % self._publish_interval == 0:
            self._publish_state()

        if self._debug and (self._update_count == 1 or self._update_count % 300 == 0):
            sub_state = "no node" if self._node is None else "subscribed"
            LOGGER(
                f"Hand heartbeat: updates={self._update_count} "
                f"poses={self._pose_count} grab_msgs={self._grab_msg_count} "
                f"grab_held={self._grab_held} grabbed={self._grabbed_name} "
                f"sub={sub_state} pos=({self._hand_pos[0]:+.3f},{self._hand_pos[1]:+.3f},{self._hand_pos[2]:+.3f})"
            )

    # ---- grab logic ----

    def _try_grab(self):
        nearest = None
        nearest_d = self.GRAB_RADIUS
        for name in self._graspable_names:
            obj = self._world.scene.get_object(name)
            if obj is None:
                continue
            try:
                pos, _ = obj.get_world_pose()
                d = float(np.linalg.norm(np.array(pos) - self._hand_pos))
            except Exception:
                continue
            if d < nearest_d:
                nearest_d = d
                nearest = name
        if nearest is not None:
            self._grabbed_name = nearest
            LOGGER(f"SpaceMouseHandController: grabbed {nearest} (d={nearest_d:.3f}m)")
        else:
            LOGGER("SpaceMouseHandController: grab requested but no object within radius")

    def _release(self):
        if self._grabbed_name is not None:
            LOGGER(f"SpaceMouseHandController: released {self._grabbed_name}")
            self._grabbed_name = None

    # ---- state computation + publishing ----

    def _compute_closest_and_moving(self):
        """Return (closest_name, moving_towards_name). Both may be 'none'."""
        if not self._graspable_names:
            return "none", "none"

        speed = float(np.linalg.norm(self._velocity))
        v_hat = self._velocity / speed if speed > 1e-6 else None

        closest_name, closest_d = "none", float("inf")
        best_name, best_score   = "none", -1.0

        for name in self._graspable_names:
            obj = self._world.scene.get_object(name)
            if obj is None:
                continue
            try:
                obj_pos, _ = obj.get_world_pose()
            except Exception:
                continue
            rel = np.array(obj_pos, dtype=np.float64) - self._hand_pos
            d = float(np.linalg.norm(rel))
            if d < closest_d:
                closest_d = d
                closest_name = name

            if v_hat is not None and d > 1e-6:
                rel_hat = rel / d
                score = float(np.dot(v_hat, rel_hat))
                if score > best_score:
                    best_score = score
                    best_name = name

        if speed < self.MIN_SPEED_FOR_INTENT or best_score < self.MOVE_COS_THRESHOLD:
            moving_name = "none"
        else:
            moving_name = best_name

        return closest_name, moving_name

    def _publish_state(self):
        if self._node is None or self._pose_pub is None:
            return
        try:
            stamp = self._node.get_clock().now().to_msg()
        except Exception:
            stamp = None

        # /hand/pose
        try:
            pose_msg = self._PoseStamped()
            pose_msg.header.frame_id = "world"
            if stamp is not None:
                pose_msg.header.stamp = stamp
            pose_msg.pose.position.x = float(self._hand_pos[0])
            pose_msg.pose.position.y = float(self._hand_pos[1])
            pose_msg.pose.position.z = float(self._hand_pos[2])
            pose_msg.pose.orientation.w = 1.0
            self._pose_pub.publish(pose_msg)
        except Exception as e:
            if self._update_count % 300 == 0:
                LOGGER(f"SpaceMouseHandController: pose publish error — {e}")

        # /hand/closest_to, /hand/moving_towards, /hand/holding
        try:
            closest_name, moving_name = self._compute_closest_and_moving()
            holding_name = self._grabbed_name if self._grabbed_name is not None else "none"
            self._closest_pub.publish(self._String(data=closest_name))
            self._moving_pub.publish(self._String(data=moving_name))
            self._holding_pub.publish(self._String(data=holding_name))
        except Exception as e:
            if self._update_count % 300 == 0:
                LOGGER(f"SpaceMouseHandController: state publish error — {e}")

    def shutdown(self):
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None


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

        # Object pose publisher (set up in setup_scene)
        self._object_pose_publisher = None

        # SpaceMouse-driven hand controller (set up in setup_scene)
        self._spacemouse_hand_controller = None

        # Cuboid bookkeeping for re-randomization on reset
        self._cuboid_specs = []
        self._cuboid_positions = {}  # name -> (x, y) last sampled
        self._cuboid_rng = np.random.default_rng()
        self._named_positions_path = None

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
            ("basket_blue.glb", "basket_blue", [0.53, -0.5, -0.02], [0.3, 0.3, 0.3], [0.70710677, 0, 0, -0.70710677]),
            # For the balls, x may be between 0.55 to 1.05 ; y may be between -0.32 to +0.30
            # ("blue_ball.glb", "blue_ball", [0.69, -0.32, -0.05], [0.06, 0.06, 0.06], [1, 0, 0, 0]),
            # ("punch.glb", "punch", [0.60, -0.10, -0.05], [0.1, 0.1, 0.1], [0.70710677, 0, 0, -0.70710677]),
            # ("purple_ball.glb", "purple_ball", [0.80352, -0.18857, -0.05], [0.06, 0.06, 0.06], [1, 0, 0, 0]),
            # ("soccer_ball.glb", "soccer_ball", [0.8695, -0.30667, -0.05], [0.06, 0.06, 0.06], [1, 0, 0, 0]),
            
            # ("box.glb", "box", [0.020250664143844695, -0.799380035436923, -0.003023436160052201], [0.3, 0.3, 0.3], [0.67167205, 0, 0, 0.7408486]),
            # ("bottle.glb", "bottle", [0.5883761388085353, 0.3891037747941267, -0.0015232706561414185], [0.15, 0.15, 0.15], [0.89517987, 0, 0, -0.44570506]),
            # ("bottle.glb", "bottle_01", [0.37399906533516813, -0.7416339292944675, -0.006991196782229553], [0.15, 0.15, 0.15], [0.87884057, 0, 0, -0.47711557]),
            # ("mold.glb", "mold", [0.9947561783427199, 0.03842857747566735, -0.03418280632764703], [0.2, 0.2, 0.2], [6.123234e-17, 1, 0, 0]),
            # ("roller.glb", "roller", [0.009414017246025964, -0.6610178444917778, 0.07770526356954245], [0.2, 0.2, 0.2], [0.9907924, -0.13035351, 0.028021853, 0.023519594]),
            # ("scale.usd", "scale", [0.9796966110997245, 0.47066618528330484, -0.050753143391285616], [0.2, 0.2, 0.2], [0.70710677, 0, 0, 0.70710677]),

            ("chair.glb", "chair", [2.654924188733815, -0.7996137486341647, -0.4433299999999999], [0.6, 0.6, 0.6], [0.6846738, 0, 0, -0.72884965]),
            ("table.glb", "table", [0.52883, -0.11806, -0.49688], [1.22, 1.27, 1.53], [0.70710677, 0, 0, -0.70710677]),           
            ("table.glb", "table_01", [-0.08974, -1.03653, -0.49688], [1.22, 1.27, 1.53], [0, 0, 0, 1]),
            ("chair.glb", "chair_01", [1.5211308724811525, 0.11280207853649299, -0.4129617690059313], [0.7, 0.7, 0.7], [0.6846738, 0, 0, -0.72884965]),
            ("chair.glb", "chair_02", [0.5518825573150291, -1.4596548944824432, -0.44333000000000217], [0.7, 0.7, 0.7], [0.021866286, 0, 0, -0.9997609]),
            ("chair.glb", "chair_03", [-0.030057869207804636, -1.4888103302584828, -0.44332999999999656], [0.7, 0.7, 0.7], [0.021866286, 0, 0, -0.9997609]),
            ("human.usd", "human", [1.55496987436995, 0.08527604402904952, -0.14375425001112657], [1, 1, 1], [0.96886265, 0, 0, 0.24759878],
             {"xformOp:rotateX:unitsResolve": (Sdf.ValueTypeNames.Double, 90.0),
              "xformOp:scale:unitsResolve": (Sdf.ValueTypeNames.Double3, Gf.Vec3d(0.75, 0.75, 0.75))}),
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

        for obj_entry in objects:
            obj_file, obj_name, position, scale, orientation = obj_entry[:5]
            extra_xforms = obj_entry[5] if len(obj_entry) > 5 else None
            try:
                obj_path = str(ur_ws / f"isaac_standalone/Objects/{obj_file}")
                if os.path.exists(obj_path):
                    prim_path = f"/World/Objects/{obj_name}"
                    add_reference_to_stage(usd_path=obj_path, prim_path=prim_path)
                    set_prim_transform(prim_path, position, scale, orientation)
                    if extra_xforms:
                        prim = stage.GetPrimAtPath(prim_path)
                        if prim.IsValid():
                            for attr_name, (type_name, value) in extra_xforms.items():
                                attr = prim.GetAttribute(attr_name)
                                if not attr.IsValid():
                                    attr = prim.CreateAttribute(attr_name, type_name)
                                attr.Set(value)
                            # Rebuild xformOpOrder to include extras
                            base_ops = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
                            order_attr = prim.GetAttribute("xformOpOrder")
                            if not order_attr.IsValid():
                                order_attr = prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray, True)
                            order_attr.Set(base_ops + list(extra_xforms.keys()))
                    if ENABLE_OBJECT_COLLISIONS:
                        # Easily reversible: change True to False to disable deformable balls
                        if "ball" in obj_name and False:
                            try:
                                from omni.physx.scripts import deformableUtils
                                from pxr import PhysxSchema
                                inner_path = f"{prim_path}/geometry_0/geometry_0"
                                
                                deformableUtils.create_auto_volume_deformable_hierarchy(
                                    stage = stage,
                                    root_prim_path = prim_path,
                                    simulation_tetmesh_path = f"{prim_path}/SimulationMesh",
                                    collision_tetmesh_path = f"{prim_path}/CollisionMesh",
                                    cooking_src_mesh_path = inner_path,
                                    simulation_hex_mesh_enabled = True,
                                    cooking_src_simplification_enabled = True,
                                    set_visibility_with_guide_purpose = True
                                )
                                
                                rootPrim = stage.GetPrimAtPath(prim_path)
                                rootPrim.ApplyAPI("PhysxBaseDeformableBodyAPI")
                                if rootPrim.HasAPI("PhysxBaseDeformableBodyAPI"):
                                    rootPrim.GetAttribute("physxDeformableBody:disableGravity").Set(False)
                                    
                                # Increase rigidity by creating and assigning a stiff material
                                from pxr import UsdShade
                                mat_path = f"{prim_path}/StiffDeformableMaterial"
                                mat = UsdShade.Material.Define(stage, mat_path)
                                mat_prim = mat.GetPrim()
                                
                                mat_prim.ApplyAPI("OmniPhysicsBaseMaterialAPI")
                                mat_prim.GetAttribute("omniphysics:dynamicFriction").Set(0.5)
                                
                                mat_prim.ApplyAPI("OmniPhysicsDeformableMaterialAPI")
                                mat_prim.GetAttribute("omniphysics:youngsModulus").Set(5e9) # High stiffness (e.g. 5e7)
                                mat_prim.GetAttribute("omniphysics:poissonsRatio").Set(0.45)
                                
                                mat_prim.ApplyAPI("PhysxDeformableMaterialAPI")
                                mat_prim.GetAttribute("physxDeformableMaterial:elasticityDamping").Set(0.05)
                                
                                binding_api = UsdShade.MaterialBindingAPI.Apply(rootPrim)
                                binding_api.Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
                                    
                                colPrim = stage.GetPrimAtPath(f"{prim_path}/CollisionMesh")
                                if colPrim.IsValid():
                                    physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(colPrim)
                                    if physxCollisionAPI:
                                        physxCollisionAPI.GetContactOffsetAttr().Set(0.01)
                                        physxCollisionAPI.GetRestOffsetAttr().Set(0.005)
                            except Exception as edef:
                                LOGGER(f"Failed to make {obj_name} deformable: {edef}")
                                add_sdf_collision(prim_path)
                        else:
                            add_sdf_collision(prim_path)
                    LOGGER(f"Loaded {obj_name}")
                else:
                    LOGGER(f"Object file not found: {obj_path}")
            except Exception as e:
                LOGGER(f"Error adding {obj_name}: {e}")

        # Spawn coloured cuboids (scale 4x4x6 cm) at random positions on the table.
        # x in [0.55, 1.05], y in [-0.32, 0.30], z = -0.05
        cuboid_specs = [
            ("cuboid_red",    np.array([1.0, 0.0, 0.0])),
            ("cuboid_green",  np.array([0.0, 0.8, 0.0])),
            ("cuboid_blue",   np.array([0.0, 0.3, 1.0])),
            ("cuboid_yellow", np.array([1.0, 0.9, 0.0])),
            ("cuboid_orange", np.array([1.0, 0.5, 0.0])),
        ]
        self._cuboid_specs = cuboid_specs
        cuboid_scale = np.array([0.04, 0.04, 0.06])
        for cub_name, cub_color in cuboid_specs:
            cx = float(self._cuboid_rng.uniform(0.40, 0.65))
            cy = float(self._cuboid_rng.uniform(-0.32, 0.30))
            self._cuboid_positions[cub_name] = (cx, cy)
            try:
                world.scene.add(
                    DynamicCuboid(
                        prim_path=f"/World/Objects/{cub_name}",
                        name=cub_name,
                        position=np.array([cx, cy, -0.05]),
                        scale=cuboid_scale,
                        color=cub_color,
                    )
                )
                LOGGER(f"Spawned {cub_name} at ({cx:.3f}, {cy:.3f}, -0.05)")
            except Exception as e:
                LOGGER(f"Error spawning {cub_name}: {e}")

        # Resolve named_positions.txt once and cache for reset-time rewrites.
        named_pos_path = None
        for candidate in [ur_ws / "src/ur5_curobo_control/config/named_positions.txt"]:
            if candidate.exists():
                named_pos_path = candidate
                break
        if named_pos_path is None:
            matches = list(ur_ws.rglob("named_positions.txt"))
            if matches:
                named_pos_path = matches[0]
        self._named_positions_path = named_pos_path

        self._write_cuboid_pick_poses()

        # Create the pose publisher for all cuboids
        cuboid_prim_paths = [f"/World/Objects/{name}" for name, _ in cuboid_specs]
        self._object_pose_publisher = ObjectPosePublisher(cuboid_prim_paths)

        # SpaceMouse-driven hand with proximity grab over the cuboids.
        self._spacemouse_hand_controller = SpaceMouseHandController(
            world,
            graspable_names=[name for name, _ in cuboid_specs],
        )

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
                trans_attr.Set(Gf.Vec3d(0, 0, -0.9271))

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
                # Manually set the order list
                order_attr.Set(["xformOp:translate", "xformOp:orient", "xformOp:scale"])

                LOGGER("Added ground plane at z=-0.9271")
            else:
                LOGGER("Could not create ground plane prim")
        except Exception as e:
            LOGGER(f"Error adding ground plane: {e}")

        # # Add deformable plane
        # try:
        #     from pxr import Vt, UsdGeom
        #     from omni.physx.scripts import deformableUtils
            
        #     def create_trimesh(stage, path, res):
        #         triMesh = UsdGeom.Mesh.Define(stage, path)
        #         step = 1.0 / res
        #         verts = [(i * step, j * step, 0.0) for j in range(res + 1) for i in range(res + 1)]
        #         idx = lambda i, j: j * (res + 1) + i
        #         tris = [(idx(i,j), idx(i+1,j), idx(i+1,j+1)) + (idx(i,j), idx(i+1,j+1), idx(i,j+1))
        #                 for j in range(res) for i in range(res)]
        #         triMesh.GetPointsAttr().Set(Vt.Vec3fArray(verts))
        #         triMesh.GetFaceVertexCountsAttr().Set([3] * (2 * res**2))
        #         triMesh.GetFaceVertexIndicesAttr().Set([i for t in tris for i in t])
        #         return triMesh

        #     deformable_path = "/World/DeformablePlane"
        #     triMesh = create_trimesh(stage, deformable_path, 20)
        #     prim = triMesh.GetPrim()

        #     # Set the offset so it falls near the table
        #     triMesh.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 0.5))
            
        #     # Use deformableUtils script
        #     success = deformableUtils.set_physics_surface_deformable_body(stage, prim.GetPath())
            
        #     prim.ApplyAPI("PhysxSurfaceDeformableBodyAPI")
        #     if prim.HasAPI("PhysxSurfaceDeformableBodyAPI"):
        #         prim.GetAttribute("physxDeformableBody:selfCollision").Set(True)
                
        #     LOGGER("Added deformable plane")
        # except Exception as e:
        #     LOGGER(f"Error adding deformable plane: {e}")

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

        # Publish object poses every step (publisher throttles internally)
        if self._object_pose_publisher is not None:
            try:
                stage = omni.usd.get_context().get_stage()
                self._object_pose_publisher.publish(stage)
            except Exception as e:
                if self._physics_step_count % 300 == 0:
                    LOGGER(f"Object pose publisher error: {e}")

        if self._spacemouse_hand_controller is not None:
            try:
                self._spacemouse_hand_controller.update()
            except Exception as e:
                if self._physics_step_count % 300 == 0:
                    LOGGER(f"SpaceMouse hand controller error: {e}")

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

    def _write_cuboid_pick_poses(self):
        """Rewrite the cuboid_* entries in named_positions.txt using the last sampled (x, y)."""
        named_pos_path = self._named_positions_path
        if named_pos_path is None or not self._cuboid_specs:
            LOGGER("named_positions.txt not found — skipping cuboid pose export")
            return
        PICK_Z = 0.06
        PICK_QUAT = "0.0 1.0 0.0 0.0"
        try:
            lines = named_pos_path.read_text().splitlines()
            lines = [l for l in lines if not l.strip().startswith("pose cuboid_")]
            while lines and lines[-1].strip() == "":
                lines.pop()
            lines.append("")
            lines.append("# Isaac Sim cuboid pick poses (auto-generated)")
            for cub_name, _ in self._cuboid_specs:
                xy = self._cuboid_positions.get(cub_name)
                if xy is None:
                    continue
                cx, cy = xy
                lines.append(f"pose {cub_name} {cx:.5f} {cy:.5f} {PICK_Z} {PICK_QUAT}")
            lines.append("")
            named_pos_path.write_text("\n".join(lines))
            LOGGER(f"Wrote cuboid poses to {named_pos_path}")
        except Exception as e:
            LOGGER(f"Error writing cuboid poses to named_positions.txt: {e}")

    async def setup_post_reset(self):
        """Re-sample cuboid spawn positions so each reset gives fresh locations."""
        world = self.get_world()
        if world is None or not self._cuboid_specs:
            return
        for cub_name, _ in self._cuboid_specs:
            cx = float(self._cuboid_rng.uniform(0.40, 0.65))
            cy = float(self._cuboid_rng.uniform(-0.32, 0.30))
            self._cuboid_positions[cub_name] = (cx, cy)
            position = np.array([cx, cy, -0.05])
            try:
                cub = world.scene.get_object(cub_name)
                if cub is None:
                    continue
                cub.set_default_state(position=position)
                cub.set_world_pose(position=position)
                if hasattr(cub, "set_linear_velocity"):
                    cub.set_linear_velocity(np.zeros(3))
                if hasattr(cub, "set_angular_velocity"):
                    cub.set_angular_velocity(np.zeros(3))
                LOGGER(f"Re-spawned {cub_name} at ({cx:.3f}, {cy:.3f}, -0.05)")
            except Exception as e:
                LOGGER(f"Error re-spawning {cub_name}: {e}")

        self._write_cuboid_pick_poses()

    def world_cleanup(self):
        if self._direct_ros2_context:
            LOGGER("Cleaning up ROS 2 follower context")
            if hasattr(self._direct_ros2_context, "shutdown"):
                self._direct_ros2_context.shutdown()
            self._direct_ros2_context = None

        if self._object_pose_publisher is not None:
            self._object_pose_publisher.shutdown()
            self._object_pose_publisher = None

        if self._spacemouse_hand_controller is not None:
            self._spacemouse_hand_controller.shutdown()
            self._spacemouse_hand_controller = None
