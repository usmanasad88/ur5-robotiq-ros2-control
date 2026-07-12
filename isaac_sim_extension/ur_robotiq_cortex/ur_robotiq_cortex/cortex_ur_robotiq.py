"""Robotiq gripper + UR10/Robotiq Cortex robot definitions, vendored locally.

RobotiqGripper, CortexUr10Robotiq, and add_ur10_robotiq_to_stage were previously
hand-added directly into NVIDIA's isaacsim.cortex.framework.robot module in the
Isaac Sim 5.0 install. Isaac Sim 6 ships a pristine robot.py, so those edits are
gone after the upgrade. We keep our own copy here instead: the definitions build
only on stock Cortex base classes (CortexGripper, MotionCommandedRobot), so they
survive Isaac Sim rebuilds/reinstalls without ever touching NVIDIA files.

Copied verbatim from the Isaac Sim 5.0.0 robot.py; only the imports were updated
to the Isaac Sim 6 module paths.
"""

from typing import Optional, Sequence

import numpy as np

import isaacsim.robot_motion.motion_generation.interface_config_loader as icl
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.cortex.framework.cortex_utils import get_assets_root_path_or_die
from isaacsim.cortex.framework.robot import CortexGripper, MotionCommandedRobot


class RobotiqGripper(CortexGripper):
    """Robotiq parallel gripper.

    Specifies the gripper joint, provides mappings from width to joint angle, and defines the
    opened and closed widths.

    Args:
        articulation: The Articulation object containing the finger joint that will be controlled
            by this gripper.
    """

    def __init__(self, articulation: SingleArticulation):
        super().__init__(
            articulation_subset=ArticulationSubset(articulation, ["finger_joint"]),
            opened_width=0.0,
            closed_width=0.8,
        )

    def joints_to_width(self, joint_positions: Sequence[float]) -> float:
        """The width is the joint position directly (single revolute joint).

        Args:
            joint_positions: The value for joint ["finger_joint"].

        Returns:
            The width of the gripper (joint angle) corresponding to the joint position.
        """
        return float(joint_positions[0])

    def width_to_joints(self, width: float) -> np.ndarray:
        """Width maps directly to the revolute joint angle.

        Args:
            width: The width (joint angle) of the gripper

        Returns:
            The value for joint ["finger_joint"] giving the requested gripper width.
        """
        return np.array([width])


class CortexUr10Robotiq(MotionCommandedRobot):
    """The Cortex UR10 Robotiq contains commanders for commanding the end-effector (a MotionCommander
    governing the full arm) and the gripper (a RobotiqGripper governing the fingers).

    Each of these commanders are accessible via members arm and gripper.

    This object only wraps an existing USD UR10 with Robotiq gripper on the stage at the specified prim_path.
    To add it to the stage first then wrap it, use the add_ur10_robotiq_to_stage() method.

    Note that position and orientation are both relative to the prim the UR10 sits on.

    Args:
        name: A name for the UR10 Robotiq robot. Robots added to the CortexWorld should all have unique names.
        prim_path: The path to the UR10 prim in the USD stage.
        position: The position of the robot. See CortexRobot's position parameter for details.
        orientation: The orientation of the robot. See CortexRobot's orientation parameter for details.
    """

    def __init__(
        self,
        name: str,
        prim_path: str,
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
    ):
        motion_policy_config = icl.load_supported_motion_policy_config("UR10", "RMPflowCortex")
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


def add_ur10_robotiq_to_stage(
    name: str,
    prim_path: str,
    usd_path: Optional[str] = None,
    position: Optional[Sequence[float]] = None,
    orientation: Optional[Sequence[float]] = None,
):
    """Adds a UR10 with Robotiq gripper to the stage at the specified prim_path, then wrap it as a CortexUr10Robotiq object.

    Args:
        For name, prim_path, position, and orientation, see the CortexUr10Robotiq doc string.

        usd_path: An optional path to the UR10 Robotiq USD asset to add. If a specific path is not
            provided, a default UR10 USD path is used.

    Returns: The constructed CortexUr10Robotiq object.
    """
    if usd_path is not None:
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
    else:
        # Use default UR10 asset if custom path not provided
        usd_path = get_assets_root_path_or_die() + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

    return CortexUr10Robotiq(name, prim_path, position, orientation)
