#!/bin/bash
# ============================================================
# SpaceMouse Hand Teleop — drives the simulated human hand in Isaac Sim
# ============================================================
# Reads SpaceMouse linear axes, integrates them into an absolute hand
# position, and publishes:
#   /hand/cmd_pose   geometry_msgs/PoseStamped  (absolute world pose)
#   /hand/grab_held  std_msgs/Bool              (toggled by BTN_1)
#
# The Isaac Sim ur_robotiq_cortex extension subscribes to these topics
# and renders/animates the hand, performs grab/release, and publishes:
#   /hand/pose             geometry_msgs/PoseStamped (authoritative)
#   /hand/closest_to       std_msgs/String           (cuboid name)
#   /hand/moving_towards   std_msgs/String           (cuboid name or "")
#   /hand/holding          std_msgs/String           (cuboid name or "")
#
# This script does NOT touch ros2_control / forward_velocity_controller —
# the hand is a simulated proxy, not the UR5.
#
# Prerequisites (one-time):
#   pip install pyspacemouse
#   udev rules for non-root SpaceMouse access (see run_spacemouse_teleop.sh)
#
# Controls:
#   Move SpaceMouse  → hand moves in 3D (x, y, z)
#   BTN_1 (idx 5)    → toggle grab / release nearest cuboid
#   BTN_3 (idx 7)    → reset hand to anchor
#   Ctrl-C           → quit
# ============================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- environment cleanup (mirrors run_spacemouse_teleop.sh) ----
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "Deactivating conda environment: $CONDA_PREFIX"
    export PATH=${PATH//$CONDA_PREFIX\/bin:/}
    export PATH=${PATH//$CONDA_PREFIX\/bin/}
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi

unset GTK_PATH GTK2_RC_FILES GTK_IM_MODULE QT_QPA_PLATFORMTHEME LD_PRELOAD PYTHONPATH
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"

# ---- ROS 2 setup ----
set +u
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

echo "Starting SpaceMouse → simulated hand teleop ..."
echo "  Move SpaceMouse → hand moves"
echo "  BTN_1 (idx 5)   → toggle grab"
echo "  BTN_3 (idx 7)   → reset hand to anchor"
echo ""

exec ros2 run ur5_spacemouse_teleop spacemouse_hand_teleop_node --ros-args \
    -p max_lin_vel:=0.30 \
    -p control_hz:=60.0 \
    -p deadzone:=0.1 \
    -p anchor:='[0.60, 0.0, 0.20]'
