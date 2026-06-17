#!/bin/bash
# ============================================================
# Quest 3S VR Teleop — Simulation (fake hardware)
# ============================================================
# Drives the simulated UR5 via ROS 2:
#   Quest 3S → quest_teleop_node → /ur5/teleop_delta
#                                → cuRobo program_executor (TELEOP mode)
#                                → scaled_joint_trajectory_controller
#
# Prerequisites (one-time):
#   pip install git+https://github.com/rail-berkeley/oculus_reader.git
#   sudo apt install android-tools-adb
#
# Quest 3S setup:
#   1. Enable Developer Mode: Meta Quest app → Devices → Developer Mode ON
#   2. Connect Quest 3S to this PC via USB-C
#   3. Put on headset — accept the ADB authorization prompt
#   4. Verify:  adb devices  (should list your Quest)
#
# Usage:
#   ./run_quest_sim_teleop.sh              # Default params, right controller
#   ./run_quest_sim_teleop.sh --left       # Left controller
#   ./run_quest_sim_teleop.sh --scale 8.0  # Higher translation sensitivity
#   ./run_quest_sim_teleop.sh --no-enable  # Skip auto-enabling cuRobo teleop mode
#
# Controls:
#   Joystick click  = reset forward direction  (do this FIRST)
#   Hold GRIP       = robot moves (deadman switch)
#   Trigger         = gripper open/close (/ur5/gripper_cmd)
#
# Axis tuning (if robot moves the wrong way):
#   Pass ROS params via --ros-args -p rmat_reorder:=[-2,-1,-3,4]
#   Negate an index to flip that axis.  e.g. [2,-1,-3,4] flips X.
# ============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- defaults ----
CONTROLLER="right"
SCALE_TRANS="5.0"
SCALE_ROT="0.5"
AUTO_ENABLE=true

# ---- parse simple flags ----
while [[ $# -gt 0 ]]; do
    case "$1" in
        --left)       CONTROLLER="left";  shift ;;
        --right)      CONTROLLER="right"; shift ;;
        --scale)      SCALE_TRANS="$2";   shift 2 ;;
        --no-enable)  AUTO_ENABLE=false;  shift ;;
        --help|-h)
            sed -n '2,40p' "$0" | grep '^#' | sed 's/^# \?//'
            exit 0 ;;
        *) echo "Unknown option: $1  (use --help)"; exit 1 ;;
    esac
done

# ---- environment cleanup (remove conda/snap conflicts) ----
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "Deactivating conda environment: $CONDA_PREFIX"
    export PATH=${PATH//$CONDA_PREFIX\/bin:/}
    export PATH=${PATH//$CONDA_PREFIX\/bin/}
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi

unset GTK_PATH GTK2_RC_FILES GTK_IM_MODULE QT_QPA_PLATFORMTHEME LD_PRELOAD PYTHONPATH
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"

# ---- ROS 2 setup (disable nounset around source — ROS scripts use unset vars) ----
set +u
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

# ---- ADB check ----
if ! command -v adb &>/dev/null; then
    echo "ERROR: adb not found.  sudo apt install android-tools-adb"
    exit 1
fi

DEVICES=$(adb devices | grep -v "List of devices" | grep -v "^$" | grep -v "offline" || true)
if [[ -z "$DEVICES" ]]; then
    echo "ERROR: No Quest found via ADB."
    echo "  • Connect Quest 3S via USB-C"
    echo "  • Put on headset and accept ADB authorization"
    echo "  • Run:  adb devices"
    exit 1
fi
echo "Quest detected:"
echo "$DEVICES"
echo ""

# ---- launch quest_teleop_node ----
echo "Controller       : $CONTROLLER"
echo "Scale (trans/rot): $SCALE_TRANS / $SCALE_ROT"
echo "Auto-enable teleop: $AUTO_ENABLE"
echo ""
echo "Starting Quest 3S sim teleop..."
echo "  Joystick click  → reset forward direction"
echo "  Hold GRIP       → robot moves"
echo "  Trigger         → gripper (published to /ur5/gripper_cmd)"
echo ""

# auto_enable_teleop=true → the node calls ~/set_teleop_mode True on startup,
# waiting up to 8 s for the program_executor_node to appear.
if [[ "$AUTO_ENABLE" == "true" ]]; then
    exec ros2 run ur5_vr_teleop quest_teleop_node --ros-args \
        -p controller_side:="$CONTROLLER" \
        -p scale_translation:="$SCALE_TRANS" \
        -p scale_rotation:="$SCALE_ROT" \
        -p publish_rate:=50.0 \
        -p auto_enable_teleop:=true
else
    exec ros2 run ur5_vr_teleop quest_teleop_node --ros-args \
        -p controller_side:="$CONTROLLER" \
        -p scale_translation:="$SCALE_TRANS" \
        -p scale_rotation:="$SCALE_ROT" \
        -p publish_rate:=50.0
fi
