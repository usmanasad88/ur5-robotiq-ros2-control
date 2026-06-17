#!/bin/bash
# Run script for UR5 Quest 3S Teleop
# Uses oculus_reader (ADB-based) instead of OpenVR/SteamVR.
#
# Prerequisites (one-time setup):
#   1. pip install git+https://github.com/rail-berkeley/oculus_reader.git
#   2. sudo apt install android-tools-adb
#   3. Enable Developer Mode on Quest 3S:
#        Meta Quest smartphone app → Devices → [your headset] → Developer Mode: ON
#   4. Connect Quest 3S via USB-C, put on headset, accept ADB authorization prompt
#   5. Verify: adb devices   (should list your Quest)
#
# Controls:
#   Grip button (side)   = deadman switch  (hold to move robot)
#   Joystick click       = reset forward direction
#   Trigger (front)      = gripper open/close (published on /ur5/gripper_cmd)

set -eo pipefail

# ---------------------------------------------------------------------------
# Environment cleanup (same as other run scripts - removes conda/snap conflicts)
# ---------------------------------------------------------------------------
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "Deactivating conda environment: $CONDA_PREFIX"
    export PATH=${PATH//$CONDA_PREFIX\/bin:/}
    export PATH=${PATH//$CONDA_PREFIX\/bin/}
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi

unset GTK_PATH GTK2_RC_FILES GTK_IM_MODULE QT_QPA_PLATFORMTHEME LD_PRELOAD PYTHONPATH
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"

# ---------------------------------------------------------------------------
# ROS 2 setup
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# ---------------------------------------------------------------------------
# ADB check
# ---------------------------------------------------------------------------
if ! command -v adb &>/dev/null; then
    echo "ERROR: adb not found. Install with: sudo apt install android-tools-adb"
    exit 1
fi

DEVICES=$(adb devices | grep -v "List of devices" | grep -v "^$" | grep -v "offline" || true)
if [[ -z "$DEVICES" ]]; then
    echo "ERROR: No ADB device found."
    echo "  - Make sure Quest 3S is connected via USB-C"
    echo "  - Put on headset and accept the ADB authorization prompt"
    echo "  - Run: adb devices"
    exit 1
fi
echo "ADB device(s) detected:"
echo "$DEVICES"
echo ""

# ---------------------------------------------------------------------------
# Launch
# ---------------------------------------------------------------------------
echo "Running UR5 Quest 3S Teleop..."

if [ $# -eq 0 ]; then
    echo "No arguments provided. Using defaults:"
    echo "  -p controller_side:=right"
    echo "  -p scale_translation:=5.0"
    echo "  -p scale_rotation:=0.5"
    echo "  -p publish_rate:=50.0"
    echo "  -p rmat_reorder:=[-2,-1,-3,4]"
    echo ""
    echo "Tip: If axes feel inverted, try adjusting rmat_reorder."
    echo "     E.g. for mirrored X: -p rmat_reorder:=[2,-1,-3,4]"
    exec ros2 run ur5_vr_teleop quest_teleop_node --ros-args \
        -p controller_side:=right \
        -p scale_translation:=5.0 \
        -p scale_rotation:=0.5 \
        -p publish_rate:=50.0
else
    echo "Arguments: $@"
    exec ros2 run ur5_vr_teleop quest_teleop_node --ros-args "$@"
fi
