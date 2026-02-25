#!/bin/bash
# ============================================================
# Quest 3S VR Teleop  —  ur_rtde Cartesian velocity control
# ============================================================
# Connects directly to the robot via RTDE (bypasses ros2_control).
# Requires launch_all.sh to be running ONLY for the UR5 driver
# (the driver must have brought the robot to EXTERNAL CONTROL mode).
#
# Prerequisites (one-time):
#   pip install ur-rtde
#   pip install git+https://github.com/rail-berkeley/oculus_reader.git
#   sudo apt install android-tools-adb
#
# Quest 3S setup:
#   1. Enable Developer Mode: Meta Quest app → Devices → Developer Mode ON
#   2. Connect Quest 3S to this PC via USB-C
#   3. Put on headset — accept the ADB authorization prompt
#   4. Verify:  adb devices  (should list your Quest)
#
# Controls:
#   Grip button (side)  = deadman switch  (hold to move robot)
#   Joystick click      = reset forward direction  ← do this FIRST
#   Trigger (front)     = gripper  (0 open, 1 closed)
#
# Axis tuning  (if robot moves in the wrong direction):
#   Edit --rmat-reorder below.  Negate an index to flip that axis.
#   e.g.  -2 -1 -3 4  →  flip X only:  2 -1 -3 4
# ============================================================

set -euo pipefail

ROBOT_IP="${ROBOT_IP:-172.17.66.105}"

# ---- ADB check ----
if ! command -v adb &>/dev/null; then
    echo "ERROR: adb not found.  sudo apt install android-tools-adb"
    exit 1
fi
QUEST=$(adb devices | grep -v "List of devices" | grep -v "^$" | grep -v "offline" || true)
if [[ -z "$QUEST" ]]; then
    echo "ERROR: No Quest found via ADB."
    echo "  • Connect Quest 3S via USB-C"
    echo "  • Put on headset and accept ADB authorization"
    echo "  • Run:  adb devices"
    exit 1
fi
echo "Quest detected:"
echo "$QUEST"
echo ""

# ---- Python environment ---
# Use the conda env that has ur_rtde + oculus_reader installed.
# Adjust if your env is named differently.
CONDA_ENV="${CONDA_ENV:-ur5_python}"
if conda run -n "$CONDA_ENV" python -c "import rtde_control" 2>/dev/null; then
    PYTHON="conda run --no-capture-output -n $CONDA_ENV python"
else
    # Fallback to whatever python is on PATH
    PYTHON="python"
fi

echo "Robot IP  : $ROBOT_IP"
echo "Controller: right (edit script to use left)"
echo ""
echo "Starting teleop — hold GRIP to move, joystick to reset forward."
echo ""

exec $PYTHON -m ur5_vr_teleop.quest_rtde_teleop \
    --robot-ip  "$ROBOT_IP" \
    --controller right \
    --pos-gain   3.0 \
    --rot-gain   2.0 \
    --max-lin-vel 0.15 \
    --max-rot-vel 0.75 \
    --hz         50 \
    --rmat-reorder -2 -1 -3 4
