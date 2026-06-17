#!/bin/bash
# ============================================================
# Quest 3S VR Teleop  —  ur_rtde Cartesian velocity control
# ============================================================
# Launches the quest_teleop package (src/quest_teleop/).
# Connects directly to the robot via RTDE (bypasses ros2_control).
# Requires launch_all.sh to be running ONLY for the UR5 driver
# (the driver must have brought the robot to EXTERNAL CONTROL mode).
#
# Usage:
#   ./run_quest_rtde_teleop.sh                       # USB ADB, real robot
#   ./run_quest_rtde_teleop.sh --dry-run             # no robot, log commands
#   QUEST_IP=192.168.0.42 ./run_quest_rtde_teleop.sh # wireless ADB
#   ./run_quest_rtde_teleop.sh --max-lin-vel 0.1     # extra flags pass through
#
# Fallback to the original single-file version:
#   python -m ur5_vr_teleop.quest_rtde_teleop  (from src/ur5_vr_teleop/)
#
# Prerequisites (one-time):
#   pip install ur-rtde
#   pip install git+https://github.com/rail-berkeley/oculus_reader.git
#   sudo apt install android-tools-adb
#
# Quest 3S setup (USB):
#   1. Enable Developer Mode: Meta Quest app → Devices → Developer Mode ON
#   2. Connect Quest 3S to this PC via USB-C
#   3. Put on headset — accept the ADB authorization prompt
#   4. Verify:  adb devices  (should list your Quest)
#
# Wireless setup (one-time per Quest reboot):
#   1. Connect via USB first (steps above)
#   2. adb tcpip 5555
#   3. adb shell ip route        # note the Quest's wlan0 IP
#   4. Unplug USB, run with QUEST_IP=<that ip>
#
# Controls:
#   Grip button (side)  = deadman switch  (hold to move robot)
#   Joystick click      = reset forward direction  ← do this FIRST
#   Trigger (front)     = gripper  (0 open, 1 closed)
#
# Axis tuning  (if robot moves in the wrong direction):
#   Pass --rmat-reorder.  Negate an index to flip that axis.
#   e.g.  -2 -1 -3 4  →  flip X only:  2 -1 -3 4
# ============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_IP="${ROBOT_IP:-172.17.66.105}"
QUEST_IP="${QUEST_IP:-}"

# ---- Python environment ---
# Use the conda env that has ur_rtde + oculus_reader installed.
# Adjust if your env is named differently.
CONDA_ENV="${CONDA_ENV:-ur5_python}"
CONDA_PY="$HOME/miniconda3/envs/$CONDA_ENV/bin/python"
if [[ -x "$CONDA_PY" ]]; then
    PYTHON="$CONDA_PY"
else
    PYTHON="python3"
fi

EXTRA_ARGS=("$@")
# Honor a --robot-ip passed as an argument (so the echo below is accurate and
# we don't pass the flag twice).
prev=""
for a in "$@"; do
    [[ "$prev" == "--robot-ip" ]] && ROBOT_IP="$a"
    prev="$a"
done
if [[ -n "$QUEST_IP" ]]; then
    EXTRA_ARGS+=(--quest-ip "$QUEST_IP")
fi

echo "Robot IP  : $ROBOT_IP"
echo "Quest     : ${QUEST_IP:-USB ADB}"
echo "Python    : $PYTHON"
echo ""
echo "Starting teleop — hold GRIP to move, joystick to reset forward."
echo ""

export PYTHONPATH="$SCRIPT_DIR/src${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON" -u -m quest_teleop \
    --robot-ip "$ROBOT_IP" \
    ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}
