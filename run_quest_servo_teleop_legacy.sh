#!/bin/bash
# ============================================================
# Quest 3S VR Teleop — Simulation via Joint Velocity Control
# ============================================================
# Direct Cartesian velocity teleop for the simulated UR5:
#   Quest → DROID P-controller → Jacobian IK → joint velocities
#                              → /forward_velocity_controller/commands
#
# No MoveIt, no cuRobo, no trajectory replanning — smooth 50 Hz streaming.
#
# Prerequisites (one-time):
#   sudo pip3 install git+https://github.com/rail-berkeley/oculus_reader.git
#   sudo apt install android-tools-adb
#   launch_all.sh must be running (provides joint_states)
#
# Quest 3S setup:
#   1. Enable Developer Mode: Meta Quest app → Devices → Developer Mode ON
#   2. Connect Quest 3S via USB-C
#   3. Put on headset — accept ADB authorization
#   4. Verify:  adb devices
#
# Usage:
#   ./run_quest_servo_teleop.sh              # Default (right controller, no auto-swap)
#   ./run_quest_servo_teleop.sh --swap       # Auto-switch controllers on start/exit
#   ./run_quest_servo_teleop.sh --left       # Left controller
#
#   To switch controllers manually (toggle UI ↔ Quest):
#   ./toggle_controller.sh                   # auto-detects and swaps
#   ./toggle_controller.sh traj              # → UI / MoveIt
#   ./toggle_controller.sh vel               # → Quest teleop
#
# Controls:
#   Joystick click  = reset forward direction  (do FIRST)
#   Hold GRIP       = robot moves (deadman)
#   Trigger         = gripper → /ur5/gripper_cmd
#   A button        = save position (auto name: pos_HHMMSS)
#   B button        = save position (type name in terminal)
#   Ctrl-C          = quit (sends zero velocities)
# ============================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- defaults ----
CONTROLLER="right"
SWAP_CTRL=false   # default: don't auto-switch (use toggle_controller.sh)

while [[ $# -gt 0 ]]; do
    case "$1" in
        --left)     CONTROLLER="left"; shift ;;
        --right)    CONTROLLER="right"; shift ;;
        --swap)     SWAP_CTRL=true; shift ;;
        --no-swap)  SWAP_CTRL=false; shift ;;
        --help|-h)
            sed -n '2,30p' "$0" | grep '^#' | sed 's/^# \?//'
            exit 0 ;;
        *) echo "Unknown option: $1  (use --help)"; exit 1 ;;
    esac
done

# ---- environment cleanup ----
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

# ---- switch to forward_velocity_controller ----
# The UR driver starts with scaled_joint_trajectory_controller active.
# We need forward_velocity_controller instead for streaming joint velocities.
if [[ "$SWAP_CTRL" == "true" ]]; then
    echo "Switching to forward_velocity_controller ..."

    # Deactivate scaled_joint_trajectory_controller (ignore errors if already inactive)
    ros2 control switch_controllers \
        --deactivate scaled_joint_trajectory_controller \
        --activate   forward_velocity_controller \
        --controller-manager /controller_manager 2>/dev/null \
    && echo "  Controller switched: forward_velocity_controller active." \
    || {
        echo "  WARNING: Could not switch controllers automatically."
        echo "  Try manually:"
        echo "    ros2 control switch_controllers \\"
        echo "      --deactivate scaled_joint_trajectory_controller \\"
        echo "      --activate   forward_velocity_controller \\"
        echo "      --controller-manager /controller_manager"
        echo ""
        echo "  If forward_velocity_controller is not loaded, load it first:"
        echo "    ros2 control load_controller forward_velocity_controller"
        echo ""
        echo "  Continuing anyway (node will still start) ..."
    }
    echo ""
fi

# ---- restore controller on exit ----
restore_controller() {
    echo ""
    echo "Restoring scaled_joint_trajectory_controller ..."
    ros2 control switch_controllers \
        --deactivate forward_velocity_controller \
        --activate   scaled_joint_trajectory_controller \
        --controller-manager /controller_manager 2>/dev/null \
    && echo "  Restored." || echo "  (Could not restore — check controller_manager)"
}
if [[ "$SWAP_CTRL" == "true" ]]; then
    trap restore_controller EXIT INT TERM
fi

# ---- launch ----
echo "Controller: $CONTROLLER"
echo ""
echo "Starting Quest 3S servo teleop (Jacobian IK velocity control) ..."
echo "  Joystick click  → reset forward direction"
echo "  Hold GRIP       → robot moves"
echo "  Trigger         → gripper (/ur5/gripper_cmd)"
echo ""

exec ros2 run ur5_vr_teleop quest_servo_teleop --ros-args \
    -p controller_side:="$CONTROLLER" \
    -p pos_gain:=3.0 \
    -p rot_gain:=2.0 \
    -p max_lin_vel:=0.15 \
    -p max_rot_vel:=0.75 \
    -p max_joint_vel:=1.5 \
    -p control_hz:=50.0 \
    -p damping:=0.05
