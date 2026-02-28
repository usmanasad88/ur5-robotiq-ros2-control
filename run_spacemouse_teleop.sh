#!/bin/bash
# ============================================================
# SpaceMouse Teleop — Simulation via Joint Velocity Control
# ============================================================
# Direct Cartesian velocity teleop for the simulated UR5:
#   SpaceMouse axes → scale to Cartesian vel → Jacobian IK → joint velocities
#                                             → /forward_velocity_controller/commands
#
# No MoveIt, no cuRobo, no trajectory replanning — smooth 50 Hz streaming.
#
# Prerequisites (one-time):
#   pip install pyspacemouse
#   Setup udev rules (see below) for non-root access
#   launch_all.sh must be running (provides joint_states)
#
# SpaceMouse setup:
#   1. Connect SpaceMouse via USB
#   2. Check detected: lsusb | grep 3Dconnexion
#   3. If permission denied, create /etc/udev/rules.d/90-spacemouse.rules:
#      SUBSYSTEM=="usb", ATTRS{idVendor}=="256f", MODE="0666"
#      SUBSYSTEM=="hidraw", ATTRS{idVendor}=="256f", MODE="0666"
#   4. Reload: sudo udevadm control --reload-rules && sudo udevadm trigger
#
# Usage:
#   ./run_spacemouse_teleop.sh              # Default (no auto-swap)
#   ./run_spacemouse_teleop.sh --swap       # Auto-switch controllers on start/exit
#
#   To switch controllers manually (toggle UI ↔ SpaceMouse):
#   ./toggle_controller.sh                  # auto-detects and swaps
#   ./toggle_controller.sh traj             # → UI / MoveIt
#   ./toggle_controller.sh vel              # → SpaceMouse teleop
#
# Controls:
#   Move SpaceMouse = robot moves in Cartesian space
#   Release         = robot stops
#   Ctrl-C          = quit (sends zero velocities)
# ============================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- defaults ----
SWAP_CTRL=false   # default: don't auto-switch (use toggle_controller.sh)

while [[ $# -gt 0 ]]; do
    case "$1" in
        --swap)     SWAP_CTRL=true; shift ;;
        --no-swap)  SWAP_CTRL=false; shift ;;
        --help|-h)
            sed -n '2,38p' "$0" | grep '^#' | sed 's/^# \?//'
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

# ---- switch to forward_velocity_controller ----
if [[ "$SWAP_CTRL" == "true" ]]; then
    echo "Switching to forward_velocity_controller ..."

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
echo "Starting SpaceMouse servo teleop (Jacobian IK velocity control) ..."
echo "  Move SpaceMouse → robot moves"
echo "  Release          → robot stops"
echo ""

exec ros2 run ur5_spacemouse_teleop spacemouse_teleop_node --ros-args \
    -p max_lin_vel:=0.15 \
    -p max_rot_vel:=0.75 \
    -p max_joint_vel:=1.5 \
    -p control_hz:=50.0 \
    -p damping:=0.05 \
    -p deadzone:=0.1
