#!/bin/bash
# ============================================================
# Quest 3S VR Teleop — SIMULATION (quest_teleop package, ROS backend)
# ============================================================
# Drives the launch_all.sh UR5 (fake OR real hardware) through
# /forward_velocity_controller, using the new quest_teleop package so the
# Phase 1-2 improvements (wireless ADB, reader watchdog, VR-pose low-pass,
# yaw-only forward reset, precision mode, uniform saturation) all apply:
#
#   Quest → DROID P-controller → Jacobian IK → joint velocities
#                              → /forward_velocity_controller/commands
#
# This is the package equivalent of run_quest_servo_teleop_legacy.sh. For the REAL
# robot over RTDE use run_quest_rtde_teleop.sh instead.
#
# Prerequisites:
#   launch_all.sh running (provides /joint_states + the controller)
#   oculus_reader importable by the ROS python (system python3)
#
# Usage:
#   ./run_quest_teleop_sim.sh                 # right controller, auto-swap controller
#   ./run_quest_teleop_sim.sh --left          # left controller
#   ./run_quest_teleop_sim.sh --no-swap       # don't switch controllers (use toggle_controller.sh)
#   QUEST_IP=192.168.0.42 ./run_quest_teleop_sim.sh   # wireless ADB
#   ./run_quest_teleop_sim.sh --pos-scale 0.5 --filter-alpha 0.9   # extra flags pass through
#
# Controls: joystick click = reset forward (yaw-only); hold GRIP = move;
#           trigger = gripper (/ur5/gripper_cmd); A/X = precision toggle.
# ============================================================

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- parse our own flags; pass the rest to the package ----
CONTROLLER="right"
SWAP_CTRL=true
QUEST_IP="${QUEST_IP:-}"
EXTRA=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --left)    CONTROLLER="left";  shift ;;
        --right)   CONTROLLER="right"; shift ;;
        --no-swap) SWAP_CTRL=false;    shift ;;
        --swap)    SWAP_CTRL=true;     shift ;;
        --help|-h) sed -n '2,33p' "$0" | grep '^#' | sed 's/^# \?//'; exit 0 ;;
        *) EXTRA+=("$1"); shift ;;
    esac
done
[[ -n "$QUEST_IP" ]] && EXTRA+=(--quest-ip "$QUEST_IP")

# ---- environment cleanup (drop conda, fix lib paths — same as servo script) ----
if [[ -n "${CONDA_PREFIX:-}" ]]; then
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
export PYTHONPATH="$SCRIPT_DIR/src:${PYTHONPATH:-}"

# ---- switch to forward_velocity_controller (holds last velocity; watchdog + ----
# ---- grip-release publish zeros to stop) ----
VEL="forward_velocity_controller"
TRAJ="scaled_joint_trajectory_controller"
if [[ "$SWAP_CTRL" == "true" ]]; then
    echo "Switching → $VEL ..."
    ros2 control switch_controllers --deactivate "$TRAJ" --activate "$VEL" \
        --controller-manager /controller_manager 2>/dev/null \
        && echo "  $VEL active." \
        || echo "  (could not auto-switch — run ./toggle_controller.sh velocity manually)"
    restore_controller() {
        echo ""; echo "Restoring $TRAJ ..."
        ros2 control switch_controllers --deactivate "$VEL" --activate "$TRAJ" \
            --controller-manager /controller_manager 2>/dev/null \
            && echo "  Restored." || echo "  (could not restore)"
    }
    trap restore_controller EXIT INT TERM
fi

echo "Controller side: $CONTROLLER | Quest: ${QUEST_IP:-USB ADB}"
echo "Starting quest_teleop (ROS backend) ..."
echo ""

# Use the SYSTEM python that ROS Humble targets (`ros2 run` uses /usr/bin/python3
# via shebang). The conda python's libstdc++ is too old for rclpy's C extension.
exec /usr/bin/python3 -u -m quest_teleop \
    --backend ros \
    --controller "$CONTROLLER" \
    ${EXTRA[@]+"${EXTRA[@]}"}
