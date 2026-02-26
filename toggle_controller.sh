#!/bin/bash
# ============================================================
# Toggle between forward_velocity_controller (Quest teleop)
# and scaled_joint_trajectory_controller (UI / MoveIt)
# ============================================================
# Run this from any terminal while launch_all.sh is running.
# Usage:
#   ./toggle_controller.sh          # auto-detects active and swaps
#   ./toggle_controller.sh velocity # force → forward_velocity_controller
#   ./toggle_controller.sh traj     # force → scaled_joint_trajectory_controller

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

VEL="forward_velocity_controller"
TRAJ="scaled_joint_trajectory_controller"

# ---- detect current active controller ----
# Strip ANSI colour codes first, then match " active" (not "inactive")
ACTIVE=$(ros2 control list_controllers 2>/dev/null \
    | sed 's/\x1b\[[0-9;]*m//g' \
    | grep -E "$VEL|$TRAJ" \
    | grep " active" \
    | awk '{print $1}' \
    | head -1 || true)

# ---- parse optional argument ----
case "${1:-}" in
    velocity|vel)   WANT="velocity" ;;
    traj|trajectory) WANT="traj" ;;
    *)
        # auto-toggle
        if [[ "$ACTIVE" == "$VEL" ]]; then
            WANT="traj"
        else
            WANT="velocity"
        fi
        ;;
esac

if [[ "$WANT" == "velocity" ]]; then
    if [[ "$ACTIVE" == "$VEL" ]]; then
        echo "forward_velocity_controller is already active."
        exit 0
    fi
    echo "Switching → forward_velocity_controller  (Quest teleop)"
    ros2 control switch_controllers \
        --deactivate "$TRAJ" \
        --activate   "$VEL" \
        --controller-manager /controller_manager
    echo "Done. Quest teleop is active."
else
    if [[ "$ACTIVE" == "$TRAJ" ]]; then
        echo "scaled_joint_trajectory_controller is already active."
        exit 0
    fi
    echo "Switching → scaled_joint_trajectory_controller  (UI / MoveIt)"
    ros2 control switch_controllers \
        --deactivate "$VEL" \
        --activate   "$TRAJ" \
        --controller-manager /controller_manager
    echo "Done. UI is active."
fi
