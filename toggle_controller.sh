#!/bin/bash
# ============================================================
# Switch the active UR5 arm controller.
#   forward_velocity_controller        — Quest / SpaceMouse velocity teleop
#   forward_position_controller        — Virtual_UR5 Unity Mode A (q[6] stream)
#   scaled_joint_trajectory_controller — UI / MoveIt (default)
# ============================================================
# Run this from any terminal while launch_all.sh is running.
# Usage:
#   ./toggle_controller.sh           # auto-toggle velocity <-> traj (legacy)
#   ./toggle_controller.sh velocity  # force → forward_velocity_controller
#   ./toggle_controller.sh position  # force → forward_position_controller
#   ./toggle_controller.sh traj      # force → scaled_joint_trajectory_controller

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

VEL="forward_velocity_controller"
POS="forward_position_controller"
TRAJ="scaled_joint_trajectory_controller"

# ---- detect which arm controller is currently active ----
# Strip ANSI colour codes first, then match " active" (not "inactive").
ACTIVE=$(ros2 control list_controllers 2>/dev/null \
    | sed 's/\x1b\[[0-9;]*m//g' \
    | grep -E "$VEL|$POS|$TRAJ" \
    | grep " active" \
    | awk '{print $1}' \
    | head -1 || true)

# ---- resolve the wanted controller ----
case "${1:-}" in
    velocity|vel)        WANT="$VEL" ;;
    position|pos)        WANT="$POS" ;;
    traj|trajectory)     WANT="$TRAJ" ;;
    "")
        # Legacy auto-toggle: velocity <-> traj.
        if [[ "$ACTIVE" == "$VEL" ]]; then WANT="$TRAJ"; else WANT="$VEL"; fi
        ;;
    *)
        echo "Unknown controller '$1'. Use: velocity | position | traj"
        exit 1
        ;;
esac

if [[ "$ACTIVE" == "$WANT" ]]; then
    echo "$WANT is already active."
    exit 0
fi

# Build args: deactivate whatever arm controller is active (if any), activate WANT.
DEACT_ARGS=()
[[ -n "$ACTIVE" ]] && DEACT_ARGS=(--deactivate "$ACTIVE")

echo "Switching → $WANT"
ros2 control switch_controllers \
    "${DEACT_ARGS[@]}" \
    --activate "$WANT" \
    --controller-manager /controller_manager
echo "Done. $WANT is active."
