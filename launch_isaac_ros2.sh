#!/bin/bash
# Launch Isaac Sim with ROS 2 enabled.
#
# Default  : Isaac Sim 6  (source build at ~/Repos/isaacsim)
# Revert   : pass --v5 to use the Isaac Sim 5.0.0 backup on the BigD drive,
#            or set ISAAC_SIM_SH=/path/to/isaac-sim.sh to point anywhere.
#
# Isaac Sim 6's own isaac-sim.sh already sets up ROS 2 (it sources
# setup_ros_env.sh). When ROS_DISTRO is left UNSET, that script adds Isaac
# Sim's *internal* ROS 2 bridge libraries to LD_LIBRARY_PATH -- the standalone
# bridge the 5.0 launch also relied on. So for v6 we intentionally clear
# ROS_DISTRO and let isaac-sim.sh do the work.

set -e

# ---- Pick which Isaac Sim to launch ---------------------------------------
ISAAC6_SH="/home/mani/Repos/isaacsim/_build/linux-x86_64/release/isaac-sim.sh"
ISAAC5_SH="/media/mani/BigD/isaac-sim-standalone-5.0.0-linux-x86_64/isaac-sim.sh"

# ISAAC_SIM_SH env var wins; otherwise default to v6.
ISAAC_SIM_SH="${ISAAC_SIM_SH:-$ISAAC6_SH}"
USE_V5=0
GDB=0

PASS_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --v5|--v5.0|--isaac5) ISAAC_SIM_SH="$ISAAC5_SH"; USE_V5=1 ;;
        --v6|--isaac6)        ISAAC_SIM_SH="$ISAAC6_SH"; USE_V5=0 ;;
        --gdb)                GDB=1 ;;
        *)                    PASS_ARGS+=("$arg") ;;
    esac
done

if [[ ! -x "$ISAAC_SIM_SH" ]]; then
    echo "ERROR: Isaac Sim launcher not found or not executable:" >&2
    echo "       $ISAAC_SIM_SH" >&2
    exit 1
fi

# ---- ROS 2 environment -----------------------------------------------------
# Clear PYTHONPATH so Isaac Sim's bundled Python does not pick up the system
# ROS 2 (Python 3.10) packages, which would clash with Isaac Sim's Python.
export PYTHONPATH=""

# Use FastDDS as the ROS 2 middleware (matches the rest of the workspace).
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [[ "$USE_V5" == "1" ]]; then
    # Isaac Sim 5.0.0: replicate the original launch behaviour exactly.
    export ROS_DISTRO=humble
else
    # Isaac Sim 6: leave ROS_DISTRO unset so isaac-sim.sh -> setup_ros_env.sh
    # loads Isaac Sim's internal ROS 2 bridge libraries.
    unset ROS_DISTRO
fi

if [[ "$GDB" == "1" ]]; then
    # Debug mode: run Kit directly under gdb so a native crash (segfault) is
    # caught and a backtrace is printed, instead of being swallowed by Isaac
    # Sim's own crash reporter. Intended for the v6 layout.
    REL_DIR="$(dirname "$ISAAC_SIM_SH")"
    # isaac-sim.sh normally sources this to set up ROS; do it ourselves here.
    [[ -f "$REL_DIR/setup_ros_env.sh" ]] && source "$REL_DIR/setup_ros_env.sh"
    echo "[launch] Running Isaac Sim under gdb (crash reporter disabled)."
    echo "[launch] Load your extension; on a crash, a backtrace prints below. Then 'quit'."
    exec gdb -q -batch \
        -ex 'set pagination off' \
        -ex 'handle SIGPIPE nostop noprint pass' \
        -ex 'handle SIG32 SIG33 SIG34 SIG35 SIG36 nostop noprint pass' \
        -ex run \
        -ex 'bt' \
        -ex 'quit' \
        --args "$REL_DIR/kit/kit" "$REL_DIR/apps/isaacsim.exp.full.kit" \
        --/crashreporter/enabled=false "${PASS_ARGS[@]}"
fi

echo "[launch] Starting Isaac Sim: $ISAAC_SIM_SH"
exec "$ISAAC_SIM_SH" "${PASS_ARGS[@]}"
