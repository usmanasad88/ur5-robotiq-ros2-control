#!/bin/bash
# ============================================================
# Newton UDP relay  —  ROS 2 -> Newton UR5 teleop bridge
# ============================================================
# Makes the Newton sim a FOLLOWER of the RViz UR5: relays /joint_states (the
# robot's actual state, whatever drives it — Quest, Unity app, prog files,
# spacemouse, cuRobo, teach pendant) to the cloth_ur5_ogc example over UDP.
#
#   1. ./launch_all.sh [--bridge ...]  # UR5 driver (+ whichever input methods)
#   2. ./run_newton_relay.sh           # this relay (ROS -> UDP)
#   3. (Newton venv) python -m newton.examples cloth_ur5_ogc --teleop
#
# Usage:
#   ./run_newton_relay.sh                          # -> 127.0.0.1:11000
#   NEWTON_HOST=192.168.0.38 ./run_newton_relay.sh # Newton on another machine
#   NEWTON_PORT=11001 ./run_newton_relay.sh
#   NEWTON_SOURCE=commands ./run_newton_relay.sh   # legacy: relay the raw
#                                                  # position-command stream
# ============================================================
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- environment cleanup (drop conda, fix lib paths — same as the other
# ---- ROS run scripts) ----
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    export PATH=${PATH//$CONDA_PREFIX\/bin:/}
    export PATH=${PATH//$CONDA_PREFIX\/bin/}
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi
unset GTK_PATH GTK2_RC_FILES GTK_IM_MODULE QT_QPA_PLATFORMTHEME LD_PRELOAD PYTHONPATH
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"

set +u
source /opt/ros/humble/setup.bash
[ -f "$SCRIPT_DIR/install/setup.bash" ] && source "$SCRIPT_DIR/install/setup.bash"
set -u

NEWTON_HOST="${NEWTON_HOST:-127.0.0.1}"
NEWTON_PORT="${NEWTON_PORT:-11000}"
NEWTON_SOURCE="${NEWTON_SOURCE:-joint_states}"

echo "Relaying ${NEWTON_SOURCE} -> UDP ${NEWTON_HOST}:${NEWTON_PORT}  (Newton follows the RViz UR5)"
echo "(Run the Newton example with: --teleop --teleop-port ${NEWTON_PORT})"
echo ""

exec python3 "$SCRIPT_DIR/newton_bridge/ur5_newton_relay.py" --ros-args \
    -p newton_host:="$NEWTON_HOST" \
    -p newton_port:="$NEWTON_PORT" \
    -p source:="$NEWTON_SOURCE"
