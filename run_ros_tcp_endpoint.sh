#!/bin/bash
# ============================================================
# ROS-TCP-Endpoint  —  Unity <-> ROS 2 bridge server
# ============================================================
# Starts the TCP server the Unity ROS-TCP-Connector connects to (Unity is the
# TCP client). Needed for PLAN.md Step 4/5 (Virtual_UR5 Unity app):
#   Step 4 (Mode A): Unity publishes q[6] -> /forward_position_controller/commands
#                    and the gripper -> /ur5/gripper_cmd.
#   Step 5 (Mode B): Unity subscribes to /joint_states.
#
# The Quest and this PC must share a LAN; point Unity's ROSConnection at this
# PC's IP, port 10000 (the default).
#
# Usage:
#   ./run_ros_tcp_endpoint.sh                 # listen on 0.0.0.0:10000
#   ROS_TCP_PORT=10001 ./run_ros_tcp_endpoint.sh
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
source "$SCRIPT_DIR/install/setup.bash"
set -u

ROS_IP="${ROS_IP:-0.0.0.0}"
ROS_TCP_PORT="${ROS_TCP_PORT:-10000}"

echo "ROS-TCP-Endpoint listening on ${ROS_IP}:${ROS_TCP_PORT}"
echo "Point Unity's ROSConnection at this PC's LAN IP, port ${ROS_TCP_PORT}."
echo ""

exec ros2 run ros_tcp_endpoint default_server_endpoint --ros-args \
    -p ROS_IP:="$ROS_IP" \
    -p ROS_TCP_PORT:="$ROS_TCP_PORT"
