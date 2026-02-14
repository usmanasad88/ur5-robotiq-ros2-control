#!/bin/bash
# Launch script for UR5 External Control REST API
#
# This starts a Flask server that exposes robot commands as REST endpoints,
# allowing an external autonomous service to drive the UR5.
#
# The API mirrors the Streamlit UI's capabilities:
#   - List available programs, named positions, gripper actions
#   - Execute / load / pause / resume / stop programs
#   - Gripper open / close / set position
#   - Move to named positions, raw joints, or Cartesian poses
#
# Usage:
#   ./run_external_api.sh                # default port 5050
#   ./run_external_api.sh --port 8080    # custom port
#   ./run_external_api.sh --no-ros       # dry-run without ROS 2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "============================================"
echo "UR5 External Control API"
echo "============================================"
echo ""
echo "Endpoints (once running):"
echo "  GET  /api/commands         - List all commands"
echo "  GET  /api/status           - Executor & joint state"
echo "  POST /api/program/execute  - Load + execute program"
echo "  POST /api/program/load     - Load program only"
echo "  POST /api/program/pause    - Pause execution"
echo "  POST /api/program/resume   - Resume execution"
echo "  POST /api/program/stop     - Stop & hold at current position"
echo "  POST /api/gripper          - Gripper control"
echo "  POST /api/move/named       - Move to named position"
echo "  POST /api/move/joints      - Move to joint angles"
echo "  POST /api/move/pose        - Move to Cartesian pose"
echo "  POST /api/move/relative    - Move by offset (left/right/up/down/...)"
echo "  POST /api/position/save    - Save current position as named"
echo "  GET  /api/speed            - Get current speed factor"
echo "  POST /api/speed            - Set speed factor {\"speed\": 0.3}"
echo ""
echo "Prerequisites:"
echo "  - Program executor must be running"
echo "  - Run: ./run_program_executor.sh"
echo "============================================"
echo ""

# Use system Python 3.10 for ROS 2 compatibility (not conda Python)
export PATH="/usr/bin:$PATH"
PYTHON_BIN="/usr/bin/python3.10"

# Preload system libstdc++ to prevent GLIBCXX version conflicts with ROS 2
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# Source ROS 2 workspace
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# Ensure ROS Python paths are at the front
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH"

# Check if flask is installed for system Python
if ! $PYTHON_BIN -c "import flask" &> /dev/null; then
    echo "📦 Installing Flask..."
    $PYTHON_BIN -m pip install --user flask
fi

# Check if psutil is installed
if ! $PYTHON_BIN -c "import psutil" &> /dev/null; then
    echo "📦 Installing psutil..."
    $PYTHON_BIN -m pip install --user psutil
fi

# Launch the API server, forwarding all arguments
exec $PYTHON_BIN "$SCRIPT_DIR/external_control_api.py" "$@"
