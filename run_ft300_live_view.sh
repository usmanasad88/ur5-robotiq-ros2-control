#!/bin/bash
# Live FT300 force/torque visualization
# Usage: ./run_ft300_live_view.sh [--zero]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || true

python3 "$SCRIPT_DIR/ft300_live_view.py" "$@"
