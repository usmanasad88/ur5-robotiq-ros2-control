#!/bin/bash
# Test force-controlled push in -Z direction
# Usage:
#   ./run_force_push_test.sh                # 10N down, 5s
#   ./run_force_push_test.sh --force 15     # 15N down
#   ./run_force_push_test.sh --time 10      # hold 10s

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || true

python3 "$SCRIPT_DIR/test_force_push.py" "$@"
