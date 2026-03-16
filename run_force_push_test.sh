#!/bin/bash
# Test force-controlled push: down (-Z) + forward via movel
# Uses URScript directly (no controller switching needed)
#
# Usage:
#   ./run_force_push_test.sh                                    # defaults: 60N, 100mm, 50mm/s
#   ./run_force_push_test.sh --force 40 --distance 0.15         # 40N, 150mm
#   ./run_force_push_test.sh --speed 0.03 --speed-limit 0.1    # slower

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash" 2>/dev/null || true

python3 "$SCRIPT_DIR/test_force_push.py" "$@"
