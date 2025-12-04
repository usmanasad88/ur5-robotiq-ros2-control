#!/bin/bash
# ============================================
# UR5 cuRobo Control Launch Script
# ============================================
# This script handles the environment setup and launching of the curobo control node.
# It can optionally launch the gesture-based safety monitor.
#
# Usage:
#   ./run_curobo.sh                  # Launch curobo control only
#   ./run_curobo.sh --gesture        # Also launch gesture safety monitor
#   ./run_curobo.sh --gesture-only   # Launch only gesture safety monitor
# ============================================

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Parse arguments
USE_GESTURE=false
GESTURE_ONLY=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --gesture|-g)
            USE_GESTURE=true
            shift
            ;;
        --gesture-only)
            GESTURE_ONLY=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --gesture, -g     Also launch gesture safety monitor"
            echo "  --gesture-only    Launch only gesture safety monitor"
            echo "  --help, -h        Show this help message"
            echo ""
            echo "Gesture Controls:"
            echo "  Open Palm / Point Up   -> STOP robot"
            echo "  Thumbs Up / Victory    -> RESUME robot"
            exit 0
            ;;
        *)
            shift
            ;;
    esac
done

# 1. Patch the shebang in the installed node script to use the Conda environment
#    This is necessary because 'colcon build' resets it to the system python.
sed -i '1s|^.*$|#!/home/rml/miniconda3/envs/ur5_python/bin/python|' install/ur5_curobo_control/lib/ur5_curobo_control/curobo_control_node
sed -i '1s|^.*$|#!/home/mani/miniconda3/envs/ur5_python/bin/python|' install/ur5_curobo_control/lib/ur5_curobo_control/gesture_safety_monitor 2>/dev/null

# 2. Export LD_PRELOAD to prevent GLIBCXX errors when using PyTorch/Curobo with ROS 2
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# 3. Source ROS workspace
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

echo "============================================"
echo "UR5 cuRobo Control"
echo "============================================"
if [ "$GESTURE_ONLY" = true ]; then
    echo "Mode: Gesture Safety Monitor Only"
elif [ "$USE_GESTURE" = true ]; then
    echo "Mode: cuRobo Control + Gesture Safety"
else
    echo "Mode: cuRobo Control Only"
fi
echo ""
echo "Gesture Controls (if enabled):"
echo "  Open Palm / Point Up  -> STOP robot"
echo "  Thumbs Up / Victory   -> RESUME robot"
echo "============================================"
echo ""

# 4. Launch the appropriate nodes
if [ "$GESTURE_ONLY" = true ]; then
    ros2 launch ur5_curobo_control gesture_safety.launch.py
elif [ "$USE_GESTURE" = true ]; then
    # Launch both in parallel
    ros2 launch ur5_curobo_control gesture_safety.launch.py &
    GESTURE_PID=$!
    sleep 2  # Give gesture monitor time to start
    ros2 launch ur5_curobo_control curobo_control.launch.py
    kill $GESTURE_PID 2>/dev/null
else
    ros2 launch ur5_curobo_control curobo_control.launch.py
fi
