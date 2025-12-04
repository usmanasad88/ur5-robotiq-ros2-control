#!/bin/bash
# ============================================
# UR5 Program Executor Launch Script
# ============================================
# This script handles the environment setup and launches the program executor node.
# It can optionally load and execute a program immediately.
#
# Usage:
#   ./run_program_executor.sh                     # Just start the node
#   ./run_program_executor.sh pick_and_place.prog # Start and load a program
#   ./run_program_executor.sh --execute prog.prog # Load and execute immediately
# ============================================

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 1. Patch the shebang in the installed node script to use the Conda environment
#    This is necessary because 'colcon build' resets it to the system python.
EXECUTOR_SCRIPT="install/ur5_curobo_control/lib/ur5_curobo_control/program_executor_node"
if [ -f "$EXECUTOR_SCRIPT" ]; then
    sed -i '1s|^.*$|#!/home/mani/miniconda3/envs/ur5_python/bin/python|' "$EXECUTOR_SCRIPT"
fi

# 2. Export LD_PRELOAD to prevent GLIBCXX errors when using PyTorch/Curobo with ROS 2
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# 3. Parse arguments
PROGRAM_FILE=""
AUTO_EXECUTE=false
USE_FAKE_HARDWARE=true
PRESENTER_CONTROL=true

while [[ $# -gt 0 ]]; do
    case $1 in
        --execute|-e)
            AUTO_EXECUTE=true
            shift
            ;;
        --no-presenter)
            PRESENTER_CONTROL=false
            shift
            ;;
        --real|-r)
            USE_FAKE_HARDWARE=false
            shift
            ;;
        --fake|-f)
            USE_FAKE_HARDWARE=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS] [PROGRAM_FILE]"
            echo ""
            echo "Options:"
            echo "  --execute, -e    Automatically execute the loaded program"
            echo "  --no-presenter   Disable presenter/keyboard control"
            echo "  --real, -r       Use real robot hardware (gripper via action)"
            echo "  --fake, -f       Use fake/simulated hardware (default)"
            echo "  --help, -h       Show this help message"
            echo ""
            echo "Presenter Controls (enabled by default):"
            echo "  Next/PageDown/Space/Right  Start or restart program"
            echo "  Prev/PageUp/Left           Pause execution"
            echo "  's' key                    Stop execution"
            echo ""
            echo "Examples:"
            echo "  $0                              # Start with presenter control"
            echo "  $0 pick_and_place.prog          # Load specific program"
            echo "  $0 --execute prog.prog          # Auto-execute (no waiting)"
            echo "  $0 --real pick_and_place.prog   # Real robot, custom program"
            exit 0
            ;;
        *)
            PROGRAM_FILE="$1"
            shift
            ;;
    esac
done

# 4. Source ROS workspace
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# 5. Build launch arguments
# Default to pick_and_place_object.prog if no program specified
if [ -z "$PROGRAM_FILE" ]; then
    PROGRAM_FILE="pick_and_place_object.prog"
fi
LAUNCH_ARGS="program_file:=$PROGRAM_FILE auto_execute:=$AUTO_EXECUTE use_fake_hardware:=$USE_FAKE_HARDWARE presenter_control:=$PRESENTER_CONTROL"

# 6. Launch the node
echo "============================================"
echo "UR5 Program Executor"
echo "============================================"
echo "Program file: $PROGRAM_FILE"
echo "Auto-execute: $AUTO_EXECUTE"
echo "Fake hardware: $USE_FAKE_HARDWARE"
echo "Presenter control: $PRESENTER_CONTROL"
echo ""
if [ "$PRESENTER_CONTROL" = true ]; then
echo "Presenter Controls:"
echo "  [Next/PageDown/Space/Right] Start/Resume or Pause program"
echo "  [Prev/PageUp/Left]          Record current pose"
echo "  [s] key                     Save recording"
echo ""
echo "Recording Mode:"
echo "  1. Move robot to desired pose (manually or via freedrive)"
echo "  2. Press [Prev] to record the pose"
echo "  3. Repeat for each waypoint"
echo "  4. Press [s] to save recording"
echo "  5. Recording saved to programs/recorded_YYYYMMDD_HHMMSS.prog"
echo ""
fi
echo "Services:"
echo "  ~/list_programs    - List available programs"
echo "  ~/load_program     - Load a program (set program_file param first)"
echo "  ~/execute_program  - Execute loaded program"
echo "  ~/pause            - Pause/resume (SetBool)"
echo "  ~/stop             - Stop execution"
echo ""
echo "CLI (after sourcing workspace):"
echo "  ros2 run ur5_curobo_control program_cli list"
echo "  ros2 run ur5_curobo_control program_cli load <program.prog>"
echo "  ros2 run ur5_curobo_control program_cli execute"
echo "============================================"
echo ""

ros2 launch ur5_curobo_control program_executor.launch.py $LAUNCH_ARGS
