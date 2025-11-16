#!/bin/bash
# Quick command reference for Isaac Sim UR robot scripts
# Source this file or copy commands as needed

# Set up paths
export ISAAC_SIM_PATH="/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64"
export ISAAC_PYTHON="$ISAAC_SIM_PATH/python.sh"
export SCRIPT_DIR="/home/mani/Repos/ur_ws/isaac_standalone"

# Aliases for convenience
alias isaacsim='$ISAAC_PYTHON'
alias test-ur='$SCRIPT_DIR/run_tests.sh'
alias ur10='$SCRIPT_DIR/run_ur_robot.sh --robot ur10'
alias ur5='$SCRIPT_DIR/run_ur_robot.sh --robot ur5'

echo "Isaac Sim environment configured!"
echo ""
echo "Available commands:"
echo "  isaacsim <script.py>  - Run any script with Isaac Sim Python"
echo "  test-ur               - Run all tests"
echo "  ur10                  - Load UR10 robot (GUI)"
echo "  ur5                   - Load UR5 robot (GUI)"
echo ""
echo "Examples:"
echo "  isaacsim $SCRIPT_DIR/simple_test.py"
echo "  isaacsim $SCRIPT_DIR/ur_robot_advanced.py --mode animated --record"
echo "  test-ur"
echo ""
