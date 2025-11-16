#!/bin/bash
# Helper script to run the UR robot standalone Isaac Sim application
# This script uses the Isaac Sim standalone Python interpreter

ISAAC_SIM_PATH="/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check if Isaac Sim path exists
if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo "Error: Isaac Sim not found at $ISAAC_SIM_PATH"
    exit 1
fi

# Set up environment
export LD_LIBRARY_PATH=$ISAAC_SIM_PATH:$LD_LIBRARY_PATH

# Run the script with Isaac Sim's Python
echo "Running UR robot simulation with Isaac Sim..."
echo "Isaac Sim Path: $ISAAC_SIM_PATH"
echo "Script: $SCRIPT_DIR/ur10_import.py"
echo ""

# Pass all arguments to the Python script
$ISAAC_SIM_PATH/python.sh "$SCRIPT_DIR/ur10_import.py" "$@"
