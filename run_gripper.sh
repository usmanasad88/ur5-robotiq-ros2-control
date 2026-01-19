#!/bin/bash
# Launch script for Robotiq 2F-85 gripper adapter
# Handles environment setup for both rml and mani users

# Deactivate conda environment if active to avoid python version conflicts
if [[ -n "$CONDA_PREFIX" ]]; then
    echo "Deactivating conda environment: $CONDA_PREFIX"
    export PATH=${PATH//$CONDA_PREFIX\/bin:/}
    export PATH=${PATH//$CONDA_PREFIX\/bin/}
    unset CONDA_PREFIX
    unset CONDA_DEFAULT_ENV
    unset CONDA_PROMPT_MODIFIER
    unset CONDA_SHLVL
    unset CONDA_PYTHON_EXE
fi

# Unset problematic variables
unset GTK_PATH
unset GTK2_RC_FILES
unset GTK_IM_MODULE
unset QT_QPA_PLATFORMTHEME
unset LD_PRELOAD
unset PYTHONPATH

# Rebuild LD_LIBRARY_PATH from scratch without snap
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"

# Detect workspace path based on user
if [ "$USER" = "mani" ]; then
    WORKSPACE="/home/mani/Repos/ur_ws"
else
    WORKSPACE="/home/rml/ur5-robotiq-ros2-control"
fi

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# Default robot IP
ROBOT_IP="${1:-172.17.66.105}"

echo "Launching Robotiq 2F-85 Gripper Adapter..."
echo "Robot IP: $ROBOT_IP"
echo ""

exec ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:="$ROBOT_IP"
