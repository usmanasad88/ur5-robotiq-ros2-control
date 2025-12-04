#!/bin/bash
# Launch script for UR5 + Robotiq that fixes RViz crash issue
# This removes snap libraries from the path to prevent conflicts

# Deactivate conda environment if active to avoid python version conflicts
if [[ -n "$CONDA_PREFIX" ]]; then
    echo "Deactivating conda environment: $CONDA_PREFIX"
    # Remove conda bin from PATH (handling both middle and end of string)
    export PATH=${PATH//$CONDA_PREFIX\/bin:/}
    export PATH=${PATH//$CONDA_PREFIX\/bin/}
    
    # Unset CONDA variables
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

# Now source ROS2 (this will append workspace paths)
cd /home/rml/ur5-robotiq-ros2-control
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch with provided arguments or defaults
ROBOT_IP="${1:-172.17.66.105}"
USE_FAKE="${2:-true}"

echo "Launching UR5 + Robotiq 2F-85..."
echo "Robot IP: $ROBOT_IP"
echo "Fake Hardware: $USE_FAKE"
echo ""

exec ros2 launch ur5_robotiq_description ur5_robotiq.launch.py \
  robot_ip:=$ROBOT_IP \
  use_fake_hardware:=$USE_FAKE
