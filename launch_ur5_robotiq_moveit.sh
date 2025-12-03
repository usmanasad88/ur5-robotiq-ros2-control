#!/bin/bash
# Launch script for UR5 + Robotiq MoveIt that fixes RViz crash/library issues
# This removes snap libraries and conda env from the path to prevent conflicts

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
cd /home/mani/Repos/ur_ws
source install/setup.bash

# Launch with provided arguments or defaults
USE_FAKE="${1:-true}"

echo "Launching UR5 + Robotiq MoveIt..."
echo "Fake Hardware: $USE_FAKE"
echo ""

exec ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 \
  description_package:=ur5_robotiq_description \
  description_file:=ur5_robotiq.urdf.xacro \
  moveit_config_package:=ur5_robotiq_description \
  moveit_config_file:=ur5_robotiq.srdf.xacro \
  use_fake_hardware:=$USE_FAKE
