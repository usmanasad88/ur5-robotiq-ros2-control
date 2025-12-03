#!/bin/bash
# Run script for UR5 VR Teleop that fixes library path issues
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

echo "Running UR5 VR Teleop..."

# Default arguments if none provided
if [ $# -eq 0 ]; then
    echo "No arguments provided. Using defaults:"
    echo "  -p controller_side:=any"
    echo "  -p scale_translation:=50.0"
    echo "  -p scale_rotation:=50.0"
    exec ros2 run ur5_vr_teleop vr_teleop_node --ros-args \
        -p controller_side:=any \
        -p scale_translation:=5.0 \
        -p scale_rotation:=0.5
else
    echo "Arguments: $@"
    exec ros2 run ur5_vr_teleop vr_teleop_node --ros-args "$@"
fi
