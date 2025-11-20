#!/bin/bash

# This script handles the environment setup and launching of the curobo control node.

# 1. Patch the shebang in the installed node script to use the Conda environment
#    This is necessary because 'colcon build' resets it to the system python.
sed -i '1s|^.*$|#!/home/mani/miniconda3/envs/ur5_python/bin/python|' install/ur5_curobo_control/lib/ur5_curobo_control/curobo_control_node

# 2. Export LD_PRELOAD to prevent GLIBCXX errors when using PyTorch/Curobo with ROS 2
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# 3. Launch the node
ros2 launch ur5_curobo_control curobo_control.launch.py
