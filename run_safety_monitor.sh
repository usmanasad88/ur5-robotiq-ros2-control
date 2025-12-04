#!/bin/bash

# 1. Patch the shebang in the installed script to use the Conda environment
sed -i '1s|^.*$|#!/home/rml/miniconda3/envs/ur5_python/bin/python|' install/ur5_curobo_control/lib/ur5_curobo_control/face_safety_monitor

# 2. Export LD_PRELOAD to prevent GLIBCXX errors when using Conda Python with ROS 2
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# 3. Run the face safety monitor, passing any arguments (like --ros-args)
ros2 run ur5_curobo_control face_safety_monitor "$@"
