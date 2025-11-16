#!/bin/bash
# Launch script for UR5 + Robotiq that fixes RViz crash issue
# This removes snap libraries from the path to prevent conflicts

# Unset problematic variables
unset GTK_PATH
unset GTK2_RC_FILES
unset GTK_IM_MODULE
unset QT_QPA_PLATFORMTHEME
unset LD_PRELOAD

# Rebuild LD_LIBRARY_PATH from scratch without snap
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"

# Now source ROS2 (this will append workspace paths)
cd /home/mani/Repos/ur_ws
source install/setup.bash

# Launch with provided arguments or defaults
ROBOT_IP="${1:-192.168.1.102}"
USE_FAKE="${2:-true}"

echo "Launching UR5 + Robotiq 2F-85..."
echo "Robot IP: $ROBOT_IP"
echo "Fake Hardware: $USE_FAKE"
echo ""

exec ros2 launch ur5_robotiq_description ur5_robotiq.launch.py \
  robot_ip:=$ROBOT_IP \
  use_fake_hardware:=$USE_FAKE
