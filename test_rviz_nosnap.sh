#!/bin/bash

# Test script to launch RViz without snap library interference

echo "=== Testing RViz without snap libraries ==="

# Kill any existing RViz instances
pkill -9 rviz2 2>/dev/null
sleep 1

# Completely unset problematic variables
unset GTK_PATH
unset GTK2_RC_FILES
unset GTK_IM_MODULE  
unset QT_QPA_PLATFORMTHEME
unset LD_PRELOAD

# Rebuild LD_LIBRARY_PATH from scratch
unset LD_LIBRARY_PATH

# System libraries first
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu"

# Then ROS
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"

# Then workspace
cd /home/mani/Repos/ur_ws
source install/setup.bash 2>/dev/null

echo "Library path (first 10):"
echo "$LD_LIBRARY_PATH" | tr ':' '\n' | head -10
echo ""
echo "Checking for snap in path:"
echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep snap || echo "None found - good!"
echo ""

echo "Launching RViz..."
timeout 15 rviz2 2>&1 &
RVIZ_PID=$!

sleep 5

if ps -p $RVIZ_PID > /dev/null 2>&1; then
    echo "SUCCESS: RViz is running (PID: $RVIZ_PID)"
    echo "Killing test instance..."
    kill $RVIZ_PID 2>/dev/null
    exit 0
else
    echo "FAILED: RViz crashed or didn't start"
    wait $RVIZ_PID 2>/dev/null
    exit 1
fi
