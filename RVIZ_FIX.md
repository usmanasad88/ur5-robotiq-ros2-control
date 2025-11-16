# RViz2 Not Opening - Troubleshooting Guide

## Problem
RViz2 crashes with error:
```
/opt/ros/humble/lib/rviz2/rviz2: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

## Root Cause
Conflict between snap packages and ROS2 libraries. The snap's libpthread.so.0 is being loaded instead of the system library.

## Solutions (Try in Order)

### Solution 1: Remove Snap Library from PATH (Recommended)
```bash
# Create a wrapper script to launch RViz without snap libraries
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Then launch normally
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py robot_ip:=192.168.1.102 use_fake_hardware:=true
```

### Solution 2: Unset Problematic Environment Variable
```bash
# Remove snap paths from library search
unset GTK_PATH
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH

# Launch
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py robot_ip:=192.168.1.102 use_fake_hardware:=true
```

### Solution 3: Launch RViz Separately (Workaround)
```bash
# Terminal 1: Launch without RViz
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py robot_ip:=192.168.1.102 use_fake_hardware:=true launch_rviz:=false

# Terminal 2: Launch RViz with clean environment
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH rviz2
```

### Solution 4: Create Permanent Fix Script
Create a script to always launch ROS2 with clean library paths.

## Quick Fix for Your Current Session

Run this before any ROS2 command:
```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib:$LD_LIBRARY_PATH
```

Or add to your `~/.bashrc`:
```bash
# Add to bottom of ~/.bashrc
if [ -d "/opt/ros/humble" ]; then
    export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib:$LD_LIBRARY_PATH
fi
```

Then:
```bash
source ~/.bashrc
```

## Alternative: Install RViz2 via Conda (If using conda)
```bash
conda install -c conda-forge -c robostack-staging ros-humble-rviz2
```

## Verify Fix
After applying a solution, test:
```bash
rviz2 --version
# Should not crash
```

Then launch your robot:
```bash
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py robot_ip:=192.168.1.102 use_fake_hardware:=true
```

## Note
Your robot system is working correctly - all controllers loaded successfully. Only the visualization (RViz) is affected by this library path issue.
