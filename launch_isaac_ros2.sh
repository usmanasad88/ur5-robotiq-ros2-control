#!/bin/bash

# Clear PYTHONPATH to avoid conflicts with system ROS 2 (Python 3.10)
# Isaac Sim uses Python 3.11 and provides its own ROS 2 libraries.
export PYTHONPATH=""

# Set ROS 2 environment variables for Isaac Sim
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Launch Isaac Sim
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/isaac-sim.sh
