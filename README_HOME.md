# ===== INSTALLATION AND SETUP =====

## Prerequisites
# Ubuntu 22.04 LTS
# ROS 2 Humble (Desktop Full)

## 1. Install ROS 2 Humble
# Follow official instructions: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop-full

## 2. Install ROS 2 Development Tools
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool

## 3. Initialize rosdep
sudo rosdep init
rosdep update

## 4. Clone this workspace
cd ~/Repos  # or your preferred location
git clone <your-repo-url> ur_ws
cd ur_ws

## 5. Install dependencies
rosdep install --from-paths src --ignore-src -r -y

## 6. Additional dependencies for UR5 and teleoperation
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install python3-pip
pip3 install flask numpy  # For HTTP server control

# For SpaceMouse support (optional)
pip3 install pyspacemouse

# For VR teleoperation (optional - requires OpenVR/SteamVR)
pip3 install openvr

## 7. Build the workspace
cd ~/Repos/ur_ws
colcon build --symlink-install

## 8. Source the workspace
source install/setup.bash

# Add to ~/.bashrc for convenience:
echo "source ~/Repos/ur_ws/install/setup.bash" >> ~/.bashrc

## 9. Apply RViz fix (prevents crashes with snap-installed apps)
# Add to ~/.bashrc:
echo 'export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"' >> ~/.bashrc
source ~/.bashrc

## 10. Verify installation
ros2 pkg list | grep ur  # Should show ur packages
ros2 pkg list | grep robotiq  # Should show robotiq packages

# ===== BASIC USAGE COMMANDS =====

# Launch UR5 controller (simulation)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=172.17.66.105 use_fake_hardware:=true

# Launch MoveIt with RViz
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

# Control real robot (NOT for simulation)
#ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=192.168.1.102

# Run general controller with kinematics
ros2 run ur5_gen_controller random_joint_goal.py --ros-args --params-file /home/mani/Repos/ur_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml

# ===== UR5 + ROBOTIQ 2F-85 GRIPPER INTEGRATION =====
# Launch UR5 with Robotiq 2F-85 gripper (RECOMMENDED - includes RViz fix)
./launch_ur5_robotiq.sh

# With real robot (replace IP as needed)
./launch_ur5_robotiq.sh 172.17.66.105 false

# Direct launch (requires RViz fix in bashrc - see RVIZ_FIX.md)
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py ur_type:=ur5 use_fake_hardware:=true

# Launch with MoveIt (gripper fully integrated with collision checking)
./launch_ur5_robotiq_moveit.sh

# Or manually (requires environment fix):
# ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 \
#   description_package:=ur5_robotiq_description \
#   description_file:=ur5_robotiq.urdf.xacro \
#   moveit_config_package:=ur5_robotiq_description \
#   moveit_config_file:=ur5_robotiq.srdf.xacro \
#   use_fake_hardware:=true

# Verify gripper segments are loaded
ros2 topic echo /robot_description --once | grep robotiq

# ===== RVIZ FIX (if RViz crashes) =====
# The fix is now in ~/.bashrc, but for manual override:
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"
source ~/Repos/ur_ws/install/setup.bash
# Then launch normally

# Test RViz without snap interference
./test_rviz_nosnap.sh


# ===== TELEOPERATION CONTROL =====
# General controller node (processes teleop commands from all sources)
# Use the helper script to avoid library conflicts:
./run_ur5_controller.sh

# Or with custom arguments:
./run_ur5_controller.sh -p use_http_server:=true -p action_server_url:=http://localhost:5000/get_action

# To control in Base Frame (useful for VR teleop):
./run_ur5_controller.sh -p control_frame:=base

# Manual command (requires environment fix):
# ros2 run ur5_gen_controller random_joint_goal --ros-args \
#   -p use_http_server:=false \
#   -p use_random_motion:=false \
#   -p max_iterations:=10000

# With HTTP server enabled (for Python control)
ros2 run ur5_gen_controller random_joint_goal --ros-args \
  -p use_http_server:=true \
  -p action_server_url:=http://localhost:5000/get_action \
  -p use_random_motion:=false

# With random motion for testing
ros2 run ur5_gen_controller random_joint_goal --ros-args \
  -p use_http_server:=false \
  -p use_random_motion:=true

# Keyboard teleoperation
# Use the helper script to avoid library conflicts:
./run_keyboard_teleop.sh

# Manual command (requires environment fix):
# ros2 run ur5_keyboard_teleop keyboard_teleop_node --ros-args \
#   -p linear_step:=0.01 \
#   -p angular_step:=0.05

# SpaceMouse teleoperation
# Use the helper script to avoid library conflicts:
./run_spacemouse_teleop.sh

# Manual command (requires environment fix):
# ros2 run ur5_spacemouse_teleop spacemouse_teleop_node --ros-args \
#   -p scale_translation:=0.001 \
#   -p scale_rotation:=0.005 \
#   -p publish_rate:=10.0

# VR teleoperation (HTC Vive)
# Use the helper script to avoid library conflicts:
./run_vr_teleop.sh

# Manual command (requires environment fix):
# ros2 run ur5_vr_teleop vr_teleop_node --ros-args \
#   -p controller_side:=any \
#   -p scale_translation:=50.0 \
#   -p scale_rotation:=50.0

# VR with custom sensitivity
ros2 run ur5_vr_teleop vr_teleop_node --ros-args \
  -p scale_translation:=100.0 \
  -p scale_rotation:=100.0

# ===== HTTP ACTION SERVER =====
# Start Python HTTP server for sending pose deltas
# Server should expose GET /get_action endpoint returning:
# {"joints": {"x": 0.01, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}}
# Example with Flask:
# @app.route('/get_action', methods=['GET'])
# def get_action():
#     return jsonify({"joints": {"x": dx, "y": dy, "z": dz, "roll": dr, "pitch": dp, "yaw": dyw}})

# ===== WORKSPACE ENVIRONMENT VISUALIZATION =====
# Add tabletop environment with boxes to RViz visualization

# Option 1: Visual markers only (no collision checking)
ros2 launch ur5_workspace_description workspace_markers.launch.py

# Option 2: Collision objects only (for MoveIt planning - requires MoveIt running)
ros2 launch ur5_workspace_description workspace_collision_objects.launch.py

# Option 3: Both visualization markers AND collision objects (RECOMMENDED)
ros2 launch ur5_workspace_description workspace_environment.launch.py

# With custom options
ros2 launch ur5_workspace_description workspace_environment.launch.py \
  use_markers:=true \
  use_collision_objects:=true

# Run standalone nodes (alternative to launch files)
ros2 run ur5_workspace_description workspace_markers
ros2 run ur5_workspace_description workspace_collision_objects

# ===== COMPLETE SETUP WITH WORKSPACE =====
# Terminal 1: Launch robot with fake hardware
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=172.17.66.105 use_fake_hardware:=true

# Terminal 2: Launch MoveIt with RViz
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

# Terminal 3: Add workspace environment
ros2 launch ur5_workspace_description workspace_environment.launch.py

# In RViz, add these displays:
# - Add -> MarkerArray -> Topic: /workspace_markers (for visual markers)
# - Planning Scene should automatically show collision objects if MoveIt is running

# ===== WORKSPACE WITH REAL ROBOT =====
# Terminal 1: Launch real robot
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=172.17.66.105 use_fake_hardware:=false

# Terminal 2: Launch MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

# Terminal 3: Add workspace environment (collision objects will prevent robot from hitting obstacles)
ros2 launch ur5_workspace_description workspace_environment.launch.py

# ===== CUROBO MOTION GENERATION =====
# This package uses NVIDIA's curobo library for motion generation.
# It requires a specific Conda environment and environment variables.

# 1. Launch the UR5 driver (simulation)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=172.17.66.105 use_fake_hardware:=true

# 2. Launch the Curobo control node (in a separate terminal)
# This script handles the environment setup (LD_PRELOAD) and python interpreter patching.
./run_curobo.sh

# The robot will cycle through a sequence of target poses continuously.
# To stop, press Ctrl+C in the terminal running run_curobo.sh.

# ===== SAFETY MONITOR (SEPARATE COMPUTER) =====
# To run the computer vision safety monitor on a separate computer:

# 1. Install dependencies
pip install ultralytics opencv-python

# 2. Ensure ROS 2 is installed and networked
# Export ROS_DOMAIN_ID to match the robot computer (e.g., 0)
export ROS_DOMAIN_ID=0

# 3. Run the monitor script
# (Copy src/ur5_curobo_control/ur5_curobo_control/face_safety_monitor.py to the other computer)
python3 face_safety_monitor.py

# Optional parameters:
# python3 face_safety_monitor.py --ros-args -p camera_id:=0 -p threshold_area:=0.2

./run_safety_monitor.sh --ros-args -p use_realsense:=True