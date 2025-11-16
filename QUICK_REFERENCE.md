# 🤖 UR5 + Robotiq 2F-85 Quick Reference

## 🚀 Quick Start (One Command!)

```bash
# Simulation Mode
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py

# Real Robot
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py use_fake_hardware:=false robot_ip:=192.168.1.102
```

## 📦 First Time Setup

```bash
cd /home/mani/Repos/ur_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5_robotiq_description
source install/setup.bash
```

## 🆚 Before vs After

### OLD (No Gripper in RViz):
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true
```

### NEW (With Gripper):
```bash
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py robot_ip:=192.168.1.102 use_fake_hardware:=true
```

## ✅ What You Get

- ✅ UR5 robot model
- ✅ Robotiq 2F-85 gripper attached
- ✅ Complete visualization in RViz  
- ✅ All TF frames published
- ✅ Compatible with MoveIt
- ✅ Existing packages still work

## 🔧 Common Commands

```bash
# Build package
colcon build --packages-select ur5_robotiq_description

# Source workspace  
source ~/Repos/ur_ws/install/setup.bash

# Check package installed
ros2 pkg list | grep ur5_robotiq

# View TF tree
ros2 run tf2_tools view_frames

# Use with MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 \
  description_package:=ur5_robotiq_description \
  description_file:=ur5_robotiq.urdf.xacro
```

## 🛠️ Troubleshooting

### Gripper not visible?
```bash
ros2 topic echo /robot_description | head -20
# Should show robotiq links
```

### Build errors?
```bash
colcon build --packages-select ur5_robotiq_description --cmake-clean-cache
```

### Package not found?
```bash
source ~/Repos/ur_ws/install/setup.bash
```

## 📂 Package Location

`/home/mani/Repos/ur_ws/src/ur5_robotiq_description/`

## 📖 Documentation

- Full details: `/home/mani/Repos/ur_ws/INTEGRATION_SUMMARY.md`
- Package README: `/home/mani/Repos/ur_ws/src/ur5_robotiq_description/README.md`

## 🎯 Key Points

1. **Non-Breaking**: Your existing setup still works
2. **Visualization Only**: For gripper control, still use `robotiq_2f_urcap_adapter`
3. **ROS2 Humble**: Compatible with your current ROS version
4. **Self-Contained**: No external package dependencies (except ur_description)

---

**Need Help?** Check the full documentation in `INTEGRATION_SUMMARY.md`
