# UR5 + Robotiq 2F-85 Description Package

ROS2 Humble package providing URDF description for UR5 robot with Robotiq 2F-85 gripper.

## Features

✅ **Complete Robot Description**: UR5 arm + Robotiq 2F-85 gripper  
✅ **ROS2 Humble Compatible**: Works with current Universal Robots ROS2 Driver  
✅ **Self-Contained**: Includes all meshes, no external dependencies (except ur_description)  
✅ **Drop-in Replacement**: Works with existing launch files  
✅ **RViz Visualization**: See complete robot with gripper in RViz  

## Quick Start

### Build the Package

```bash
cd /home/mani/Repos/ur_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5_robotiq_description
source install/setup.bash

# Or if already built other packages:
colcon build
source install/setup.bash
```

### Launch (Simulation)

```bash
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py \
  robot_ip:=192.168.1.102 \
  use_fake_hardware:=true
```

### Launch (Real Robot)

```bash
# Terminal 1: Robot control with gripper visualization
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py \
  robot_ip:=192.168.1.102 \
  use_fake_hardware:=false

# Terminal 2: Gripper control (when needed)
ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py \
  --ros-args -p robot_ip:=192.168.1.102
```

## What's Different from Standard Setup

### Before
```bash
# Old commands - gripper not visible
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true
# ❌ No gripper in RViz
```

### After
```bash
# New command - gripper included
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py robot_ip:=192.168.1.102 use_fake_hardware:=true
# ✅ Gripper visible in RViz!
```

## Using with MoveIt

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5 \
  description_package:=ur5_robotiq_description \
  description_file:=ur5_robotiq.urdf.xacro
```

## Package Contents

```
ur5_robotiq_description/
├── urdf/
│   ├── ur5_robotiq.urdf.xacro      # Main URDF (UR5 + gripper)
│   └── robotiq_2f85.urdf.xacro     # Gripper macro
├── meshes/                          # Robotiq gripper meshes
│   ├── visual/                      # Visual meshes (.dae)
│   └── collision/                   # Collision meshes (.stl)
├── launch/
│   └── ur5_robotiq.launch.py       # Main launch file
└── rviz/                            # RViz configs (optional)
```

## Frame Hierarchy

```
world
└── base_link (UR5)
    └── ... (UR5 joints)
        └── tool0
            └── robotiq_coupler
                └── robotiq_85_base_link
                    ├── robotiq_85_left_knuckle_link
                    │   └── robotiq_85_left_finger_link
                    │       └── robotiq_85_left_finger_tip_link
                    ├── robotiq_85_right_knuckle_link
                    │   └── robotiq_85_right_finger_link
                    │       └── robotiq_85_right_finger_tip_link
                    └── gripper_tcp (Tool Center Point)
```

## Important Notes

### Does NOT Break Existing Setup
- Your existing UR5 packages remain untouched
- Standard `ur_robot_driver` launch files still work
- This is an **additional** package, not a replacement

### Gripper Control vs Visualization
- **This package**: Provides gripper **visualization** in RViz
- **robotiq_2f_urcap_adapter**: Provides gripper **control** (run separately when needed)

### ROS2 Humble Compatibility
- Adapted from IFRA Cranfield's ROS2 framework
- Works with current Universal Robots ROS2 Driver (Humble)
- No ROS Noetic dependencies

## Troubleshooting

### Meshes not loading
Ensure the package is installed:
```bash
ros2 pkg prefix ur5_robotiq_description
# Should output the install path
```

### TF frames missing
Check TF tree:
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
# Look for robotiq_ frames
```

### Build errors
Clean build:
```bash
cd /home/mani/Repos/ur_ws
colcon build --packages-select ur5_robotiq_description --cmake-clean-cache
```

## Credits

- Gripper URDF adapted from [IFRA Cranfield ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl)
- UR5 integration uses [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

## License

Apache-2.0 (same as source repositories)
