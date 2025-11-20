# UR5 ROS 2 Quick Start Guide

## Fixed Issues ✅

### 1. RViz Snap Library Conflict
**Problem**: RViz crashes with `symbol lookup error: libpthread.so.0: undefined symbol: __libc_pthread_init`

**Solution**: Use wrapper scripts that clean snap environment variables.

### 2. Missing moveit-servo Package
**Problem**: `package 'moveit_servo' not found`

**Solution**: Installed with `sudo apt install ros-humble-moveit-servo`

### 3. Incorrect File Paths
**Problem**: Commands referenced wrong user directory (`/home/mani/` instead of `/home/rml/`)

**Solution**: Updated all paths to use correct workspace location

## Available Launch Scripts

### 1. UR5 Controller (Simulation)
```bash
# Standard launch (may fail with snap VS Code)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true

# RECOMMENDED: Snap-safe wrapper script
./launch_ur5_nosnap.sh 192.168.1.102 true
```

**What it does**: Launches UR5 robot controller with fake hardware (simulation mode) and RViz visualization.

### 2. MoveIt Motion Planning
```bash
# Standard launch (may fail with snap VS Code)
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

# RECOMMENDED: Snap-safe wrapper script
./launch_moveit_nosnap.sh ur5 true
```

**What it does**: Launches MoveIt for motion planning with RViz interface for planning and executing trajectories.

### 3. UR5 + Robotiq Gripper
```bash
./launch_ur5_robotiq.sh 192.168.1.102 true
```

**What it does**: Launches UR5 with Robotiq 2F-85 gripper in simulation mode.

## Common Commands

### General Controller
```bash
# Run without HTTP server
ros2 run ur5_gen_controller random_joint_goal --ros-args \
  -p use_http_server:=false \
  -p use_random_motion:=false \
  -p max_iterations:=10000

# With kinematics configuration
ros2 run ur5_gen_controller random_joint_goal --ros-args \
  --params-file /home/rml/ur_ws/src/Universal_Robots_ROS2_Driver/ur_moveit_config/config/kinematics.yaml
```

### Keyboard Teleoperation
```bash
ros2 run ur5_keyboard_teleop keyboard_teleop_node --ros-args \
  -p linear_step:=0.01 \
  -p angular_step:=0.05
```

### SpaceMouse Teleoperation
```bash
ros2 run ur5_spacemouse_teleop spacemouse_teleop_node --ros-args \
  -p scale_translation:=0.001 \
  -p scale_rotation:=0.005 \
  -p publish_rate:=10.0
```

## Why Wrapper Scripts?

When running from snap-installed VS Code, the terminal inherits snap environment variables that cause Qt applications (like RViz) to load incompatible libraries. The wrapper scripts:

1. **Clean snap environment variables** (GTK_PATH, GIO_MODULE_DIR, etc.)
2. **Set correct LD_LIBRARY_PATH** (prioritize system libraries)
3. **Remove snap paths from PATH**
4. **Launch in isolated environment**

## Alternative Solution

Launch from a regular (non-snap) terminal:
1. Press `Ctrl+Alt+T` to open GNOME Terminal
2. `source ~/ur5-robotiq-ros2-control/install/setup.bash`
3. Run standard launch commands directly

## Workspace Structure
```
/home/rml/ur5-robotiq-ros2-control/
├── launch_ur5_nosnap.sh          # UR5 controller launcher
├── launch_moveit_nosnap.sh       # MoveIt launcher
├── launch_ur5_robotiq.sh         # UR5+Gripper launcher
├── Commands                      # Full command reference
├── RVIZ_SNAP_FIX.md             # Detailed RViz fix documentation
└── src/                         # Source packages
```

## Troubleshooting

### RViz Not Appearing
- **Symptom**: `symbol lookup error: libpthread.so.0`
- **Fix**: Use `./launch_*_nosnap.sh` wrapper scripts

### No Joint States
- **Symptom**: Robot model appears gray/transparent in RViz
- **Fix**: Ensure controller is running (`ros2 launch ur_robot_driver...`)

### Controller Spawner Errors
- **Symptom**: `[ERROR] [spawner]: process has died`
- **Note**: These are often timing issues and may not prevent visualization

### MoveIt Warnings
- **Symptom**: `No 3D sensor plugin(s) defined for octomap updates`
- **Note**: Normal when no depth sensor is configured (simulation only)

## Dependencies Installed
- ✅ ROS 2 Humble Desktop Full
- ✅ MoveIt 2
- ✅ ros2_control & ros2_controllers
- ✅ moveit-servo
- ✅ UR Robot Driver packages
- ✅ Custom teleop packages (keyboard, spacemouse, VR)

## Next Steps
1. Test motion planning in MoveIt
2. Configure your specific hardware (real robot IP, gripper, sensors)
3. Customize teleoperation parameters
4. Develop your application using provided examples

For more details, see:
- `Commands` - Complete command reference
- `RVIZ_SNAP_FIX.md` - Technical details on RViz fix
- `SETUP_COMPLETE.md` - Installation documentation
