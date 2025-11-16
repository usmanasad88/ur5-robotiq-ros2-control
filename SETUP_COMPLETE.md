# ✅ UR5 + Robotiq 2F-85 Integration Complete

## What Was Accomplished

Successfully integrated the Robotiq 2F-85 gripper with the UR5 robot for ROS2 Humble, including:

1. **Created `ur5_robotiq_description` package** - Self-contained package with gripper URDF and meshes
2. **Fixed RViz crash** - Resolved snap library conflict preventing visualization  
3. **Complete visualization** - Robot + gripper visible in RViz with all segments
4. **Maintained compatibility** - All existing UR packages remain functional

## Launch Commands

### Quick Start (Fake Hardware)
```bash
# Use the fixed launch script (recommended)
cd ~/Repos/ur_ws
./launch_ur5_robotiq.sh

# Or use ros2 launch directly
source ~/.bashrc  # Ensure RViz fix is applied
cd ~/Repos/ur_ws
source install/setup.bash
ros2 launch ur5_robotiq_description ur5_robotiq.launch.py use_fake_hardware:=true
```

**Note**: RViz will now open with a custom configuration showing the UR5 + Robotiq gripper with:
- Grid display
- RobotModel with all gripper links visible
- TF tree showing robot and gripper frames
- Fixed frame set to `world`

### Real Robot
```bash
./launch_ur5_robotiq.sh 192.168.1.102 false
```

## What's in the Workspace

### New Package: `ur5_robotiq_description`
```
ur5_robotiq_description/
├── urdf/
│   ├── robotiq_2f85.urdf.xacro      # Gripper macro (adapted from IFRA)
│   └── ur5_robotiq.urdf.xacro       # Combined UR5 + gripper
├── meshes/robotiq_2f_85/            # STL files (copied from IFRA)
├── config/ur5/                      # Joint limits, kinematics (from ur_description)
├── launch/
│   └── ur5_robotiq.launch.py        # Main launch file
└── README.md                        # Package documentation
```

### Helper Scripts
- `launch_ur5_robotiq.sh` - Launch script with RViz snap library fix
- `test_rviz_nosnap.sh` - Diagnostic script to test RViz

### Documentation
- `INTEGRATION_SUMMARY.md` - Complete technical details
- `QUICK_REFERENCE.md` - One-page quick reference
- `RVIZ_FIX.md` - RViz troubleshooting guide
- `SETUP_COMPLETE.md` - This file

## RViz Snap Library Fix

### The Problem
RViz was crashing with: `symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init`

### The Solution
Modified `~/.bashrc` to rebuild `LD_LIBRARY_PATH` before sourcing ROS, preventing snap libraries from interfering:

```bash
# This is now in your ~/.bashrc (already applied)
unset GTK_PATH GTK2_RC_FILES GTK_IM_MODULE QT_QPA_PLATFORMTHEME 2>/dev/null
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"
```

**To apply the fix**: Open a new terminal or run `source ~/.bashrc`

## Verification

### Check Gripper Segments Are Loaded
```bash
ros2 topic echo /robot_description --once | grep robotiq
```

Should show:
- `robotiq_coupler`
- `robotiq_85_base_link`
- `robotiq_85_left_finger_link`, `robotiq_85_right_finger_link`
- `robotiq_85_left_knuckle_link`, `robotiq_85_right_knuckle_link`
- `robotiq_85_left_finger_tip_link`, `robotiq_85_right_finger_tip_link`
- `gripper_tcp`

### Check RViz is Running
```bash
ps aux | grep rviz2 | grep -v grep
```

### Test with MoveIt
```bash
cd ~/Repos/ur_ws
source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 use_fake_hardware:=true
```

## Key Technical Details

### Gripper Attachment
- Attached to UR5 `tool0` frame via `robotiq_coupler` fixed joint
- Tool Center Point (TCP) at `gripper_tcp` link (+16.6cm from tool0)

### Gripper Kinematics
- 2-finger parallel jaw gripper with 85mm stroke
- Mimic joints: right finger mirrors left finger motion
- 4 knuckles per side for realistic finger articulation

### URDF Structure
```
world → base_link → UR5 arm → tool0 → robotiq_coupler → robotiq_85_base_link → fingers
```

## MoveIt Integration

The gripper is defined in the robot description but **not yet configured in MoveIt** for planning. To enable gripper control in MoveIt:

1. Update MoveIt configuration to include gripper group
2. Define gripper controllers
3. Configure planning groups

See `INTEGRATION_SUMMARY.md` section "MoveIt Configuration" for details.

## Troubleshooting

### RViz still crashes after bashrc fix
1. Open a **new terminal** (old terminals keep old environment)
2. Verify: `echo $LD_LIBRARY_PATH | grep snap` (should return nothing)
3. Use the launch script: `./launch_ur5_robotiq.sh`

### Gripper not visible in RViz
1. Verify robot state publisher loaded gripper:
   ```bash
   ros2 topic echo /robot_description --once | grep robotiq
   ```
2. In RViz, check that RobotModel display is enabled
3. Verify TF tree: `ros2 run tf2_tools view_frames`

### Package build errors
```bash
cd ~/Repos/ur_ws
colcon build --packages-select ur5_robotiq_description
source install/setup.bash
```

## Next Steps

1. **Test with Real Robot**
   - Update `robot_ip` parameter to your UR5's IP
   - Test visualization before enabling hardware control

2. **Add Gripper Control**
   - Integrate Robotiq ROS2 driver for gripper actuation
   - Configure MoveIt for coordinated arm+gripper planning

3. **Customize TCP**
   - Adjust `gripper_tcp` position if using custom tools
   - Update in `urdf/robotiq_2f85.urdf.xacro`

## References

- ROS1 Reference: `ros_1_for_reference/ur5_robot/`
- ROS2 Reference: `ifra_cranfield/ros2_SimRealRobotControl/`
- UR Driver Docs: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- Robotiq Gripper: https://robotiq.com/products/2f85-140-adaptive-robot-gripper

---

**Status**: ✅ Complete and Tested  
**ROS2**: Humble  
**Robot**: UR5  
**Gripper**: Robotiq 2F-85  
**Date**: November 2025
