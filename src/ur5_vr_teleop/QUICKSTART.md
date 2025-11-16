# HTC Vive VR Teleoperation - Quick Start

## Installation

```bash
# 1. Install Python dependencies
pip install openvr numpy

# 2. Build the package
cd ~/Repos/ur_ws
colcon build --packages-select ur5_vr_teleop
source install/setup.bash
```

## Running

### Terminal Commands (Copy-Paste Ready)

**Terminal 1: Start SteamVR**
```bash
~/.steam/steam/steamapps/common/SteamVR/bin/vrstartup.sh
# Or launch from Steam GUI
```

**Terminal 2: Robot Driver**
```bash
cd ~/Repos/ur_ws && source install/setup.bash && ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true
```

**Terminal 3: MoveIt**
```bash
cd ~/Repos/ur_ws && source install/setup.bash && ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

**Terminal 4: Controller**
```bash
cd ~/Repos/ur_ws && source install/setup.bash && ros2 run ur5_gen_controller random_joint_goal --ros-args -p use_http_server:=false -p use_random_motion:=false -p max_iterations:=10000
```

**Terminal 5: VR Teleop (Right Controller)**
```bash
cd ~/Repos/ur_ws && source install/setup.bash && ros2 run ur5_vr_teleop vr_teleop_node --ros-args -p controller_side:=right -p scale_translation:=0.1 -p scale_rotation:=0.1
```

## Usage

1. **Start all terminals** in order
2. Pick up your **right Vive controller**
3. **Hold the trigger** to start controlling
4. **Move the controller** - robot follows
5. **Release trigger** to stop

## Tips

- Start with **small movements** to get a feel for the control
- The robot moves in **relative mode** - it follows your controller movements
- You can **reposition** the controller while trigger is released
- **Lower scales** = more precise control
- **Higher scales** = faster movement

## Troubleshooting

**Robot not moving?**
- Make sure you're holding the trigger
- Try: `ros2 topic echo /ur5/teleop_delta`
- Increase scales if needed

**Controller not found?**
- Check SteamVR shows controller as green
- Try other hand: `-p controller_side:=left`

**SteamVR issues?**
- Close and reopen SteamVR
- Restart the vr_teleop_node

## Safety

⚠️ **Always start with simulation** (use_fake_hardware:=true)  
⚠️ **Test with low scales first** (0.05-0.1)  
⚠️ **Keep trigger released when not controlling**
