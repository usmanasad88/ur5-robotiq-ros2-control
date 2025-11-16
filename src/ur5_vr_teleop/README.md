# UR5 HTC Vive VR Teleoperation

This package provides HTC Vive VR controller-based teleoperation for the UR5 robot using pose deltas.

## Prerequisites

### 1. Install SteamVR
- Install Steam from: https://store.steampowered.com/
- Install SteamVR from Steam Library
- Set up your HTC Vive headset and controllers

### 2. Install Python dependencies
```bash
pip install openvr numpy
```

### 3. Test SteamVR
- Launch SteamVR
- Make sure controllers are tracked (green indicators in SteamVR status window)
- You don't need to wear the headset - just have the controllers tracked

## Build

```bash
cd ~/Repos/ur_ws
colcon build --packages-select ur5_vr_teleop
source install/setup.bash
```

## Usage

### Complete System

**Terminal 1: Start SteamVR**
```bash
# Launch SteamVR from Steam or:
~/.steam/steam/steamapps/common/SteamVR/bin/vrstartup.sh
```

**Terminal 2: Robot Driver**
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true
```

**Terminal 3: MoveIt**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

**Terminal 4: Controller**
```bash
ros2 run ur5_gen_controller random_joint_goal --ros-args \
  -p use_http_server:=false \
  -p use_random_motion:=false \
  -p max_iterations:=10000
```

**Terminal 5: VR Teleop**
```bash
ros2 run ur5_vr_teleop vr_teleop_node --ros-args \
  -p controller_side:=right \
  -p scale_translation:=0.1 \
  -p scale_rotation:=0.1 \
  -p publish_rate:=10.0 \
  -p use_relative_mode:=true
```

## Parameters

- **`controller_side`**: Which controller to use (default: `'right'`)
  - Options: `'left'` or `'right'`
  
- **`scale_translation`**: Scale factor for translation (default: 0.1)
  - Higher values = more sensitive translation
  - Typical range: 0.05 - 0.5
  
- **`scale_rotation`**: Scale factor for rotation (default: 0.1)
  - Higher values = more sensitive rotation
  - Typical range: 0.05 - 0.5
  
- **`publish_rate`**: Publishing frequency in Hz (default: 10.0)
  - Typical range: 5.0 - 20.0 Hz
  - Higher rate = smoother but more computational load
  
- **`trigger_threshold`**: Trigger activation threshold (default: 0.1)
  - Range: 0.0 - 1.0
  - Lower = more sensitive trigger
  
- **`use_relative_mode`**: Use relative position deltas (default: true)
  - `true`: Robot moves relative to controller movement
  - `false`: Robot moves to absolute controller position (not recommended)

## How to Use

### Basic Operation

1. **Launch all terminals** as shown above
2. **Hold the trigger** on your right (or left) controller
3. **Move the controller** in 3D space:
   - The robot end-effector will follow your movements
   - Release trigger to stop sending commands
   - Pull trigger again to resume control

### Controller Mapping

**HTC Vive Controller:**
```
        ╔═══════════╗
        ║  Trackpad ║  (Not used by default)
        ╚═══════════╝
            
     [Grip Button]      (Not used by default)
            
     [System Button]    (SteamVR menu)
            
     [Trigger] ──────   **HOLD TO CONTROL ROBOT**
            
      (Controller 
       Position & 
       Orientation)  ──  **ROBOT FOLLOWS THIS**
```

### Control Modes

**Relative Mode (Recommended):**
- Pull trigger
- Move controller
- Robot follows your movement deltas
- Release trigger to pause
- Can reposition controller while trigger is released
- Pull trigger again to resume from new position

**Absolute Mode (Not Recommended):**
- Robot tries to match controller's absolute position
- Can be dangerous if controller is far from robot's current position

## Coordinate System

The VR coordinate system is mapped to the robot end-effector frame:

| VR Controller | Robot End-Effector |
|---------------|-------------------|
| Right         | +X (forward)      |
| Up            | +Z (up)           |
| Forward       | -Y (backward)     |
| Roll          | Roll              |
| Pitch         | Pitch             |
| Yaw           | Yaw               |

## Tuning

### If robot moves too fast:
```bash
ros2 run ur5_vr_teleop vr_teleop_node --ros-args \
  -p scale_translation:=0.05 \
  -p scale_rotation:=0.05
```

### If robot moves too slow:
```bash
ros2 run ur5_vr_teleop vr_teleop_node --ros-args \
  -p scale_translation:=0.2 \
  -p scale_rotation:=0.2
```

### For smoother motion (higher rate):
```bash
ros2 run ur5_vr_teleop vr_teleop_node --ros-args \
  -p publish_rate:=20.0 \
  -p scale_translation:=0.05 \
  -p scale_rotation:=0.05
```

**Note:** Higher publish rates require lower scales to maintain similar speeds.

## Troubleshooting

### SteamVR not starting
```bash
# Check if SteamVR process is running
ps aux | grep vrcompositor

# Kill existing instances if needed
killall vrcompositor

# Restart SteamVR
~/.steam/steam/steamapps/common/SteamVR/bin/vrstartup.sh
```

### Controller not found
- Make sure SteamVR shows controllers as tracked (green)
- Try turning controllers off and on
- Check `controller_side` parameter matches your active controller
- Check with: `ros2 run ur5_vr_teleop vr_teleop_node --ros-args -p controller_side:=left`

### OpenVR initialization failed
- SteamVR must be running **before** starting the node
- If SteamVR crashes, restart both SteamVR and the node

### Robot not moving
- Pull and hold the trigger
- Check topic: `ros2 topic echo /ur5/teleop_delta`
- Increase scale values if deltas are too small
- Make sure controller node is running

### Motion is jittery
- Lower the `publish_rate`
- Increase scale values proportionally
- Check SteamVR performance (lighthouse positioning)

## Safety Notes

⚠️ **IMPORTANT SAFETY CONSIDERATIONS:**

1. **Start with low scales** (0.05-0.1) and increase gradually
2. **Keep trigger released** when not actively controlling
3. **Test in simulation first** before using with real robot
4. **Be aware of workspace limits** - VR space is larger than robot workspace
5. **Use e-stop if available** when working with real hardware
6. **Don't make sudden movements** - smooth controller motion = smooth robot motion

## Testing Without Robot

Test the VR node independently:

```bash
# Terminal 1: Start SteamVR
~/.steam/steam/steamapps/common/SteamVR/bin/vrstartup.sh

# Terminal 2: Run VR node
ros2 run ur5_vr_teleop vr_teleop_node

# Terminal 3: Monitor output
ros2 topic echo /ur5/teleop_delta
```

Hold trigger and move controller - you should see PoseDelta messages.

## Architecture

```
┌─────────────────────────┐
│   SteamVR/OpenVR        │
│   (HTC Vive Runtime)    │
└────────────┬────────────┘
             │
┌────────────▼────────────┐
│   vr_teleop_node        │
│   Reads Controller      │
│   Publishes PoseDelta   │
└────────────┬────────────┘
             │
             │ /ur5/teleop_delta
             ▼
┌─────────────────────────┐
│ random_joint_goal       │
│ Subscribes & Executes   │
└─────────────────────────┘
```

## Advanced: Multiple Input Devices

You can switch between keyboard, SpaceMouse, and VR without changing the controller:

**Use Keyboard:**
```bash
ros2 run ur5_keyboard_teleop keyboard_teleop_node
```

**Use SpaceMouse:**
```bash
ros2 run ur5_spacemouse_teleop spacemouse_teleop_node
```

**Use VR:**
```bash
ros2 run ur5_vr_teleop vr_teleop_node
```

All publish to the same `/ur5/teleop_delta` topic!
