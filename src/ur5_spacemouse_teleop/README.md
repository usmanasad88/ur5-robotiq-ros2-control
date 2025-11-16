# UR5 SpaceMouse Teleoperation

This package provides SpaceMouse-based teleoperation for the UR5 robot using pose deltas.

## Prerequisites

### 1. Install pyspacemouse library
```bash
pip install pyspacemouse
```

### 2. Setup udev rules for non-root access
Create `/etc/udev/rules.d/90-spacemouse.rules`:
```bash
sudo nano /etc/udev/rules.d/90-spacemouse.rules
```

Add this line:
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="256f", MODE="0666"
```

Reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 3. (Optional) Stop 3Dconnexion driver if cursor moves
If your cursor moves when you use the SpaceMouse:
```bash
sudo systemctl stop 3dxsrv
# Or disable permanently:
sudo systemctl disable 3dxsrv
```

## Build

```bash
cd ~/Repos/ur_ws
colcon build --packages-select ur5_spacemouse_teleop
source install/setup.bash
```

## Usage

### Complete System

**Terminal 1: Robot Driver**
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true
```

**Terminal 2: MoveIt**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

**Terminal 3: Controller**
```bash
ros2 run ur5_gen_controller random_joint_goal --ros-args \
  -p use_http_server:=false \
  -p use_random_motion:=false \
  -p max_iterations:=10000
```

**Terminal 4: SpaceMouse Teleop**
```bash
ros2 run ur5_spacemouse_teleop spacemouse_teleop_node --ros-args \
  -p scale_translation:=0.0001 \
  -p scale_rotation:=0.0005 \
  -p deadzone:=0.05 \
  -p publish_rate:=50.0
```

## Parameters

- **`device_number`**: SpaceMouse device number (default: 0)
- **`deadzone`**: Ignore values below this threshold (default: 0.05)
- **`scale_translation`**: Scale factor for translation (default: 0.0001)
  - Raw SpaceMouse values are typically -350 to +350
  - Scale converts to meters per timer tick
  - Example: 350 * 0.0001 = 0.035m = 3.5cm per tick (at 50Hz)
- **`scale_rotation`**: Scale factor for rotation (default: 0.0005)
  - Raw rotation values are typically -350 to +350
  - Scale converts to radians per timer tick
  - Example: 350 * 0.0005 = 0.175 rad ≈ 10 degrees per tick (at 50Hz)
- **`publish_rate`**: Publishing frequency in Hz (default: 50.0)
  - Higher rate = smoother motion
  - Lower rate = less CPU usage

## Tuning the Scales

If the robot moves too fast or too slow, adjust the scales:

**For slower motion (more precise control):**
```bash
ros2 run ur5_spacemouse_teleop spacemouse_teleop_node --ros-args \
  -p scale_translation:=0.00005 \
  -p scale_rotation:=0.00025
```

**For faster motion:**
```bash
ros2 run ur5_spacemouse_teleop spacemouse_teleop_node --ros-args \
  -p scale_translation:=0.0002 \
  -p scale_rotation:=0.001
```

## Troubleshooting

### Device not found
```bash
# Check if SpaceMouse is detected
lsusb | grep 3Dconnexion

# Should see something like:
# Bus 001 Device 005: ID 256f:c62e 3Dconnexion SpaceMouse Pro
```

### Permission denied
- Make sure udev rules are set up (see Prerequisites)
- Reconnect the device after setting up udev rules

### Cursor still moves
- Stop the 3Dconnexion system driver: `sudo systemctl stop 3dxsrv`

### Robot doesn't move
- Check that the controller is running and subscribed to `/ur5/teleop_delta`
- Try: `ros2 topic echo /ur5/teleop_delta` to see if messages are published
- Increase the scale values if motion is too small

## SpaceMouse Controls

The SpaceMouse Pro provides 6 degrees of freedom:

### Translation
- **Push/Pull**: Forward/Backward (Y axis)
- **Left/Right**: Left/Right (X axis)  
- **Up/Down**: Up/Down (Z axis)

### Rotation
- **Tilt Left/Right**: Roll
- **Tilt Forward/Back**: Pitch
- **Twist Left/Right**: Yaw

All motions are in the end-effector (tool) frame.

## Architecture

```
┌─────────────────────────┐
│ spacemouse_teleop_node  │
│ Reads SpaceMouse        │
│ Publishes PoseDelta     │
└───────────┬─────────────┘
            │
            │ /ur5/teleop_delta
            ▼
┌─────────────────────────┐
│ random_joint_goal       │
│ Subscribes & Executes   │
└─────────────────────────┘
```

## Testing Without Robot

You can test the SpaceMouse node without the robot:

```bash
# Terminal 1: Run the node
ros2 run ur5_spacemouse_teleop spacemouse_teleop_node

# Terminal 2: Echo the topic
ros2 topic echo /ur5/teleop_delta
```

Move the SpaceMouse and you should see PoseDelta messages being published.
