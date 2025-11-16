# UR5 Keyboard Teleoperation

This package provides keyboard-based teleoperation for the UR5 robot using pose deltas.

## Usage

### 1. Build the workspace
```bash
cd ~/Repos/ur_ws
colcon build --packages-select ur5_teleop_msgs ur5_keyboard_teleop ur5_gen_controller
source install/setup.bash
```

### 2. Launch the robot and MoveIt
```bash
# Terminal 1: Robot driver
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true

# Terminal 2: MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true
```

### 3. Start the controller
```bash
# Terminal 3: Controller (subscribes to keyboard and optionally HTTP)
ros2 run ur5_gen_controller random_joint_goal --ros-args \
  -p use_http_server:=false \
  -p use_random_motion:=false \
  -p max_iterations:=10000
```

### 4. Start keyboard teleoperation
```bash
# Terminal 4: Keyboard teleop
ros2 run ur5_keyboard_teleop keyboard_teleop_node --ros-args \
  -p linear_step:=0.01 \
  -p angular_step:=0.05
```

## Keyboard Controls

### Translation
- **W/S**: Move forward/backward (+X/-X)
- **A/D**: Move left/right (+Y/-Y)
- **Q/E**: Move up/down (+Z/-Z)

### Rotation
- **R/F**: Roll +/-
- **T/G**: Pitch +/-
- **Y/H**: Yaw +/-

### Commands
- **I**: Show instructions
- **X** or **ESC**: Quit

## Parameters

### keyboard_teleop_node
- `linear_step`: Distance per keypress in meters (default: 0.01 = 1cm)
- `angular_step`: Rotation per keypress in radians (default: 0.05 ≈ 3°)

### random_joint_goal (controller)
- `use_http_server`: Enable HTTP server input (default: false)
- `use_random_motion`: Enable random motion when no input (default: false)
- `max_iterations`: Maximum number of motions to execute (default: 1000)
- `action_server_url`: URL for HTTP server (default: http://localhost:5000/joint_actions)

## Architecture

```
┌─────────────────────────┐
│ keyboard_teleop_node    │
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

## Future Extensions

- `ur5_spacemouse_teleop`: SpaceMouse support
- `ur5_vr_teleop`: VR controller support

All will publish to the same `/ur5/teleop_delta` topic.
