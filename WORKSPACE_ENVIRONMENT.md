# Workspace Environment Quick Reference

## What is this?

A tabletop environment with boxes for the UR5 robot that can be:
- **Visualized in RViz** (markers)
- **Used for collision avoidance** (MoveIt planning)
- **Both** (recommended)

## Objects Included

- **Table**: 120cm x 80cm brown table (15cm below robot base)
- **4 Boxes**: Red, Green, Blue, Yellow boxes of various sizes
- **1 Cylinder**: Orange cylinder obstacle

## Quick Start

### Complete Setup (3 terminals)

```bash
# Terminal 1: Launch robot (simulation)
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true

# Terminal 2: Launch MoveIt with RViz
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

# Terminal 3: Add workspace environment
ros2 launch ur5_workspace_description workspace_environment.launch.py
```

### In RViz

1. **Add MarkerArray display**:
   - Click `Add` button
   - Select `MarkerArray`
   - Set Topic to `/workspace_markers`

2. **Collision objects automatically appear** in the `Planning Scene` display

## Usage Options

### Option 1: Visual Markers Only (No Collision Checking)
```bash
ros2 launch ur5_workspace_description workspace_markers.launch.py
```

### Option 2: Collision Objects Only (MoveIt Integration)
```bash
ros2 launch ur5_workspace_description workspace_collision_objects.launch.py
```

### Option 3: Complete Environment (Both - RECOMMENDED)
```bash
ros2 launch ur5_workspace_description workspace_environment.launch.py
```

## With Real Robot

```bash
# Terminal 1
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=false

# Terminal 2
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

# Terminal 3
ros2 launch ur5_workspace_description workspace_environment.launch.py
```

**⚠️ Important**: Collision objects will prevent the robot from hitting the table/boxes during motion planning.

## Customization

Edit the object positions/sizes in:
- `src/ur5_workspace_description/ur5_workspace_description/workspace_markers.py`
- `src/ur5_workspace_description/ur5_workspace_description/workspace_collision_objects.py`

Then rebuild:
```bash
cd ~/Repos/ur_ws
colcon build --packages-select ur5_workspace_description --symlink-install
source install/setup.bash
```

## Troubleshooting

**Markers don't appear in RViz:**
- Add MarkerArray display with topic `/workspace_markers`
- Verify node is running: `ros2 node list | grep workspace`

**Collision objects not working:**
- Make sure MoveIt is running first
- Enable Planning Scene display in RViz
- Check: `ros2 topic echo /planning_scene --once`

## See Also

- Full documentation: `src/ur5_workspace_description/README.md`
- Commands reference: `Commands` file in workspace root
