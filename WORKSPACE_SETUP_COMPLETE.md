# UR5 Workspace Environment - Installation Complete ✓

## What Was Created

A complete ROS2 package (`ur5_workspace_description`) that adds a tabletop environment with objects to your UR5 workspace visualization.

### Package Structure

```
ur5_workspace_description/
├── ur5_workspace_description/
│   ├── __init__.py
│   ├── workspace_markers.py              # Option 4: Visual markers for RViz
│   └── workspace_collision_objects.py    # Option 5: Collision objects for MoveIt
├── launch/
│   ├── workspace_markers.launch.py
│   ├── workspace_collision_objects.launch.py
│   └── workspace_environment.launch.py   # Complete environment (both)
├── scripts/
│   ├── workspace_layout.py               # Visual diagram of workspace
│   └── test_workspace.py                 # Verification script
├── package.xml
├── setup.py
└── README.md
```

## Workspace Contents

### Objects Created
- **Table**: 1.2m × 0.8m brown tabletop (15cm below robot base)
- **Box 1** (Red): 10cm cube on left side
- **Box 2** (Green): 12×8×15cm rectangular box in center
- **Box 3** (Blue): 8×8×20cm tall thin box on right
- **Box 4** (Yellow): 15×15×10cm flat box near robot
- **Cylinder** (Orange): 20cm tall, 8cm diameter obstacle

All objects have text labels in RViz for easy identification.

## Quick Start

### 1. Complete Setup (RECOMMENDED)

Open 3 terminals and run:

```bash
# Terminal 1: Launch robot
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true

# Terminal 2: Launch MoveIt with RViz  
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

# Terminal 3: Add workspace environment
ros2 launch ur5_workspace_description workspace_environment.launch.py
```

### 2. Configure RViz

In RViz:
1. Click **Add** button (bottom left)
2. Select **MarkerArray** 
3. Set Topic to: `/workspace_markers`
4. Collision objects automatically appear in **Planning Scene** display

## Usage Options

### Option 4: Visual Markers Only
```bash
ros2 launch ur5_workspace_description workspace_markers.launch.py
```
- Shows objects in RViz
- No collision checking
- Good for visualization and debugging

### Option 5: Collision Objects Only  
```bash
ros2 launch ur5_workspace_description workspace_collision_objects.launch.py
```
- Adds objects to MoveIt planning scene
- Enables collision avoidance
- Requires MoveIt to be running

### Both (Complete Environment)
```bash
ros2 launch ur5_workspace_description workspace_environment.launch.py
```
- Visual markers AND collision objects
- Best option for most use cases

## With Real Robot

```bash
# Terminal 1: Real robot
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=false

# Terminal 2: MoveIt  
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

# Terminal 3: Workspace (with collision protection!)
ros2 launch ur5_workspace_description workspace_environment.launch.py
```

⚠️ **Important**: With collision objects enabled, MoveIt will prevent the robot from planning paths that would collide with the table or boxes.

## Verification

View the workspace layout:
```bash
python3 src/ur5_workspace_description/scripts/workspace_layout.py
```

Check package is installed:
```bash
source install/setup.bash
ros2 pkg list | grep ur5_workspace
ros2 pkg executables ur5_workspace_description
```

## Customization

To modify object positions/sizes, edit:
- `src/ur5_workspace_description/ur5_workspace_description/workspace_markers.py`
- `src/ur5_workspace_description/ur5_workspace_description/workspace_collision_objects.py`

Then rebuild:
```bash
cd ~/Repos/ur_ws
colcon build --packages-select ur5_workspace_description --symlink-install
source install/setup.bash
```

## Topics Published

- `/workspace_markers` - MarkerArray for RViz visualization
- `/planning_scene` - PlanningScene for MoveIt collision objects
- `/collision_object` - Individual CollisionObject messages

## Documentation

- **Quick Reference**: `/home/mani/Repos/ur_ws/WORKSPACE_ENVIRONMENT.md`
- **Full Documentation**: `src/ur5_workspace_description/README.md`
- **Commands**: See `/home/mani/Repos/ur_ws/Commands` (updated with new commands)

## Troubleshooting

**Markers don't appear:**
- Add MarkerArray display in RViz
- Set topic to `/workspace_markers`
- Verify node is running: `ros2 node list | grep workspace`

**Collision objects not working:**
- Ensure MoveIt is launched first
- Enable Planning Scene display in RViz
- Check: `ros2 topic echo /planning_scene --once`

**Objects in wrong position:**
- All positions are relative to `base_link`
- Verify TF tree: `ros2 run tf2_tools view_frames`

## Status

✓ Package created and built successfully  
✓ Two publisher nodes implemented (markers + collision objects)  
✓ Three launch files created  
✓ Documentation and quick reference guides added  
✓ Commands file updated with usage instructions  
✓ Verification tests created  
✓ Workspace layout diagram available  

## Next Steps

1. **Launch the environment** using the commands above
2. **Add MarkerArray display** in RViz to see objects
3. **Test motion planning** with collision avoidance enabled
4. **Customize objects** if needed for your specific use case

Enjoy your new tabletop workspace environment! 🎉
