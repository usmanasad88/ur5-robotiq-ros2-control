# UR5 Workspace Description

ROS2 package for visualizing and adding collision objects to the UR5 robot workspace. This package provides a tabletop environment with a table and several boxes that can be:

1. **Visualized in RViz** (Option 4) - Visual markers only
2. **Used for collision avoidance** (Option 5) - MoveIt planning scene integration
3. **Both** - Complete workspace environment (recommended)

## Features

- **Tabletop Environment**: 120cm x 80cm table positioned below the robot
- **Multiple Objects**: 
  - 4 boxes of different sizes and colors (Red, Green, Blue, Yellow)
  - 1 cylinder obstacle (Orange)
  - Text labels for easy identification
- **Dual Publishing**:
  - Visualization markers (`/workspace_markers`) for RViz display
  - Collision objects (`/planning_scene`) for MoveIt collision avoidance

## Installation

The package is already part of your workspace. Build it with:

```bash
cd ~/Repos/ur_ws
colcon build --packages-select ur5_workspace_description --symlink-install
source install/setup.bash
```

## Usage

### Quick Start - Complete Environment (RECOMMENDED)

Launch both visualization and collision objects:

```bash
# Terminal 1: Launch robot
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true

# Terminal 2: Launch MoveIt with RViz
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

# Terminal 3: Add workspace environment
ros2 launch ur5_workspace_description workspace_environment.launch.py
```

**In RViz:**
1. Add display: `Add` → `MarkerArray` → Topic: `/workspace_markers`
2. The collision objects will automatically appear in the `Planning Scene` display

### Option 1: Visualization Markers Only

For visual reference without collision checking:

```bash
ros2 launch ur5_workspace_description workspace_markers.launch.py
```

Or run the node directly:
```bash
ros2 run ur5_workspace_description workspace_markers
```

**In RViz:** Add → MarkerArray → Topic: `/workspace_markers`

### Option 2: Collision Objects Only

For collision avoidance in MoveIt (requires MoveIt to be running):

```bash
ros2 launch ur5_workspace_description workspace_collision_objects.launch.py
```

Or run the node directly:
```bash
ros2 run ur5_workspace_description workspace_collision_objects
```

The objects will appear in RViz's Planning Scene display automatically.

### Option 3: Both (Complete Environment)

```bash
# Default - both markers and collision objects
ros2 launch ur5_workspace_description workspace_environment.launch.py

# Only visualization markers
ros2 launch ur5_workspace_description workspace_environment.launch.py use_collision_objects:=false

# Only collision objects
ros2 launch ur5_workspace_description workspace_environment.launch.py use_markers:=false
```

## Workspace Objects

The environment includes the following objects (all positioned relative to `base_link`):

### Table
- **Type**: Box
- **Dimensions**: 1.2m x 0.8m x 0.1m (Length x Width x Height)
- **Position**: [0.0, 0.0, -0.15] (15cm below robot base)
- **Color**: Brown

### Box 1 (Red)
- **Dimensions**: 0.1m x 0.1m x 0.1m (cube)
- **Position**: [0.3, 0.2, -0.05]
- **Color**: Red

### Box 2 (Green)
- **Dimensions**: 0.12m x 0.08m x 0.15m (rectangular)
- **Position**: [0.35, -0.1, -0.05]
- **Color**: Green

### Box 3 (Blue)
- **Dimensions**: 0.08m x 0.08m x 0.2m (tall thin)
- **Position**: [0.25, -0.25, -0.05]
- **Color**: Blue

### Box 4 (Yellow)
- **Dimensions**: 0.15m x 0.15m x 0.1m (wide flat)
- **Position**: [0.15, 0.0, -0.05]
- **Color**: Yellow

### Cylinder (Orange)
- **Dimensions**: Height=0.2m, Radius=0.04m
- **Position**: [0.4, 0.3, 0.0]
- **Color**: Orange

## Using with Real Robot

```bash
# Terminal 1: Launch real robot
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=false

# Terminal 2: Launch MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true

# Terminal 3: Add workspace environment
ros2 launch ur5_workspace_description workspace_environment.launch.py
```

The collision objects will prevent MoveIt from planning paths that would collide with the table or boxes.

## Customization

To modify the workspace objects, edit:
- **Visualization markers**: `ur5_workspace_description/workspace_markers.py`
- **Collision objects**: `ur5_workspace_description/workspace_collision_objects.py`

After editing, rebuild:
```bash
cd ~/Repos/ur_ws
colcon build --packages-select ur5_workspace_description --symlink-install
```

## Topics Published

- `/workspace_markers` (visualization_msgs/MarkerArray) - Visual markers for RViz
- `/planning_scene` (moveit_msgs/PlanningScene) - Collision objects for MoveIt
- `/collision_object` (moveit_msgs/CollisionObject) - Individual collision objects

## Troubleshooting

### Objects don't appear in RViz
1. Make sure you've added the MarkerArray display to RViz
2. Check that the topic is `/workspace_markers`
3. Verify the node is running: `ros2 node list | grep workspace`

### Collision objects not working
1. Ensure MoveIt is running first
2. Check that the Planning Scene display is enabled in RViz
3. Verify objects are published: `ros2 topic echo /planning_scene --once`

### Objects in wrong position
- All positions are relative to `base_link` frame
- Make sure your robot's base_link is correctly positioned
- Check TF tree: `ros2 run tf2_tools view_frames`

## License

BSD License
