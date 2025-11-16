# Quick Reference - Running Isaac Sim Scripts

## Quick Start (5 minutes)

### 1. Test Isaac Sim Installation
```bash
cd /home/mani/Repos/ur_ws/isaac_standalone
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh simple_test.py
```

### 2. Load UR10 Robot (Easy Way)
```bash
./run_ur_robot.sh --robot ur10
```

### 3. Load UR5 Robot (Easy Way)
```bash
./run_ur_robot.sh --robot ur5
```

## All Scripts Summary

| Script | Purpose | Difficulty |
|--------|---------|------------|
| `simple_test.py` | Minimal example - verify installation | ⭐ Beginner |
| `ur10_import.py` | Basic UR5/UR10 loader | ⭐ Beginner |
| `ur_robot_advanced.py` | Advanced control with recording | ⭐⭐ Intermediate |
| `ur_from_ros2_urdf.py` | Load from ROS 2 workspace | ⭐⭐⭐ Advanced |

## Common Commands

### Using Isaac Sim Python Directly
```bash
ISAAC_PYTHON=/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh

# Simple test
$ISAAC_PYTHON simple_test.py

# Load UR10
$ISAAC_PYTHON ur10_import.py --robot ur10

# Load UR5
$ISAAC_PYTHON ur10_import.py --robot ur5

# Animated UR10 with recording
$ISAAC_PYTHON ur_robot_advanced.py --robot ur10 --mode animated --record

# Load from ROS 2 workspace
$ISAAC_PYTHON ur_from_ros2_urdf.py --robot ur5
```

### Using Helper Script
```bash
# Basic UR10
./run_ur_robot.sh --robot ur10

# Basic UR5
./run_ur_robot.sh --robot ur5

# Headless mode (no GUI)
./run_ur_robot.sh --robot ur10 --headless

# Test mode (quick run)
./run_ur_robot.sh --robot ur5 --test
```

## Control Modes (ur_robot_advanced.py)

### Static Mode
Robot stays at home position (good for testing)
```bash
$ISAAC_PYTHON ur_robot_advanced.py --mode static --robot ur10
```

### Animated Mode
Robot moves in smooth sinusoidal motion
```bash
$ISAAC_PYTHON ur_robot_advanced.py --mode animated --robot ur5
```

### Interactive Mode
Robot cycles through predefined poses
```bash
$ISAAC_PYTHON ur_robot_advanced.py --mode interactive --robot ur10
```

## Recording Joint States

Record joint positions and velocities to CSV:
```bash
$ISAAC_PYTHON ur_robot_advanced.py \
    --robot ur10 \
    --mode animated \
    --record \
    --frames 1000
```

Output: `ur10_recording_animated.csv`

## Loading from ROS 2 URDF

### From installed packages
```bash
$ISAAC_PYTHON ur_from_ros2_urdf.py \
    --robot ur5 \
    --workspace /home/mani/Repos/ur_ws
```

### Prerequisites
Make sure ur_description is built:
```bash
cd /home/mani/Repos/ur_ws
colcon build --packages-select ur_description
source install/setup.bash
```

## Troubleshooting

### "Could not find Isaac Sim assets folder"
Check Isaac Sim path in scripts:
```bash
ISAAC_SIM_PATH="/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64"
```

### Import errors in VS Code
These are normal! Isaac Sim modules only work with Isaac Sim's Python interpreter.

### Robot doesn't move
- Check that you're using a motion mode (animated/interactive)
- In static mode, robot stays at home position
- Use `--mode animated` for motion

### Performance issues
- Use `--headless` to disable rendering
- Reduce `--frames` for shorter simulation
- Close other applications using GPU

## Next Steps

1. **Modify joint trajectories** in `ur_robot_advanced.py` (line ~110)
2. **Add end-effector control** using inverse kinematics
3. **Import custom grippers** from URDF
4. **Add objects to pick and place**
5. **Connect to ROS 2** for real-time control

## File Locations

- **Scripts**: `/home/mani/Repos/ur_ws/isaac_standalone/`
- **Isaac Sim**: `/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/`
- **ROS 2 WS**: `/home/mani/Repos/ur_ws/`
- **UR Description**: `/home/mani/Repos/ur_ws/src/Universal_Robots_ROS2_Driver/ur_description/`

## Example Workflow

1. Test installation:
   ```bash
   ./run_ur_robot.sh --robot ur10 --test
   ```

2. Run animated simulation:
   ```bash
   $ISAAC_PYTHON ur_robot_advanced.py --robot ur10 --mode animated --frames 500
   ```

3. Record data:
   ```bash
   $ISAAC_PYTHON ur_robot_advanced.py --robot ur10 --mode animated --record --frames 1000
   ```

4. Analyze recording:
   ```bash
   python3 -c "import pandas as pd; df = pd.read_csv('ur10_recording_animated.csv'); print(df.describe())"
   ```

## Help Commands

```bash
# Get help for basic loader
$ISAAC_PYTHON ur10_import.py --help

# Get help for advanced controller
$ISAAC_PYTHON ur_robot_advanced.py --help

# Get help for ROS 2 URDF loader
$ISAAC_PYTHON ur_from_ros2_urdf.py --help
```
