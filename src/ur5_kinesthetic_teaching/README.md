# UR5 Kinesthetic Teaching Module

A ROS 2 package for kinesthetic teaching (learning by demonstration) of the UR5 robot. This package allows you to manually guide the robot and record trajectories in both **joint space** and **task space**, then replay them.

## Features

- **Dual-space trajectory recording**: Captures both joint angles and end-effector poses
- **Real-time visualization**: See recorded paths in RViz as you teach
- **Flexible playback**: Replay trajectories at different speeds with loop support
- **Smart recording**: Only records when robot moves (configurable threshold)
- **Auto-save**: Automatically saves trajectories when recording stops
- **JSON format**: Human-readable trajectory files for easy analysis
- **Post-processing**: Remove idle time from recordings for cleaner trajectories (see [POST_PROCESSING_GUIDE.md](POST_PROCESSING_GUIDE.md))

## Package Contents

### Nodes

1. **`trajectory_recorder.py`** - Records robot trajectories during manual manipulation
   - Subscribes to `/joint_states` for joint space data
   - Uses TF for task space (end-effector pose) data
   - Provides services to control recording
   - Publishes visualization markers

2. **`trajectory_player.py`** - Replays recorded trajectories
   - Loads trajectories from JSON files
   - Executes via `FollowJointTrajectory` action
   - Supports speed control and looping
   - Lists available trajectories

3. **`trajectory_visualizer.py`** - Visualizes recorded trajectories in RViz
   - Shows trajectory path as colored line strip
   - Displays waypoints and orientations
   - Highlights start (green) and end (red) points

### Scripts

4. **`trajectory_postprocess.py`** - Post-processes trajectories to remove idle time
   - Detects and removes waypoints where robot is not moving
   - Configurable motion threshold
   - Preserves all meaningful motion data
   - See [POST_PROCESSING_GUIDE.md](POST_PROCESSING_GUIDE.md) for details

## Installation

1. Navigate to your workspace:
```bash
cd ~/ur5-robotiq-ros2-control
```

2. Build the package:
```bash
colcon build --packages-select ur5_kinesthetic_teaching --symlink-install
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Prerequisites

**⚠️ IMPORTANT**: The kinesthetic teaching system requires the UR5 robot controller to be running FIRST!

Before launching any kinesthetic teaching nodes, start the UR5 controller:

```bash
# Terminal 1: Launch UR5 robot controller
cd /home/rml/ur5-robotiq-ros2-control
./launch_ur5_nosnap.sh 192.168.1.102 true  # Use 'false' for real robot

# Wait until joint_states and TF are publishing
# Verify with: ros2 topic echo /joint_states
```

### Quick Start - Complete System

After the UR5 controller is running, launch the complete kinesthetic teaching system in a new terminal:

```bash
# Terminal 2: Launch kinesthetic teaching
ros2 launch ur5_kinesthetic_teaching kinesthetic_teaching.launch.py
```

### Step-by-Step Usage

#### 1. Start the UR5 Robot

Launch your UR5 robot (simulation or real):

```bash
# For simulation with fake hardware
./launch_ur5_nosnap.sh 192.168.1.102 true

# OR for real robot
./launch_ur5_nosnap.sh 192.168.1.102 false
```

**Wait for the controller to fully initialize** before proceeding to step 2.

#### 2. Launch Trajectory Recorder

```bash
ros2 launch ur5_kinesthetic_teaching recorder.launch.py
```

#### 3. Record a Trajectory

Put the robot in **freedrive mode** (if using real hardware) or manually move it in simulation, then:

```bash
# Start recording
ros2 service call /trajectory_recorder/start_recording std_srvs/srv/Trigger

# Manually guide the robot through desired motion

# Stop recording (auto-saves)
ros2 service call /trajectory_recorder/stop_recording std_srvs/srv/Trigger
```

#### 4. Replay the Trajectory

```bash
# Launch player
ros2 launch ur5_kinesthetic_teaching player.launch.py

# Load most recent trajectory
ros2 service call /trajectory_player/load_trajectory std_srvs/srv/Trigger

# Play the trajectory
ros2 service call /trajectory_player/play_trajectory std_srvs/srv/Trigger
```

#### 5. Visualize Trajectory

```bash
# Launch visualizer
ros2 run ur5_kinesthetic_teaching trajectory_visualizer.py

# Load and visualize latest trajectory
ros2 service call /trajectory_visualizer/load_latest std_srvs/srv/Trigger
```

## Services API

### Trajectory Recorder

| Service | Type | Description |
|---------|------|-------------|
| `~/start_recording` | `std_srvs/Trigger` | Start recording a new trajectory |
| `~/stop_recording` | `std_srvs/Trigger` | Stop recording and auto-save |
| `~/save_trajectory` | `std_srvs/Trigger` | Manually save current trajectory |
| `~/clear_trajectory` | `std_srvs/Trigger` | Clear recorded data without saving |

### Trajectory Player

| Service | Type | Description |
|---------|------|-------------|
| `~/load_trajectory` | `std_srvs/Trigger` | Load most recent trajectory |
| `~/play_trajectory` | `std_srvs/Trigger` | Play loaded trajectory |
| `~/stop_playback` | `std_srvs/Trigger` | Stop current playback |
| `~/list_trajectories` | `std_srvs/Trigger` | List available trajectory files |

### Trajectory Visualizer

| Service | Type | Description |
|---------|------|-------------|
| `~/load_latest` | `std_srvs/Trigger` | Load and visualize latest trajectory |
| `~/clear_visualization` | `std_srvs/Trigger` | Clear all visualization markers |

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/trajectory_recorder/recording_status` | `std_msgs/String` | Recording status (RECORDING/STOPPED) |
| `/trajectory_recorder/trajectory_markers` | `visualization_msgs/MarkerArray` | Live trajectory markers |
| `/trajectory_player/playback_status` | `std_msgs/String` | Playback status |
| `/trajectory_visualizer/trajectory_visualization` | `visualization_msgs/MarkerArray` | Trajectory visualization |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Robot joint states |
| `/trajectory_player/set_playback_speed` | `std_msgs/Float32` | Adjust playback speed |

## Parameters

### Recorder Parameters

- `recording_rate` (double, default: 30.0) - Recording frequency in Hz
- `end_effector_frame` (string, default: "tool0") - End-effector TF frame
- `base_frame` (string, default: "base_link") - Base TF frame
- `trajectory_dir` (string, default: "~/ur5_trajectories") - Save directory
- `min_position_change` (double, default: 0.001) - Minimum change to record
- `auto_save` (bool, default: true) - Auto-save on stop

### Player Parameters

- `trajectory_dir` (string, default: "~/ur5_trajectories") - Load directory
- `playback_speed` (double, default: 1.0) - Speed multiplier (0.1-5.0)
- `controller_name` (string, default: "scaled_joint_trajectory_controller") - Controller to use
- `loop_playback` (bool, default: false) - Loop trajectory continuously

### Visualizer Parameters

- `trajectory_dir` (string, default: "~/ur5_trajectories") - Load directory
- `base_frame` (string, default: "base_link") - Base frame for markers
- `visualization_rate` (double, default: 1.0) - Update frequency in Hz

## Trajectory File Format

Trajectories are saved as JSON files in `~/ur5_trajectories/` with the following structure:

```json
{
  "name": "trajectory_20251119_143022",
  "timestamp": "2025-11-19T14:30:22.123456",
  "recording_rate": 30.0,
  "frames": {
    "base_frame": "base_link",
    "end_effector_frame": "tool0"
  },
  "joint_space": {
    "joint_names": ["shoulder_pan_joint", "shoulder_lift_joint", ...],
    "positions": [[0.0, -1.57, ...], [0.1, -1.56, ...], ...],
    "velocities": [[0.0, 0.0, ...], [0.05, 0.01, ...], ...],
    "timestamps": [1234567890.123, 1234567890.156, ...]
  },
  "task_space": {
    "poses": [
      {
        "position": {"x": 0.5, "y": 0.2, "z": 0.3},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
      },
      ...
    ],
    "timestamps": [1234567890.123, 1234567890.156, ...]
  }
}
```

## RViz Visualization

To view trajectory markers in RViz:

1. Add a **MarkerArray** display
2. Set topic to `/trajectory_recorder/trajectory_markers` (for live recording)
3. Or `/trajectory_visualizer/trajectory_visualization` (for saved trajectories)

The visualization shows:
- **Blue-to-red gradient path**: Full trajectory (blue=start, red=end)
- **Green sphere**: Start point
- **Red sphere**: End point
- **Small colored spheres**: Waypoints along path
- **Yellow arrows**: End-effector orientation at waypoints

## Examples

### Example 1: Simple Pick-and-Place

```bash
# 1. Start robot and recorder
ros2 launch ur5_kinesthetic_teaching recorder.launch.py

# 2. Start recording
ros2 service call /trajectory_recorder/start_recording std_srvs/srv/Trigger

# 3. Manually guide robot: approach → grasp → lift → move → place

# 4. Stop recording
ros2 service call /trajectory_recorder/stop_recording std_srvs/srv/Trigger

# 5. Replay
ros2 launch ur5_kinesthetic_teaching player.launch.py
ros2 service call /trajectory_player/load_trajectory std_srvs/srv/Trigger
ros2 service call /trajectory_player/play_trajectory std_srvs/srv/Trigger
```

### Example 2: Adjust Playback Speed

```bash
# Play at half speed
ros2 topic pub /trajectory_player/set_playback_speed std_msgs/Float32 "data: 0.5" --once

# Play at double speed
ros2 topic pub /trajectory_player/set_playback_speed std_msgs/Float32 "data: 2.0" --once
```

### Example 3: Continuous Loop

```bash
ros2 launch ur5_kinesthetic_teaching player.launch.py loop_playback:=true
ros2 service call /trajectory_player/play_trajectory std_srvs/srv/Trigger
```

## Tips

1. **Smooth motions**: Move the robot slowly and smoothly during recording for better trajectory quality
2. **Adjust recording rate**: Lower `recording_rate` for slower motions, higher for fast motions
3. **Filter noise**: Increase `min_position_change` to avoid recording small jitters
4. **Check before playing**: Use the visualizer to inspect trajectories before playback
5. **Backup trajectories**: Files are saved in `~/ur5_trajectories/`, back them up regularly
6. **Remove idle time**: Use `trajectory_postprocess.py` to clean up recordings (see [POST_PROCESSING_GUIDE.md](POST_PROCESSING_GUIDE.md))

## Post-Processing Trajectories

Remove idle time (startup delays, pauses, end delays) from recorded trajectories:

```bash
# Basic usage - removes idle time with default threshold
python3 scripts/trajectory_postprocess.py ~/ur5_trajectories/trajectory_20251119_132731.json

# See what's being removed with verbose mode
python3 scripts/trajectory_postprocess.py ~/ur5_trajectories/trajectory_20251119_132731.json --verbose

# For micro-motions, use more sensitive threshold
python3 scripts/trajectory_postprocess.py ~/ur5_trajectories/micro_motion.json --threshold 0.0005
```

**📖 Full documentation**: See [POST_PROCESSING_GUIDE.md](POST_PROCESSING_GUIDE.md) for complete guide  
**📋 Quick reference**: See [POSTPROCESSING_QUICKREF.md](POSTPROCESSING_QUICKREF.md) for command cheat sheet

## Troubleshooting

**Problem**: Recorder not capturing points
- **Solution**: Check that robot is moving more than `min_position_change` threshold
- **Solution**: Verify `/joint_states` topic is publishing: `ros2 topic echo /joint_states`

**Problem**: Player can't find trajectory
- **Solution**: Check `~/ur5_trajectories/` directory exists and contains `.json` files
- **Solution**: Verify trajectory_dir parameter matches between recorder and player

**Problem**: Playback rejected by controller
- **Solution**: Ensure controller is running: `ros2 control list_controllers`
- **Solution**: Check controller name matches parameter

**Problem**: No visualization in RViz
- **Solution**: Add MarkerArray display in RViz
- **Solution**: Verify correct topic is selected
- **Solution**: Check that `base_frame` exists in TF tree

## Future Enhancements

- [ ] MoveIt integration for task-space playback
- [ ] Trajectory editing and modification
- [ ] Multiple trajectory blending
- [ ] Waypoint insertion/deletion
- [ ] Force/torque recording (if sensor available)
- [ ] GUI for easier control
- [ ] Trajectory comparison tools

## Dependencies

- ROS 2 Humble
- `sensor_msgs`
- `geometry_msgs`
- `trajectory_msgs`
- `control_msgs`
- `visualization_msgs`
- `tf2_ros`
- `moveit_msgs`

## License

MIT

## Author

RML Lab

## See Also

- [Commands](../../Commands) - Full command reference for UR5 workspace
- [QUICKSTART.md](../../QUICKSTART.md) - UR5 setup guide
