# Kinesthetic Teaching - Quick Start Guide

This guide will help you get started with recording and replaying robot trajectories through kinesthetic teaching (manual guidance).

## Overview

The kinesthetic teaching module allows you to:
1. **Record** trajectories by manually moving the robot
2. **Save** trajectories in both joint space and task space
3. **Replay** trajectories at variable speeds
4. **Visualize** trajectories in RViz

## Prerequisites

1. Built and sourced workspace:
```bash
cd ~/ur5-robotiq-ros2-control
colcon build --packages-select ur5_kinesthetic_teaching --symlink-install
source install/setup.bash
```

2. UR5 robot running (simulation or real hardware)

## Complete Workflow Example

### Step 1: Launch UR5 Robot

**For Simulation:**
```bash
# Terminal 1: Launch UR5 with fake hardware
./launch_ur5_nosnap.sh 192.168.1.102 true
```

**For Real Robot:**
```bash
# Terminal 1: Launch UR5 with real hardware
./launch_ur5_nosnap.sh 192.168.1.102 false

# Enable freedrive mode on teach pendant to manually move robot
```

### Step 2: Start Kinesthetic Teaching System

```bash
# Terminal 2: Launch complete system
ros2 launch ur5_kinesthetic_teaching kinesthetic_teaching.launch.py
```

This starts:
- Trajectory recorder (records movements)
- Trajectory player (replays movements)
- Trajectory visualizer (shows paths in RViz)

### Step 3: Open RViz (Optional)

```bash
# Terminal 3: Launch RViz if not already running
rviz2
```

In RViz:
1. Click **Add** → **By topic**
2. Add `/trajectory_recorder/trajectory_markers` → **MarkerArray**
3. Add `/trajectory_visualizer/trajectory_visualization` → **MarkerArray**

### Step 4: Record a Trajectory

```bash
# Terminal 4: Start recording
ros2 service call /trajectory_recorder/start_recording std_srvs/srv/Trigger
```

**Expected output:**
```
waiting for service to become available...
response:
std_srvs.srv.Trigger_Response(success=True, message='Started recording: trajectory_20251119_143022')
```

**Now:**
- Manually move the robot through desired motion
- In RViz, you'll see a green/blue path appear as you move
- Move slowly and smoothly for best results

**When finished:**
```bash
# Stop recording (auto-saves)
ros2 service call /trajectory_recorder/stop_recording std_srvs/srv/Trigger
```

**Expected output:**
```
response:
std_srvs.srv.Trigger_Response(success=True, message='Stopped recording. Captured 342 joint space points and 338 task space points. Auto-saved to: /home/rml/ur5_trajectories/trajectory_20251119_143022.json')
```

### Step 5: Replay the Trajectory

```bash
# Load the most recent trajectory
ros2 service call /trajectory_player/load_trajectory std_srvs/srv/Trigger
```

**Expected output:**
```
response:
std_srvs.srv.Trigger_Response(success=True, message='Loaded trajectory: trajectory_20251119_143022 (342 joint points, 338 task points)')
```

**Play it:**
```bash
ros2 service call /trajectory_player/play_trajectory std_srvs/srv/Trigger
```

**Expected output:**
```
response:
std_srvs.srv.Trigger_Response(success=True, message='Started playback of 342 points at 1.0x speed')
```

Watch the robot replay your taught motion!

## Common Use Cases

### Use Case 1: Pick and Place

```bash
# 1. Start recording
ros2 service call /trajectory_recorder/start_recording std_srvs/srv/Trigger

# 2. Manually guide robot:
#    - Move to object
#    - Lower to grasp height
#    - Move to place location
#    - Raise to safe height
#    - Return to home

# 3. Stop recording
ros2 service call /trajectory_recorder/stop_recording std_srvs/srv/Trigger

# 4. Replay
ros2 service call /trajectory_player/load_trajectory std_srvs/srv/Trigger
ros2 service call /trajectory_player/play_trajectory std_srvs/srv/Trigger
```

### Use Case 2: Slow Motion Playback

```bash
# Play at half speed for safety
ros2 topic pub /trajectory_player/set_playback_speed std_msgs/Float32 "data: 0.5" --once

ros2 service call /trajectory_player/play_trajectory std_srvs/srv/Trigger
```

### Use Case 3: Continuous Looping

```bash
# Launch with loop enabled
ros2 launch ur5_kinesthetic_teaching player.launch.py loop_playback:=true

# Load and play
ros2 service call /trajectory_player/load_trajectory std_srvs/srv/Trigger
ros2 service call /trajectory_player/play_trajectory std_srvs/srv/Trigger

# Robot will continuously repeat the trajectory until stopped
```

### Use Case 4: Visualize Without Playing

```bash
# Just visualize without moving robot
ros2 run ur5_kinesthetic_teaching trajectory_visualizer.py

# Load latest
ros2 service call /trajectory_visualizer/load_latest std_srvs/srv/Trigger

# View in RViz - no robot motion
```

## Tips for Good Recordings

### ✅ Do's

- **Move slowly and smoothly** - Jerky motions create poor trajectories
- **Stay within workspace** - Don't approach joint limits
- **Consistent speed** - Maintain relatively constant speed during motion
- **Wait before stopping** - Hold final position for 1 second before stopping
- **Test in slow motion** - First replay at 0.5x speed to verify safety

### ❌ Don'ts

- **Don't rush** - Fast motions may be unsafe during playback
- **Don't stop mid-motion** - Complete the full desired path
- **Don't hit obstacles** - Trajectory won't avoid obstacles during replay
- **Don't record too long** - Keep trajectories under 30 seconds for best results

## Monitoring and Debugging

### Check Recording Status

```bash
# Monitor recording status
ros2 topic echo /trajectory_recorder/recording_status
```

### Check Playback Status

```bash
# Monitor playback status
ros2 topic echo /trajectory_player/playback_status
```

### List Saved Trajectories

```bash
# See all available trajectories
ros2 service call /trajectory_player/list_trajectories std_srvs/srv/Trigger
```

### View Trajectory Files

```bash
# Go to trajectory directory
cd ~/ur5_trajectories

# List files
ls -lh

# View a trajectory (pretty-printed JSON)
cat trajectory_20251119_143022.json | python3 -m json.tool | less
```

### Check Joint States

```bash
# Verify robot is publishing joint states
ros2 topic echo /joint_states --once
```

## Troubleshooting

### Problem: "No trajectory data to save"

**Cause:** Robot didn't move enough during recording

**Solution:**
- Ensure `min_position_change` threshold is met (default: 0.001 rad/m)
- Move robot more during recording
- Check that `/joint_states` is publishing

### Problem: "Joint trajectory controller not available"

**Cause:** Controller not running or wrong controller name

**Solution:**
```bash
# List available controllers
ros2 control list_controllers

# Use correct controller name in launch
ros2 launch ur5_kinesthetic_teaching player.launch.py \
  controller_name:=scaled_joint_trajectory_controller
```

### Problem: Playback is jerky or unstable

**Cause:** Recording rate too low or motion too fast

**Solution:**
```bash
# Increase recording rate
ros2 launch ur5_kinesthetic_teaching recorder.launch.py recording_rate:=50.0

# Play slower
ros2 topic pub /trajectory_player/set_playback_speed std_msgs/Float32 "data: 0.7" --once
```

### Problem: No visualization in RViz

**Cause:** MarkerArray display not added or wrong topic

**Solution:**
1. Add **MarkerArray** display in RViz
2. Set topic to `/trajectory_recorder/trajectory_markers`
3. Ensure `base_link` frame exists
4. Check Fixed Frame is set to `base_link` or `world`

## Advanced Configuration

### Custom Recording Parameters

```bash
ros2 launch ur5_kinesthetic_teaching recorder.launch.py \
  recording_rate:=50.0 \
  min_position_change:=0.0005 \
  end_effector_frame:=tool0 \
  base_frame:=base_link \
  trajectory_dir:=~/custom_trajectories \
  auto_save:=false
```

### Custom Playback Parameters

```bash
ros2 launch ur5_kinesthetic_teaching player.launch.py \
  playback_speed:=0.8 \
  controller_name:=joint_trajectory_controller \
  loop_playback:=true
```

## Trajectory File Location

All trajectories are saved in:
```
~/ur5_trajectories/
```

Files are named:
```
trajectory_YYYYMMDD_HHMMSS.json
```

Example:
```
~/ur5_trajectories/trajectory_20251119_143022.json
```

## Next Steps

1. **Practice** - Record several simple motions to get comfortable
2. **Experiment** - Try different speeds, recording rates, and parameters
3. **Integrate** - Use recorded trajectories in your applications
4. **Customize** - Modify the nodes for your specific needs

## See Also

- [README.md](../ur5_kinesthetic_teaching/README.md) - Full package documentation
- [Commands](../../Commands) - Complete command reference
- [QUICKSTART.md](../../QUICKSTART.md) - UR5 workspace setup

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review ROS 2 logs: `ros2 topic echo /rosout`
3. Check trajectory files in `~/ur5_trajectories/`
4. Verify robot and controllers are running properly
