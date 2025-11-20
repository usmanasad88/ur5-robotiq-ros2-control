# Trajectory Post-Processing Guide

## Overview

The `trajectory_postprocess.py` script removes idle time from recorded trajectories by detecting when the robot position doesn't change significantly. This is especially useful for:

- Recording micro-motions where you need to manually position the robot
- Removing startup delay before the robot begins moving
- Removing end delay after stopping the recording
- Cleaning up trajectories with unintentional pauses

## Usage

### Basic Usage

Process a trajectory with default settings (0.001 rad threshold):

```bash
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    ~/ur5_trajectories/trajectory_20251119_132731.json
```

**Output:**
```
Saved: /home/rml/ur5_trajectories/trajectory_20251119_132731_processed.json
Waypoints: 638 → 425 (213 removed)
```

### Verbose Output

See detailed information about motion detection:

```bash
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    ~/ur5_trajectories/trajectory_20251119_132731.json --verbose
```

**Output:**
```
Loading trajectory from: /home/rml/ur5_trajectories/trajectory_20251119_132731.json

Original waypoints: 638
Motion threshold: 0.001 rad
Found 3 motion segment(s):
  Segment 1: waypoints 45 to 178 (134 points)
  Segment 2: waypoints 195 to 423 (229 points)
  Segment 3: waypoints 530 to 592 (63 points)

After merging nearby segments: 2 segment(s):
  Segment 1: waypoints 45 to 423 (379 points)
  Segment 2: waypoints 530 to 592 (63 points)

Note: Task space has fewer poses (100) than joint positions (638)
      Keeping 100 task space poses

Processed waypoints: 442
Removed waypoints: 196 (30.7%)

Processed trajectory saved to: /home/rml/ur5_trajectories/trajectory_20251119_132731_processed.json
```

### Custom Threshold

Adjust the motion detection sensitivity:

```bash
# More sensitive - detects smaller movements (keeps more waypoints)
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    ~/ur5_trajectories/my_trajectory.json --threshold 0.0005

# Less sensitive - only detects larger movements (removes more waypoints)
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    ~/ur5_trajectories/my_trajectory.json --threshold 0.005
```

**Threshold Guidelines:**
- **0.0005 rad** (0.029°) - Very sensitive, for detecting micro-motions
- **0.001 rad** (0.057°) - Default, good for normal recordings
- **0.005 rad** (0.29°) - Less sensitive, removes small jitters
- **0.01 rad** (0.57°) - Only keeps significant motions

### Custom Output File

Specify a custom output file path:

```bash
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    ~/ur5_trajectories/raw_trajectory.json \
    --output ~/ur5_trajectories/clean_trajectory.json
```

## How It Works

### 1. Motion Detection

The script calculates the Euclidean distance between consecutive joint configurations:

```
distance = sqrt(Σ(joint[i+1] - joint[i])²)
```

If the distance exceeds the threshold, those waypoints are marked as "in motion."

### 2. Segment Identification

Consecutive waypoints with motion are grouped into segments:

```
Original: [idle][idle][move][move][move][idle][move][move][idle]
Segments:              [  Segment 1   ]      [ Segment 2 ]
```

### 3. Segment Merging

Small gaps between segments (≤3 waypoints) are merged to preserve micro-motions:

```
Before merge: [Seg1][2-waypoint gap][Seg2]
After merge:  [    Merged Segment        ]
```

### 4. Waypoint Extraction

Only waypoints within motion segments are kept in the final trajectory.

## Examples

### Example 1: Recording with Startup Delay

**Scenario:** You start recording, wait 2 seconds, then move the robot.

```bash
# Original: 638 waypoints (60 at start are idle, 20 at end are idle)
# Recording rate: 30 Hz → ~2 sec idle at start, ~0.7 sec idle at end

python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    ~/ur5_trajectories/trajectory_with_delay.json --verbose
```

**Result:**
```
Original waypoints: 638
Found 1 motion segment(s):
  Segment 1: waypoints 60 to 618 (559 points)

Processed waypoints: 559
Removed waypoints: 79 (12.4%)
```

### Example 2: Multiple Motion Segments

**Scenario:** Move robot → pause → move again → pause → move again

```bash
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    ~/ur5_trajectories/multi_segment.json --verbose
```

**Result:**
```
Original waypoints: 450
Found 3 motion segment(s):
  Segment 1: waypoints 10 to 120 (111 points)
  Segment 2: waypoints 180 to 290 (111 points)
  Segment 3: waypoints 350 to 440 (91 points)

Processed waypoints: 313
Removed waypoints: 137 (30.4%)
```

### Example 3: Micro-Motion Recording

**Scenario:** Very slow, precise adjustments

```bash
# Use more sensitive threshold to preserve small movements
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    ~/ur5_trajectories/micro_motion.json --threshold 0.0005 --verbose
```

## Integration with Kinesthetic Teaching Workflow

### Complete Workflow

```bash
# Terminal 1: Launch UR5
cd ~/ur5-robotiq-ros2-control
./launch_ur5_nosnap.sh 192.168.1.102 false

# Terminal 2: Launch kinesthetic teaching
source install/setup.bash
ros2 launch ur5_kinesthetic_teaching kinesthetic_teaching.launch.py

# Terminal 3: Record trajectory
ros2 service call /trajectory_recorder/start_recording std_srvs/srv/Trigger
# Move robot...
ros2 service call /trajectory_recorder/stop_recording std_srvs/srv/Trigger

# Terminal 3: Post-process to remove idle time
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    ~/ur5_trajectories/trajectory_20251119_132731.json --verbose

# Terminal 3: Play processed trajectory
ros2 service call /trajectory_player/load_trajectory std_srvs/srv/Trigger "{}"
# Edit: manually specify the processed file in trajectory_player config, or
# rename processed file to be the newest one
```

### Batch Processing

Process multiple trajectories:

```bash
for file in ~/ur5_trajectories/trajectory_*.json; do
    echo "Processing: $file"
    python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
        "$file" --threshold 0.001
done
```

## Output File Format

The processed trajectory maintains the same JSON structure as the original:

```json
{
  "name": "trajectory_20251119_132731",
  "timestamp": "2025-11-19T13:29:38.934748",
  "recording_rate": 30.0,
  "frames": { ... },
  "joint_space": {
    "joint_names": [ ... ],
    "positions": [ ... ]  // Only motion waypoints
  },
  "task_space": {
    "poses": [ ... ]  // Only motion waypoints
  },
  "metadata": {
    "original_waypoints": 638,
    "processed_waypoints": 425,
    "removed_waypoints": 213,
    "motion_threshold": 0.001,
    "processing": "idle_time_removed"
  }
}
```

## Troubleshooting

### No Waypoints Removed

```
Processed waypoints: 638
Removed waypoints: 0 (0.0%)
```

**Cause:** Robot was continuously moving throughout recording.

**Solution:** This is normal if there's no idle time. If you expected idle time to be removed, try a higher threshold.

### Too Many Waypoints Removed

```
Processed waypoints: 2
Removed waypoints: 636 (99.7%)
```

**Cause:** Threshold too high, script only kept start and end points.

**Solution:** Use a lower threshold (e.g., `--threshold 0.0005`).

### Task Space Mismatch Warning

```
Note: Task space has fewer poses (100) than joint positions (638)
      Keeping 100 task space poses
```

**Cause:** Task space is recorded at a slower rate or recording stopped early.

**Solution:** This is handled automatically. The script keeps as many task poses as available.

## Command Reference

```bash
# Show help
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py --help

# Basic usage
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py <input_file>

# Full options
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    <input_file> \
    --threshold <float> \
    --output <output_file> \
    --min-waypoints <int> \
    --verbose
```

## Tips

1. **Always use --verbose first**: See what's being removed before committing
2. **Backup originals**: Script creates new file by default, originals are safe
3. **Start with default threshold**: 0.001 rad works well for most cases
4. **Micro-motions**: Use lower threshold (0.0005) for very precise work
5. **Batch processing**: Process multiple files with a for loop
