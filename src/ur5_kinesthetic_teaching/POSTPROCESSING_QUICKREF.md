# Trajectory Post-Processing - Quick Reference

## One-Line Commands

```bash
# Basic: Remove idle time with default settings
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py ~/ur5_trajectories/trajectory.json

# Verbose: See what's being removed
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py ~/ur5_trajectories/trajectory.json --verbose

# Micro-motions: More sensitive threshold
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py ~/ur5_trajectories/trajectory.json --threshold 0.0005

# Custom output: Specify output file
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py ~/ur5_trajectories/raw.json --output ~/ur5_trajectories/clean.json

# Batch: Process all trajectories
for f in ~/ur5_trajectories/*.json; do python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py "$f"; done
```

## Threshold Guide

| Value | Description | Use When |
|-------|-------------|----------|
| `0.0005` | Very sensitive | Micro-motions, precise adjustments |
| `0.001` | **Default** | Normal kinesthetic recordings |
| `0.005` | Less sensitive | Remove small jitters/noise |
| `0.01` | Coarse | Only major movements |

## Typical Workflow

```bash
# 1. Launch robot
./launch_ur5_nosnap.sh 192.168.1.102 false

# 2. Launch kinesthetic teaching
ros2 launch ur5_kinesthetic_teaching kinesthetic_teaching.launch.py

# 3. Record (in freedrive mode)
ros2 service call /trajectory_recorder/start_recording std_srvs/srv/Trigger
# ... move robot ...
ros2 service call /trajectory_recorder/stop_recording std_srvs/srv/Trigger

# 4. Post-process
python3 src/ur5_kinesthetic_teaching/scripts/trajectory_postprocess.py \
    ~/ur5_trajectories/trajectory_YYYYMMDD_HHMMSS.json --verbose

# 5. Play cleaned trajectory
# (Load the _processed.json file)
```

## Options

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--threshold` | float | 0.001 | Motion detection threshold (radians) |
| `--output` | path | `<input>_processed.json` | Output file path |
| `--min-waypoints` | int | 2 | Minimum waypoints to preserve |
| `--verbose` | flag | off | Show detailed processing info |
| `--help` | flag | - | Show full help message |

## Output Explained

```
Saved: /home/rml/ur5_trajectories/trajectory_20251119_132731_processed.json
Waypoints: 638 → 425 (213 removed)
```

- **638**: Original waypoint count
- **425**: Waypoints after removing idle time
- **213 removed**: 33.4% reduction

## Verbose Output Explained

```
Found 3 motion segment(s):
  Segment 1: waypoints 45 to 178 (134 points)   ← First movement
  Segment 2: waypoints 195 to 423 (229 points)  ← Second movement
  Segment 3: waypoints 530 to 592 (63 points)   ← Third movement

After merging nearby segments: 2 segment(s):
  Segment 1: waypoints 45 to 423 (379 points)   ← Merged 1 & 2
  Segment 2: waypoints 530 to 592 (63 points)   ← Kept separate
```

- Waypoints 0-44: **Removed** (idle at start)
- Waypoints 45-178: **Kept** (movement)
- Waypoints 179-194: **Kept** (small gap, merged)
- Waypoints 195-423: **Kept** (movement)
- Waypoints 424-529: **Removed** (idle pause)
- Waypoints 530-592: **Kept** (movement)
- Waypoints 593-637: **Removed** (idle at end)

## Common Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| No waypoints removed | Robot continuously moving | Normal, no action needed |
| Too many removed | Threshold too high | Lower threshold (e.g., `--threshold 0.0005`) |
| Only 2 waypoints kept | No motion detected | Much lower threshold or check recording |

## Files Generated

Input: `trajectory_20251119_132731.json`  
Output: `trajectory_20251119_132731_processed.json` (default)

Original file is **never modified** - always safe to re-run with different thresholds.

## Documentation

- **Full guide**: `POST_PROCESSING_GUIDE.md`
- **Summary**: `TRAJECTORY_POSTPROCESSING_SUMMARY.md`
- **This file**: `POSTPROCESSING_QUICKREF.md`
