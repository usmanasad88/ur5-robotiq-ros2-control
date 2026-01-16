# GoPro Integration - Quick Reference

## Overview

The episode recording system now captures synchronized data from multiple sources:
- **Webcam video** (OpenCV, 30 Hz)
- **GoPro video** (FFmpeg UDP stream)
- **Robot joint states** (ROS 2, 50 Hz, 6 DOF)
- **Gripper state** (ROS 2, 50 Hz, 1 DOF)
- **Program events** (execution timestamps)

## Prerequisites

### 1. Start GoPro Streaming

Before recording episodes, make sure the GoPro is in webcam mode:

```bash
cd /home/mani/Repos/ur_ws/gopro_streaming
conda activate ur5_python
python connect_gopro.py
```

**Expected output:**
```
✅ GoPro webcam mode started!
   Camera IP: 172.20.110.51
   UDP Stream: udp://@:8554
```

### 2. Test GoPro Connection

Verify the GoPro stream is working:

```bash
cd /home/mani/Repos/ur_ws
python test_gopro_integration.py
```

This will record a 5-second test video and verify everything works.

## Using the UI

### 1. Launch the Program Selector UI

```bash
cd /home/mani/Repos/ur_ws
./run_program_ui.sh
```

### 2. Check GoPro Status

In the UI sidebar under "Episode Recording", you'll see:
- **📹 GoPro: Streaming** - GoPro is ready to record
- **📹 GoPro: Not streaming** - Run `connect_gopro.py` first
- **📹 GoPro: Not available** - `gopro_recorder.py` module not found

### 3. Start Recording

1. Enter experiment name (e.g., "pick_and_place_demo")
2. Click **🔴 Record** button
3. Execute robot programs as normal
4. Click **⏹️ End** when finished

### 4. Recording Output

Each recording creates a directory: `recordings/{experiment_name}_{timestamp}/`

Files created:
- `video.mp4` - Webcam video (640x480, 30 fps)
- `gopro_video.mp4` - GoPro video (if streaming detected)
- `joint_states.h5` - Robot joint data (HDF5 format)
- `metadata.json` - Recording metadata
- `program_events.json` - Program execution timeline

## Troubleshooting

### GoPro not streaming

**Problem:** UI shows "GoPro: Not streaming"

**Solution:**
1. Make sure GoPro is connected via USB
2. Run `python gopro_streaming/connect_gopro.py`
3. Verify stream with `./gopro_streaming/view_stream.sh`

### GoPro recording file missing

**Problem:** Recording completes but no `gopro_video.mp4` file

**Possible causes:**
- GoPro stream dropped during recording
- FFmpeg process crashed
- Insufficient disk space

**Check:**
```bash
# View terminal output for FFmpeg errors
# The gopro_recorder prints status messages
```

### GoPro stream quality issues

**Problem:** GoPro video is choppy or has artifacts

**Solution:**
1. Check USB connection (use USB 3.0 port)
2. Verify network: `ping 172.20.110.51`
3. Restart GoPro and reconnect

## Architecture

### GoProRecorder Class

Located in: `/home/mani/Repos/ur_ws/gopro_recorder.py`

**Methods:**
- `start_recording(output_path)` - Start FFmpeg capture
- `stop_recording()` - Stop capture and finalize video
- `is_gopro_streaming()` - Check if UDP stream is active

**How it works:**
1. Connects to GoPro UDP stream at `udp://0.0.0.0:8554`
2. Spawns FFmpeg subprocess to capture stream
3. Copies video codec (no re-encoding for efficiency)
4. Encodes audio as AAC
5. Outputs to MP4 file

### Integration with EpisodeRecorder

The `EpisodeRecorder` class in `program_selector_ui.py`:

1. **Initialization:** Creates `GoProRecorder` instance if available
2. **Start recording:** Calls `gopro_recorder.start_recording()` alongside webcam
3. **Stop recording:** Calls `gopro_recorder.stop_recording()` and updates metadata
4. **Metadata:** Adds `gopro_video_file` entry if GoPro was used

## Data Synchronization

All data sources are synchronized to a common timeline:

```
t=0.0s: Recording start
   ├─ Webcam capture starts (30 Hz)
   ├─ GoPro capture starts (stream rate)
   ├─ Joint state recording starts (50 Hz)
   └─ Event logging starts

t=2.5s: Execute program "pick_object.prog"
   └─ Event logged with timestamp

t=5.8s: Pause program
   └─ Event logged with timestamp

t=8.3s: Resume program
   └─ Event logged with timestamp

t=12.0s: Program completes
   └─ Event logged with timestamp

t=15.0s: Recording stop
   ├─ All captures stopped
   ├─ HDF5 file saved
   └─ Metadata.json created
```

## File Format Details

### metadata.json
```json
{
  "experiment_name": "pick_and_place_demo",
  "timestamp": "2024-01-15T14:30:00.123456",
  "duration_seconds": 15.2,
  "num_joint_samples": 760,
  "sample_rate_hz": 50.0,
  "video_file": "video.mp4",
  "gopro_video_file": "gopro_video.mp4",
  "joint_data_file": "joint_states.h5",
  "program_events_file": "program_events.json"
}
```

### program_events.json
```json
{
  "experiment_name": "pick_and_place_demo",
  "start_time": "2024-01-15T14:30:00.123456",
  "duration_seconds": 15.2,
  "events": [
    {
      "event": "recording_start",
      "timestamp": 0.0,
      "program": null
    },
    {
      "event": "execute",
      "timestamp": 2.5,
      "program": "pick_object.prog"
    },
    {
      "event": "pause",
      "timestamp": 5.8,
      "program": "pick_object.prog"
    },
    {
      "event": "resume",
      "timestamp": 8.3,
      "program": "pick_object.prog"
    },
    {
      "event": "stop",
      "timestamp": 12.0,
      "program": "pick_object.prog"
    }
  ]
}
```

### joint_states.h5
```
Datasets:
  - timestamps: (N,) float32 - Relative time in seconds
  - positions: (N, 6) float32 - Robot joint positions (radians)
  - velocities: (N, 6) float32 - Robot joint velocities (rad/s)
  - efforts: (N, 6) float32 - Robot joint efforts (N·m)
  - gripper_position: (N,) float32 - Gripper position
  - gripper_velocity: (N,) float32 - Gripper velocity
  - gripper_effort: (N,) float32 - Gripper effort
  - joint_names: (6,) string - Joint names
```

## Command Summary

```bash
# Setup (one-time)
cd /home/mani/Repos/ur_ws/gopro_streaming
./setup_environment.sh

# Start GoPro streaming (before each recording session)
conda activate ur5_python
cd /home/mani/Repos/ur_ws/gopro_streaming
python connect_gopro.py

# Test GoPro integration
cd /home/mani/Repos/ur_ws
python test_gopro_integration.py

# View GoPro stream
cd /home/mani/Repos/ur_ws/gopro_streaming
./view_stream.sh

# Start UI for recording
cd /home/mani/Repos/ur_ws
./run_program_ui.sh

# Inspect recorded HDF5 files
python inspect_h5.py recordings/experiment_20240115_143000/joint_states.h5
```

## Tips

1. **Always start GoPro streaming first** - The UI will detect if streaming is active
2. **Check GoPro status indicator** - Make sure it shows "Streaming" before recording
3. **Test GoPro separately** - Run `test_gopro_integration.py` to verify it works
4. **Monitor disk space** - GoPro videos can be large (several GB per minute)
5. **Use good USB connection** - USB 3.0 recommended for stable streaming
6. **Keep GoPro charged** - Long recording sessions drain battery

## Known Limitations

1. **GoPro must be pre-configured** - Webcam mode must be started manually via `connect_gopro.py`
2. **No auto-reconnect** - If stream drops, must restart webcam mode
3. **Fixed stream URL** - Hardcoded to `udp://0.0.0.0:8554`
4. **No stream quality control** - Uses GoPro's default streaming settings
5. **FFmpeg dependency** - Requires FFmpeg installed on system
