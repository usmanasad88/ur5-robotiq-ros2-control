# GR00T Integration for UR10 Robotiq Cortex

This extension now supports **GR00T** as a third VLA option alongside Octo and Gemini models.

## Prerequisites

GR00T support requires additional Python packages. Install them into Isaac Sim's Python environment:

```bash
cd /home/mani/isaac-sim-standalone-5.0.0-linux-x86_64
./python.sh -m pip install msgpack-python pyzmq
```

**Important**: Use Isaac Sim's `python.sh` script, not your system Python, to ensure packages are installed in the correct environment.

## Quick Start

### 1. Install Required Packages (see Prerequisites above)

### 2. Start the GR00T Inference Server

Start the GR00T inference server with the OXE_DROID embodiment:

```bash
python scripts/inference_service.py --server --embodiment-tag oxe_droid --data-config oxe_droid
```

The server will listen on `tcp://localhost:5555` by default.

### 3. Configure the VLA Behavior

Edit `ur10_vla_behavior.py` and set:

```python
VLA_MODEL = "gr00t"  # Change from "octo" or "gemini-flash" to "gr00t"
VLA_SERVER_URL = "localhost"  # Host where GR00T server is running
```

### 4. Run the Extension

Load the UR10 Robotiq Cortex extension in Isaac Sim:
1. Select "VLA Control" behavior from the dropdown
2. Click "Load" to load the world and robot
3. Click "Start" to begin GR00T-controlled behavior

## How It Works

### Observation Format

GR00T receives observations in the OXE_DROID format:

- **Video observations** (3 cameras):
  - `video.exterior_image_1`: Third-person view from VLA camera
  - `video.exterior_image_2`: Same as exterior_image_1 (reused)
  - `video.wrist_image`: Same as exterior_image_1 (reused)
  - Shape: `(1, H, W, 3)` uint8, where H,W = 256x256

- **State observations**:
  - `state.eef_position`: End-effector position `(1, 3)` - ABSOLUTE in robot base frame (meters)
  - `state.eef_rotation`: End-effector orientation `(1, 3)` - Euler angles (roll, pitch, yaw) in RADIANS, ABSOLUTE in robot base frame
  - `state.gripper_position`: Gripper state `(1, 1)` - 0.0=open, 1.0=closed, ABSOLUTE

- **Language instruction**:
  - `annotation.language.language_instruction`: Task description string (e.g., "pick up the red cube")

### Action Format

GR00T returns actions over a 16-timestep horizon:

- `action.eef_position_delta`: Position deltas `(16, 3)` - RELATIVE to current position (meters)
- `action.eef_rotation_delta`: Rotation deltas `(16, 3)` - Axis-angle representation, RELATIVE
- `action.gripper_position`: Gripper commands `(16, 1)` - ABSOLUTE (0=open, 1=closed)

**Receding Horizon Control**: The extension uses only the first timestep (index 0) and re-queries the policy at each step for reactive control.

### Reference Frames

All **absolute** coordinates (positions and orientations in observations) are in the **robot base frame**:
- Origin: Center of robot base mounting point
- Right-handed coordinate system
- Use UR10's `get_ee_pose()` to get current state in this frame

All **deltas** (position and rotation in actions) are also in the robot base frame and can be directly added to current pose.

## Model Configuration

GR00T is configured with:
- **Timeout**: 5.0s (local inference)
- **Min call interval**: 0.1s (~10 Hz inference rate)
- **Communication**: ZMQ on port 5555 (not HTTP)

This is much faster than cloud-based models (Gemini) which are rate-limited.

## Key Differences from Octo/Gemini

| Feature | Octo/Gemini | GR00T |
|---------|-------------|-------|
| Communication | HTTP REST API | ZMQ with msgpack |
| Observations | Not required | Required (video + state + language) |
| Action format | Joint deltas only | Position/rotation deltas + gripper |
| Rotation format | Quaternion | Axis-angle (converted to quat) |
| Action horizon | Single timestep | 16 timesteps (use first) |
| Rate | Varies | ~10 Hz |

## Troubleshooting

**"No module named 'msgpack'" or "No module named 'zmq'"**
- You need to install the packages into Isaac Sim's Python environment
- Run: `cd /home/mani/isaac-sim-standalone-5.0.0-linux-x86_64 && ./python.sh -m pip install msgpack-python pyzmq`
- **Do NOT use system pip** - it must be installed via Isaac Sim's `python.sh`

**"Cannot connect to GR00T server"**
- Ensure the inference server is running: `python scripts/inference_service.py --server --embodiment-tag oxe_droid --data-config oxe_droid`
- Check that `VLA_SERVER_URL` points to the correct host
- Verify port 5555 is not blocked by firewall

**"Failed to capture viewport frame"**
- The VLA camera should be automatically created at `/World/vla_camera`
- Check that the camera prim exists in the stage
- Fallback: A dummy black frame will be used if capture fails

**Robot doesn't move**
- Check the console for `[VLA] Action #N` messages
- Verify GR00T is returning non-zero deltas
- Ensure robot is initialized properly (check `desired_position` is not None)

## Camera Setup

The extension creates a third-person camera at:
- **Position**: (2.72, 4.77, 2.52)
- **Orientation**: Euler angles (-65°, 24°, 169°)
- **Resolution**: 256x256
- **Location**: `/World/vla_camera`

You can adjust the camera pose by:
1. Selecting the camera prim in the stage
2. Modifying its transform properties
3. Updating the hardcoded values in `capture_viewport_frame()` function

## Code Reference

See `robot_inference_test.py` for a standalone example of GR00T inference without the Cortex framework.

Key files modified:
- `ur10_vla_behavior.py`: Added GR00TClient class and gr00t model support
- `ur10_robotiq_cortex_extension.py`: No changes needed (already supports VLA Control)

## Advanced: Multi-Camera Setup

To use actual separate camera views (wrist camera, multiple exterior cameras):

1. Create additional camera prims in the scene
2. Modify `fetch_and_apply_action()` to capture from different cameras:
   ```python
   exterior_frame_1 = capture_from_camera("/World/exterior_camera_1")
   exterior_frame_2 = capture_from_camera("/World/exterior_camera_2")  
   wrist_frame = capture_from_camera("/World/wrist_camera")
   ```
3. Update the observations dictionary with the separate frames

Currently all three video observations use the same third-person view as a simplified setup.
