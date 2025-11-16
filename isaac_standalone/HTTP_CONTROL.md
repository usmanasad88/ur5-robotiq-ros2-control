# UR5 HTTP End-Effector Control

## Overview
This script creates a standalone Isaac Sim application that:
1. Loads a UR5 robot
2. Continuously reads target end-effector poses from an HTTP endpoint
3. Updates the robot's end-effector position/orientation accordingly

## Flask Server Format

The script expects your Flask server at `http://localhost:5000/joint_actions` to return JSON in this format:

```json
{
    "joint_actions": [x, y, z, roll, pitch, yaw, gripper],
    "joints": {
        "x": -0.00030576539575122297,
        "y": -0.00032838108018040657,
        "z": -0.0014036695938557386,
        "roll": -0.0065353563986718655,
        "pitch": -0.0010407345835119486,
        "yaw": -0.008116843178868294,
        "gripper": 0.989782452583313
    },
    "timestamp": 1759376486.8638608
}
```

The values represent **delta/incremental** changes to the end-effector pose.

## Usage

### Basic Usage
```bash
# Run with default settings (http://localhost:5000/joint_actions)
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur5_http_control.py

# Specify custom server URL
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur5_http_control.py \
    --server http://localhost:8080

# Specify custom endpoint
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur5_http_control.py \
    --endpoint /api/robot/pose

# Run headless (no GUI)
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur5_http_control.py \
    --headless

# Custom update rate
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur5_http_control.py \
    --update-rate 20.0
```

### All Arguments
```
--server URL          Flask server URL (default: http://localhost:5000)
--endpoint PATH       Endpoint path (default: /joint_actions)
--headless            Run without GUI
--update-rate HZ      Control update rate in Hz (default: 10.0)
--timeout SECONDS     HTTP request timeout (default: 1.0)
```

## How It Works

### 1. Initialization
- Loads UR5 robot from Isaac Sim assets
- Creates end-effector controller
- Sets robot to home position
- Connects to Flask server

### 2. Control Loop
- Runs at specified update rate (default: 10 Hz)
- Fetches target pose from HTTP endpoint
- Parses JSON response
- Computes new end-effector pose (current + delta)
- Updates robot using inverse kinematics

### 3. Pose Interpretation
The script treats received values as **deltas** (incremental changes):
```python
new_position = current_position + target_position
new_orientation = current_orientation + target_orientation
```

If your server sends **absolute** poses instead, modify line ~336 in the script.

## Coordinate System

### Position (x, y, z)
- Units: meters
- Origin: World origin (0, 0, 0)
- Axes: X (forward), Y (left), Z (up)

### Orientation (roll, pitch, yaw)
- Units: radians
- Convention: Intrinsic XYZ Euler angles
- Order: Roll → Pitch → Yaw

## Examples

### Example 1: Start Simulation and Connect to Server
```bash
# Terminal 1: Start your Flask server
python flask_server.py

# Terminal 2: Start Isaac Sim simulation
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur5_http_control.py
```

### Example 2: High-Frequency Control
```bash
# Update at 30 Hz
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur5_http_control.py \
    --update-rate 30.0
```

### Example 3: Custom Server
```bash
# Connect to different server
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur5_http_control.py \
    --server http://192.168.1.100:8000 \
    --endpoint /robot/target_pose
```

## Output

The script prints:
- Connection status
- Current end-effector pose
- Target pose from server
- Frame count and timing
- Errors and warnings

Example output:
```
╔══════════════════════════════════════════════════════════════════╗
║ HTTP End-Effector Controller initialized                        ║
╔══════════════════════════════════════════════════════════════════╗
Server URL: http://localhost:5000/joint_actions
Update Rate: 10.0 Hz
Timeout: 1.0s
╔══════════════════════════════════════════════════════════════════╗

✓ Connected to server: http://localhost:5000/joint_actions

Frame 60:
  Target position: [-0.0003 -0.0003 -0.0014]
  New position: [0.5234 0.1234 0.5123]
  Target euler: [-0.0065 -0.0010 -0.0081]

[2.0s] Connected | Target: pos=[-0.0003, -0.0003, -0.0014] quat=[1.000, 0.003, 0.005, 0.004]
```

## Troubleshooting

### Server Not Running
```
Cannot connect to http://localhost:5000/joint_actions (is server running?)
```
**Solution:** Start your Flask server first

### Connection Timeout
```
Request timeout (waiting for server...)
```
**Solution:** Check server is responding, increase `--timeout`

### Wrong Data Format
```
Error parsing response: ...
```
**Solution:** Verify your server returns the expected JSON format

### Robot Not Moving
- Check that server is sending non-zero values
- Verify values are reasonable (not too large)
- Check console for IK solver errors

## Integration with Your Flask Server

Make sure your Flask server (`flask_server.py`) has an endpoint like:

```python
from flask import Flask, jsonify
import time

app = Flask(__name__)

@app.route('/joint_actions', methods=['GET'])
def joint_actions():
    return jsonify({
        "joint_actions": [
            x, y, z,           # Position deltas (meters)
            roll, pitch, yaw,  # Orientation deltas (radians)
            gripper            # Gripper state (0-1)
        ],
        "joints": {
            "x": x,
            "y": y,
            "z": z,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "gripper": gripper
        },
        "timestamp": time.time()
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
```

## Advanced Usage

### Using Absolute Poses Instead of Deltas

If your server sends absolute poses, modify the script around line 336:

```python
# Instead of:
new_position = current_position + target_position

# Use:
new_position = target_position
```

### Adding Gripper Control

The gripper value is currently received but not used. To add gripper control:

1. Check if robot has gripper
2. Extract gripper value from response
3. Set gripper position

```python
if len(actions) >= 7:
    gripper_value = actions[6]
    # Set gripper (implementation depends on gripper type)
```

### Recording Poses

To record all received poses to a file, add to the script:

```python
import csv

# In main():
with open('poses.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(['time', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
    
    # In loop:
    if target_pose:
        pos, quat = target_pose
        euler = quat_to_euler_angles(quat)
        writer.writerow([current_time, *pos, *euler])
```

## Performance Tips

1. **Update Rate**: Start with 10 Hz, increase if needed
2. **Timeout**: Keep at 1.0s for good responsiveness
3. **Headless Mode**: Use `--headless` for better performance
4. **Server Latency**: Minimize network latency for smooth control

## Dependencies

- Isaac Sim 5.0.0+
- Python `requests` library (included with Isaac Sim)
- Flask server running separately

## Known Limitations

1. Currently prints target poses but requires full IK integration for motion
2. Gripper control not implemented
3. No collision avoidance
4. No trajectory smoothing

## Next Steps

To make the robot actually move:
1. Integrate full IK solver (Lula or other)
2. Add motion planning
3. Implement collision avoidance
4. Add trajectory smoothing
5. Implement gripper control

## Related Files

- `ur5_http_control.py` - Main script (this)
- `ur10_import.py` - Basic robot loader
- `ur_robot_advanced.py` - Advanced control modes
- `flask_server.py` - Example Flask server (in scripts_for_coding_agent_reference/)

## Support

For issues or questions:
1. Check Isaac Sim logs: `~/.nvidia-omniverse/logs/`
2. Verify Flask server is responding: `curl http://localhost:5000/joint_actions`
3. Check firewall settings
4. Review Isaac Sim documentation: https://docs.isaacsim.omniverse.nvidia.com/
