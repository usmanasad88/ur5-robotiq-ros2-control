# Expected Output - Kinesthetic Teaching System

## Standalone Launch (No Robot Connected)

When launching without the UR5 controller running, you'll see:

```
[INFO] [launch]: All log files can be found below /home/rml/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [trajectory_recorder.py-1]: process started with pid [...]
[INFO] [trajectory_player.py-2]: process started with pid [...]
[INFO] [trajectory_visualizer.py-3]: process started with pid [...]

[trajectory_visualizer.py-3] [INFO] [...] Trajectory Visualizer initialized
[trajectory_recorder.py-1] [INFO] [...] Trajectory Recorder initialized
[trajectory_recorder.py-1] [INFO] [...] Trajectory directory: /home/rml/ur5_trajectories
[trajectory_recorder.py-1] [INFO] [...] Recording rate: 30.0 Hz
[trajectory_player.py-2] [INFO] [...] Trajectory Player initialized
[trajectory_player.py-2] [INFO] [...] Trajectory directory: /home/rml/ur5_trajectories
[trajectory_player.py-2] [INFO] [...] Controller: scaled_joint_trajectory_controller
```

**Status**: ✅ All nodes initialized, but waiting for robot data (no joint_states or TF available)

---

## With UR5 Robot Connected (Full System)

### Terminal 1: UR5 Controller
```bash
./launch_ur5_nosnap.sh 192.168.1.102 false
```

**Expected output includes:**
```
[ur_control_node]: Connecting to robot at 192.168.1.102
[ur_control_node]: Robot connected successfully
[controller_manager]: Loaded controller 'scaled_joint_trajectory_controller'
[controller_manager]: Configured controller 'scaled_joint_trajectory_controller'
[controller_manager]: Started controller 'scaled_joint_trajectory_controller'
```

### Terminal 2: Kinesthetic Teaching System
```bash
ros2 launch ur5_kinesthetic_teaching kinesthetic_teaching.launch.py
```

**Expected output:**
```
[INFO] [launch]: All log files can be found below /home/rml/.ros/log/...
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [trajectory_recorder.py-1]: process started with pid [...]
[INFO] [trajectory_player.py-2]: process started with pid [...]
[INFO] [trajectory_visualizer.py-3]: process started with pid [...]

[trajectory_visualizer.py-3] [INFO] [...] Trajectory Visualizer initialized
[trajectory_recorder.py-1] [INFO] [...] Trajectory Recorder initialized
[trajectory_recorder.py-1] [INFO] [...] Trajectory directory: /home/rml/ur5_trajectories
[trajectory_recorder.py-1] [INFO] [...] Recording rate: 30.0 Hz
[trajectory_recorder.py-1] [INFO] [...] Subscribed to joint_states topic        # NEW!
[trajectory_recorder.py-1] [INFO] [...] TF listener ready, tracking tool0       # NEW!
[trajectory_player.py-2] [INFO] [...] Trajectory Player initialized
[trajectory_player.py-2] [INFO] [...] Trajectory directory: /home/rml/ur5_trajectories
[trajectory_player.py-2] [INFO] [...] Controller: scaled_joint_trajectory_controller
[trajectory_player.py-2] [INFO] [...] Action client connected to controller    # NEW!
```

**Key differences:**
- ✅ Recorder confirms it's receiving joint_states
- ✅ Recorder confirms TF transforms are available for end-effector
- ✅ Player confirms action client connected to the trajectory controller

---

## During Recording Session

### 1. Start Recording
```bash
ros2 service call /trajectory_recorder/start_recording std_srvs/srv/Trigger
```

**Expected output in Terminal 2:**
```
[trajectory_recorder.py-1] [INFO] [...] Recording started
[trajectory_recorder.py-1] [INFO] [...] Move the robot to record trajectory
```

### 2. Moving the Robot (in freedrive mode)
```
[trajectory_recorder.py-1] [INFO] [...] Recording waypoint 1: joints=[...], pose=[...]
[trajectory_recorder.py-1] [INFO] [...] Recording waypoint 2: joints=[...], pose=[...]
[trajectory_recorder.py-1] [INFO] [...] Recording waypoint 3: joints=[...], pose=[...]
...
[trajectory_recorder.py-1] [INFO] [...] Recording waypoint N: joints=[...], pose=[...]
```

**Note**: Only logs when robot moves beyond threshold (default 0.001 radians)

### 3. Stop Recording
```bash
ros2 service call /trajectory_recorder/stop_recording std_srvs/srv/Trigger
```

**Expected output:**
```
[trajectory_recorder.py-1] [INFO] [...] Recording stopped
[trajectory_recorder.py-1] [INFO] [...] Recorded 45 waypoints
[trajectory_recorder.py-1] [INFO] [...] Trajectory saved to: /home/rml/ur5_trajectories/trajectory_2025-11-19_13-45-22.json
```

---

## During Playback Session

### 1. List Available Trajectories
```bash
ros2 service call /trajectory_player/list_trajectories std_srvs/srv/Trigger
```

**Expected output:**
```
[trajectory_player.py-2] [INFO] [...] Available trajectories:
[trajectory_player.py-2] [INFO] [...]   - trajectory_2025-11-19_13-45-22.json
[trajectory_player.py-2] [INFO] [...]   - trajectory_2025-11-19_14-02-15.json
```

### 2. Load a Trajectory
```bash
ros2 service call /trajectory_player/load_trajectory std_srvs/srv/Trigger "{}"
# This loads the most recent trajectory by default
```

**Expected output:**
```
[trajectory_player.py-2] [INFO] [...] Loading trajectory: trajectory_2025-11-19_14-02-15.json
[trajectory_player.py-2] [INFO] [...] Loaded trajectory with 45 waypoints
[trajectory_player.py-2] [INFO] [...] Duration: 15.0 seconds
```

### 3. Play the Trajectory
```bash
ros2 service call /trajectory_player/play_trajectory std_srvs/srv/Trigger
```

**Expected output:**
```
[trajectory_player.py-2] [INFO] [...] Starting trajectory execution
[trajectory_player.py-2] [INFO] [...] Sending goal to action server...
[trajectory_player.py-2] [INFO] [...] Goal accepted by controller
[trajectory_player.py-2] [INFO] [...] Robot is executing trajectory...
[trajectory_player.py-2] [INFO] [...] Trajectory execution completed successfully!
```

**If playing with loop enabled:**
```
[trajectory_player.py-2] [INFO] [...] Trajectory execution completed successfully!
[trajectory_player.py-2] [INFO] [...] Looping enabled, restarting trajectory...
[trajectory_player.py-2] [INFO] [...] Sending goal to action server...
[trajectory_player.py-2] [INFO] [...] Goal accepted by controller
...
```

---

## RViz Visualization

When you open RViz with the trajectory visualizer running:

1. **Before Recording**: No markers visible
2. **During Recording**: 
   - Green sphere at start point
   - Blue line following the end-effector path
   - Small blue arrows showing orientation at each waypoint
3. **After Recording**:
   - Red sphere at end point
   - Complete trajectory path visible
4. **During Playback**: Robot follows the visualized path

---

## Error Scenarios

### Robot Not Connected
```
[trajectory_recorder.py-1] [WARN] [...] No joint_states received yet
[trajectory_player.py-2] [ERROR] [...] Action server not available: scaled_joint_trajectory_controller/follow_joint_trajectory
```
**Solution**: Launch UR5 controller first

### No TF Available
```
[trajectory_recorder.py-1] [WARN] [...] Could not get transform from base to tool0
```
**Solution**: Ensure robot_state_publisher is running (included in UR5 launch)

### Empty Trajectory
```
[trajectory_recorder.py-1] [WARN] [...] No waypoints recorded (robot did not move)
```
**Solution**: Move the robot more during recording, or reduce motion threshold

### Action Fails
```
[trajectory_player.py-2] [ERROR] [...] Trajectory execution failed: EXECUTION_FAILED
```
**Solution**: Check that robot is not in protective stop, check joint limits

---

## Verification Commands

### Check if topics are publishing:
```bash
ros2 topic list | grep -E "(joint_states|trajectory)"
ros2 topic hz /joint_states
ros2 topic echo /trajectory_visualization --no-arr
```

### Check if services are available:
```bash
ros2 service list | grep trajectory
```

### Check action server:
```bash
ros2 action list
ros2 action info /scaled_joint_trajectory_controller/follow_joint_trajectory
```

### Monitor TF:
```bash
ros2 run tf2_ros tf2_echo base tool0
```

---

## Summary

**Current Status**: ✅ System initialized correctly, waiting for robot connection

**Next Step**: Connect to real UR5 robot using `./launch_ur5_nosnap.sh 192.168.1.102 false` in Terminal 1, then you'll see the additional connection messages listed above.
