# Bug Fix: Service Type Error in trajectory_player.py

## Issue
When launching the kinesthetic teaching system, a `RuntimeError` occurred:

```
RuntimeError: String is a message type and should only be used with publishers and subscribers
```

## Root Cause
In `trajectory_player.py`, line 60 incorrectly used `String` (a message type) instead of `Trigger` (a service type) for the `load_trajectory` service:

```python
# WRONG - String is a message type, not a service type
self.load_srv = self.create_service(String, '~/load_trajectory', self.load_callback)
```

## Solution
Changed the service definition to use `Trigger` from `std_srvs.srv`:

```python
# CORRECT - Trigger is a service type
self.load_srv = self.create_service(Trigger, '~/load_trajectory', self.load_callback)
```

This matches the other service definitions in the file which correctly use `Trigger`:
- `start_recording` service
- `stop_recording` service  
- `play_trajectory` service

## Additional Fix
Also updated the default controller name from `'joint_trajectory_controller'` to `'scaled_joint_trajectory_controller'` to match the UR5 robot driver's actual controller name.

## Testing
After rebuilding with `colcon build --packages-select ur5_kinesthetic_teaching --symlink-install`, the service now correctly initializes.

## Proper Launch Sequence
**Always launch the UR5 robot controller BEFORE the kinesthetic teaching system:**

```bash
# Terminal 1: UR5 Controller
cd /home/rml/ur5-robotiq-ros2-control
./launch_ur5_nosnap.sh 192.168.1.102 true

# Terminal 2: Kinesthetic Teaching (wait for Terminal 1 to fully initialize)
source install/setup.bash
ros2 launch ur5_kinesthetic_teaching kinesthetic_teaching.launch.py
```

## Status
✅ Fixed - Package rebuilt and documentation updated
