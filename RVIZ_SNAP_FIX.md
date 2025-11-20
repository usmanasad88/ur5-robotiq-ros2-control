# RViz Snap Library Conflict - Fixed

## Problem
When launching `ros2 launch ur_robot_driver ur_control.launch.py` from a snap-installed VS Code terminal, RViz crashes with:
```
/opt/ros/humble/lib/rviz2/rviz2: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
```

## Root Cause
VS Code installed via snap sets environment variables that cause Qt applications (like RViz) to use snap's incompatible `libpthread.so.0` library instead of the system library.

## Solution

### Quick Fix: Use the Wrapper Script
```bash
./launch_ur5_nosnap.sh [robot_ip] [use_fake_hardware]
```

**Examples:**
```bash
# Launch with fake hardware (simulation)
./launch_ur5_nosnap.sh 192.168.1.102 true

# Launch with real robot
./launch_ur5_nosnap.sh 192.168.1.102 false
```

### What the Script Does
The `launch_ur5_nosnap.sh` script:
1. Cleans all snap-related environment variables (GTK_PATH, GIO_MODULE_DIR, XDG_DATA_DIRS, etc.)
2. Sets a clean LD_LIBRARY_PATH prioritizing system libraries
3. Removes snap paths from PATH
4. Launches ROS 2 in this clean environment

### Alternative: Launch from Non-Snap Terminal
If you prefer not to use the wrapper script, launch from a regular terminal (not snap-based VS Code):
1. Open a regular GNOME Terminal (Ctrl+Alt+T)
2. Source your workspace: `source ~/ur5-robotiq-ros2-control/install/setup.bash`
3. Run the standard launch command:
   ```bash
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 robot_ip:=192.168.1.102 use_fake_hardware:=true
   ```

## Verification
When RViz launches successfully, you should see:
```
[rviz2-4] [INFO] [timestamp] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-4] [INFO] [timestamp] [rviz2]: OpenGl version: 4.6 (GLSL 4.6)
```

## Technical Details

### Problematic Environment Variables from Snap
- `GTK_PATH=/snap/code/207/usr/lib/x86_64-linux-gnu/gtk-3.0`
- `GIO_MODULE_DIR=/home/rml/snap/code/common/.cache/gio-modules`
- `XDG_DATA_DIRS` containing snap paths
- Various `*_VSCODE_SNAP_ORIG` variables

### Clean Environment Requirements
- `LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib`
- `PATH` without `/snap/bin`
- `XDG_DATA_DIRS=/usr/local/share:/usr/share`
- All snap-specific variables unset

## Related Files
- `/home/rml/ur5-robotiq-ros2-control/launch_ur5_nosnap.sh` - Wrapper script for clean launch
- `/home/rml/ur5-robotiq-ros2-control/launch_ur5_robotiq.sh` - Wrapper for UR5+Robotiq gripper
- `~/.bashrc` - Contains commented RViz LD_LIBRARY_PATH fix (alternative approach)

## Notes
- The real-time kernel warning (`[WARN] No real-time kernel detected`) is normal and doesn't affect simulation.
- Controller spawner errors are unrelated to the RViz snap issue and may need separate investigation.
