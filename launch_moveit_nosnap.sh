#!/bin/bash

# Wrapper script to launch MoveIt with RViz without snap library conflicts
# This script cleans snap environment variables and library paths

# Save essential variables
SAVED_HOME=$HOME
SAVED_USER=$USER
SAVED_SHELL=$SHELL
SAVED_TERM=$TERM
SAVED_DISPLAY=$DISPLAY

# Get parameters
UR_TYPE=${1:-ur5}
LAUNCH_RVIZ=${2:-true}

# Create a clean environment script
cat > /tmp/launch_moveit_clean.sh << 'EOFSCRIPT'
#!/bin/bash

# Restore essential variables
export HOME="$1"
export USER="$2"
export SHELL="$3"
export TERM="$4"
export DISPLAY="$5"
UR_TYPE="$6"
LAUNCH_RVIZ="$7"

# Set clean PATH (no snap)
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/humble/bin"

# Set clean LD_LIBRARY_PATH (system libraries first, no snap)
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"

# Clean XDG paths (remove snap)
export XDG_DATA_DIRS="/usr/local/share:/usr/share"
export XDG_CONFIG_DIRS="/etc/xdg"
unset XDG_DATA_HOME

# Unset all snap-related variables
unset GDK_BACKEND_VSCODE_SNAP_ORIG
unset GIO_MODULE_DIR_VSCODE_SNAP_ORIG
unset GIO_MODULE_DIR
unset GTK_EXE_PREFIX
unset GTK_PATH
unset GTK_IM_MODULE_FILE
unset LOCPATH
unset GSETTINGS_SCHEMA_DIR
unset BAMF_DESKTOP_FILE_HINT
unset GIO_LAUNCHED_DESKTOP_FILE

# Source ROS
source /opt/ros/humble/setup.bash
source $HOME/ur5-robotiq-ros2-control/install/setup.bash
source $HOME/ur_ws/src/install/setup.bash

echo "=== MoveIt Launch (Clean Environment) ==="
echo "UR Type: $UR_TYPE"
echo "Launch RViz: $LAUNCH_RVIZ"
echo "=========================================="

# Launch MoveIt
exec ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=$UR_TYPE \
  launch_rviz:=$LAUNCH_RVIZ
EOFSCRIPT

chmod +x /tmp/launch_moveit_clean.sh

# Execute in clean environment
exec /tmp/launch_moveit_clean.sh "$SAVED_HOME" "$SAVED_USER" "$SAVED_SHELL" "$SAVED_TERM" "$SAVED_DISPLAY" "$UR_TYPE" "$LAUNCH_RVIZ"
