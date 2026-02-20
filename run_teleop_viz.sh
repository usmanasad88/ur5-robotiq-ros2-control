#!/bin/bash
# Launch the SpaceMouse teleop box visualizer with a clean library environment.

# Deactivate conda — strip bin from PATH and remove all CONDA_* vars
if [[ -n "$CONDA_PREFIX" ]]; then
    echo "Deactivating conda environment: $CONDA_PREFIX"
    export PATH=${PATH//$CONDA_PREFIX\/bin:/}
    export PATH=${PATH//$CONDA_PREFIX\/bin/}
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER CONDA_SHLVL CONDA_PYTHON_EXE
fi
# Also remove conda base if present
if [[ -n "$CONDA_PYTHON_EXE" ]]; then
    CONDA_BASE="$(dirname "$(dirname "$CONDA_PYTHON_EXE")")"
    export PATH=${PATH//$CONDA_BASE\/bin:/}
    export PATH=${PATH//$CONDA_BASE\/bin/}
fi

# Strip conflicting libraries
unset GTK_PATH GTK2_RC_FILES GTK_IM_MODULE QT_QPA_PLATFORMTHEME LD_PRELOAD PYTHONPATH
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/opt/ros/humble/opt/rviz_ogre_vendor/lib"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# Use system python3 explicitly to avoid any conda python leaking in
exec /usr/bin/python3 "$SCRIPT_DIR/teleop_box_viz.py"
