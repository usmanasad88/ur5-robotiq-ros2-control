#!/bin/bash
# Launch script for UR5 Program Selector UI

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "============================================"
echo "UR5 Program Selector UI"
echo "============================================"
echo ""
echo "Starting Streamlit UI..."
echo "The UI will open in your default browser"
echo ""
echo "Controls:"
echo "  - Click program buttons to load & execute"
echo "  - Use sidebar controls for pause/resume/stop"
echo "  - Refresh to update program list"
echo ""
echo "Prerequisites:"
echo "  - Program executor must be running"
echo "  - Run: ./run_program_executor.sh"
echo ""
echo "Note: The program executor already has keyboard/presenter controls:"
echo "  [Next/PageDown/Space/Right] - Start/Resume execution"
echo "  [Prev/PageUp/Left]          - Pause or record pose"  
echo "  [S/ESC]                     - Stop execution"
echo "============================================"
echo ""

# Use system Python 3.10 for ROS 2 compatibility (not conda Python)
export PATH="/usr/bin:$PATH"
PYTHON_BIN="/usr/bin/python3.10"

# Preload system libstdc++ to prevent GLIBCXX version conflicts with ROS 2
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# Source ROS 2 workspace
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

# Ensure ROS Python paths are at the front
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH"

# Check if streamlit is installed for system Python
if ! $PYTHON_BIN -m streamlit version &> /dev/null; then
    echo "❌ Streamlit not found for system Python 3.10. Installing..."
    $PYTHON_BIN -m pip install --user streamlit psutil opencv-python h5py
else
    # Check if required packages are installed
    if ! $PYTHON_BIN -c "import cv2" &> /dev/null; then
        echo "📦 Installing OpenCV..."
        $PYTHON_BIN -m pip install --user opencv-python
    fi
    if ! $PYTHON_BIN -c "import h5py" &> /dev/null; then
        echo "📦 Installing h5py..."
        $PYTHON_BIN -m pip install --user h5py
    fi
fi

# Launch Streamlit with system Python 3.10
exec $PYTHON_BIN -m streamlit run program_selector_ui.py --server.port 8501 --server.headless false
