#!/bin/bash
# ============================================
# Record UR5 Program Episodes
# ============================================
# Automation wrapper that records a .prog file execution
# with joint states + camera feeds, producing data suitable
# for LeRobot SmolVLA finetuning.
#
# Prerequisites:
#   - The ur5 tmux session must be running (./launch_all.sh)
#   - ROS 2 sourced
#
# Usage:
#   ./record_program_episode.sh --program pick_and_place_object.prog --episodes 5
#   ./record_program_episode.sh -p pick_and_place_object.prog -n 5 --task "pick and place" --convert --repo-id myuser/ur5_data
# ============================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2 workspace
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
fi

# ---- GoPro network setup (USB Ethernet) --------------------------------
# The GoPro Max 2 streams over a USB-Ethernet adapter. The host side needs
# an IP on the same subnet (172.29.170.0/24) before the stream is reachable.
GOPRO_IFACE="enx04574796c048"
GOPRO_HOST_IP="172.29.170.50/24"
if ip link show "$GOPRO_IFACE" &>/dev/null; then
    if ! ip addr show "$GOPRO_IFACE" | grep -q "172.29.170.50"; then
        echo "Setting up GoPro network: $GOPRO_HOST_IP on $GOPRO_IFACE"
        sudo ip addr add "$GOPRO_HOST_IP" dev "$GOPRO_IFACE" 2>/dev/null || true
    fi
fi

# Find conda Python with necessary packages (h5py, cv2, etc.)
if [ -f "$HOME/miniconda3/envs/ur5_python/bin/python" ]; then
    PYTHON_EXE="$HOME/miniconda3/envs/ur5_python/bin/python"
elif [ -f "$HOME/anaconda3/envs/ur5_python/bin/python" ]; then
    PYTHON_EXE="$HOME/anaconda3/envs/ur5_python/bin/python"
else
    PYTHON_EXE=$(which python3)
    echo "Warning: ur5_python conda env not found, using $PYTHON_EXE"
fi

# Add ROS 2 Python packages to PYTHONPATH so rclpy is available in conda env
export PYTHONPATH="/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH"

cd "$SCRIPT_DIR"

echo "============================================"
echo "  UR5 Program Episode Recorder"
echo "  Python: $PYTHON_EXE"
echo "============================================"

$PYTHON_EXE "$SCRIPT_DIR/record_program_episode.py" "$@"
