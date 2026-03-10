#!/bin/bash
# ============================================
# Test FT300 Force/Torque Sensor Reading
# ============================================
# Run this AFTER the UR5 driver is launched with real hardware.
# The force_torque_sensor_broadcaster is auto-started by the UR driver.
#
# Usage:
#   ./test_ft300_read.sh
# ============================================

set -e
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash" 2>/dev/null || true

echo "=== Step 1: Check if force_torque_sensor_broadcaster is active ==="
ros2 control list_controllers 2>/dev/null | grep -i force || echo "(controller_manager not reachable — is the UR5 driver running?)"

echo ""
echo "=== Step 2: List wrench / ft topics ==="
ros2 topic list 2>/dev/null | grep -iE "force|torque|wrench|ft" || echo "(no matching topics found)"

echo ""
echo "=== Step 3: Echo FT data (Ctrl+C to stop) ==="
echo "Trying /force_torque_sensor_broadcaster/wrench ..."

# Try the standard topic name first, fall back to ft_data
TOPIC="/force_torque_sensor_broadcaster/wrench"
if ! ros2 topic info "$TOPIC" 2>/dev/null | grep -q "Type"; then
    echo "Topic $TOPIC not found, trying /force_torque_sensor_broadcaster/ft_data ..."
    TOPIC="/force_torque_sensor_broadcaster/ft_data"
fi

echo "Listening on: $TOPIC"
echo "You should see force (N) and torque (Nm) values. Move the robot or press on the sensor to see changes."
echo "---"
ros2 topic echo "$TOPIC" --once || echo "Could not read from $TOPIC"
echo ""
echo "=== Continuous stream (5 seconds) ==="
timeout 5 ros2 topic hz "$TOPIC" 2>/dev/null || echo "(topic not publishing)"
