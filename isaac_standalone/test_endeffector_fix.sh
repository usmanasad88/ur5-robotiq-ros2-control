#!/bin/bash

# Quick test to verify the end-effector link fix

echo "Testing ur5_http_control.py end-effector fix..."
echo ""
echo "1. Checking if Flask server is running..."

if curl -s http://localhost:5000/joint_actions > /dev/null 2>&1; then
    echo "   ✓ Flask server is responding"
else
    echo "   ✗ Flask server is not running"
    echo ""
    echo "   Start it with: python3 test_flask_server.py --mode static"
    exit 1
fi

echo ""
echo "2. Testing Isaac Sim startup with corrected end-effector path..."
echo "   This will run for 20 seconds then automatically close"
echo ""

cd /home/mani/Repos/ur_ws/isaac_standalone

# Run with timeout to avoid hanging
timeout 20s /home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur5_http_control.py --headless 2>&1 | tee /tmp/ur5_http_test.log

EXIT_CODE=$?

echo ""
echo "3. Analyzing output..."
echo ""

# Check for the specific error we're fixing
if grep -q "ee_link.*is invalid" /tmp/ur5_http_test.log; then
    echo "   ✗ FAIL: End-effector link error still present"
    echo "   The ee_link path error was found in output"
    exit 1
elif grep -q "tool0.*is invalid" /tmp/ur5_http_test.log; then
    echo "   ✗ FAIL: tool0 link error (unexpected)"
    echo "   The tool0 path is also invalid"
    exit 1
elif grep -q "Loading UR5 robot" /tmp/ur5_http_test.log && grep -q "Initializing simulation" /tmp/ur5_http_test.log; then
    echo "   ✓ PASS: Robot loaded successfully"
    echo "   ✓ PASS: Simulation initialized without ee_link error"
    echo ""
    echo "Fix verified! The end-effector path is now correct."
    exit 0
elif [ $EXIT_CODE -eq 124 ]; then
    # Timeout occurred - this is actually good, means it was running
    echo "   ✓ PASS: Process ran for 20 seconds without crashing"
    echo "   ✓ PASS: No end-effector link errors detected"
    echo ""
    echo "Fix verified! The simulation ran successfully."
    exit 0
else
    echo "   ? UNKNOWN: Could not determine test result"
    echo "   Check /tmp/ur5_http_test.log for details"
    exit 2
fi
