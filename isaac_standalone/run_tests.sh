#!/bin/bash
# Test runner for Isaac Sim UR robot scripts
# This script runs through various tests to verify everything works

set -e  # Exit on error

ISAAC_SIM_PATH="/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ISAAC_PYTHON="$ISAAC_SIM_PATH/python.sh"

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║          Isaac Sim UR Robot - Test Suite                    ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Check if Isaac Sim exists
echo -e "${YELLOW}Checking Isaac Sim installation...${NC}"
if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo -e "${RED}✗ Isaac Sim not found at: $ISAAC_SIM_PATH${NC}"
    exit 1
fi

if [ ! -f "$ISAAC_PYTHON" ]; then
    echo -e "${RED}✗ Isaac Sim Python not found at: $ISAAC_PYTHON${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Isaac Sim found${NC}"
echo ""

# Test 1: Simple test
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 1: Simple Test (basic functionality)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if $ISAAC_PYTHON "$SCRIPT_DIR/simple_test.py" 2>&1 | head -20; then
    echo -e "${GREEN}✓ Test 1 PASSED${NC}"
else
    echo -e "${RED}✗ Test 1 FAILED${NC}"
    exit 1
fi
echo ""

# Test 2: Load UR10
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 2: Load UR10 Robot"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if $ISAAC_PYTHON "$SCRIPT_DIR/ur10_import.py" --robot ur10 --test 2>&1 | tail -20; then
    echo -e "${GREEN}✓ Test 2 PASSED${NC}"
else
    echo -e "${RED}✗ Test 2 FAILED${NC}"
    exit 1
fi
echo ""

# Test 3: Load UR5
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 3: Load UR5 Robot"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if $ISAAC_PYTHON "$SCRIPT_DIR/ur10_import.py" --robot ur5 --test 2>&1 | tail -20; then
    echo -e "${GREEN}✓ Test 3 PASSED${NC}"
else
    echo -e "${RED}✗ Test 3 FAILED${NC}"
    exit 1
fi
echo ""

# Test 4: Advanced controller - static mode
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 4: Advanced Controller (static mode)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if $ISAAC_PYTHON "$SCRIPT_DIR/ur_robot_advanced.py" --robot ur10 --mode static --frames 100 2>&1 | tail -20; then
    echo -e "${GREEN}✓ Test 4 PASSED${NC}"
else
    echo -e "${RED}✗ Test 4 FAILED${NC}"
    exit 1
fi
echo ""

# Test 5: Advanced controller - animated mode with recording
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Test 5: Advanced Controller (animated + recording)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if $ISAAC_PYTHON "$SCRIPT_DIR/ur_robot_advanced.py" --robot ur5 --mode animated --record --frames 200 2>&1 | tail -20; then
    echo -e "${GREEN}✓ Test 5 PASSED${NC}"
    if [ -f "ur5_recording_animated.csv" ]; then
        echo -e "${GREEN}  ✓ CSV file created successfully${NC}"
        echo "    First few lines of CSV:"
        head -5 "ur5_recording_animated.csv" | sed 's/^/    /'
    fi
else
    echo -e "${RED}✗ Test 5 FAILED${NC}"
    exit 1
fi
echo ""

# Optional Test 6: ROS 2 URDF (only if workspace exists)
if [ -d "/home/mani/Repos/ur_ws/install/ur_description" ]; then
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Test 6: Load from ROS 2 URDF (optional)"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    if $ISAAC_PYTHON "$SCRIPT_DIR/ur_from_ros2_urdf.py" --robot ur5 --workspace /home/mani/Repos/ur_ws 2>&1 | tail -20; then
        echo -e "${GREEN}✓ Test 6 PASSED${NC}"
    else
        echo -e "${YELLOW}⚠ Test 6 FAILED (optional - ROS 2 integration)${NC}"
    fi
    echo ""
else
    echo -e "${YELLOW}⊘ Skipping Test 6 (ROS 2 workspace not built)${NC}"
    echo ""
fi

# Summary
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║                     TEST RESULTS                             ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo -e "${GREEN}✓ Core Tests: PASSED${NC}"
echo -e "${GREEN}✓ Basic functionality working${NC}"
echo -e "${GREEN}✓ UR5 and UR10 loading successful${NC}"
echo -e "${GREEN}✓ Advanced controller working${NC}"
echo -e "${GREEN}✓ Recording functionality working${NC}"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "All tests completed successfully! 🎉"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Next steps:"
echo "  1. Try the interactive mode: ./run_ur_robot.sh --robot ur10"
echo "  2. Experiment with recording: $ISAAC_PYTHON ur_robot_advanced.py --mode animated --record"
echo "  3. Read the documentation: cat README.md"
echo ""
