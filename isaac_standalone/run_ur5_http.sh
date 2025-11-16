#!/bin/bash
# Helper script to run UR5 HTTP control with Isaac Sim

ISAAC_SIM_PATH="/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║          UR5 HTTP End-Effector Control Runner                ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# Check if Isaac Sim exists
if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo -e "${RED}Error: Isaac Sim not found at: $ISAAC_SIM_PATH${NC}"
    exit 1
fi

# Check if Flask server is running (only if --server argument is provided)
echo -e "${YELLOW}Checking Flask server...${NC}"

# Parse arguments to find server URL
SERVER_URL="http://localhost:5000"  # Default
ENDPOINT="/joint_actions"           # Default
CHECK_SERVER=true

# Look for --server argument
for ((i=1; i<=$#; i++)); do
    if [[ "${!i}" == "--server" ]]; then
        j=$((i+1))
        if [[ $j -le $# ]]; then
            SERVER_URL="${!j}"
        fi
        break
    fi
done

# Look for --endpoint argument
for ((i=1; i<=$#; i++)); do
    if [[ "${!i}" == "--endpoint" ]]; then
        j=$((i+1))
        if [[ $j -le $# ]]; then
            ENDPOINT="${!j}"
        fi
        break
    fi
done

# Check if --test-motion is specified (skip server check for test mode)
for arg in "$@"; do
    if [[ "$arg" == "--test-motion" ]]; then
        CHECK_SERVER=false
        echo "Test motion mode detected - skipping server check"
        break
    fi
done

if $CHECK_SERVER; then
    if curl -s --max-time 2 "$SERVER_URL$ENDPOINT" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ Flask server is running at $SERVER_URL$ENDPOINT${NC}"
    else
        echo -e "${YELLOW}⚠ Warning: Cannot reach Flask server at $SERVER_URL$ENDPOINT${NC}"
        echo "  Make sure your Flask server is running!"
        echo "  Example: python /home/mani/Repos/ur_ws/scripts_for_coding_agent_reference/flask_server.py"
        echo ""
        echo "  The simulation will start anyway, but robot won't receive commands."
        echo ""
        read -p "Continue anyway? (y/N) " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

echo ""
echo "Starting UR5 HTTP control simulation..."
echo "Arguments: $@"
echo ""

# Run the script - pass all arguments directly to Python script
$ISAAC_SIM_PATH/python.sh "$SCRIPT_DIR/ur5_http_control.py" "$@"
