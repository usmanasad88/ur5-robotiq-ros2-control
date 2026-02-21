#!/bin/bash
# ============================================
# Launch All UR5 Services
# ============================================
# Opens all services as named windows inside a single tmux session,
# then attaches to it in a new gnome-terminal window.
#
# Usage:
#   ./launch_all.sh              # All services (fake hardware)
#   ./launch_all.sh --real       # Real robot + gripper adapter
#   ./launch_all.sh --no-ui      # Skip the Streamlit UI
# ============================================


# To kill: tmux kill-session -t ur5


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="ur5"

USE_FAKE="false"
LAUNCH_UI=true

for arg in "$@"; do
    case $arg in
        --real|-r)  USE_FAKE="false" ;;
        --no-ui)    LAUNCH_UI=false ;;
        --help|-h)
            echo "Usage: $0 [--real] [--no-ui]"
            echo "  --real, -r   Use real robot hardware + gripper adapter"
            echo "  --no-ui      Skip launching the Streamlit UI"
            exit 0
            ;;
    esac
done

# Require tmux
if ! command -v tmux &>/dev/null; then
    echo "ERROR: tmux is not installed. Install with: sudo apt install tmux"
    exit 1
fi

echo "Launching UR5 services  (fake=$USE_FAKE, ui=$LAUNCH_UI)"

# Kill any stale session from a previous run
tmux kill-session -t "$SESSION" 2>/dev/null

# ---- Create session + windows ------------------------------------------

# Window 1: UR5 driver
tmux new-session -d -s "$SESSION" -n "UR5-Driver" \
    "echo '[UR5 Driver] Starting...'; \"$SCRIPT_DIR/launch_ur5_robotiq.sh\" 172.17.66.105 $USE_FAKE; exec bash"

# Window 2 (real only): Robotiq gripper adapter
if [ "$USE_FAKE" = "false" ]; then
    tmux new-window -t "$SESSION" -n "Gripper" \
        "echo '[Gripper] Waiting 8s for UR5 driver...'; sleep 8; echo '[Gripper] Starting Robotiq adapter...'; \"$SCRIPT_DIR/run_gripper.sh\" 172.17.66.105; exec bash"
fi

# Window: cuRobo motion planner
tmux new-window -t "$SESSION" -n "cuRobo" \
    "echo '[cuRobo] Waiting 2s...'; sleep 2; echo '[cuRobo] Starting cuRobo control node...'; \"$SCRIPT_DIR/run_curobo.sh\"; exec bash"

# Window: Program executor
tmux new-window -t "$SESSION" -n "Executor" \
    "echo '[Executor] Waiting 4s...'; sleep 4; echo '[Executor] Starting program executor...'; \"$SCRIPT_DIR/run_program_executor.sh\"; exec bash"

# Window: Streamlit UI
if [ "$LAUNCH_UI" = true ]; then
    tmux new-window -t "$SESSION" -n "UI" \
        "echo '[UI] Waiting 6s...'; sleep 6; echo '[UI] Starting Streamlit UI...'; \"$SCRIPT_DIR/run_program_ui.sh\"; exec bash"
fi

# Select the first window
tmux select-window -t "$SESSION:0"

# ---- Open gnome-terminal attached to the tmux session ------------------
gnome-terminal --title="UR5 Control" -- tmux attach-session -t "$SESSION" 2>/dev/null

echo ""
echo "tmux session '$SESSION' running."
echo "Switch windows: Ctrl-b + [0-4]  or  Ctrl-b + n/p"
echo "Detach:         Ctrl-b + d"
echo "Re-attach:      tmux attach -t $SESSION"
