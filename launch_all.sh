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

USE_FAKE="true"
LAUNCH_UI=true
LAUNCH_QUEST=false
LAUNCH_QUEST_SERVO=false
LAUNCH_SPACEMOUSE=false

for arg in "$@"; do
    case $arg in
        --real|-r)         USE_FAKE="false" ;;
        --no-ui)           LAUNCH_UI=false ;;
        --quest|-q)        LAUNCH_QUEST=true ;;
        --quest-servo|-qs) LAUNCH_QUEST_SERVO=true ;;
        --spacemouse|-sm)  LAUNCH_SPACEMOUSE=true ;;
        --help|-h)
            echo "Usage: $0 [--real] [--no-ui] [--quest] [--quest-servo] [--spacemouse]"
            echo "  --real, -r          Use real robot hardware + gripper adapter"
            echo "  --no-ui             Skip launching the Streamlit UI"
            echo "  --quest, -q         Quest sim teleop via cuRobo (PoseDelta, jerky)"
            echo "  --quest-servo, -qs  Quest sim teleop via Jacobian IK velocity (smooth)"
            echo "  --spacemouse, -sm   SpaceMouse teleop via Jacobian IK velocity (smooth)"
            exit 0
            ;;
    esac
done

# Require tmux
if ! command -v tmux &>/dev/null; then
    echo "ERROR: tmux is not installed. Install with: sudo apt install tmux"
    exit 1
fi

echo "Launching UR5 services  (fake=$USE_FAKE, ui=$LAUNCH_UI, quest=$LAUNCH_QUEST, spacemouse=$LAUNCH_SPACEMOUSE)"

# Export USE_FAKE_HARDWARE so child processes (API, UI) can detect sim mode
export USE_FAKE_HARDWARE="$USE_FAKE"

# Kill any stale session from a previous run
tmux kill-session -t "$SESSION" 2>/dev/null

# ---- Create session + windows ------------------------------------------

# Window 1: UR5 driver - IP: 172.17.66.105 OR 192.168.0.105 ; Host IP: 172.17.66.133 OR 192.168.0.38
tmux new-session -d -s "$SESSION" -n "UR5-Driver" \
    "echo '[UR5 Driver] Starting...'; \"$SCRIPT_DIR/launch_ur5_robotiq.sh\" 192.168.0.105 $USE_FAKE; exec bash"

# Window 2 (real only): Robotiq gripper adapter
if [ "$USE_FAKE" = "false" ]; then
    tmux new-window -t "$SESSION" -n "Gripper" \
        "echo '[Gripper] Waiting 8s for UR5 driver...'; sleep 8; echo '[Gripper] Starting Robotiq adapter...'; \"$SCRIPT_DIR/run_gripper.sh\" 192.168.0.105; exec bash"
fi

# Window: cuRobo motion planner
tmux new-window -t "$SESSION" -n "cuRobo" \
    "echo '[cuRobo] Waiting 2s...'; sleep 2; echo '[cuRobo] Starting cuRobo control node...'; \"$SCRIPT_DIR/run_curobo.sh\"; exec bash"

# Window: Program executor
EXECUTOR_FAKE_FLAG=""
[ "$USE_FAKE" = "true" ] && EXECUTOR_FAKE_FLAG="--fake"
tmux new-window -t "$SESSION" -n "Executor" \
    "echo '[Executor] Waiting 4s...'; sleep 4; echo '[Executor] Starting program executor...'; \"$SCRIPT_DIR/run_program_executor.sh\" $EXECUTOR_FAKE_FLAG; exec bash"

# Window: External Control API
tmux new-window -t "$SESSION" -n "API" \
    "echo '[API] Waiting 6s...'; sleep 6; echo '[API] Starting External Control API...'; \"$SCRIPT_DIR/run_external_api.sh\"; exec bash"

# Window: Streamlit UI
if [ "$LAUNCH_UI" = true ]; then
    tmux new-window -t "$SESSION" -n "UI" \
        "echo '[UI] Waiting 6s...'; sleep 6; echo '[UI] Starting Streamlit UI...'; \"$SCRIPT_DIR/run_program_ui.sh\"; exec bash"
fi

# Window: Quest 3S sim teleop via cuRobo/PoseDelta (--quest)
if [ "$LAUNCH_QUEST" = true ] && [ "$USE_FAKE" = "true" ]; then
    tmux new-window -t "$SESSION" -n "Quest-Teleop" \
        "echo '[Quest] Waiting 10s for cuRobo executor...'; sleep 10; echo '[Quest] Starting Quest 3S sim teleop...'; \"$SCRIPT_DIR/run_quest_sim_teleop.sh\"; exec bash"
elif [ "$LAUNCH_QUEST" = true ] && [ "$USE_FAKE" = "false" ]; then
    echo "NOTE: --quest uses the ROS 2 / cuRobo path (sim only)."
    echo "      For real robot teleop use: bash run_quest_rtde_teleop.sh"
fi

# Window: Quest 3S servo teleop via Jacobian IK velocity control (--quest-servo)
# Smoother than cuRobo — no trajectory replanning, direct 50 Hz velocity streaming.
if [ "$LAUNCH_QUEST_SERVO" = true ] && [ "$USE_FAKE" = "true" ]; then
    tmux new-window -t "$SESSION" -n "Quest-Servo" \
        "echo '[Quest-Servo] Waiting 5s for UR5 driver...'; sleep 5; echo '[Quest-Servo] Starting Quest servo teleop...'; \"$SCRIPT_DIR/run_quest_servo_teleop.sh\"; exec bash"
elif [ "$LAUNCH_QUEST_SERVO" = true ] && [ "$USE_FAKE" = "false" ]; then
    echo "NOTE: --quest-servo targets the sim velocity controller (sim only)."
    echo "      For real robot teleop use: bash run_quest_rtde_teleop.sh"
fi

# Window: SpaceMouse teleop via Jacobian IK velocity control (--spacemouse)
if [ "$LAUNCH_SPACEMOUSE" = true ] && [ "$USE_FAKE" = "true" ]; then
    tmux new-window -t "$SESSION" -n "SpaceMouse" \
        "echo '[SpaceMouse] Waiting 5s for UR5 driver...'; sleep 5; echo '[SpaceMouse] Starting SpaceMouse servo teleop...'; \"$SCRIPT_DIR/run_spacemouse_teleop.sh\" --swap; exec bash"
elif [ "$LAUNCH_SPACEMOUSE" = true ] && [ "$USE_FAKE" = "false" ]; then
    echo "NOTE: --spacemouse targets the sim velocity controller (sim only)."
fi

# Select the first window
tmux select-window -t "$SESSION:0"

# ---- Open gnome-terminal attached to the tmux session -------------------
# Strip snap/VSCode environment variables that break gnome-terminal when
# this script is launched from the VS Code integrated terminal.
env -u GIO_MODULE_DIR -u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR \
    -u GTK_PATH -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE \
    -u LOCPATH -u SNAP_LIBRARY_PATH -u GSETTINGS_SCHEMA_DIR \
    gnome-terminal --title="UR5 Control" -- tmux attach-session -t "$SESSION"

echo ""
echo "tmux session '$SESSION' running."
echo "Switch windows: Ctrl-b + [0-4]  or  Ctrl-b + n/p"
echo "Detach:         Ctrl-b + d"
echo "Re-attach:      tmux attach -t $SESSION"
