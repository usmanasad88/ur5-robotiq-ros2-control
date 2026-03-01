#!/bin/bash
# ============================================================
# SmolVLA Inference — Absolute Joint Position Control for UR5
# ============================================================
# Runs a finetuned SmolVLA policy and publishes absolute joint
# positions to /scaled_joint_trajectory_controller/joint_trajectory.
#
# No controller switching needed — uses the default trajectory
# controller (same one MoveIt / cuRobo / UI use).
#
# Architecture:
#   Camera + JointState → SmolVLA (10 Hz) → absolute joint positions
#                                            → /scaled_joint_trajectory_controller
#   Gripper from action[6]
#                                            → /robotiq_2f_urcap_adapter
#
# Prerequisites:
#   1. launch_all.sh must be running (provides joint_states + controllers)
#   2. lerobot + smolvla deps installed:
#        cd ~/Repos/lerobot && pip install -e ".[smolvla]"
#   3. A finetuned SmolVLA checkpoint
#
# Usage:
#   ./run_smolvla_inference.sh                          # defaults
#   ./run_smolvla_inference.sh --task "pick red block"  # custom task
#   ./run_smolvla_inference.sh --dry-run                # test without moving
# ============================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LEROBOT_DIR="${LEROBOT_DIR:-$HOME/Repos/lerobot}"
CONDA_ENV="${CONDA_ENV:-lerobot}"

# ---- defaults (override via env vars) ----
# MODEL_PATH can be a local dir OR a HuggingFace Hub ID (e.g. lerobot/smolvla_base)
MODEL_PATH="${MODEL_PATH:-lerobot/smolvla_base}"
TASK="${TASK:-robot task}"
DEVICE="${DEVICE:-cuda}"
WEBCAM="${WEBCAM:-0}"
WRIST="${WRIST:-}"
IMAGE_H="${IMAGE_H:-480}"
IMAGE_W="${IMAGE_W:-640}"
INFERENCE_FPS="${INFERENCE_FPS:-10}"
DURATION_SCALE="${DURATION_SCALE:-1.0}"
MAX_DELTA="${MAX_DELTA:-0.1}"

# ---- parse extra CLI args (passed through to Python) ----
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --task)            shift; TASK="$1"; shift ;;
        --model)           shift; MODEL_PATH="$1"; shift ;;
        --webcam)          shift; WEBCAM="$1"; shift ;;
        --wrist)           shift; WRIST="$1"; shift ;;
        --device)          shift; DEVICE="$1"; shift ;;
        --max-delta)       shift; MAX_DELTA="$1"; shift ;;
        --duration-scale)  shift; DURATION_SCALE="$1"; shift ;;
        --dry-run)         EXTRA_ARGS+=("--dry-run"); shift ;;
        --help|-h)
            sed -n '2,28p' "$0" | grep '^#' | sed 's/^# \?//'
            exit 0 ;;
        *)                 EXTRA_ARGS+=("$1"); shift ;;
    esac
done

# ---- ROS 2 setup ----
set +u
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

# ---- resolve model path ----
# If it looks like a HuggingFace Hub ID (contains / but not an absolute path),
# pass it through directly — from_pretrained will download it.
# Otherwise, resolve relative paths from LEROBOT_DIR.
IS_HUB_ID=false
if [[ "$MODEL_PATH" == */* && ! "$MODEL_PATH" == /* ]]; then
    # Could be hub ID (org/model) or relative path — check if local dir exists
    if [[ -d "$LEROBOT_DIR/$MODEL_PATH" ]]; then
        MODEL_PATH="$LEROBOT_DIR/$MODEL_PATH"
    elif [[ -d "$MODEL_PATH" ]]; then
        MODEL_PATH="$(realpath "$MODEL_PATH")"
    else
        IS_HUB_ID=true
        echo "  (Treating '$MODEL_PATH' as HuggingFace Hub ID)"
    fi
elif [[ ! "$MODEL_PATH" == /* && -d "$LEROBOT_DIR/$MODEL_PATH" ]]; then
    MODEL_PATH="$LEROBOT_DIR/$MODEL_PATH"
fi

if [[ "$IS_HUB_ID" == false && ! -d "$MODEL_PATH" ]]; then
    echo "ERROR: Model directory not found: $MODEL_PATH"
    echo "  Use a HuggingFace Hub ID:  MODEL_PATH=lerobot/smolvla_base"
    echo "  Or finetune first:         ./run_smolvla_finetune.sh"
    exit 1
fi

# ---- resolve conda Python ----
CONDA_PYTHON="$HOME/miniconda3/envs/$CONDA_ENV/bin/python"
if [[ ! -x "$CONDA_PYTHON" ]]; then
    echo "WARNING: Conda env '$CONDA_ENV' not found at $CONDA_PYTHON"
    echo "  Falling back to system python3"
    CONDA_PYTHON=python3
fi

# ---- build camera args ----
CAM_ARGS=""
if [[ -n "$WEBCAM" ]]; then
    CAM_ARGS="$CAM_ARGS --webcam $WEBCAM"
fi
if [[ -n "$WRIST" ]]; then
    CAM_ARGS="$CAM_ARGS --wrist $WRIST"
fi

# ---- launch ----
echo "============================================"
echo "  SmolVLA Inference → UR5 Position Control"
echo "============================================"
echo "  Model     : $MODEL_PATH"
echo "  Task      : $TASK"
echo "  Device    : $DEVICE"
echo "  Webcam    : ${WEBCAM:-none}"
echo "  Wrist cam : ${WRIST:-none}"
echo "  Inference : ${INFERENCE_FPS} Hz"
echo "  Max delta : ${MAX_DELTA} rad/step"
echo "============================================"
echo ""

exec "$CONDA_PYTHON" "$SCRIPT_DIR/smolvla_inference_node.py" \
    --model "$MODEL_PATH" \
    --task "$TASK" \
    --device "$DEVICE" \
    $CAM_ARGS \
    --image-size "$IMAGE_H" "$IMAGE_W" \
    --inference-fps "$INFERENCE_FPS" \
    --duration-scale "$DURATION_SCALE" \
    --max-delta "$MAX_DELTA" \
    "${EXTRA_ARGS[@]}"
