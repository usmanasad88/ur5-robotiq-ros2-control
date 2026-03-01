#!/bin/bash
# ============================================
# Finetune SmolVLA on UR5 dataset
# ============================================
# Uses lerobot-train to finetune the SmolVLA base model
# on a local LeRobot dataset created by convert_to_lerobot.py.
#
# Prerequisites:
#   pip install -e ".[smolvla]"   (in the lerobot repo)
#
# Usage:
#   ./run_smolvla_finetune.sh                              # defaults
#   REPO_ID=myuser/ur5_data ./run_smolvla_finetune.sh      # custom dataset
#   STEPS=50000 BATCH=16 ./run_smolvla_finetune.sh         # override training
# ============================================

set -e

# ---- Configuration (override via environment) -------------------------
LEROBOT_DIR="${LEROBOT_DIR:-$HOME/Repos/lerobot}"
REPO_ID="${REPO_ID:-local/ur5_programs}"
DATASET_ROOT="${DATASET_ROOT:-}"               # empty = default HF cache
STEPS="${STEPS:-20000}"
BATCH="${BATCH:-32}"
DEVICE="${DEVICE:-cuda}"
OUTPUT_DIR="${OUTPUT_DIR:-outputs/train/ur5_smolvla}"
JOB_NAME="${JOB_NAME:-ur5_smolvla_finetune}"
WANDB="${WANDB:-false}"

# PEFT / LoRA settings (recommended for small datasets)
USE_PEFT="${USE_PEFT:-true}"
PEFT_R="${PEFT_R:-64}"
LR="${LR:-1e-3}"
DECAY_LR="${DECAY_LR:-1e-4}"

# ---- Build the command ------------------------------------------------
cd "$LEROBOT_DIR"

CMD="lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=${REPO_ID} \
  --batch_size=${BATCH} \
  --steps=${STEPS} \
  --output_dir=${OUTPUT_DIR} \
  --job_name=${JOB_NAME} \
  --policy.device=${DEVICE} \
  --policy.input_features=null \
  --policy.output_features=null"

if [ -n "$DATASET_ROOT" ]; then
    CMD="$CMD --dataset.root=${DATASET_ROOT}"
fi

if [ "$WANDB" = "true" ]; then
    CMD="$CMD --wandb.enable=true"
fi

if [ "$USE_PEFT" = "true" ]; then
    CMD="$CMD \
      --peft.method_type=LORA \
      --peft.r=${PEFT_R} \
      --policy.optimizer_lr=${LR} \
      --policy.scheduler_decay_lr=${DECAY_LR}"
fi

echo "============================================"
echo "  SmolVLA Finetuning on UR5 Data"
echo "============================================"
echo "  Dataset  : ${REPO_ID}"
echo "  Steps    : ${STEPS}"
echo "  Batch    : ${BATCH}"
echo "  Device   : ${DEVICE}"
echo "  PEFT     : ${USE_PEFT} (r=${PEFT_R})"
echo "  Output   : ${OUTPUT_DIR}"
echo "============================================"
echo ""

eval $CMD
