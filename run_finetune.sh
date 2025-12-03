#!/bin/bash

# Setup Environment
source /home/mani/miniconda3/bin/activate gr00t
export PYTHONPATH=$PYTHONPATH:/home/mani/Repos/Isaac-GR00T:/home/mani/Repos/ur_ws

# Run Finetuning
python3 /home/mani/Repos/Isaac-GR00T/scripts/gr00t_finetune.py \
    --dataset_path /home/mani/Repos/ur_ws/generated_dataset \
    --data_config ur5_config:UR5DataConfig \
    --output_dir /home/mani/Repos/ur_ws/finetune_output \
    --max_steps 50 \
    --save_steps 25 \
    --batch_size 1 \
    --num_gpus 1 \
    --embodiment_tag new_embodiment
