#!/bin/bash

# This script launches the dataset generator node.

# 1. Setup Environment
source /home/mani/miniconda3/bin/activate ur5_python
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6

# 2. Define Paths
WORKSPACE_DIR=$(pwd)
CONFIG_DIR=$WORKSPACE_DIR/src/ur5_curobo_control/config
NODE_SCRIPT=$WORKSPACE_DIR/src/ur5_curobo_control/ur5_curobo_control/dataset_generator_node.py

# 3. Run the Node
# We run the python script directly to avoid rebuilding the package.
# We pass the config files as ROS parameters.
echo "Starting Dataset Generator..."
echo "Output Directory: $WORKSPACE_DIR/generated_dataset"

python3 $NODE_SCRIPT --ros-args \
    -p robot_config_file:=$CONFIG_DIR/ur5_existing.yml \
    -p world_config_file:=$CONFIG_DIR/collision_base.yml \
    -p output_dir:=$WORKSPACE_DIR/generated_dataset \
    -p episodes:=5

echo "Generator finished."
