# Synthetic Dataset Generation Task

## Objective
Create a set of scripts to generate a synthetic dataset for the UR5 robot. The dataset should be recorded in real-time (1x speed) while the robot performs a sequence of movements driven by `curobo`.

## Context
*   **Control Logic**: The robot is currently controlled by `src/ur5_curobo_control/ur5_curobo_control/curobo_control_node.py`, which uses NVIDIA's `curobo` library to generate trajectories between keypoints.
*   **Target Format**: The dataset must match the **LeRobot** format used by the `Isaac-GR00T` project.
    *   Reference Dataset: `Isaac-GR00T/demo_data/robot_sim.PickNPlace`
    *   Analysis Tool: `Isaac-GR00T/scripts/analyze_dataset.py`

## Requirements

### 1. Non-Destructive Implementation
*   **Do NOT modify** the existing `run_curobo.sh` or `curobo_control_node.py`.
*   Create **NEW** scripts (e.g., `src/ur5_curobo_control/ur5_curobo_control/dataset_generator_node.py` and `generate_dataset.sh`).

### 2. Dataset Generator Node
Create a new ROS 2 node (Python) that performs the following:
*   **Motion Generation**: Replicate the logic from `curobo_control_node.py` to move the robot between defined keypoints.
*   **Data Recording**:
    *   **Frequency**: Record at a fixed rate (e.g., 10Hz or 30Hz).
    *   **Images**: Capture the current viewport.
        *   *Note*: Since we are using RViz/ROS, check if an image topic (e.g., `/camera/image_raw`) is available. If not, use a screen capture library (like `mss` or `pyautogui`) to grab the RViz window region, or create a dummy image generator if strictly testing logic.
    *   **State**: Record joint positions (7 DOF: 6 joints + gripper) from `/joint_states`.
    *   **Action**: Record the target joint velocities or positions.
    *   **Language Instruction**: Save a text string corresponding to the current goal (e.g., "Move to Keypoint 1", "Move to Keypoint 2").
*   **Output Format**:
    *   Save the data as **Parquet** files (`data/train-00000-of-00001.parquet`).
    *   Generate the metadata files (`meta/info.json`, `meta/episodes.jsonl`).
    *   Ensure the schema matches the reference dataset (check `analyze_dataset.py` output for shapes and data types).

### 3. Execution Script
Create a shell script `generate_dataset.sh` that:
*   Sets up the environment (similar to `run_curobo.sh`).
*   Launches the new generator node.

## Instructions for the Agent
1.  **Analyze the Target**: Run `python3 Isaac-GR00T/scripts/analyze_dataset.py --dataset-path Isaac-GR00T/demo_data/robot_sim.PickNPlace` to understand the exact column names, shapes, and metadata structure required.
2.  **Develop the Node**: Write the `dataset_generator_node.py`. Ensure it handles the `curobo` imports correctly (refer to `curobo_control_node.py` for the `sys.path` setup).
3.  **Handle Images**: Implement a robust way to get images. If using screen capture, allow the user to define the screen region.
4.  **Verify**: The generated dataset should be readable by `analyze_dataset.py`.
