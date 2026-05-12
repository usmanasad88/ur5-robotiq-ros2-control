# UR5 Control Workspace for AURA

ROS 2 workspace and tooling that backs the robot side of the AURA framework
described in *"Modular framework for responsive and explainable robotic
assistance with intention prediction using human-centric digital twins"*
(Asad et al., 2026). It implements the multi-modal control infrastructure of
the paper:

- a `.prog` DSL with a cuRobo-planned Program Executor,
- VR (Meta Quest) and SpaceMouse teleoperation,
- a Flask REST API used by AURA's decision engine,
- a Streamlit operator UI and an Episode Recorder,
- an Isaac Sim digital twin via the [`ur_robotiq_cortex`](isaac_sim_extension/ur_robotiq_cortex/)
  extension.

The agentic framework that consumes this stack lives in the companion
[aura](../aura) repository.

---

## 1. Install

Tested on Ubuntu 22.04 + ROS 2 Humble + NVIDIA GPU.

```bash
# ROS 2 Humble (one-time, system-wide)
sudo apt install ros-humble-desktop ros-humble-ur python3-colcon-common-extensions tmux

# Workspace dependencies
cd ~/Repos/ur_ws
rosdep install --from-paths src --ignore-src -r -y
pip install -r requirements.txt          # cuRobo, pyspacemouse, flask, streamlit, ...

# Build
colcon build --symlink-install
source install/setup.bash
```

Add to `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
source ~/Repos/ur_ws/install/setup.bash
```

cuRobo must be installed against the same Python ROS 2 uses; see the
[cuRobo install guide](https://curobo.org/get_started/1_install_instructions.html).

---

## 3. Launching the control stack — `launch_all.sh`

The single entry point starts the UR driver, gripper adapter, cuRobo planner,
Program Executor, REST API, and Streamlit UI, each in its own tmux window,
then attaches a terminal to the session.

```bash
./launch_all.sh                 # all services, fake hardware (simulation)
./launch_all.sh --real          # real UR5 + Robotiq gripper adapter
./launch_all.sh --no-ui         # skip Streamlit UI

# Add a teleop modality (sim only):
./launch_all.sh --quest-servo   # Quest 3S over Jacobian-IK velocity control
./launch_all.sh --spacemouse    # SpaceMouse over Jacobian-IK velocity control
./launch_all.sh --quest         # Quest via cuRobo (PoseDelta, less smooth)
```

Useful tmux keys: `Ctrl-b n/p` to switch windows, `Ctrl-b d` to detach.
Kill the whole stack with `tmux kill-session -t ur5`.

The robot IP is hard-coded to `192.168.0.105` in `launch_all.sh`; edit that
line for a different network.

### What the REST API exposes

`run_external_api.sh` (window **API**) serves on
`http://localhost:5050`. AURA's `run_aura.py --live --robot-url
http://localhost:5050` dispatches `EXECUTE_PROGRAM`, `MOVE_TO_POSE`, and
gripper primitives through this endpoint.

---

## 4. Teleoperation

### Quest 3S (VR)

For **simulation** (sourced from `launch_all.sh --quest-servo`):

```bash
# one-time
sudo pip3 install git+https://github.com/rail-berkeley/oculus_reader.git
sudo apt install android-tools-adb

# Quest 3S: enable Developer Mode, connect USB-C, accept ADB prompt
adb devices            # confirm device visible
```

For the **real robot**, bypass `launch_all.sh` and use the RTDE driver:

```bash
bash run_quest_rtde_teleop.sh
```

Trigger = gripper proportional, grip button = deadman, A/B buttons save the
current configuration as a named position (used by `.prog` files).

### SpaceMouse

```bash
pip install pyspacemouse
# Allow non-root access
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="256f", MODE="0666"' | \
    sudo tee /etc/udev/rules.d/90-spacemouse.rules
sudo udevadm control --reload && sudo udevadm trigger
```

Then:

```bash
./launch_all.sh --spacemouse           # sim, with full stack
# or, standalone (assumes launch_all.sh is already running):
bash run_spacemouse_teleop.sh --swap
```

`run_spacemouse_hand_teleop.sh` drives the simulated *human hand* in the
Isaac Sim extension (publishes `/hand/cmd_pose`) — used for collecting
proactive-assistance ground truth in the digital twin.

---

## 5. Isaac Sim digital twin — `ur_robotiq_cortex` extension

A portable Isaac Sim 5.0 extension lives in
[isaac_sim_extension/ur_robotiq_cortex/](isaac_sim_extension/ur_robotiq_cortex/).
It loads the UR5/UR10 + Robotiq USD and supports two behaviours:

| Behaviour | Description |
|---|---|
| **ROS 2 Follower** | Mirrors `/joint_states` from the real / simulated UR5 into Isaac Sim each physics step. |
| **VLA Control**    | Executes Cartesian actions from a VLA inference server via HTTP/ZMQ. |

### One-time setup

```bash
cd isaac_sim_extension
./setup_isaac_extension.sh                                    # default Isaac path
./setup_isaac_extension.sh --isaac-sim-path /opt/isaac-sim    # custom path
source ~/.bashrc                                              # picks up UR_WS_PATH
```

The script symlinks the extension into `<isaac-sim>/extsUser/` and exports
`UR_WS_PATH` so the extension can locate the USD assets at
`isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/`.

### Run

```bash
# Robot side
./launch_all.sh                     # provides /joint_states (fake or real)

# Isaac Sim — must inherit a ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/Repos/ur_ws/install/setup.bash
~/isaac-sim-standalone-5.0.0-linux-x86_64/isaac-sim.sh
```

In Isaac Sim: **Examples → Cortex → UR Robotiq Cortex Examples**, pick
*ROS 2 Follower* or *VLA Control*, then **Load → Start**.

