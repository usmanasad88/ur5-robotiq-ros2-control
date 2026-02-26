# UR Robotiq Cortex – Isaac Sim Extension

A portable Isaac Sim extension that simulates a **UR5 or UR10 robot with a
Robotiq gripper** using the NVIDIA Cortex framework.

Two behaviors are provided:

| Behavior | Description |
|---|---|
| **ROS 2 Follower** | Subscribes to `/joint_states` from a real/simulated UR robot and mirrors the joints in Isaac Sim using Curobo motion planning. |
| **VLA Control** | Receives Cartesian action targets from a Vision-Language-Action model (e.g. GR00T, Octo) over HTTP or ZMQ. |

---

## Prerequisites

| Requirement | Notes |
|---|---|
| Ubuntu 22.04 | Tested platform |
| Isaac Sim 5.0.0 | Installed from [isaac-sim/IsaacSim](https://github.com/isaac-sim/IsaacSim) |
| ROS 2 Humble | Required for ROS 2 Follower behavior; source before launching Isaac Sim |
| [cuRobo](https://curobo.org/) | Motion planning library; used by ROS 2 Follower |
| ur_ws cloned | This extension lives inside `ur_ws` |

---

## Directory layout

```
ur_ws/
└── isaac_sim_extension/
    ├── README.md                        ← you are here
    ├── setup_isaac_extension.sh         ← run once per machine
    └── ur_robotiq_cortex/               ← Isaac Sim extension root
        ├── config/
        │   └── extension.toml
        └── ur_robotiq_cortex/           ← Python package
            ├── __init__.py
            ├── ur_robotiq_cortex.py          main sample logic
            ├── ur_robotiq_cortex_extension.py  UI / extension entry-point
            ├── ur_ros2_follower_behavior.py   ROS 2 joint-state follower
            └── ur_vla_behavior.py             VLA model interface
```

The extension depends on USD robot assets that are stored in the same repo:

```
ur_ws/
└── isaac_standalone/
    └── Collected_ur10e_robotiq2f-140_ROS/
        ├── ur5_robotiq2f-85.usd
        └── ur10e_robotiq2f-140_ROS.usd
```

---

## Installation (new machine)

### 1. Install Isaac Sim

Follow the official instructions at
<https://github.com/isaac-sim/IsaacSim>.

Default installation path used by the setup script:
```
~/isaac-sim-standalone-5.0.0-linux-x86_64
```

### 2. Clone ur_ws

```bash
git clone <your-ur_ws-repo-url> ~/Repos/ur_ws
cd ~/Repos/ur_ws
```

### 3. Run the setup script

```bash
cd ~/Repos/ur_ws/isaac_sim_extension
./setup_isaac_extension.sh
```

The script:
- Creates a symlink `<isaac-sim>/extsUser/ur_robotiq_cortex` → `ur_ws/isaac_sim_extension/ur_robotiq_cortex`. Isaac Sim scans `extsUser/` automatically on every launch — no other config changes needed.
- Exports `UR_WS_PATH` in your shell RC file so the extension can find the USD assets.

If Isaac Sim is installed at a non-default path:

```bash
./setup_isaac_extension.sh --isaac-sim-path /opt/nvidia/isaac-sim-5.0.0
```

### 4. Source your shell RC (or open a new terminal)

```bash
source ~/.bashrc   # or ~/.zshrc
```

---

## Usage

### ROS 2 Follower

1. Source ROS 2 and your `ur_ws` workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/Repos/ur_ws/install/setup.bash
   ```

2. Launch the UR driver (real robot or `ur_robot_driver` simulation):
   ```bash
   # Example for real UR5:
   ros2 launch ur_robot_driver ur_control.launch.py \
       ur_type:=ur5 robot_ip:=<ROBOT_IP>
   ```
   The robot will publish joint states on `/joint_states`.

3. Launch Isaac Sim (from a terminal where `UR_WS_PATH` is set and ROS 2 is
   sourced):
   ```bash
   ~/isaac-sim-standalone-5.0.0-linux-x86_64/isaac-sim.sh
   ```

4. In the Isaac Sim Examples Browser: **Cortex → UR Robotiq Cortex Examples**
   - Select **ROS 2 Follower** from the dropdown
   - Click **Load** → **Start**

Isaac Sim will now mirror the real robot's joint positions in real time.

### VLA Control

Configure the server URL and task description inside `ur_vla_behavior.py`
(search for `SERVER_URL` and `TASK_DESCRIPTION`), then:

1. Start your VLA inference server.
2. Launch Isaac Sim as above.
3. Select **VLA Control** from the dropdown, click **Load** → **Start**.

---

## Environment variables

| Variable | Default | Description |
|---|---|---|
| `UR_WS_PATH` | auto-detected | Absolute path to the `ur_ws` root. Set this if the setup script auto-detection fails. |
| `UR_ROBOT_TYPE` | `ur5` | Robot model to load: `ur5` or `ur10`. Set before launching Isaac Sim. |

Example:
```bash
export UR_WS_PATH=~/Repos/ur_ws
export UR_ROBOT_TYPE=ur10
~/isaac-sim-standalone-5.0.0-linux-x86_64/isaac-sim.sh
```

---

## Troubleshooting

### Extension not visible in Isaac Sim

Check that the symlink was created in `extsUser/`:

```bash
ls -la ~/isaac-sim-standalone-5.0.0-linux-x86_64/extsUser/
```

You should see:
```
ur_robotiq_cortex -> /home/<user>/Repos/ur_ws/isaac_sim_extension/ur_robotiq_cortex
```

If the symlink is missing, re-run the setup script.

### USD assets not found

Verify `UR_WS_PATH` is set and the USD files exist:
```bash
echo $UR_WS_PATH
ls $UR_WS_PATH/isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/
```

### ROS 2 not available in Isaac Sim

Isaac Sim must be launched from a terminal where ROS 2 is already sourced:
```bash
source /opt/ros/humble/setup.bash
source ~/Repos/ur_ws/install/setup.bash
~/isaac-sim-standalone-5.0.0-linux-x86_64/isaac-sim.sh
```

Do **not** use the desktop shortcut — it does not inherit the ROS 2 environment.

### Curobo not available

Curobo must be installed into Isaac Sim's Python environment.  From the
`ur_ws` root:
```bash
~/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh -m pip install curobo
```

Or follow the [cuRobo installation guide](https://curobo.org/get_started/1_install_instructions.html).
