# Isaac Sim Standalone Scripts for UR Robots

This directory contains standalone Python scripts for simulating Universal Robots (UR5/UR10) in NVIDIA Isaac Sim.

## Prerequisites

- Isaac Sim installed at: `/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64`
- ROS 2 workspace with Universal Robots packages (optional, for URDF files)
- Python packages: `requests`, `flask` (for HTTP control)

## Scripts

### 1. `ur10_import.py` - Basic Robot Loader

Simple standalone script that loads and simulates a UR5 or UR10 robot.

**Features:**
- Load UR5 or UR10 from Isaac Sim assets
- Basic simulation with default joint positions
- Minimal example for getting started

**Usage:**
```bash
# Using the helper script (recommended)
./run_ur_robot.sh --robot ur10

# Or directly with Isaac Sim Python
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur10_import.py --robot ur5

# Headless mode
./run_ur_robot.sh --robot ur10 --headless

# Test mode (shorter simulation)
./run_ur_robot.sh --robot ur5 --test
```

**Arguments:**
- `--robot {ur5,ur10}`: Select robot model (default: ur10)
- `--headless`: Run without GUI
- `--test`: Run in test mode (shorter simulation)

### 2. `ur_robot_advanced.py` - Advanced Robot Controller

Enhanced script with multiple control modes, joint control, and recording capabilities.

**Features:**
- Multiple control modes: static, animated, interactive
- Joint position and velocity control
- CSV recording of joint states
- Custom URDF import support
- Predefined motion sequences

**Usage:**
```bash
# Static mode (robot at home position)
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur_robot_advanced.py --robot ur10 --mode static

# Animated mode (sinusoidal motion)
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur_robot_advanced.py --robot ur5 --mode animated

# Interactive mode (cycling through poses)
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur_robot_advanced.py --robot ur10 --mode interactive

# With recording
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur_robot_advanced.py --robot ur10 --mode animated --record

# Load custom URDF
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur_robot_advanced.py --robot ur5 --urdf /path/to/ur5.urdf

# Specify simulation duration
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur_robot_advanced.py --robot ur10 --frames 2000
```

**Arguments:**
- `--robot {ur5,ur10}`: Select robot model (default: ur10)
- `--urdf PATH`: Load from custom URDF file instead of Isaac Sim assets
- `--mode {static,animated,interactive}`: Control mode (default: static)
- `--headless`: Run without GUI
- `--record`: Record joint states to CSV file
- `--frames N`: Number of simulation frames (default: 1000)

### 3. `ur5_http_control.py` - HTTP-Based End-Effector Control ⭐ NEW

Standalone script that loads a UR5 robot and continuously reads target end-effector poses from an HTTP endpoint.

**Features:**
- Reads target poses from Flask server via HTTP GET requests
- Parses JSON format: `{"joint_actions": [x, y, z, roll, pitch, yaw, gripper], ...}`
- Configurable update rate (default: 10 Hz)
- Automatic connection retry on failures
- Status reporting every 2 seconds
- Visualizes target pose with coordinate frame marker

**Usage:**
```bash
# Start the test Flask server (in another terminal)
python3 test_flask_server.py --mode sine

# Run the HTTP control script
./run_ur5_http.sh

# With custom URL and rate
./run_ur5_http.sh --url http://localhost:5000/joint_actions --rate 20

# Headless mode
./run_ur5_http.sh --headless
```

**Arguments:**
- `--url URL`: Flask endpoint URL (default: http://localhost:5000/joint_actions)
- `--rate HZ`: Update rate in Hz (default: 10)
- `--headless`: Run without GUI
- `--timeout SEC`: Request timeout in seconds (default: 1.0)

**Test Flask Server:**
The included `test_flask_server.py` provides three motion modes for development:
- `static`: Fixed position (0.3, 0.0, 0.5)
- `sine`: Sinusoidal motion in X-axis
- `circle`: Circular motion in XY plane

See `HTTP_CONTROL.md` and `HTTP_QUICKSTART.txt` for detailed documentation.

### 4. `run_ur_robot.sh` - Helper Script

Bash script that sets up the environment and runs the basic loader script.

**Usage:**
```bash
./run_ur_robot.sh [ARGUMENTS]
```

All arguments are passed through to `ur10_import.py`.

## File Structure

```
isaac_standalone/
├── ur10_import.py           # Basic robot loader
├── ur_robot_advanced.py     # Advanced controller
├── run_ur_robot.sh          # Helper script
├── README.md                # This file
├── HTTP_CONTROL.md          # HTTP control documentation
├── HTTP_QUICKSTART.txt      # HTTP quick start guide
└── standalone_examples/     # Isaac Sim example scripts
```

## Examples

### Example 1: Quick Start with UR10
```bash
./run_ur_robot.sh --robot ur10
```

### Example 2: Animated UR5 with Recording
```bash
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur_robot_advanced.py \
    --robot ur5 \
    --mode animated \
    --record \
    --frames 1000
```

### Example 3: HTTP-Based End-Effector Control ⭐ NEW
```bash
# Terminal 1: Start test Flask server
python3 test_flask_server.py --mode sine --amplitude 0.1

# Terminal 2: Start Isaac Sim with HTTP control
./run_ur5_http.sh --url http://localhost:5000/joint_actions --rate 10

# The robot will continuously read target poses from the Flask endpoint
# See HTTP_CONTROL.md for full documentation
```

### Example 4: Load Custom URDF from ROS 2 Workspace
```bash
URDF_PATH="/home/mani/Repos/ur_ws/install/ur_description/share/ur_description/urdf/ur5/ur5.urdf"
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh ur_robot_advanced.py \
    --robot ur5 \
    --urdf $URDF_PATH \
    --mode interactive
```

## Understanding the Code

### Basic Structure

All standalone Isaac Sim applications follow this pattern:

```python
from isaacsim import SimulationApp

# Create SimulationApp FIRST (before other imports)
simulation_app = SimulationApp({"headless": False})

# Import Isaac Sim modules AFTER SimulationApp
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
# ... other imports

# Create world and add objects
my_world = World()
my_world.scene.add_default_ground_plane()

# Add robot
robot = Articulation(prim_path="/World/Robot")

# Simulation loop
while simulation_app.is_running():
    my_world.step(render=True)

simulation_app.close()
```

### Key Concepts

1. **SimulationApp**: Must be created before any other Isaac Sim imports
2. **World**: Main simulation container with physics
3. **Articulation**: Represents articulated robots with joints
4. **Stage**: USD scene graph containing all objects
5. **Prim Path**: Unique identifier for objects in the scene (e.g., "/World/Robot")

### Robot Models in Isaac Sim

Isaac Sim provides pre-made UR robot models at:
- UR5: `/Isaac/Robots/UniversalRobots/ur5/ur5.usd`
- UR10: `/Isaac/Robots/UniversalRobots/ur10/ur10.usd`

These models include:
- Accurate geometry and collision meshes
- Proper joint configurations
- Physics properties
- Optional gripper variants

## Troubleshooting

### Import Errors
If you see import errors like `Import "isaacsim" could not be resolved`, this is expected in VS Code. The modules are only available when running with Isaac Sim's Python interpreter.

### Isaac Sim Not Found
Update the path in `run_ur_robot.sh` to match your Isaac Sim installation:
```bash
ISAAC_SIM_PATH="/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64"
```

### Robot Not Loading
- Verify Isaac Sim assets are properly installed
- Check that the robot model exists in the assets folder
- For URDF import, ensure the URDF file path is correct

### Performance Issues
- Use `--headless` for faster simulation without rendering
- Reduce `--frames` for shorter simulations
- Close other GPU-intensive applications

## Debugging in VS Code

The repository includes a ready-to-use launch configuration for stepping through `ur5_http_control.py` without relying on the helper shell script.

1. Ensure the Isaac Sim Python environment has `debugpy` available (only needed once):

    ```bash
    /home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh -m pip install --user debugpy
    ```

2. Open the **Run and Debug** panel in VS Code and select **UR5 HTTP Control (Isaac Sim)**.
3. Adjust command-line arguments in `.vscode/launch.json` if you need different server URLs, endpoints, or flags such as `--headless`.
4. Press **F5** to start debugging. The session uses the same Isaac Sim interpreter and environment as `run_ur5_http.sh`, so breakpoints, variable inspection, and the integrated terminal all work as expected.

## Additional Resources

- [Isaac Sim Documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Manual Standalone Python](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/python_scripting/manual_standalone_python.html)
- [Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

## Notes

- The import errors shown in the IDE are expected - these modules are only available when running with Isaac Sim's Python
- Always create `SimulationApp` before importing other Isaac Sim modules
- The scripts are designed to work with Isaac Sim 5.0.0 but should be compatible with other versions
- Joint states are recorded at 6 Hz when using `--record` (every 10th frame at 60 Hz physics)

## License

SPDX-License-Identifier: Apache-2.0
