# Copilot Instructions for `ur5-robotiq-ros2-control`

These rules are for AI coding agents working on this workspace (ROS 2 UR5/UR10 + Isaac Sim standalone scripts + cuRobo).

## High‑level architecture

- **ROS 2 workspace (`/home/mani/Repos/ur_ws`)**: Standard `colcon` layout under `src/`, plus teleop, gripper integration, cuRobo control, and documentation.
- **Isaac Sim standalone scripts (`isaac_standalone/`)**: Python apps that launch Isaac Sim 5.0.0 standalone to simulate UR5/UR10, with HTTP control and ROS 2 URDF import.
- **Isaac Sim install (`/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64`)**: Vendor drop; treat as read‑only. Only reference its `python.sh`, `standalone_examples/`, and `curobo/` APIs, don’t modify core files.
- **cuRobo integration**: Motion generation for UR5 via separate environment, triggered from helper scripts like `run_curobo.sh`.

## Environment & workflows

- **ROS 2 build & source**
  - Build: `colcon build --symlink-install` from workspace root.
  - Source for new terminals:
    ```bash
    source /home/mani/Repos/ur_ws/install/setup.bash
    ```
- **Isaac Sim standalone (UR scripts)**
  - Use the helper script from `isaac_standalone/` (preferred over calling `python.sh` directly):
    ```bash
    cd /home/mani/Repos/ur_ws/isaac_standalone
    ./run_ur_robot.sh --robot ur10 --mode animated
    ```
  - HTTP control workflow (two terminals): see `isaac_standalone/HTTP_QUICKSTART.txt` and `HTTP_CONTROL.md`.
- **Isaac Sim environment helpers**
  - For generic Isaac Sim apps in the vendor tree, there is a VS Code task `setup_python_env` (see `.vscode/tasks.json`) which runs `setup_python_env.sh` and exports env vars.
- **cuRobo motion generation**
  - Use `./run_curobo.sh` from workspace root; it handles conda env, `LD_PRELOAD`, and Isaac Sim Python patching (see `README_HOME.md`).

## Project‑specific code patterns

- **Isaac Sim scripts (`isaac_standalone/*.py`)**
  - Always create `SimulationApp` *before* most Isaac imports (see `isaac_standalone/README.md`):
    ```python
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": False})
    # Only then import World/Robot, etc.
    ```
  - Scripts follow a consistent structure: `argparse` → `SimulationApp` → `World`/robot setup → main loop `while simulation_app.is_running(): world.step(render=...)`.
  - Use existing examples as templates: `simple_test.py`, `ur10_import.py`, `ur_robot_advanced.py`, `ur5_http_control.py`.
  - For HTTP workflows, mirror request/JSON formats defined in `HTTP_CONTROL.md` and `test_flask_server.py` instead of inventing new schemas.

- **ROS 2 nodes/packages (`src/`)**
  - Use standard ROS 2 Humble layout and `rclcpp`/`rclpy` conventions already present in `src/` packages (e.g. teleop nodes, `ur5_gen_controller`).
  - Prefer updating existing node parameters and launch files over creating ad‑hoc scripts; follow argument patterns from `README_HOME.md` (e.g. `use_fake_hardware`, `use_http_server`, `action_server_url`).

- **UR + Robotiq integration**
  - Respect the existing integration pipeline driven by `launch_ur5_robotiq.sh` and `ur5_robotiq_description` launch/URDF files.
  - When adding frames or links, update appropriate `ur5_robotiq_description` xacro/URDF and MoveIt SRDF, not Isaac Sim assets.

## Files and docs to consult before big changes

- **Workspace‑level usage & commands**: `README_HOME.md`.
- **Isaac Sim standalone apps**: `isaac_standalone/START_HERE.md`, `isaac_standalone/README.md`, `isaac_standalone/DOCUMENTATION_INDEX.txt`, `isaac_standalone/QUICKSTART.md`.
- **HTTP control architecture & API**: `isaac_standalone/HTTP_CONTROL.md`, `HTTP_QUICKSTART.txt`, `SYSTEM_DIAGRAM.txt`, `HTTP_FEATURE_SUMMARY.txt`.
- **Workspace environment & obstacles**: `WORKSPACE_ENVIRONMENT.md`, `src/ur5_workspace_description` (ROS planning scene & RViz markers).

## Conventions & constraints

- Treat `/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64` as **external dependency**. Do not edit files in `exts/`, `curobo/`, or `kit/`; only read and reuse patterns.
- Keep Isaac Sim scripts self‑contained and runnable via `python.sh` or the existing shell helpers; avoid adding hardcoded absolute paths beyond what the docs already use.
- Follow existing CLI patterns: short `argparse` flags, `--headless`, `--test`, `--record`, and ROS 2 `--ros-args -p name:=value` style parameters.
- Prefer reusing and extending current controllers (e.g. `ur_robot_advanced.py`, `ur5_http_control.py`, `ur5_gen_controller`) over starting entirely new control stacks.
- When adding new docs, link them from `isaac_standalone/DOCUMENTATION_INDEX.txt` if they relate to Isaac Sim UR scripts.

## How agents should validate changes

- For ROS 2 code: run targeted packages or launch files described in `README_HOME.md` rather than whole‑workspace tests when possible.
- For Isaac Sim scripts: run via helper scripts (`run_ur_robot.sh`, `run_ur5_http.sh`) or `python.sh` in `isaac_standalone/` with `--test` or short frame counts.
- For cuRobo‑related changes: use `./run_curobo.sh` and keep iterations/trajectory counts low for quick checks.

If something seems to conflict with these patterns, prefer matching existing scripts and READMEs, and note the deviation in a short comment or README update.