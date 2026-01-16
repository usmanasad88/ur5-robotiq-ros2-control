# UR5 Program Control Guide

This guide explains the two ways to control program execution: **Keyboard/Presenter** and **Web UI**.

---

## Method 1: Keyboard / Logitech Presenter Control (Built-in)

The program executor node has **built-in presenter/keyboard control** that is **enabled by default**.

### Starting the Executor

```bash
./run_program_executor.sh [PROGRAM_FILE]
```

Example:
```bash
./run_program_executor.sh pick_and_place_object.prog
```

### Keyboard/Presenter Controls

The executor listens for the following keys (works with Logitech presenters):

| Button/Key | Action | Description |
|------------|--------|-------------|
| **Next / PageDown** | Start/Resume | Start or resume program execution |
| **Space / Right** | Start/Resume | Alternative keys for start/resume |
| **Prev / PageUp** | Record Pose | Record current robot pose (recording mode) |
| **Left / P** | Pause | Pause execution (alternative) |
| **S / ESC** | Stop | Stop program execution |

### How It Works

1. **Load a program:**
   ```bash
   ./run_program_executor.sh my_program.prog
   ```

2. **The executor will:**
   - Load the program
   - Wait for you to press **[Next]** or **[PageDown]**
   - Execute the program step-by-step

3. **During execution:**
   - Press **[Next]** again to resume if paused
   - Press **[S]** to stop

### Recording Mode

You can record robot poses using the presenter:

1. Start executor without a program:
   ```bash
   ./run_program_executor.sh
   ```

2. Move the robot to desired pose (manually or via freedrive)

3. Press **[Prev]** to record the current pose

4. Repeat for each waypoint

5. Press **[s]** to save the recording to `programs/recorded_YYYYMMDD_HHMMSS.prog`

---

## Method 2: Web UI (Streamlit)

For a more visual interface, use the Streamlit web UI.

### Starting the Web UI

**Terminal 1 - Start Program Executor:**
```bash
./run_program_executor.sh
```

**Terminal 2 - Start Web UI:**
```bash
./run_program_ui.sh
```

The UI will open in your browser at `http://localhost:8501`

### Web UI Features

- **Program Grid:** Visual buttons for all available programs
- **One-Click Execution:** Load & execute programs with a single click
- **Status Display:** See current program and execution status
- **Control Panel:** Pause, resume, and stop buttons in the sidebar
- **Auto-Refresh:** Updates available programs automatically

### Using the Web UI

1. **Select a program** - Click any "▶️ Load & Execute" button
2. **Monitor status** - Check sidebar for execution status
3. **Control execution:**
   - Click "⏸️ Pause" to pause
   - Click "▶️ Resume" to continue
   - Click "🛑 Stop" to stop

---

## Comparison

| Feature | Keyboard/Presenter | Web UI |
|---------|-------------------|---------|
| **Setup** | None (built-in) | Requires Streamlit |
| **Visual Feedback** | Terminal logs only | Status indicators & colors |
| **Program Selection** | Command-line argument | Click buttons |
| **Recording** | ✅ Yes | ❌ No |
| **Remote Control** | ❌ Local only | ✅ Network access |
| **Best For** | Quick testing, demos | Multiple programs, remote control |

---

## Troubleshooting

### Presenter Not Working

If your Logitech presenter isn't detected:

1. **Install evdev:**
   ```bash
   pip install evdev
   ```

2. **Check device permissions:**
   ```bash
   sudo usermod -a -G input $USER
   # Log out and log back in
   ```

3. **List detected devices:**
   The executor logs show detected input devices:
   ```
   [INFO] Found input device: Logitech USB Receiver
   ```

### Web UI Shows "ROS 2 not available"

This means the Streamlit app can't connect to ROS 2. Make sure:

1. **Program executor is running** in another terminal
2. **ROS 2 is sourced** before starting the UI
3. The UI script should handle this automatically, but you can manually check:
   ```bash
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   python3 program_selector_ui.py
   ```

### Program Won't Start

Check that:
- Program file exists in `programs/` directory
- Robot is receiving joint states
- No safety triggers are active

---

## Advanced Usage

### Custom Programs

Place `.prog` files in:
- `src/ur5_curobo_control/programs/` (source)
- Or `install/ur5_curobo_control/share/ur5_curobo_control/programs/` (installed)

Program format:
```
# My Custom Program
movetopose 0.3 0.2 0.4 0 180 0
opengripper
wait 1.0
movetojoint 0.0 -1.57 1.57 -1.57 -1.57 0.0
closegripper
```

### Disable Presenter Control

To disable keyboard/presenter control:
```bash
./run_program_executor.sh --no-presenter my_program.prog
```

### Auto-Execute (No Waiting)

To execute immediately without waiting for [Next]:
```bash
./run_program_executor.sh --execute my_program.prog
```

---

## ROS 2 Services (Advanced)

You can also control the executor via ROS 2 services:

```bash
# List available programs
ros2 service call /ur5_program_executor/list_programs std_srvs/srv/Trigger

# Load a program (set parameter first)
ros2 param set /ur5_program_executor program_file pick_and_place_object.prog
ros2 service call /ur5_program_executor/load_program std_srvs/srv/Trigger

# Execute loaded program
ros2 service call /ur5_program_executor/execute_program std_srvs/srv/Trigger

# Pause/Resume
ros2 service call /ur5_program_executor/pause std_srvs/srv/SetBool "{data: true}"  # Pause
ros2 service call /ur5_program_executor/pause std_srvs/srv/SetBool "{data: false}" # Resume

# Stop
ros2 service call /ur5_program_executor/stop std_srvs/srv/Trigger
```

Or use the CLI helper:
```bash
ros2 run ur5_curobo_control program_cli list
ros2 run ur5_curobo_control program_cli load pick_and_place_object.prog
ros2 run ur5_curobo_control program_cli execute
```
