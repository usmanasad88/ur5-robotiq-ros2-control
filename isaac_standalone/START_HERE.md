# Isaac Sim UR Robot Scripts - Complete Package

## 🎯 What You Got

A complete suite of standalone Isaac Sim applications for simulating UR5 and UR10 robots, with comprehensive documentation and testing tools.

## 📦 Files Created (15 total)

### Python Scripts (7 files)
1. **simple_test.py** (982 bytes) - Minimal example
2. **ur10_import.py** (4.9K) - Basic UR5/UR10 loader  
3. **ur_robot_advanced.py** (11K) - Advanced controller with modes
4. **ur_from_ros2_urdf.py** (6.8K) - ROS 2 workspace integration
5. **ur5_http_control.py** (15K) ⭐ NEW - HTTP-based end-effector control
6. **test_flask_server.py** (8K) - Test Flask server for development

### Shell Scripts (4 files)
7. **run_ur_robot.sh** (782 bytes) - Helper script for easy running
8. **run_ur5_http.sh** - Helper for HTTP control
9. **run_tests.sh** (7.4K) - Automated test suite
10. **setup_env.sh** (993 bytes) - Environment configuration

### Documentation (5 files)
11. **README.md** (6.6K) - Full documentation
12. **QUICKSTART.md** (4.4K) - Quick reference guide
13. **HTTP_CONTROL.md** ⭐ NEW - HTTP control documentation
14. **HTTP_QUICKSTART.txt** ⭐ NEW - HTTP quick start guide
15. **SUMMARY.txt** (8.7K) - Visual summary

**Total Size:** ~85KB of well-documented, production-ready code!

## 🚀 Quick Start Options

### Option 1: Basic Robot Loading
```bash
# Simple test (recommended for first-time users)
./run_ur_robot.sh --test

# Interactive UR10 with keyboard control
./run_ur_robot.sh --robot ur10 --mode interactive

# Animated demo with CSV recording
./run_ur_robot.sh --robot ur5 --mode animated --record
```

### Option 2: HTTP-Based Control ⭐ NEW
```bash
# Terminal 1: Start the test Flask server
python3 test_flask_server.py --mode sine

# Terminal 2: Start Isaac Sim with HTTP control
./run_ur5_http.sh

# The robot will read target poses from http://localhost:5000/joint_actions
# See HTTP_QUICKSTART.txt for details
```

### Option 3: ROS 2 Integration
```bash
# Load robot from your ROS 2 workspace
./run_ur_robot.sh --urdf
```

That's it! 🎉

## 📊 Feature Matrix

| Feature | simple_test | ur10_import | ur_robot_advanced | ur_from_ros2 |
|---------|-------------|-------------|-------------------|--------------|
| Load UR5 | ❌ | ✅ | ✅ | ✅ |
| Load UR10 | ✅ | ✅ | ✅ | ✅ |
| GUI Mode | ✅ | ✅ | ✅ | ✅ |
| Headless | ❌ | ✅ | ✅ | ✅ |
| Static Pose | ✅ | ✅ | ✅ | ✅ |
| Animation | ❌ | ❌ | ✅ | ✅ |
| Interactive | ❌ | ❌ | ✅ | ❌ |
| CSV Recording | ❌ | ❌ | ✅ | ❌ |
| Custom URDF | ❌ | ❌ | ✅ | ✅ |
| ROS 2 Integration | ❌ | ❌ | ❌ | ✅ |

## 🎓 Learning Path

```
1. simple_test.py
   └─> Verify Isaac Sim works
       └─> Takes 30 seconds
   
2. ur10_import.py --robot ur10
   └─> Learn basic robot loading
       └─> Takes 2 minutes
   
3. ur_robot_advanced.py --mode animated
   └─> Learn motion control
       └─> Takes 5 minutes
   
4. ur_robot_advanced.py --mode animated --record
   └─> Learn data recording
       └─> Creates CSV file
   
5. ur_from_ros2_urdf.py
   └─> Learn ROS 2 integration
       └─> Advanced usage
```

## 🎬 Usage Examples

### Example 1: Quick Test
```bash
./run_ur_robot.sh --robot ur10 --test
```
**What it does:** Loads UR10, runs for 100 frames, exits
**Time:** ~30 seconds

### Example 2: Animated with Recording
```bash
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh \
    ur_robot_advanced.py \
    --robot ur5 \
    --mode animated \
    --record \
    --frames 1000
```
**What it does:** 
- Loads UR5 robot
- Animates with sinusoidal motion
- Records joint states to CSV
- Runs for 1000 frames

**Output:** `ur5_recording_animated.csv`

### Example 3: ROS 2 Workspace URDF
```bash
/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/python.sh \
    ur_from_ros2_urdf.py \
    --robot ur5 \
    --workspace /home/mani/Repos/ur_ws
```
**What it does:**
- Finds UR5 URDF in your ROS 2 workspace
- Imports with proper physics
- Runs simple motion test

## 🔧 Configuration

All scripts use these default settings:
- **Physics Rate:** 60 Hz
- **Isaac Sim Path:** `/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64`
- **ROS 2 Workspace:** `/home/mani/Repos/ur_ws`
- **Recording Rate:** 6 Hz (every 10th frame)

To change, edit the respective script or pass command-line arguments.

## 📈 What Each Script Does Best

### simple_test.py ⭐
**Best for:** Verifying Isaac Sim installation
**Time:** 30 seconds
**Output:** Visual confirmation UR10 loads

### ur10_import.py ⭐
**Best for:** Learning Isaac Sim basics
**Time:** 2-5 minutes  
**Output:** Static robot visualization

### ur_robot_advanced.py ⭐⭐
**Best for:** Research, data collection, testing control
**Time:** Configurable (100-10000 frames)
**Output:** CSV files with joint trajectories

### ur_from_ros2_urdf.py ⭐⭐⭐
**Best for:** Using your own robot descriptions
**Time:** 2-5 minutes
**Output:** Robot from your URDF files

## 🐛 Common Issues & Solutions

### Issue: "Command not found"
```bash
# Solution: Make scripts executable
chmod +x *.sh
```

### Issue: Import errors in VS Code
```
Solution: This is NORMAL! Isaac Sim modules only work with 
Isaac Sim's Python. VS Code doesn't know about them.
```

### Issue: "Could not find Isaac Sim assets"
```bash
# Solution: Check path in scripts
echo $ISAAC_SIM_PATH
# Should output: /home/mani/isaac-sim-standalone-5.0.0-linux-x86_64
```

### Issue: Robot doesn't move
```bash
# Solution: Use motion modes
python.sh ur_robot_advanced.py --mode animated  # Not static
```

## 🎯 Next Steps

### Beginner
1. Run `./run_tests.sh` to verify everything works
2. Try `./run_ur_robot.sh --robot ur10`
3. Experiment with different robots: `--robot ur5`

### Intermediate  
1. Try animated mode: `--mode animated`
2. Record data: `--record --frames 1000`
3. Analyze CSV: `import pandas; df.plot()`

### Advanced
1. Load your own URDF files
2. Modify motion patterns in `ur_robot_advanced.py`
3. Add end-effector control (IK)
4. Implement pick-and-place tasks
5. Connect to ROS 2 for real-time control

## 📚 Documentation Hierarchy

```
QUICKSTART.md ←── Start here (Quick commands)
    ↓
README.md ←────── Full documentation
    ↓
Code files ←────── Detailed implementation
    ↓
Isaac Sim Docs ←── API reference
```

## 💡 Pro Tips

1. **Use test mode** for quick iterations: `--test`
2. **Use headless mode** for batch processing: `--headless`  
3. **Record data** for analysis: `--record`
4. **Check the CSVs** to verify joint states
5. **Source setup_env.sh** for aliases

## 🎪 Cool Things You Can Do

- ✅ Load UR5 and UR10 robots
- ✅ Control joint positions
- ✅ Record trajectories to CSV
- ✅ Import custom URDFs
- ✅ Run headless for automation
- ✅ Use with ROS 2 descriptions
- 🔜 Add grippers (modify scripts)
- 🔜 Implement IK (use Isaac Sim APIs)
- 🔜 Add cameras (use sensor APIs)
- 🔜 Connect to ROS 2 (use ros2 bridge)

## 🎓 Code Structure Pattern

All scripts follow this pattern:
```python
# 1. Parse arguments FIRST
parser = argparse.ArgumentParser()
args = parser.parse_args()

# 2. Create SimulationApp SECOND
from isaacsim import SimulationApp
app = SimulationApp({"headless": False})

# 3. Import Isaac modules THIRD
from isaacsim.core.api import World
# ... other imports

# 4. Main logic FOURTH  
world = World()
# ... add robot, run simulation

# 5. Cleanup LAST
app.close()
```

## 🎉 You're All Set!

Everything is configured and ready to use. Start with `simple_test.py` and work your way up!

**Questions?** Check README.md or QUICKSTART.md

**Need help?** Refer to Isaac Sim docs: https://docs.isaacsim.omniverse.nvidia.com/

Happy simulating! 🤖
