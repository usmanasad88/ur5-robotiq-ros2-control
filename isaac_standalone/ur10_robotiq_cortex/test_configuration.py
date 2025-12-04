#!/usr/bin/env python3
"""
Diagnostic test script for UR10 Robotiq Cortex example.
This script can be used to verify robot configuration and USD structure.
"""

import os
import sys

# Add the Isaac Sim path
isaac_sim_path = "/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64"
sys.path.insert(0, os.path.join(isaac_sim_path, "exts/isaacsim.core.api/isaacsim/core/api"))

def check_usd_file():
    """Check if the USD file exists and has the expected structure."""
    usd_path = "/home/mani/Repos/ur_ws/isaac_standalone/Collected_ur10e_robotiq2f-140_ROS/ur10e_robotiq2f-140_ROS.usd"
    
    print(f"\n=== Checking USD File ===")
    print(f"USD Path: {usd_path}")
    
    if not os.path.exists(usd_path):
        print(f"❌ USD file NOT FOUND at: {usd_path}")
        return False
    
    print(f"✓ USD file exists")
    
    # Check file size
    size = os.path.getsize(usd_path)
    print(f"✓ File size: {size} bytes")
    
    # Try to parse it as text (USD files are ASCII)
    try:
        with open(usd_path, 'r') as f:
            content = f.read(1000)  # Read first 1000 chars
            if 'def' in content or 'prim' in content:
                print(f"✓ USD file appears valid (contains USD structure)")
            else:
                print(f"⚠ USD file structure unclear")
    except Exception as e:
        print(f"⚠ Error reading USD file: {e}")
    
    return True


def check_behavior_file():
    """Check if the behavior file exists."""
    isaac_sim_path = "/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64"
    behavior_path = os.path.join(isaac_sim_path, "exts/isaacsim.cortex.behaviors/isaacsim/cortex/behaviors/ur10/bin_stacking_behavior.py")
    
    print(f"\n=== Checking Behavior File ===")
    print(f"Behavior Path: {behavior_path}")
    
    if not os.path.exists(behavior_path):
        print(f"❌ Behavior file NOT FOUND at: {behavior_path}")
        return False
    
    print(f"✓ Behavior file exists")
    
    # Check if it has make_decider_network function
    try:
        with open(behavior_path, 'r') as f:
            content = f.read()
            if 'def make_decider_network' in content:
                print(f"✓ Behavior file contains 'make_decider_network' function")
            else:
                print(f"❌ Behavior file missing 'make_decider_network' function")
                return False
    except Exception as e:
        print(f"⚠ Error reading behavior file: {e}")
    
    return True


def check_robot_configuration():
    """Check if robot configuration files exist."""
    isaac_sim_path = "/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64"
    robot_py = os.path.join(isaac_sim_path, "exts/isaacsim.cortex.framework/isaacsim/cortex/framework/robot.py")
    
    print(f"\n=== Checking Robot Configuration ===")
    print(f"Robot Config Path: {robot_py}")
    
    if not os.path.exists(robot_py):
        print(f"❌ Robot configuration file NOT FOUND")
        return False
    
    print(f"✓ Robot configuration file exists")
    
    # Check for CortexUr10Robotiq class
    try:
        with open(robot_py, 'r') as f:
            content = f.read()
            if 'class CortexUr10Robotiq' in content:
                print(f"✓ CortexUr10Robotiq class found")
            else:
                print(f"❌ CortexUr10Robotiq class NOT FOUND")
                return False
            
            if 'def add_ur10_robotiq_to_stage' in content:
                print(f"✓ add_ur10_robotiq_to_stage function found")
            else:
                print(f"❌ add_ur10_robotiq_to_stage function NOT FOUND")
                return False
                
            if 'class RobotiqGripper' in content:
                print(f"✓ RobotiqGripper class found")
            else:
                print(f"❌ RobotiqGripper class NOT FOUND")
                return False
    except Exception as e:
        print(f"⚠ Error reading robot configuration: {e}")
        return False
    
    return True


def main():
    print("=" * 60)
    print("UR10 Robotiq Cortex - Configuration Diagnostic")
    print("=" * 60)
    
    results = {
        "USD File": check_usd_file(),
        "Behavior File": check_behavior_file(),
        "Robot Configuration": check_robot_configuration(),
    }
    
    print(f"\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    for check, passed in results.items():
        status = "✓ PASS" if passed else "❌ FAIL"
        print(f"{check}: {status}")
    
    all_passed = all(results.values())
    
    if all_passed:
        print(f"\n✓ All checks passed! Configuration looks good.")
        print(f"\nNext steps:")
        print(f"1. Launch Isaac Sim with the UR10 Robotiq Cortex example")
        print(f"2. Click LOAD to load the world")
        print(f"3. Click START to begin the behavior")
        print(f"4. Check the console for diagnostic messages starting with [UR10 Robotiq]")
        print(f"5. Refer to DIAGNOSTIC_GUIDE.md for troubleshooting")
    else:
        print(f"\n❌ Some checks failed. Please fix the issues above before running the example.")
    
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
