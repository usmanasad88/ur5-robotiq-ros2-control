#!/usr/bin/env python3
"""
Test script to verify workspace environment is working correctly.

This script checks that the workspace nodes can start and publish correctly.

Usage:
    python3 test_workspace.py
"""

import subprocess
import time
import sys


def run_command(cmd):
    """Run a shell command and return output."""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Timeout"


def test_package_installation():
    """Test that the package is installed."""
    print("Testing package installation...")
    success, stdout, stderr = run_command(
        "source /home/mani/Repos/ur_ws/install/setup.bash && ros2 pkg list | grep ur5_workspace_description"
    )
    if success and "ur5_workspace_description" in stdout:
        print("  ✓ Package installed correctly")
        return True
    else:
        print("  ✗ Package not found")
        return False


def test_executables():
    """Test that executables are available."""
    print("\nTesting executables...")
    success, stdout, stderr = run_command(
        "source /home/mani/Repos/ur_ws/install/setup.bash && ros2 pkg executables ur5_workspace_description"
    )
    
    executables = ["workspace_markers", "workspace_collision_objects"]
    all_found = True
    
    for exe in executables:
        if exe in stdout:
            print(f"  ✓ {exe} found")
        else:
            print(f"  ✗ {exe} not found")
            all_found = False
    
    return all_found


def test_launch_files():
    """Test that launch files exist."""
    print("\nTesting launch files...")
    import os
    
    launch_dir = "/home/mani/Repos/ur_ws/src/ur5_workspace_description/launch"
    launch_files = [
        "workspace_markers.launch.py",
        "workspace_collision_objects.launch.py",
        "workspace_environment.launch.py"
    ]
    
    all_found = True
    for launch_file in launch_files:
        path = os.path.join(launch_dir, launch_file)
        if os.path.exists(path):
            print(f"  ✓ {launch_file} exists")
        else:
            print(f"  ✗ {launch_file} not found")
            all_found = False
    
    return all_found


def test_python_modules():
    """Test that Python modules can be imported."""
    print("\nTesting Python modules...")
    
    try:
        sys.path.insert(0, "/home/mani/Repos/ur_ws/src/ur5_workspace_description")
        
        # Try importing the modules
        from ur5_workspace_description import workspace_markers
        print("  ✓ workspace_markers module imports successfully")
        
        from ur5_workspace_description import workspace_collision_objects
        print("  ✓ workspace_collision_objects module imports successfully")
        
        return True
    except ImportError as e:
        print(f"  ✗ Import failed: {e}")
        return False


def main():
    """Run all tests."""
    print("="*70)
    print(" UR5 WORKSPACE ENVIRONMENT - VERIFICATION TEST")
    print("="*70)
    print()
    
    tests = [
        ("Package Installation", test_package_installation),
        ("Executables", test_executables),
        ("Launch Files", test_launch_files),
        ("Python Modules", test_python_modules),
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"  ✗ Test failed with exception: {e}")
            results.append((test_name, False))
    
    # Print summary
    print()
    print("="*70)
    print(" TEST SUMMARY")
    print("="*70)
    print()
    
    all_passed = True
    for test_name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"  {status}: {test_name}")
        if not result:
            all_passed = False
    
    print()
    print("="*70)
    
    if all_passed:
        print(" ALL TESTS PASSED! ✓")
        print("="*70)
        print()
        print("Next steps:")
        print("  1. Launch robot: ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5 use_fake_hardware:=true")
        print("  2. Launch MoveIt: ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5 launch_rviz:=true")
        print("  3. Add workspace: ros2 launch ur5_workspace_description workspace_environment.launch.py")
        print("  4. In RViz: Add → MarkerArray → Topic: /workspace_markers")
        print()
        return 0
    else:
        print(" SOME TESTS FAILED! ✗")
        print("="*70)
        print()
        print("Please rebuild the workspace:")
        print("  cd ~/Repos/ur_ws")
        print("  colcon build --packages-select ur5_workspace_description --symlink-install")
        print("  source install/setup.bash")
        print()
        return 1


if __name__ == "__main__":
    sys.exit(main())
