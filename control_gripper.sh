#!/bin/bash

# UR5 Robotiq Gripper Control Script

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=== UR5 Robotiq Gripper Control ===${NC}"

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Sourcing ROS setup...${NC}"
    source /opt/ros/*/setup.bash
fi

# Create Python script
PYTHON_SCRIPT="/tmp/gripper_control.py"

cat > $PYTHON_SCRIPT << 'EOF'
#!/usr/bin/env python3
import rospy
from robotiq_2f_gripper_control.msg import Robotiq2FGripperRobotOutput
import sys

class GripperController:
    def __init__(self):
        rospy.init_node('gripper_controller')
        self.pub = rospy.Publisher('/Robotiq2FGripperRobotOutData', 
                                   Robotiq2FGripperRobotOutput, queue_size=10)
        rospy.sleep(0.5)
    
    def move_gripper(self, position, speed=150, force=150):
        """Move gripper to specific position (0-255)"""
        cmd = Robotiq2FGripperRobotOutput()
        cmd.rACT = 1      # Activate
        cmd.rGTO = 1      # Go to position
        cmd.rPR = int(position)
        cmd.rSP = speed
        cmd.rFR = force
        
        self.pub.publish(cmd)
        rospy.sleep(0.5)
    
    def open_gripper(self):
        self.move_gripper(0)
    
    def close_gripper(self):
        self.move_gripper(255)
    
    def half_open(self):
        self.move_gripper(128)

if __name__ == '__main__':
    try:
        gripper = GripperController()
        
        if len(sys.argv) > 1:
            cmd = sys.argv[1].lower()
            if cmd == "open":
                print("[INFO] Opening gripper...")
                gripper.open_gripper()
                print("[✓] Gripper opened")
            elif cmd == "close":
                print("[INFO] Closing gripper...")
                gripper.close_gripper()
                print("[✓] Gripper closed")
            elif cmd == "half":
                print("[INFO] Half opening gripper...")
                gripper.half_open()
                print("[✓] Gripper half open")
            elif cmd.isdigit():
                pos = int(cmd)
                print(f"[INFO] Moving to position {pos}...")
                gripper.move_gripper(pos)
                print(f"[✓] Position set to {pos}")
            else:
                print("Usage: gripper_control.py [open|close|half|0-255]")
        else:
            print("[INFO] Running test sequence...")
            gripper.open_gripper()
            print("[✓] Open")
            rospy.sleep(1)
            
            gripper.half_open()
            print("[✓] Half")
            rospy.sleep(1)
            
            gripper.close_gripper()
            print("[✓] Close")
            
    except rospy.ROSInterruptException:
        print("[✗] Interrupted")
EOF

# Run the Python script
echo -e "${GREEN}Running gripper control...${NC}"
python3 $PYTHON_SCRIPT "$@"

echo -e "${GREEN}Done!${NC}"
