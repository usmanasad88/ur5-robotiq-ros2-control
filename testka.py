#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def close_gripper():
    rospy.init_node('ur5_gripper_control')
    
    # Publisher for gripper command
    gripper_pub = rospy.Publisher('/gripper_cmd', Float64, queue_size=10)
    
    rospy.sleep(1)  # Wait for connection
    
    # Close gripper (value between 0.0 to 1.0, where 1.0 = fully closed)
    gripper_pub.publish(Float64(1.0))
    
    print("Gripper closing command sent")
    rospy.sleep(2)  # Wait for gripper to close

if __name__ == '__main__':
    close_gripper()
    from robotiq_2f_gripper_control.msg import Robotiq2FGripperRobotOutput

def close_gripper_robotiq():
    rospy.init_node('gripper_close')
    pub = rospy.Publisher('Robotiq2FGripperRobotOutData', Robotiq2FGripperRobotOutput, queue_size=10)
    
    rospy.sleep(1)
    
    # Create gripper command
    cmd = Robotiq2FGripperRobotOutput()
    cmd.rACT = 1      # Activate gripper
    cmd.rGTO = 1      # Go to position
    cmd.rPR = 255     # Position (0-255, 255 = fully closed)
    cmd.rSP = 150     # Speed
    cmd.rFR = 150     # Force
    
    pub.publish(cmd)
    print("Gripper close command sent")
    