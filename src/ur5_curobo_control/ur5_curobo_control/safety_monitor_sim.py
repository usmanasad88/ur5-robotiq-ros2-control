#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import sys
import termios
import tty
import select

class SafetyMonitorSim(Node):
    def __init__(self):
        super().__init__('safety_monitor_sim')
        self.publisher_ = self.create_publisher(Bool, '/human_safety', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.safety_active = False
        self.print_instructions()

    def print_instructions(self):
        print("\nSafety Monitor Simulator")
        print("------------------------")
        print("Press 's' to TRIGGER safety stop (Human Detected)")
        print("Press 'r' to RESET safety (Human Gone)")
        print("Press 'q' to Quit")
        print("Current State: " + ("SAFE" if not self.safety_active else "DANGER - STOP"))

    def timer_callback(self):
        msg = Bool()
        msg.data = self.safety_active
        self.publisher_.publish(msg)
        
        if self.is_data():
            c = sys.stdin.read(1)
            if c == 's':
                self.safety_active = True
                print("\n!!! SAFETY TRIGGERED !!!")
            elif c == 'r':
                self.safety_active = False
                print("\n... Safety Reset ...")
            elif c == 'q':
                rclpy.shutdown()
                exit()

    def is_data(self):
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def main(args=None):
    # Settings for non-blocking input
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        rclpy.init(args=args)
        node = SafetyMonitorSim()
        rclpy.spin(node)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main()
