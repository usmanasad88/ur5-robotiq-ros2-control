#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, select, termios, tty

msg = """
Control UR5 arm using keyboard (MoveIt Servo):
---------------------------------------------
w/s : move in z-axis
a/d : move in x-axis
q/e : move in y-axis

i/k : rotate around x
j/l : rotate around y
u/o : rotate around z

CTRL-C to quit
"""

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = rlist[0].read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)

class UR5Teleop(Node):
    def __init__(self):
        super().__init__('ur5_teleop')
        self.publisher = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_twist = TwistStamped()

    def timer_callback(self):
        key = get_key()
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()

        if key == 'w':
            twist.twist.linear.z = 0.1
        elif key == 's':
            twist.twist.linear.z = -0.1
        elif key == 'a':
            twist.twist.linear.x = -0.1
        elif key == 'd':
            twist.twist.linear.x = 0.1
        elif key == 'q':
            twist.twist.linear.y = 0.1
        elif key == 'e':
            twist.twist.linear.y = -0.1
        elif key == 'i':
            twist.twist.angular.x = 0.1
        elif key == 'k':
            twist.twist.angular.x = -0.1
        elif key == 'j':
            twist.twist.angular.y = 0.1
        elif key == 'l':
            twist.twist.angular.y = -0.1
        elif key == 'u':
            twist.twist.angular.z = 0.1
        elif key == 'o':
            twist.twist.angular.z = -0.1
        elif key == '\x03':
            rclpy.shutdown()
            return

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = UR5Teleop()
    print(msg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
