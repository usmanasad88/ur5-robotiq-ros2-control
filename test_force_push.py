#!/usr/bin/env python3
"""
Test force-controlled push: down (-Z) + forward via movel.

Replicates the UR teach pendant pattern:
  Force node: Base frame, Z compliant, -60N, 150mm/s limit
  MoveL under Force: direction [0,-1,-1], distance 100mm

Uses URScript directly (like program_executor) so force_mode and
movel run in the same URScript context on the controller.

Usage:
    python3 test_force_push.py                          # defaults
    python3 test_force_push.py --force 60 --distance 0.1 --speed 0.05
"""

import math
import time
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ForcePushTest(Node):
    def __init__(self, force_n, speed_limit, distance, move_speed):
        super().__init__("force_push_test")
        self.force_n = force_n
        self.speed_limit = speed_limit
        self.distance = distance
        self.move_speed = move_speed

        # URScript interface - same as program_executor uses
        self._urscript_pub = self.create_publisher(
            String, "/urscript_interface/script_command", 1
        )

    def send_urscript(self, script):
        msg = String()
        msg.data = script
        self._urscript_pub.publish(msg)
        self.get_logger().info(f"Sent URScript ({len(script)} bytes)")

    def run(self):
        # Wait for publisher to connect
        time.sleep(1.0)

        # Direction vector [0, -1, -1] normalized
        dx, dy, dz = 0.0, -1.0, -1.0
        norm = math.sqrt(dx**2 + dy**2 + dz**2)
        dx, dy, dz = dx / norm, dy / norm, dz / norm

        # Compute offset from direction and distance
        ox = dx * self.distance
        oy = dy * self.distance
        oz = dz * self.distance

        # Estimate move duration
        move_duration = self.distance / self.move_speed + 1.0

        force = self.force_n
        sl = self.speed_limit

        print(f"\n  Force push test (URScript)")
        print(f"  Force: -{force}N in Z (downward)")
        print(f"  Speed limit on compliant axis: {sl*1000:.0f}mm/s")
        print(f"  MoveL direction: [0, -1, -1] (normalized)")
        print(f"  MoveL distance: {self.distance*1000:.0f}mm")
        print(f"  MoveL speed: {self.move_speed*1000:.0f}mm/s")
        print(f"  Estimated duration: {move_duration:.1f}s")
        print(f"\n  Press Ctrl+C at any time to stop.\n")

        # URScript program that runs entirely on the UR controller:
        # 1. Start force mode (Z compliant, -60N, 150mm/s limit)
        # 2. Wait for force to engage
        # 3. movel in direction [0,-1,-1] for the given distance
        # 4. Hold briefly
        # 5. End force mode
        urscript = (
            f"def force_push():\n"
            f"  force_mode(p[0,0,0,0,0,0], [0,0,1,0,0,0], "
            f"[0,0,{-force},0,0,0], 2, [{sl},{sl},{sl},{sl*4},{sl*4},{sl*4}])\n"
            f"  sleep(1.0)\n"
            f"  local tcp = get_actual_tcp_pose()\n"
            f"  local tgt = p[tcp[0]+({ox:.6f}), tcp[1]+({oy:.6f}), "
            f"tcp[2]+({oz:.6f}), tcp[3], tcp[4], tcp[5]]\n"
            f"  movel(tgt, a=0.3, v={self.move_speed:.4f})\n"
            f"  sleep(0.5)\n"
            f"  end_force_mode()\n"
            f"end\n"
        )

        try:
            self.get_logger().info("Starting force_push URScript program...")
            self.send_urscript(urscript)

            # Wait for the program to complete on the UR controller
            total_wait = 1.0 + move_duration + 0.5 + 1.0  # force settle + move + hold + buffer
            print(f"  Running on UR controller (~{total_wait:.0f}s)...")
            for i in range(int(total_wait * 10)):
                time.sleep(0.1)
                elapsed = (i + 1) * 0.1
                if (i + 1) % 10 == 0:
                    print(f"  {elapsed:.0f}s / ~{total_wait:.0f}s ...")

            print("  URScript program should be complete.")

        except KeyboardInterrupt:
            print("\n  Interrupted! Sending end_force_mode...")
            self.send_urscript(
                "def stop_fm():\n"
                "  end_force_mode()\n"
                "  stopj(2.0)\n"
                "end\n"
            )
            time.sleep(0.5)

        print("  Done.")


def main():
    parser = argparse.ArgumentParser(description="Test force push down + forward")
    parser.add_argument("--force", type=float, default=60.0,
                        help="Downward force in N (default: 60)")
    parser.add_argument("--speed-limit", type=float, default=0.15,
                        help="Force mode speed limit in m/s (default: 0.15)")
    parser.add_argument("--distance", type=float, default=0.1,
                        help="MoveL distance in meters (default: 0.1 = 100mm)")
    parser.add_argument("--speed", type=float, default=0.05,
                        help="MoveL speed in m/s (default: 0.05 = 50mm/s)")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ForcePushTest(args.force, args.speed_limit, args.distance, args.speed)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
