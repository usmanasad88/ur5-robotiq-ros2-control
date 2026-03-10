#!/usr/bin/env python3
"""
Test force-controlled push in -Z (base frame).

Activates the force_mode_controller with Z-axis compliance,
applies a gentle downward force, waits, then stops.

Usage:
    ./run_force_push_test.sh              # default 10N down for 5s
    ./run_force_push_test.sh --force 15   # 15N down
    ./run_force_push_test.sh --time 10    # hold for 10s
"""

import sys
import time
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Wrench, Twist
from ur_msgs.srv import SetForceMode
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController


class ForcePushTest(Node):
    def __init__(self, force_n, duration_s):
        super().__init__("force_push_test")
        self.force_n = force_n
        self.duration_s = duration_s

        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.start_force_client = self.create_client(
            SetForceMode, "/force_mode_controller/start_force_mode"
        )
        self.stop_force_client = self.create_client(
            Trigger, "/force_mode_controller/stop_force_mode"
        )

    def wait_for_service(self, client, name, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Service {name} not available")
            return False
        return True

    def call_sync(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()

    def activate_force_controller(self):
        """Activate force_mode_controller (deactivates trajectory controller)."""
        req = SwitchController.Request()
        req.activate_controllers = ["force_mode_controller"]
        req.deactivate_controllers = ["scaled_joint_trajectory_controller"]
        req.strictness = SwitchController.Request.BEST_EFFORT
        result = self.call_sync(self.switch_client, req)
        if result and result.ok:
            self.get_logger().info("force_mode_controller activated")
            return True
        self.get_logger().error(f"Failed to activate force_mode_controller: {result}")
        return False

    def deactivate_force_controller(self):
        """Deactivate force_mode_controller, reactivate trajectory controller."""
        req = SwitchController.Request()
        req.activate_controllers = ["scaled_joint_trajectory_controller"]
        req.deactivate_controllers = ["force_mode_controller"]
        req.strictness = SwitchController.Request.BEST_EFFORT
        result = self.call_sync(self.switch_client, req)
        if result and result.ok:
            self.get_logger().info("Restored scaled_joint_trajectory_controller")

    def start_force_mode(self):
        """Start force mode: compliant in Z, apply downward force."""
        req = SetForceMode.Request()

        # Task frame at base origin, identity orientation
        req.task_frame = PoseStamped()
        req.task_frame.header.frame_id = "base"
        req.task_frame.pose.orientation.w = 1.0

        # Only Z is compliant (force-controlled)
        req.selection_vector_x = False
        req.selection_vector_y = False
        req.selection_vector_z = True
        req.selection_vector_rx = False
        req.selection_vector_ry = False
        req.selection_vector_rz = False

        # Apply force in -Z (downward in base frame)
        req.wrench = Wrench()
        req.wrench.force.z = -self.force_n

        # Frame type 2 = no transform (use task frame as-is)
        req.type = SetForceMode.Request.NO_TRANSFORM

        # Speed limit for compliant axis
        req.speed_limits = Twist()
        req.speed_limits.linear.z = 0.05  # max 50mm/s in Z

        # Deviation limits for non-compliant axes (hold position)
        req.deviation_limits = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]

        # Conservative damping and gain
        req.damping_factor = 0.1
        req.gain_scaling = 0.5

        result = self.call_sync(self.start_force_client, req)
        if result and result.success:
            self.get_logger().info(
                f"Force mode active: {self.force_n}N downward, Z speed limit 50mm/s"
            )
            return True
        self.get_logger().error(f"Failed to start force mode: {result}")
        return False

    def stop_force_mode(self):
        result = self.call_sync(self.stop_force_client, Trigger.Request())
        if result and result.success:
            self.get_logger().info("Force mode stopped")

    def run(self):
        # Check services
        for client, name in [
            (self.switch_client, "switch_controller"),
            (self.start_force_client, "start_force_mode"),
            (self.stop_force_client, "stop_force_mode"),
        ]:
            if not self.wait_for_service(client, name):
                return

        print(f"\n  Force push test: {self.force_n}N downward for {self.duration_s}s")
        print(f"  Speed limit: 50mm/s in Z")
        print(f"  Non-compliant axes: X, Y, Rx, Ry, Rz (position hold)")
        print(f"\n  Press Ctrl+C at any time to stop.\n")

        try:
            # Step 1: Activate force controller
            if not self.activate_force_controller():
                return

            time.sleep(0.5)  # let controller settle

            # Step 2: Start force mode
            if not self.start_force_mode():
                self.deactivate_force_controller()
                return

            # Step 3: Wait
            print(f"  Holding for {self.duration_s}s...")
            for i in range(int(self.duration_s * 10)):
                time.sleep(0.1)
                remaining = self.duration_s - (i + 1) * 0.1
                if remaining > 0 and (i + 1) % 10 == 0:
                    print(f"  {remaining:.0f}s remaining...")

        except KeyboardInterrupt:
            print("\n  Interrupted!")
        finally:
            # Always clean up
            print("  Stopping force mode...")
            self.stop_force_mode()
            time.sleep(0.3)
            self.deactivate_force_controller()
            print("  Done.")


def main():
    parser = argparse.ArgumentParser(description="Test force push in -Z")
    parser.add_argument("--force", type=float, default=10.0, help="Force in N (default: 10)")
    parser.add_argument("--time", type=float, default=5.0, help="Duration in seconds (default: 5)")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ForcePushTest(args.force, args.time)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
