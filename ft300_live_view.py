#!/usr/bin/env python3
"""
Live FT300 Force/Torque Sensor Visualization
=============================================
Subscribes to /force_torque_sensor_broadcaster/wrench and displays
a real-time terminal dashboard. Runs standalone alongside all other nodes.

Usage:
    ./ft300_live_view.py                # tool0 frame, raw
    ./ft300_live_view.py --zero         # Auto-zero on startup (captures bias)
    ./ft300_live_view.py --base         # Show values in base frame
    ./ft300_live_view.py --base --zero  # Both
    Press 'z' + Enter to zero/tare at any time
    Press 'f' + Enter to toggle between tool0 and base frame
    Press 'q' + Enter or Ctrl+C to quit
"""

import sys
import math
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from tf2_ros import Buffer, TransformListener

TOPIC = "/force_torque_sensor_broadcaster/wrench"
SOURCE_FRAME = "tool0"
BASE_FRAME = "base"

# Terminal bar width
BAR_WIDTH = 30
FORCE_RANGE = 50.0   # N full scale for bar display
TORQUE_RANGE = 10.0   # Nm full scale for bar display


class FT300LiveView(Node):
    def __init__(self, auto_zero=False, use_base_frame=False):
        super().__init__("ft300_live_view")
        self.bias_force = [0.0, 0.0, 0.0]
        self.bias_torque = [0.0, 0.0, 0.0]
        self.auto_zero = auto_zero
        self.use_base_frame = use_base_frame
        self.zero_samples = []
        self.zero_count = 50  # average over 50 samples for zeroing
        self.zeroed = not auto_zero
        self.msg_count = 0
        self.last_msg = None
        self.rotation_matrix = np.eye(3)

        # TF2 for tool0 -> base rotation
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            WrenchStamped, TOPIC, self.wrench_cb, 10
        )
        self.display_timer = self.create_timer(0.1, self.display)  # 10 Hz refresh

        # Keyboard input thread
        self.running = True
        self.key_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.key_thread.start()

        if auto_zero:
            print("Collecting bias samples... hold still.")
        else:
            print("Waiting for FT data...")

    def quat_to_rotation_matrix(self, q):
        """Convert quaternion [x, y, z, w] to 3x3 rotation matrix."""
        x, y, z, w = q
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])

    def update_transform(self):
        """Look up tool0 -> base rotation from TF."""
        try:
            tf = self.tf_buffer.lookup_transform(BASE_FRAME, SOURCE_FRAME, rclpy.time.Time())
            q = tf.transform.rotation
            self.rotation_matrix = self.quat_to_rotation_matrix([q.x, q.y, q.z, q.w])
        except Exception:
            pass  # keep last known rotation

    def rotate_to_base(self, vec):
        """Rotate a 3-vector from tool0 frame to base frame."""
        return self.rotation_matrix @ np.array(vec)

    def wrench_cb(self, msg):
        self.msg_count += 1

        if self.use_base_frame:
            self.update_transform()

        if not self.zeroed:
            f = msg.wrench.force
            t = msg.wrench.torque
            fv = [f.x, f.y, f.z]
            tv = [t.x, t.y, t.z]
            if self.use_base_frame:
                fv = self.rotate_to_base(fv).tolist()
                tv = self.rotate_to_base(tv).tolist()
            self.zero_samples.append((fv, tv))
            if len(self.zero_samples) >= self.zero_count:
                self.apply_zero()
            return

        self.last_msg = msg

    def apply_zero(self):
        n = len(self.zero_samples)
        if n == 0:
            return
        self.bias_force = [
            sum(s[0][i] for s in self.zero_samples) / n for i in range(3)
        ]
        self.bias_torque = [
            sum(s[1][i] for s in self.zero_samples) / n for i in range(3)
        ]
        self.zero_samples = []
        self.zeroed = True

    def zero_now(self):
        """Start collecting samples for a new zero/tare."""
        self.zeroed = False
        self.zero_samples = []
        self.auto_zero = True
        print("\nZeroing... hold still.")

    def bar(self, value, max_val):
        """Render a centered bar: negative left, positive right."""
        half = BAR_WIDTH // 2
        normalized = value / max_val  # -1 to +1
        normalized = max(-1.0, min(1.0, normalized))
        fill = int(abs(normalized) * half)

        if normalized >= 0:
            left = " " * half
            right = "\033[92m" + "█" * fill + "\033[0m" + " " * (half - fill)
        else:
            pad = half - fill
            left = " " * pad + "\033[91m" + "█" * fill + "\033[0m"
            right = " " * half

        return f"{left}│{right}"

    def toggle_frame(self):
        """Switch between tool0 and base frame display."""
        self.use_base_frame = not self.use_base_frame
        # Re-zero in the new frame if we had a bias
        if self.bias_force != [0.0, 0.0, 0.0]:
            self.zero_now()

    def display(self):
        if self.last_msg is None:
            return

        msg = self.last_msg
        f = msg.wrench.force
        t = msg.wrench.torque

        fv = [f.x, f.y, f.z]
        tv = [t.x, t.y, t.z]

        if self.use_base_frame:
            fv = self.rotate_to_base(fv).tolist()
            tv = self.rotate_to_base(tv).tolist()

        fx = fv[0] - self.bias_force[0]
        fy = fv[1] - self.bias_force[1]
        fz = fv[2] - self.bias_force[2]
        tx = tv[0] - self.bias_torque[0]
        ty = tv[1] - self.bias_torque[1]
        tz = tv[2] - self.bias_torque[2]

        f_mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        t_mag = math.sqrt(tx * tx + ty * ty + tz * tz)

        frame_name = BASE_FRAME if self.use_base_frame else SOURCE_FRAME

        # Move cursor up and overwrite (after first print)
        lines = 13
        sys.stdout.write(f"\033[{lines}A\033[J")

        print("┌─────────────────────────────────────────────────────────────────┐")
        print(f"│  FT300 Live View  [{frame_name:>5s}]     [z]=zero [f]=frame [q]=quit  │")
        print("├─────────────────────────────────────────────────────────────────┤")
        print(f"│  Fx: {fx:+8.2f} N   {self.bar(fx, FORCE_RANGE)}  │")
        print(f"│  Fy: {fy:+8.2f} N   {self.bar(fy, FORCE_RANGE)}  │")
        print(f"│  Fz: {fz:+8.2f} N   {self.bar(fz, FORCE_RANGE)}  │")
        print(f"│  Tx: {tx:+8.2f} Nm  {self.bar(tx, TORQUE_RANGE)}  │")
        print(f"│  Ty: {ty:+8.2f} Nm  {self.bar(ty, TORQUE_RANGE)}  │")
        print(f"│  Tz: {tz:+8.2f} Nm  {self.bar(tz, TORQUE_RANGE)}  │")
        print("├─────────────────────────────────────────────────────────────────┤")
        print(f"│  |F|: {f_mag:7.2f} N    |T|: {t_mag:7.2f} Nm     rate: {125:>3d} Hz       │")
        print("└─────────────────────────────────────────────────────────────────┘")

    def key_listener(self):
        while self.running:
            try:
                line = input()
                if line.strip().lower() == "z":
                    self.zero_now()
                elif line.strip().lower() == "f":
                    self.toggle_frame()
                elif line.strip().lower() == "q":
                    self.running = False
                    raise SystemExit
            except (EOFError, SystemExit):
                self.running = False
                rclpy.shutdown()
                break


def main():
    auto_zero = "--zero" in sys.argv
    use_base = "--base" in sys.argv
    rclpy.init()
    # Print blank lines so the cursor-up trick works on first frame
    print("\n" * 13)
    node = FT300LiveView(auto_zero=auto_zero, use_base_frame=use_base)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\nDone.")


if __name__ == "__main__":
    main()
