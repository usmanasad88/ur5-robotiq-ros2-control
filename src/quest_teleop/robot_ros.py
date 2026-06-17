"""ROS 2 control back-end: drive the launch_all.sh (fake-hardware or real)
UR5 through /forward_velocity_controller instead of RTDE.

The QuestTeleop loop emits a Cartesian twist via speed_l(); this back-end
converts it to joint velocities with the UR5 Jacobian (damped least squares)
and publishes Float64MultiArray to /forward_velocity_controller/commands —
the same path quest_servo_teleop.py uses, which is what actually moves the
fake-hardware sim.

rclpy is imported lazily so the rtde / dry-run paths stay ROS-free.

IMPORTANT: forward_velocity_controller HOLDS the last commanded velocity until
a new one arrives (no auto-decay). So speed_stop() MUST publish zeros, and the
teleop watchdog / grip-release path relies on it. Activate the controller first
with `./toggle_controller.sh velocity` (or run_quest_teleop_sim.sh does it).
"""

import threading
import time

import numpy as np

from .kinematics import UR_JOINT_NAMES, cartesian_to_joint_vel, ur5_fk_and_jacobian
from .transforms import quat_to_rotvec


class ROSControlRobot:
    """ur_rtde-shaped wrapper over a forward_velocity_controller ROS node."""

    def __init__(self, max_joint_vel=1.5, damping=0.05,
                 vel_topic='/forward_velocity_controller/commands',
                 joint_topic='/joint_states', wait_timeout=10.0):
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float64, Float64MultiArray
        except ImportError as exc:
            raise SystemExit(
                "ROS 2 (rclpy) not found for --backend ros. Source the "
                "workspace first:  source /opt/ros/humble/setup.bash && "
                f"source install/setup.bash  ({exc})")

        self._rclpy = rclpy
        self._Float64 = Float64
        self._Float64MultiArray = Float64MultiArray
        self.max_joint_vel = max_joint_vel
        self.damping = damping

        if not rclpy.ok():
            rclpy.init()
        self._node = Node('quest_teleop')
        self._vel_pub = self._node.create_publisher(Float64MultiArray, vel_topic, 10)
        self._grip_pub = self._node.create_publisher(Float64, '/ur5/gripper_cmd', 10)

        self._js_lock = threading.Lock()
        self._joint_pos = None
        self._last_J = None
        self._node.create_subscription(JointState, joint_topic, self._js_cb, 10)

        # Spin the node in the background; the QuestTeleop control loop runs in
        # the main thread and calls get_tcp_pose()/speed_l() into this node.
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name='quest_ros_spin')
        self._spin_thread.start()

        print(f"ROS backend: publishing to {vel_topic} "
              f"(make sure forward_velocity_controller is active)")
        self._wait_for_joints(wait_timeout)

    # -- ROS callbacks --------------------------------------------------
    def _js_cb(self, msg):
        try:
            q = [msg.position[msg.name.index(n)] for n in UR_JOINT_NAMES]
        except (ValueError, IndexError):
            return
        with self._js_lock:
            self._joint_pos = np.array(q)

    def _wait_for_joints(self, timeout):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self._js_lock:
                if self._joint_pos is not None:
                    print("ROS backend: received /joint_states.")
                    return
            time.sleep(0.05)
        raise SystemExit(
            "No /joint_states within %.0fs — is launch_all.sh running?" % timeout)

    # -- robot interface (matches URRobot / DryRunRobot) ----------------
    def get_tcp_pose(self):
        with self._js_lock:
            q = None if self._joint_pos is None else self._joint_pos.copy()
        if q is None:
            return [0.0]*6
        pos, _rot, quat, J = ur5_fk_and_jacobian(q)
        self._last_J = J        # reused by the speed_l() in the same tick
        return [*pos.tolist(), *quat_to_rotvec(quat).tolist()]

    def speed_l(self, velocity, accel, t):
        # accel / t are RTDE concepts; ignored here. Uses the Jacobian cached
        # by the get_tcp_pose() call that precedes every speed_l() in the loop.
        if self._last_J is None:
            return
        q_dot = cartesian_to_joint_vel(
            self._last_J, np.asarray(velocity, dtype=float),
            self.max_joint_vel, self.damping)
        self._publish(q_dot)

    def speed_stop(self, accel=0.0):
        self._publish(np.zeros(6))

    def stop_script(self):
        # No URScript program; just make sure the robot is commanded to stop.
        self._publish(np.zeros(6))

    def disconnect(self):
        self._publish(np.zeros(6))
        try:
            self._executor.shutdown()
            self._node.destroy_node()
        except Exception:
            pass

    def _publish(self, q_dot):
        msg = self._Float64MultiArray()
        msg.data = [float(v) for v in q_dot]
        self._vel_pub.publish(msg)

    # -- gripper handle sharing this node -------------------------------
    def make_gripper(self):
        return ROSGripper(self._node, self._grip_pub, self._Float64)


class ROSGripper:
    """Publishes a normalised [0,1] command to /ur5/gripper_cmd.

    Same interface as gripper.GripperController (update/open/disconnect) so the
    teleop loop never special-cases it. Trigger 0→open, 1→closed.
    """

    def __init__(self, node, pub, Float64, deadband=0.05):
        self._pub = pub
        self._Float64 = Float64
        self._deadband = deadband
        self._last = -1.0
        self._closed = False

    def update(self, trigger):
        trigger = float(np.clip(trigger, 0.0, 1.0))
        if abs(trigger - self._last) < self._deadband:
            return
        self._last = trigger
        msg = self._Float64()
        msg.data = trigger
        self._pub.publish(msg)

    def toggle(self):
        self._closed = not self._closed
        msg = self._Float64()
        msg.data = 1.0 if self._closed else 0.0
        self._pub.publish(msg)

    def open(self):
        self._closed = False
        msg = self._Float64()
        msg.data = 0.0
        self._pub.publish(msg)

    def disconnect(self):
        pass
