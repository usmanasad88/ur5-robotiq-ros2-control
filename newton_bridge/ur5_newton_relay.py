#!/usr/bin/env python3
# ============================================================
# ROS 2 -> Newton UDP relay for UR5 teleop
# ============================================================
# Makes the Newton cloth_ur5_ogc sim a FOLLOWER of the RViz UR5: it relays the
# robot's actual joint state (/joint_states, published by the joint-state
# broadcaster no matter what drives the robot -- Quest VR teleop, the Unity
# app, prog files, spacemouse, cuRobo, or the real teach pendant) as a UDP
# packet to the Newton example's teleop input. Newton has no ROS dependencies
# (it runs in a separate Python 3.12 venv), so this small node is the bridge
# across the ROS (py3.10) <-> Newton (py3.12) gap.
#
#   Topics in : /joint_states (sensor_msgs/JointState)  [default source]
#               -- mapped BY NAME to UR order, so publisher joint order
#                  doesn't matter. Messages without all 6 arm joints are
#                  skipped (other nodes may publish partial states).
#               -- if robotiq_85_left_knuckle_joint is present (real gripper
#                  adapter), the gripper value follows it; otherwise the
#                  cached /ur5/gripper_cmd is used.
#               /ur5/gripper_cmd (std_msgs/Float64, 0 = open .. 1 = closed)
#               [legacy source:=commands] /forward_position_controller/commands
#               (std_msgs/Float64MultiArray, q[6] rad, UR joint order) -- the
#               old direct-command path; only sees ONE input method.
#   UDP out   : 7 little-endian float64  ->  newton_host:newton_port
#               [q0..q5, gripper]  (default 127.0.0.1:11000; must match the
#               example's --teleop-port)
#
# Run via run_newton_relay.sh (which sets up the ROS env), or directly:
#   python3 ur5_newton_relay.py --ros-args \
#       -p newton_host:=127.0.0.1 -p newton_port:=11000 -p source:=joint_states
# ============================================================

import socket
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

# packet format -- must match JointStreamReceiver in the Newton example
FMT = "<7d"

# UR controller joint order == the Newton example's arm dof order
ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# Robotiq 2F-85 driver joint; all other finger joints mimic it
GRIPPER_JOINT = "robotiq_85_left_knuckle_joint"
GRIPPER_CLOSED_RAD = 0.803  # knuckle angle at fully closed


def extract_q(names, positions):
    """Map a JointState (name, position) pair to (q[6] in UR order, gripper).

    Returns (None, None) if any of the 6 arm joints is missing (e.g. a partial
    joint_states message from another publisher). gripper is the normalized
    0..1 close fraction if the Robotiq knuckle joint is present, else None.
    """
    idx = {n: i for i, n in enumerate(names)}
    try:
        q = [float(positions[idx[j]]) for j in ARM_JOINTS]
    except KeyError:
        return None, None
    g = None
    gi = idx.get(GRIPPER_JOINT)
    if gi is not None:
        g = min(max(float(positions[gi]) / GRIPPER_CLOSED_RAD, 0.0), 1.0)
    return q, g


class NewtonRelay(Node):
    def __init__(self):
        super().__init__("ur5_newton_relay")

        self.declare_parameter("newton_host", "127.0.0.1")
        self.declare_parameter("newton_port", 11000)
        # "joint_states" = follow the RViz robot whatever drives it (default);
        # "commands" = legacy direct relay of the position-command stream
        self.declare_parameter("source", "joint_states")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("commands_topic", "/forward_position_controller/commands")
        self.declare_parameter("gripper_topic", "/ur5/gripper_cmd")
        # UDP send rate cap; /joint_states can arrive at 500 Hz on the real
        # robot and Newton only reads the latest packet per frame anyway
        self.declare_parameter("max_hz", 120.0)

        host = self.get_parameter("newton_host").get_parameter_value().string_value
        port = int(self.get_parameter("newton_port").get_parameter_value().integer_value)
        source = self.get_parameter("source").get_parameter_value().string_value
        js_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        cmd_topic = self.get_parameter("commands_topic").get_parameter_value().string_value
        grip_topic = self.get_parameter("gripper_topic").get_parameter_value().string_value
        max_hz = float(self.get_parameter("max_hz").get_parameter_value().double_value)

        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._count = 0
        self._gripper = 0.0  # latest gripper value (0 = open), appended to every packet
        self._min_interval = 1.0 / max_hz if max_hz > 0.0 else 0.0
        self._last_send = 0.0

        if source == "commands":
            self.create_subscription(Float64MultiArray, cmd_topic, self._cmd_cb, 10)
            arm_desc = f"{cmd_topic} (q[6] commands, legacy)"
        else:
            self.create_subscription(JointState, js_topic, self._js_cb, 10)
            arm_desc = f"{js_topic} (robot state, follower mode)"
        self.create_subscription(Float64, grip_topic, self._grip_cb, 10)
        self.get_logger().info(f"Relaying {arm_desc} + {grip_topic} (gripper) -> UDP {host}:{port}")

    def _send(self, q):
        self._sock.sendto(struct.pack(FMT, *q, self._gripper), self._addr)
        self._count += 1
        if self._count % 100 == 1:
            self.get_logger().info(
                f"relayed {self._count} packets (q[0]={q[0]:.3f}, gripper={self._gripper:.2f})"
            )

    def _js_cb(self, msg: JointState):
        now = time.monotonic()
        if now - self._last_send < self._min_interval:
            return
        q, g = extract_q(msg.name, msg.position)
        if q is None:
            return  # partial joint_states from some other publisher
        if g is not None:
            self._gripper = g  # real gripper state beats the cached command
        self._last_send = now
        self._send(q)

    def _cmd_cb(self, msg: Float64MultiArray):
        q = list(msg.data)[:6]
        if len(q) < 6:
            self.get_logger().warn(f"command has {len(q)} values, expected 6; ignoring")
            return
        self._send(q)

    def _grip_cb(self, msg: Float64):
        # cache only; the value rides out on the next arm packet (arm streams
        # continuously). Overridden by the knuckle joint state when present.
        self._gripper = float(msg.data)


def main():
    rclpy.init()
    node = NewtonRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
