#!/usr/bin/env python3
"""
Workspace Markers Publisher for UR5 Robot Environment

Publishes GLB mesh markers for RViz. Each object's pose and scale are
exposed as ROS parameters so they can be tuned live:

    ros2 param set /workspace_markers_publisher table.x  0.1
    ros2 param set /workspace_markers_publisher table.yaw  45.0
    ros2 param set /workspace_markers_publisher table.scale_x  1.2

Usage:
    ros2 run ur5_workspace_description workspace_markers

In RViz:
    Add -> MarkerArray -> Topic: /workspace_markers
"""

import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from visualization_msgs.msg import Marker, MarkerArray
import os


class WorkspaceMarkersPublisher(Node):
    """Publishes GLB mesh markers with live-tunable ROS parameters."""

    MESH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../isaac_standalone/Objects")

    # Default objects — edit here to add/remove meshes.
    # Each: (name, glb_file, x, y, z, roll, pitch, yaw, sx, sy, sz)
    DEFAULTS = [
        ("table1", "table.glb",      0.547, -0.371, -0.5,  90.0, 0.0, 0.0,  1.0, 1.0, 1.0),
        ("table2", "table.glb",      0.684,  0.474, -0.5,  90.0, 0.0, 90.0, 1.0, 1.0, 1.0),
        ("base",   "robot_base.glb", 0.0,    0.0,   -0.1,  90.0, 0.0, 0.0,  0.8, 0.8, 0.8),
        ("bottle", "bottle.glb",     0.6,   -0.2,   -0.5,  90.0, 0.0, 0.0,  0.4, 0.4, 0.4),
        ("roller", "roller.glb",     0.6,   -0.6,   -0.5,  90.0, 0.0, 0.0,  0.4, 0.4, 0.4),
    ]

    PARAM_KEYS = ("x", "y", "z", "roll", "pitch", "yaw", "scale_x", "scale_y", "scale_z")

    def __init__(self):
        super().__init__('workspace_markers_publisher')

        self.objects = []  # list of (name, glb_file)

        # Declare parameters for each object
        for name, glb, x, y, z, r, p, yw, sx, sy, sz in self.DEFAULTS:
            self.objects.append((name, glb))
            vals = (x, y, z, r, p, yw, sx, sy, sz)
            for key, val in zip(self.PARAM_KEYS, vals):
                self.declare_parameter(f"{name}.{key}", val)

        # React to parameter changes immediately
        self.add_on_set_parameters_callback(self._on_param_change)

        self.publisher = self.create_publisher(MarkerArray, 'workspace_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info('Workspace markers publisher started (live-tunable)')
        self.get_logger().info(f'Objects: {[n for n, _ in self.objects]}')
        self.get_logger().info('Tune with: ros2 param set /workspace_markers_publisher <name>.<key> <value>')
        self.get_logger().info(f'  keys: {", ".join(self.PARAM_KEYS)}')

    def _on_param_change(self, params):
        """Accept any parameter change — the new values are picked up on next publish."""
        return SetParametersResult(successful=True)

    @staticmethod
    def _euler_to_quat(roll_deg, pitch_deg, yaw_deg):
        """RPY (degrees) -> quaternion (x, y, z, w)."""
        r, p, y = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
        cx, sx = math.cos(r / 2), math.sin(r / 2)
        cy, sy = math.cos(p / 2), math.sin(p / 2)
        cz, sz = math.cos(y / 2), math.sin(y / 2)
        return (
            sx * cy * cz - cx * sy * sz,
            cx * sy * cz + sx * cy * sz,
            cx * cy * sz - sx * sy * cz,
            cx * cy * cz + sx * sy * sz,
        )

    def publish_markers(self):
        """Read current parameter values and publish markers."""
        marker_array = MarkerArray()

        for obj_id, (name, glb_file) in enumerate(self.objects):
            x  = self.get_parameter(f"{name}.x").value
            y  = self.get_parameter(f"{name}.y").value
            z  = self.get_parameter(f"{name}.z").value
            ro = self.get_parameter(f"{name}.roll").value
            pi = self.get_parameter(f"{name}.pitch").value
            ya = self.get_parameter(f"{name}.yaw").value
            sx = self.get_parameter(f"{name}.scale_x").value
            sy = self.get_parameter(f"{name}.scale_y").value
            sz = self.get_parameter(f"{name}.scale_z").value

            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "workspace_meshes"
            m.id = obj_id
            m.type = Marker.MESH_RESOURCE
            m.action = Marker.ADD
            m.mesh_resource = f"file://{self.MESH_DIR}/{glb_file}"
            m.mesh_use_embedded_materials = True

            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = float(z)
            qx, qy, qz, qw = self._euler_to_quat(ro, pi, ya)
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw

            m.scale.x = float(sx)
            m.scale.y = float(sy)
            m.scale.z = float(sz)

            # alpha=0 lets RViz use the GLB's embedded materials/textures
            m.color.a = 0.0

            marker_array.markers.append(m)

        self.publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceMarkersPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
