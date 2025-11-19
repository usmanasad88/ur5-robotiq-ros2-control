#!/usr/bin/env python3
"""
Workspace Markers Publisher for UR5 Robot Environment

Publishes visualization markers for RViz showing a tabletop environment
with a table and several boxes. These are visual-only and do not affect
collision planning.

Usage:
    ros2 run ur5_workspace_description workspace_markers

In RViz:
    Add -> Marker -> Topic: /workspace_markers
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class WorkspaceMarkersPublisher(Node):
    """Publishes visualization markers for a tabletop workspace environment."""
    
    def __init__(self):
        super().__init__('workspace_markers_publisher')
        
        # Publisher
        self.publisher = self.create_publisher(MarkerArray, 'workspace_markers', 10)
        
        # Timer - publish at 1Hz
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info('Workspace markers publisher started')
        self.get_logger().info('Publishing markers to /workspace_markers')
        self.get_logger().info('In RViz: Add -> MarkerArray -> Topic: /workspace_markers')
        
    def create_marker(self, marker_id, marker_type, name, pose, scale, color):
        """Helper function to create a marker."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "workspace"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        
        # Position and orientation
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = pose[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Scale
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        
        # Color (RGBA)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        
        # Lifetime (0 = forever)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        # Add text label
        marker.text = name
        
        return marker
        
    def publish_markers(self):
        """Publish all workspace markers."""
        marker_array = MarkerArray()
        
        # Table - positioned below robot base
        table = self.create_marker(
            marker_id=0,
            marker_type=Marker.CUBE,
            name="Table",
            pose=[0.0, 0.0, -0.15],  # 15cm below base_link
            scale=[1.2, 0.8, 0.1],   # 120cm x 80cm x 10cm
            color=[0.6, 0.4, 0.2, 1.0]  # Brown
        )
        marker_array.markers.append(table)
        
        # Box 1 - Red box on left side of table
        box1 = self.create_marker(
            marker_id=1,
            marker_type=Marker.CUBE,
            name="Box_1",
            pose=[0.3, 0.2, -0.05],  # On top of table
            scale=[0.1, 0.1, 0.1],   # 10cm cube
            color=[0.8, 0.1, 0.1, 1.0]  # Red
        )
        marker_array.markers.append(box1)
        
        # Box 2 - Green box in center
        box2 = self.create_marker(
            marker_id=2,
            marker_type=Marker.CUBE,
            name="Box_2",
            pose=[0.35, -0.1, -0.05],
            scale=[0.12, 0.08, 0.15],  # Rectangular box
            color=[0.1, 0.8, 0.1, 1.0]  # Green
        )
        marker_array.markers.append(box2)
        
        # Box 3 - Blue box on right side
        box3 = self.create_marker(
            marker_id=3,
            marker_type=Marker.CUBE,
            name="Box_3",
            pose=[0.25, -0.25, -0.05],
            scale=[0.08, 0.08, 0.2],  # Tall thin box
            color=[0.1, 0.1, 0.8, 1.0]  # Blue
        )
        marker_array.markers.append(box3)
        
        # Box 4 - Yellow box near robot
        box4 = self.create_marker(
            marker_id=4,
            marker_type=Marker.CUBE,
            name="Box_4",
            pose=[0.15, 0.0, -0.05],
            scale=[0.15, 0.15, 0.1],  # Wide flat box
            color=[0.9, 0.9, 0.1, 1.0]  # Yellow
        )
        marker_array.markers.append(box4)
        
        # Cylinder obstacle - Orange cylinder
        cylinder = self.create_marker(
            marker_id=5,
            marker_type=Marker.CYLINDER,
            name="Cylinder",
            pose=[0.4, 0.3, 0.0],
            scale=[0.08, 0.08, 0.2],  # Diameter 8cm, height 20cm
            color=[1.0, 0.5, 0.0, 1.0]  # Orange
        )
        marker_array.markers.append(cylinder)
        
        # Add text labels above each object
        label_height = 0.15
        labels = [
            (0, "TABLE", [0.0, 0.0, -0.05], [1.0, 1.0, 1.0, 1.0]),
            (10, "Red Box", [0.3, 0.2, label_height], [1.0, 1.0, 1.0, 0.8]),
            (11, "Green Box", [0.35, -0.1, label_height], [1.0, 1.0, 1.0, 0.8]),
            (12, "Blue Box", [0.25, -0.25, label_height], [1.0, 1.0, 1.0, 0.8]),
            (13, "Yellow Box", [0.15, 0.0, label_height], [1.0, 1.0, 1.0, 0.8]),
            (14, "Cylinder", [0.4, 0.3, label_height], [1.0, 1.0, 1.0, 0.8]),
        ]
        
        for label_id, text, pos, color in labels:
            label = Marker()
            label.header.frame_id = "base_link"
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = "labels"
            label.id = label_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = pos[0]
            label.pose.position.y = pos[1]
            label.pose.position.z = pos[2]
            label.pose.orientation.w = 1.0
            label.scale.z = 0.03  # Text height
            label.color.r = color[0]
            label.color.g = color[1]
            label.color.b = color[2]
            label.color.a = color[3]
            label.text = text
            marker_array.markers.append(label)
        
        # Publish marker array
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
