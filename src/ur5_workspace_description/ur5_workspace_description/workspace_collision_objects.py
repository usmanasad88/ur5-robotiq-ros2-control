#!/usr/bin/env python3
"""
Workspace Collision Objects Publisher for UR5 Robot Environment

Publishes collision objects to the MoveIt planning scene. These objects
will appear in RViz AND be used for collision avoidance during motion planning.

Usage:
    ros2 run ur5_workspace_description workspace_collision_objects

In RViz:
    The objects will automatically appear in the Planning Scene display
    Make sure you have launched MoveIt first with ur_moveit.launch.py
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneWorld
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class WorkspaceCollisionObjectsPublisher(Node):
    """Publishes collision objects to the MoveIt planning scene."""
    
    def __init__(self):
        super().__init__('workspace_collision_objects_publisher')
        
        # Publisher for planning scene
        self.scene_publisher = self.create_publisher(
            PlanningScene, 
            'planning_scene', 
            10
        )
        
        # Also publish to collision_object topic (alternative method)
        self.collision_publisher = self.create_publisher(
            CollisionObject,
            'collision_object',
            10
        )
        
        # Timer - publish at 2Hz to ensure objects are added
        self.timer = self.create_timer(0.5, self.publish_collision_objects)
        
        # Track if objects have been published
        self.objects_published = False
        self.publish_count = 0
        
        self.get_logger().info('Workspace collision objects publisher started')
        self.get_logger().info('Publishing to /planning_scene and /collision_object')
        self.get_logger().info('Objects will be added to MoveIt planning scene for collision avoidance')
        
    def create_collision_object(self, obj_id, primitive_type, dimensions, pose_values):
        """
        Helper function to create a collision object.
        
        Args:
            obj_id: Unique identifier for the object
            primitive_type: SolidPrimitive type (BOX or CYLINDER)
            dimensions: List of dimensions (for BOX: [x, y, z], for CYLINDER: [height, radius])
            pose_values: [x, y, z] position
        """
        collision_object = CollisionObject()
        collision_object.header = Header()
        collision_object.header.frame_id = "base_link"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = obj_id
        
        # Define the primitive shape
        primitive = SolidPrimitive()
        primitive.type = primitive_type
        primitive.dimensions = dimensions
        
        # Define the pose
        pose = Pose()
        pose.position.x = pose_values[0]
        pose.position.y = pose_values[1]
        pose.position.z = pose_values[2]
        pose.orientation.w = 1.0
        
        # Add to collision object
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        
        return collision_object
        
    def publish_collision_objects(self):
        """Publish all workspace collision objects to the planning scene."""
        
        # Only publish intensively for the first few seconds, then reduce frequency
        self.publish_count += 1
        
        if self.publish_count > 10 and self.objects_published:
            # After initial publishing, slow down to once every 5 seconds
            if self.publish_count % 10 != 0:
                return
        
        # Create all collision objects
        collision_objects = []
        
        # Table - Large flat surface below robot
        table = self.create_collision_object(
            obj_id="table",
            primitive_type=SolidPrimitive.BOX,
            dimensions=[1.2, 0.8, 0.1],  # 120cm x 80cm x 10cm
            pose_values=[0.0, 0.0, -0.15]  # 15cm below base_link
        )
        collision_objects.append(table)
        
        # Box 1 - Red box on left side of table
        box1 = self.create_collision_object(
            obj_id="box_1_red",
            primitive_type=SolidPrimitive.BOX,
            dimensions=[0.1, 0.1, 0.1],  # 10cm cube
            pose_values=[0.3, 0.2, -0.05]  # On top of table
        )
        collision_objects.append(box1)
        
        # Box 2 - Green box in center
        box2 = self.create_collision_object(
            obj_id="box_2_green",
            primitive_type=SolidPrimitive.BOX,
            dimensions=[0.12, 0.08, 0.15],  # Rectangular box
            pose_values=[0.35, -0.1, -0.05]
        )
        collision_objects.append(box2)
        
        # Box 3 - Blue box on right side
        box3 = self.create_collision_object(
            obj_id="box_3_blue",
            primitive_type=SolidPrimitive.BOX,
            dimensions=[0.08, 0.08, 0.2],  # Tall thin box
            pose_values=[0.25, -0.25, -0.05]
        )
        collision_objects.append(box3)
        
        # Box 4 - Yellow box near robot
        box4 = self.create_collision_object(
            obj_id="box_4_yellow",
            primitive_type=SolidPrimitive.BOX,
            dimensions=[0.15, 0.15, 0.1],  # Wide flat box
            pose_values=[0.15, 0.0, -0.05]
        )
        collision_objects.append(box4)
        
        # Cylinder obstacle - Orange cylinder
        cylinder = self.create_collision_object(
            obj_id="cylinder_orange",
            primitive_type=SolidPrimitive.CYLINDER,
            dimensions=[0.2, 0.04],  # height=20cm, radius=4cm
            pose_values=[0.4, 0.3, 0.0]
        )
        collision_objects.append(cylinder)
        
        # Create planning scene message
        planning_scene = PlanningScene()
        planning_scene.is_diff = True  # Only add new objects, don't replace entire scene
        planning_scene.world.collision_objects = collision_objects
        
        # Publish to planning scene
        self.scene_publisher.publish(planning_scene)
        
        # Also publish individual collision objects (alternative method for compatibility)
        for obj in collision_objects:
            self.collision_publisher.publish(obj)
        
        if not self.objects_published:
            self.get_logger().info(f'Published {len(collision_objects)} collision objects to planning scene')
            self.get_logger().info('Objects: table, box_1_red, box_2_green, box_3_blue, box_4_yellow, cylinder_orange')
            self.objects_published = True


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceCollisionObjectsPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
