#!/home/mani/miniconda3/envs/ur5_python/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Bool

import torch
import numpy as np
import time
import os
import sys
import pandas as pd
import json
import shutil
from datetime import datetime
import cv2
import mss
import imageio
import pyarrow as pa
import pyarrow.parquet as pq

# Add curobo to path if not installed
sys.path.append('/home/mani/isaac-sim-standalone-5.0.0-linux-x86_64/curobo/src')

from curobo.geom.types import WorldConfig, Cuboid
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose as CuroboPose
from curobo.types.robot import JointState as CuroboJointState
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.util_file import load_yaml, join_path

class DatasetGeneratorNode(Node):
    def __init__(self):
        super().__init__('dataset_generator_node')
        
        # Parameters
        self.declare_parameter('robot_config_file', '')
        self.declare_parameter('world_config_file', '')
        self.declare_parameter('output_dir', 'generated_dataset')
        self.declare_parameter('episodes', 5)
        
        robot_config_file = self.get_parameter('robot_config_file').get_parameter_value().string_value
        world_config_file = self.get_parameter('world_config_file').get_parameter_value().string_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.target_episodes = self.get_parameter('episodes').get_parameter_value().integer_value
        
        if not robot_config_file:
            self.get_logger().error("robot_config_file parameter is required")
            return

        self.get_logger().info(f"Loading robot config: {robot_config_file}")
        self.get_logger().info(f"Loading world config: {world_config_file}")

        # Initialize Curobo
        self.tensor_args = TensorDeviceType()
        
        self.dt = 0.05 
        self.motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_config_file,
            world_config_file,
            self.tensor_args,
            trajopt_tsteps=32,
            use_cuda_graph=False,
            interpolation_dt=self.dt,
        )
        self.motion_gen = MotionGen(self.motion_gen_config)
        
        # ROS Interfaces
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.controller_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/scaled_joint_trajectory_controller/state',
            self.controller_state_callback,
            10
        )
        
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.current_joint_state = None
        self.latest_controller_state = None
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Define a sequence of target poses (position x,y,z, quaternion w,x,y,z)
        self.target_poses = [
            ([0.4, 0.3, 0.4], [0.0, 0.707, 0.707, 0.0], "Move to Left"), 
            ([0.4, -0.3, 0.4], [0.0, 0.707, 0.707, 0.0], "Move to Right"),
            ([0.5, 0.0, 0.5], [0.0, 1.0, 0.0, 0.0], "Move to Center"),
        ]
        self.current_target_idx = 0
        self.task_map = {desc: i for i, (_, _, desc) in enumerate(self.target_poses)}
        self.idx_to_task = {i: desc for desc, i in self.task_map.items()}
        
        # Dataset State
        self.setup_dataset_dir()
        self.episode_data = []
        self.episodes_meta = []
        self.current_episode_idx = 0
        self.frame_index = 0
        self.is_moving = False
        self.move_start_time = 0
        self.move_duration = 0
        
        # Timers
        self.control_timer = self.create_timer(1.0, self.control_loop) # Check logic every 1s
        self.record_timer = self.create_timer(0.1, self.record_step) # Record at 10Hz
        
        # Screen Capture
        self.sct = mss.mss()
        # Define capture region - adjust these values to match your RViz window
        # For now, we capture a central region of the primary monitor
        monitor = self.sct.monitors[1] # Primary monitor
        width = 800
        height = 800
        left = monitor["left"] + (monitor["width"] - width) // 2
        top = monitor["top"] + (monitor["height"] - height) // 2
        self.capture_region = {"top": top, "left": left, "width": width, "height": height}
        
        self.get_logger().info("Dataset Generator Node Initialized")

    def setup_dataset_dir(self):
        if os.path.exists(self.output_dir):
            shutil.rmtree(self.output_dir)
        os.makedirs(os.path.join(self.output_dir, "data", "chunk-000"))
        os.makedirs(os.path.join(self.output_dir, "videos", "chunk-000"))
        os.makedirs(os.path.join(self.output_dir, "meta"))
        
        # Save info.json
        info = {
            "codebase_version": "v2.0",
            "robot_type": "UR5",
            "total_episodes": self.target_episodes,
            "total_chunks": 1,
            "chunks_size": 1000,
            "fps": 10.0,
            "data_path": "data/chunk-000/train-{episode_index:05d}-of-" + f"{self.target_episodes:05d}.parquet",
            "video_path": "videos/chunk-000/train-{episode_index:05d}-of-" + f"{self.target_episodes:05d}.mp4",
            "features": {
                "observation.state": {
                    "dtype": "float64", 
                    "shape": [7],
                    "names": ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"]
                },
                "action": {
                    "dtype": "float64", 
                    "shape": [7],
                    "names": ["joint_0", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "gripper"]
                },
                "observation.images.egoview": {
                    "dtype": "video", 
                    "shape": [256, 256, 3],
                    "names": ["height", "width", "channel"],
                    "video_info": {
                        "video.fps": 10.0,
                        "video.codec": "av1",
                        "video.pix_fmt": "yuv420p",
                        "video.is_depth_map": False,
                        "has_audio": False
                    }
                }
            }
        }
        with open(os.path.join(self.output_dir, "meta", "info.json"), 'w') as f:
            json.dump(info, f, indent=2)

        # Save modality.json
        modality = {
            "state": {
                "arm": {"start": 0, "end": 6},
                "gripper": {"start": 6, "end": 7}
            },
            "action": {
                "arm": {"start": 0, "end": 6},
                "gripper": {"start": 6, "end": 7}
            },
            "video": {
                "egoview": {"original_key": "observation.images.egoview"}
            },
            "annotation": {
                "human.action.task_description": {}
            }
        }
        with open(os.path.join(self.output_dir, "meta", "modality.json"), 'w') as f:
            json.dump(modality, f, indent=2)

    def joint_state_callback(self, msg):
        # Map joint state to correct order
        if self.current_joint_state is None:
            self.current_joint_state = [0.0] * 6
            
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joint_state[i] = msg.position[idx]

    def controller_state_callback(self, msg):
        self.latest_controller_state = msg

    def generate_dummy_image(self):
        # Capture screen using mss
        try:
            sct_img = self.sct.grab(self.capture_region)
            # mss returns a ScreenShot object, use PIL for conversion
            from PIL import Image
            # Convert to PIL Image (mss provides RGB data via .rgb property)
            pil_img = Image.frombytes('RGB', (sct_img.width, sct_img.height), sct_img.rgb)
            
            # Resize to 256x256
            pil_img = pil_img.resize((256, 256), Image.LANCZOS)
            
            # Convert to numpy array
            img = np.array(pil_img, dtype=np.uint8)
            
            if img.size == 0:
                raise ValueError("Captured image is empty")
            
            return img
        except Exception as e:
            self.get_logger().error(f"Screen capture failed: {e}")
            return np.zeros((256, 256, 3), dtype=np.uint8)

    def record_step(self):
        if not self.is_moving or self.latest_controller_state is None:
            return
            
        # Get State (Actual)
        # We need to map controller state back to our joint order if needed, 
        # but usually controller state is ordered by joint_names in the message.
        # Let's assume the controller state follows the standard UR order or we map it.
        # For simplicity, let's use the values directly if names match, or map them.
        
        state_map = {name: 0.0 for name in self.joint_names}
        action_map = {name: 0.0 for name in self.joint_names}
        
        # Map Actual
        for i, name in enumerate(self.latest_controller_state.joint_names):
            if name in state_map:
                state_map[name] = self.latest_controller_state.actual.positions[i]
                action_map[name] = self.latest_controller_state.desired.positions[i]
                
        state_vec = [float(state_map[n]) for n in self.joint_names] + [0.0] # Add dummy gripper
        action_vec = [float(action_map[n]) for n in self.joint_names] + [0.0] # Add dummy gripper
        
        # Image
        img = self.generate_dummy_image()
        # In a real scenario, we would encode this or save frames. 
        # For Parquet, we can store flattened arrays or save video separately.
        # LeRobot often uses video files referenced in parquet or embedded.
        # The analysis script showed "video" type. 
        # For simplicity in this generator, we will NOT save the full video to parquet to avoid huge files,
        # but we will save the image as a flattened array or just skip if it's too heavy, 
        # OR we can save individual frames to a folder.
        # However, the requirement says "Save the data as Parquet files".
        # Let's try to store the image in the parquet for small datasets, or just metadata.
        # The analysis script showed `observation.images.egoview: video`.
        # Usually this means the parquet contains paths or the data is in a separate video file.
        # Given the constraints, I will just store the image as a flattened list in the parquet for now,
        # or better, just store the state/action to ensure it works first.
        # Wait, the user explicitly asked for "Images: Capture the current viewport".
        # I will save the image to a separate folder and put the path in parquet, OR embed it.
        # Let's embed it as a byte array or list for now to be self-contained in parquet.
        # Actually, `analyze_dataset.py` didn't show image columns in the `head()` print because it filtered.
        # I'll store it as a flattened float array normalized 0-1 or uint8.
        
        # Current Task
        target_info = self.target_poses[self.current_target_idx]
        task_desc = target_info[2]
        task_idx = self.task_map[task_desc]
        
        frame_data = {
            "observation.state": state_vec,
            "action": action_vec,
            "timestamp": time.time(),
            "episode_index": self.current_episode_idx,
            "index": self.frame_index,
            "task_index": task_idx, 
            "annotation.human.action.task_description": task_idx, # Store ID
            "observation.images.egoview": img # Keep as numpy array for video writing
        }
        # Note: Parquet handles binary well.
        
        self.episode_data.append(frame_data)
        self.frame_index += 1

    def control_loop(self):
        if self.current_episode_idx >= self.target_episodes:
            self.get_logger().info("Finished generating dataset.")
            self.save_dataset()
            raise SystemExit
            return

        if self.is_moving:
            # Check if move is done
            if time.time() - self.move_start_time > self.move_duration + 1.0: # +1s buffer
                self.is_moving = False
                self.get_logger().info("Move complete.")
                
                # Prepare next move
                self.current_target_idx = (self.current_target_idx + 1) % len(self.target_poses)
                
                # If we completed a full cycle or just one move? 
                # Let's say 1 move = 1 episode for simplicity, or a sequence.
                # Let's make 1 episode = 1 move.
                self.save_episode()
                self.current_episode_idx += 1
                self.episode_data = []
                self.frame_index = 0
        else:
            # Plan and Execute
            self.execute_next_move()

    def execute_next_move(self):
        if self.current_joint_state is None:
            self.get_logger().warn("Waiting for joint state...")
            return

        target_pos, target_quat, task_desc = self.target_poses[self.current_target_idx]
        self.get_logger().info(f"Planning move to: {task_desc}")
        
        # Create Goal
        goal = CuroboPose(
            position=torch.tensor([target_pos], device=self.tensor_args.device, dtype=torch.float32),
            quaternion=torch.tensor([target_quat], device=self.tensor_args.device, dtype=torch.float32),
        )
        
        # Current State
        start_state = CuroboJointState.from_position(
            torch.tensor([self.current_joint_state], device=self.tensor_args.device, dtype=torch.float32),
            joint_names=self.joint_names,
        )
        
        # Plan
        result = self.motion_gen.plan_single(start_state, goal, MotionGenPlanConfig(max_attempts=1))
        
        if result.success.item():
            traj = result.interpolated_plan
            self.get_logger().info(f"Plan found! Duration: {traj.position.shape[0] * self.dt:.2f}s")
            
            # Publish Trajectory
            ros_traj = JointTrajectory()
            ros_traj.joint_names = self.joint_names
            
            # Use interpolated plan directly
            # traj is a JointState with [batch, steps, dof]
            positions = traj.position.squeeze(0)
            velocities = traj.velocity.squeeze(0) if traj.velocity is not None else None
            accelerations = traj.acceleration.squeeze(0) if traj.acceleration is not None else None
            
            steps = positions.shape[0]
            
            # Convert to ROS msg
            for i in range(steps):
                point = JointTrajectoryPoint()
                point.positions = positions[i].tolist()
                
                if velocities is not None:
                    point.velocities = velocities[i].tolist()
                
                if accelerations is not None:
                    point.accelerations = accelerations[i].tolist()
                
                point.time_from_start.sec = int(i * self.dt)
                point.time_from_start.nanosec = int((i * self.dt - int(i * self.dt)) * 1e9)
                ros_traj.points.append(point)
                
            self.traj_pub.publish(ros_traj)
            
            self.is_moving = True
            self.move_start_time = time.time()
            self.move_duration = steps * self.dt
        else:
            self.get_logger().error("Planning failed!")

    def save_episode(self):
        if not self.episode_data:
            return
            
        # Filenames
        filename_base = f"train-{self.current_episode_idx:05d}-of-{self.target_episodes:05d}"
        parquet_path = os.path.join(self.output_dir, "data", "chunk-000", filename_base + ".parquet")
        video_path = os.path.join(self.output_dir, "videos", "chunk-000", filename_base + ".mp4")
        
        try:
            # 1. Save Video
            images = [d["observation.images.egoview"] for d in self.episode_data]
            if images:
                # Ensure images are clean numpy arrays
                clean_images = []
                for img in images:
                    if not isinstance(img, np.ndarray):
                        img = np.array(img)
                    clean_images.append(img)
                
                # Save using imageio (expects RGB)
                try:
                    # Use macro_block_size=None to avoid resizing if dimensions are not divisible by 16
                    imageio.mimsave(video_path, clean_images, fps=10.0, codec='libx264', macro_block_size=None)
                except Exception as e:
                    self.get_logger().error(f"Failed to save video with imageio: {e}")
                    # Fallback or ignore? If video fails, training will fail.
                    # But let's continue to save parquet so we don't lose data.
            
            # 2. Prepare Data for Parquet (Remove images, normalize timestamps)
            start_time = self.episode_data[0]["timestamp"]
            clean_data = []
            for d in self.episode_data:
                new_d = d.copy()
                del new_d["observation.images.egoview"] # Remove image
                new_d["timestamp"] = new_d["timestamp"] - start_time # Normalize timestamp
                clean_data.append(new_d)
            
            # 3. Save Parquet
            data_dict = {k: [d[k] for d in clean_data] for k in clean_data[0].keys()}
            table = pa.Table.from_pydict(data_dict)
            pq.write_table(table, parquet_path)
            
            # Update Meta
            task_idx = self.episode_data[0]["annotation.human.action.task_description"]
            task_desc = self.idx_to_task[task_idx]
            self.episodes_meta.append({
                "episode_index": self.current_episode_idx,
                "tasks": [task_desc],
                "length": len(self.episode_data)
            })
            self.get_logger().info(f"Saved episode {self.current_episode_idx} to {parquet_path} and {video_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save episode: {e}")

    def save_dataset(self):
        # Save tasks.jsonl
        with open(os.path.join(self.output_dir, "meta", "tasks.jsonl"), 'w') as f:
            for desc, idx in self.task_map.items():
                f.write(json.dumps({"task_index": idx, "task": desc}) + "\n")

        # Save episodes.jsonl
        with open(os.path.join(self.output_dir, "meta", "episodes.jsonl"), 'w') as f:
            for ep in self.episodes_meta:
                f.write(json.dumps(ep) + "\n")
        self.get_logger().info("Dataset generation complete.")

def main(args=None):
    rclpy.init(args=args)
    node = DatasetGeneratorNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
