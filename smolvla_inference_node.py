#!/usr/bin/env python3
"""
SmolVLA Inference Node for UR5 Robot — Absolute Joint Position Control
======================================================================

Runs a finetuned SmolVLA policy and sends absolute joint-position targets
to the scaled_joint_trajectory_controller — no controller switching needed.

    Camera + JointState → SmolVLA (10 Hz) → absolute joint positions (7D)
                                             ↓
    Publish JointTrajectory (single-point, duration = 1/fps)
        → /scaled_joint_trajectory_controller/joint_trajectory
    Gripper from target[6]
        → /robotiq_2f_urcap_adapter/gripper_command

This keeps the default trajectory controller active (same one used by
MoveIt, the UI, cuRobo, etc.), so it works alongside the normal stack.

Prerequisites:
    - launch_all.sh must be running (UR5 driver + joint_states)
    - lerobot + smolvla dependencies installed:
        pip install -e ".[smolvla]"  (in the lerobot repo)
    - A finetuned SmolVLA checkpoint (or the base model for testing)

Usage:
    # Via helper script (recommended):
    ./run_smolvla_inference.sh

    # Direct:
    python smolvla_inference_node.py \\
        --model outputs/train/ur5_smolvla/checkpoints/last/pretrained_model \\
        --task "pick and place object" \\
        --webcam 0

Controls:
    Ctrl-C = quit (stops cleanly)
"""

import argparse
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

# ---- ROS 2 ----
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# UR joint name order as reported by joint_state_broadcaster
_UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# State / action dim: 6 joints + 1 gripper (matches convert_to_lerobot.py)
_STATE_DIM = 7


class SmolVLAInferenceNode(Node):
    """
    SmolVLA policy → absolute joint positions → scaled_joint_trajectory_controller.

    ┌─────────────────────────────────────────────────────────────────┐
    │  Inference loop (10 Hz):                                        │
    │    capture cameras → build observation → model.select_action    │
    │    → publish JointTrajectory (single waypoint, dur = 1/fps)     │
    │    → handle gripper from action[6]                              │
    └─────────────────────────────────────────────────────────────────┘
    """

    def __init__(
        self,
        model_path: str,
        task: str,
        device: str,
        webcam_index: int | None,
        wrist_index: int | None,
        image_size: tuple[int, int],
        inference_fps: float,
        duration_scale: float,
        max_delta: float,
        gripper_threshold: float,
        dry_run: bool,
    ):
        super().__init__("smolvla_inference")

        self._model_path = model_path
        self._task = task
        self._device_str = device
        self._webcam_index = webcam_index
        self._wrist_index = wrist_index
        self._image_size = image_size  # (H, W)
        self._inference_fps = inference_fps
        self._duration_scale = duration_scale
        self._max_delta = max_delta  # max rad change per step (safety clamp)
        self._gripper_threshold = gripper_threshold
        self._dry_run = dry_run

        # ---- state ----
        self._js_lock = threading.Lock()
        self._joint_pos = None       # ndarray (6,) — current joint positions
        self._gripper_pos = 0.0      # 0=open, 1=closed

        self._prev_gripper_cmd = -1.0
        self._gripper_busy = False

        # ---- publishers ----
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            "/scaled_joint_trajectory_controller/joint_trajectory",
            10,
        )
        self._grip_viz_pub = self.create_publisher(
            Float64, "/gripper_position_command", 10
        )

        # ---- joint state subscriber ----
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        # ---- load model (blocking) ----
        self.get_logger().info(f"Loading SmolVLA model from: {model_path}")
        self._load_model()
        self.get_logger().info("Model loaded successfully!")

        # ---- open cameras ----
        self._webcam = None
        self._wrist_cam = None
        if webcam_index is not None:
            self._webcam = cv2.VideoCapture(webcam_index)
            if not self._webcam.isOpened():
                self.get_logger().error(f"Cannot open webcam index {webcam_index}")
                sys.exit(1)
            self.get_logger().info(f"Webcam opened: index {webcam_index}")
        if wrist_index is not None:
            self._wrist_cam = cv2.VideoCapture(wrist_index)
            if not self._wrist_cam.isOpened():
                self.get_logger().error(f"Cannot open wrist camera index {wrist_index}")
                sys.exit(1)
            self.get_logger().info(f"Wrist camera opened: index {wrist_index}")

        # ---- start inference loop ----
        self._inference_running = True
        self._inference_thread = threading.Thread(
            target=self._inference_loop, daemon=True, name="smolvla_inference"
        )
        self._inference_thread.start()

        self.get_logger().info(
            f"SmolVLA Inference ready  (fps={inference_fps}, "
            f"duration_scale={duration_scale}, max_delta={max_delta} rad)"
        )
        self.get_logger().info(f'  Task: "{task}"')
        if dry_run:
            self.get_logger().warn("DRY RUN mode — commands will NOT be published")

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------

    def _load_model(self):
        import torch
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

        self._torch = torch
        self._device = torch.device(self._device_str)

        self._model = SmolVLAPolicy.from_pretrained(self._model_path)
        self._model.eval()

        self._preprocess, self._postprocess = make_pre_post_processors(
            self._model.config,
            self._model_path,
            preprocessor_overrides={"device_processor": {"device": self._device_str}},
        )

        self._dataset_features = self._build_dataset_features()

    def _build_dataset_features(self) -> dict:
        """Match the features from convert_to_lerobot.py."""
        motor_names = _UR_JOINT_NAMES + ["gripper"]

        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (_STATE_DIM,),
                "names": motor_names,
            },
            "action": {
                "dtype": "float32",
                "shape": (_STATE_DIM,),
                "names": motor_names,
            },
        }

        if self._webcam is not None:
            features["observation.images.webcam"] = {
                "dtype": "video",
                "shape": (self._image_size[0], self._image_size[1], 3),
                "names": ["height", "width", "channels"],
            }
        if self._wrist_cam is not None:
            features["observation.images.wrist"] = {
                "dtype": "video",
                "shape": (self._image_size[0], self._image_size[1], 3),
                "names": ["height", "width", "channels"],
            }

        return features

    # ------------------------------------------------------------------
    # Joint state callback
    # ------------------------------------------------------------------

    def _js_cb(self, msg: JointState):
        try:
            q = [msg.position[msg.name.index(n)] for n in _UR_JOINT_NAMES]
        except (ValueError, IndexError):
            return
        with self._js_lock:
            self._joint_pos = np.array(q, dtype=np.float32)

        # Gripper position from same message
        for gname in ("finger_joint", "robotiq_85_left_knuckle_joint"):
            if gname in msg.name:
                idx = msg.name.index(gname)
                raw = msg.position[idx]
                self._gripper_pos = float(np.clip(raw / 0.8, 0.0, 1.0))
                break

    # ------------------------------------------------------------------
    # Camera capture
    # ------------------------------------------------------------------

    def _capture_cameras(self) -> dict:
        images = {}
        if self._webcam is not None:
            ret, frame = self._webcam.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = cv2.resize(frame, (self._image_size[1], self._image_size[0]))
                images["observation.images.webcam"] = frame.astype(np.uint8)
            else:
                images["observation.images.webcam"] = np.zeros(
                    (*self._image_size, 3), dtype=np.uint8
                )
        if self._wrist_cam is not None:
            ret, frame = self._wrist_cam.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = cv2.resize(frame, (self._image_size[1], self._image_size[0]))
                images["observation.images.wrist"] = frame.astype(np.uint8)
            else:
                images["observation.images.wrist"] = np.zeros(
                    (*self._image_size, 3), dtype=np.uint8
                )
        return images

    # ------------------------------------------------------------------
    # Inference loop  (10 Hz)
    # ------------------------------------------------------------------

    def _inference_loop(self):
        import torch
        from lerobot.policies.utils import build_inference_frame

        period = 1.0 / self._inference_fps

        # Wait for first joint state
        self.get_logger().info("Waiting for /joint_states ...")
        while self._inference_running:
            with self._js_lock:
                if self._joint_pos is not None:
                    break
            time.sleep(0.1)
        self.get_logger().info("Joint states received — starting inference loop.")

        step = 0
        while self._inference_running:
            t0 = time.monotonic()

            # ---- build observation ----
            with self._js_lock:
                q = self._joint_pos.copy()
            state = np.zeros(_STATE_DIM, dtype=np.float32)
            state[:6] = q
            state[6] = self._gripper_pos

            images = self._capture_cameras()

            obs_raw = {"observation.state": state}
            obs_raw.update(images)

            obs_frame = build_inference_frame(
                observation=obs_raw,
                ds_features=self._dataset_features,
                device=self._device,
                task=self._task,
                robot_type="ur5",
            )

            obs = self._preprocess(obs_frame)

            with torch.no_grad():
                action = self._model.select_action(obs)

            action = self._postprocess(action)

            # action is (1, 7) or (7,) — squeeze to numpy
            action_np = action.squeeze(0).cpu().numpy()  # (7,)

            # ---- safety clamp: limit per-step joint delta ----
            target_joints = action_np[:6].copy()
            delta = target_joints - q
            clipped = np.clip(delta, -self._max_delta, self._max_delta)
            if not np.allclose(delta, clipped):
                self.get_logger().warn(
                    f"Clamped delta: max |delta|={np.max(np.abs(delta)):.4f} → {self._max_delta:.4f}"
                )
                target_joints = q + clipped

            # ---- publish trajectory ----
            if not self._dry_run:
                self._publish_trajectory(target_joints)

            # ---- gripper ----
            gripper_target = float(action_np[6])
            gripper_cmd = 1.0 if gripper_target > self._gripper_threshold else 0.0
            if gripper_cmd != self._prev_gripper_cmd and not self._gripper_busy:
                self._prev_gripper_cmd = gripper_cmd
                viz_msg = Float64()
                viz_msg.data = gripper_cmd
                self._grip_viz_pub.publish(viz_msg)
                if not self._dry_run:
                    threading.Thread(
                        target=self._send_gripper_cmd,
                        args=(gripper_cmd,),
                        daemon=True,
                    ).start()

            # ---- logging ----
            step += 1
            if step % 10 == 0:
                self.get_logger().info(
                    f"[step {step}] delta: [{', '.join(f'{d:+.4f}' for d in (target_joints - q))}]  "
                    f"gripper: {gripper_target:.2f}"
                )

            # ---- sleep remainder ----
            elapsed = time.monotonic() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    # ------------------------------------------------------------------
    # Publish single-point trajectory
    # ------------------------------------------------------------------

    def _publish_trajectory(self, target_positions: np.ndarray):
        """Publish a single-waypoint JointTrajectory to the trajectory controller."""
        msg = JointTrajectory()
        msg.joint_names = list(_UR_JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = target_positions.tolist()
        point.velocities = [0.0] * 6

        # Duration for the move: one inference period, scaled
        dur = self._duration_scale / self._inference_fps
        point.time_from_start = Duration(
            sec=int(dur),
            nanosec=int((dur % 1) * 1e9),
        )
        msg.points.append(point)

        self._traj_pub.publish(msg)

    # ------------------------------------------------------------------
    # Gripper command (background thread)
    # ------------------------------------------------------------------

    def _send_gripper_cmd(self, position: float):
        self._gripper_busy = True

        # Robotiq 2F-85: 0.085m = fully open, 0.0m = fully closed
        robotiq_pos = (1.0 - position) * 0.085

        cmd = (
            "unset LD_PRELOAD && "
            "source /opt/ros/humble/setup.bash && "
            "source /home/rml/ur5-robotiq-ros2-control/install/setup.bash && "
            "ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command "
            "robotiq_2f_urcap_adapter/action/GripperCommand "
            f"'{{command: {{position: {robotiq_pos}, max_effort: 100.0, max_speed: 0.1}}}}'"
        )
        try:
            result = subprocess.run(
                cmd, shell=True, capture_output=True, text=True,
                timeout=15, executable="/bin/bash",
            )
            if result.returncode == 0:
                self.get_logger().info(
                    f'Gripper {"closed" if position >= 0.5 else "opened"}'
                )
            else:
                self.get_logger().warn(
                    f"Gripper action returned {result.returncode}: {result.stderr.strip()}"
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Gripper action timed out")
        except Exception as e:
            self.get_logger().warn(f"Gripper action error: {e}")
        finally:
            self._gripper_busy = False

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info("Stopping ...")
        self._inference_running = False
        if self._webcam is not None:
            self._webcam.release()
        if self._wrist_cam is not None:
            self._wrist_cam.release()
        super().destroy_node()


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="SmolVLA inference → UR5 absolute joint position control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--model", required=True,
        help="Path to finetuned SmolVLA checkpoint directory "
             "(e.g. outputs/train/ur5_smolvla/checkpoints/last/pretrained_model)",
    )
    p.add_argument(
        "--task", default="robot task",
        help="Language task description for the policy (default: 'robot task')",
    )
    p.add_argument(
        "--device", default="cuda",
        help="Torch device: cuda, cpu, mps (default: cuda)",
    )
    p.add_argument(
        "--webcam", type=int, default=None,
        help="OpenCV camera index for webcam (observation.images.webcam)",
    )
    p.add_argument(
        "--wrist", type=int, default=None,
        help="OpenCV camera index for wrist camera (observation.images.wrist)",
    )
    p.add_argument(
        "--image-size", type=int, nargs=2, default=[480, 640], metavar=("H", "W"),
        help="Camera capture resolution (H W). Default: 480 640",
    )
    p.add_argument(
        "--inference-fps", type=float, default=10.0,
        help="Inference rate in Hz (should match dataset FPS). Default: 10",
    )
    p.add_argument(
        "--duration-scale", type=float, default=1.0,
        help="Scale factor for trajectory point duration. "
             "1.0 = real-time (1/fps per step). >1 = slower. Default: 1.0",
    )
    p.add_argument(
        "--max-delta", type=float, default=0.1,
        help="Max joint position change per step in rad (safety clamp). Default: 0.1",
    )
    p.add_argument(
        "--gripper-threshold", type=float, default=0.5,
        help="Gripper action threshold (>threshold = close). Default: 0.5",
    )
    p.add_argument(
        "--dry-run", action="store_true",
        help="Run inference but don't publish commands (for testing)",
    )
    return p.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = None
    try:
        node = SmolVLAInferenceNode(
            model_path=args.model,
            task=args.task,
            device=args.device,
            webcam_index=args.webcam,
            wrist_index=args.wrist,
            image_size=tuple(args.image_size),
            inference_fps=args.inference_fps,
            duration_scale=args.duration_scale,
            max_delta=args.max_delta,
            gripper_threshold=args.gripper_threshold,
            dry_run=args.dry_run,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
