#!/usr/bin/env python3
"""
Automated program episode recorder for UR5.

Runs a .prog file on the UR5 via the program executor ROS 2 services,
records joint states + camera feeds using EpisodeRecorder, and (optionally)
converts the resulting recordings into a LeRobot dataset.

Usage
-----
    # Record 5 episodes of pick_and_place_object.prog
    python record_program_episode.py \
        --program pick_and_place_object.prog \
        --episodes 5 \
        --task "pick and place object"

    # Record and auto-convert to LeRobot format
    python record_program_episode.py \
        --program pick_and_place_object.prog \
        --episodes 3 \
        --task "pick and place object" \
        --convert --repo-id myuser/ur5_pick_place

Prerequisites
-------------
- The ``ur5`` tmux session must already be running (``./launch_all.sh``).
- ``/ur5_program_executor`` services must be available.
"""

import argparse
import os
import sys
import time

# Add ROS 2 paths
ros_paths = [
    "/opt/ros/humble/local/lib/python3.10/dist-packages",
    "/opt/ros/humble/lib/python3.10/site-packages",
]
for p in ros_paths:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger, SetBool
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("ERROR: ROS 2 Python packages not found. Source your ROS 2 workspace first.")
    sys.exit(1)

from experiment_recorder import EpisodeRecorder


class RecordingController(Node):
    """Lightweight ROS 2 node that drives the program executor and records data."""

    EXECUTOR_NS = "/ur5_program_executor"

    def __init__(self, webcam_index: int | None = 0, gopro_ip: str | None = None):
        super().__init__("recording_controller")

        # Service clients
        self.set_params_cli = self.create_client(
            SetParameters, f"{self.EXECUTOR_NS}/set_parameters"
        )
        self.load_cli = self.create_client(Trigger, f"{self.EXECUTOR_NS}/load_program")
        self.execute_cli = self.create_client(Trigger, f"{self.EXECUTOR_NS}/execute_program")
        self.stop_cli = self.create_client(Trigger, f"{self.EXECUTOR_NS}/stop")

        # Status subscriber
        self._latest_status = ""
        self.create_subscription(String, f"{self.EXECUTOR_NS}/status", self._status_cb, 10)

        # Joint state subscriber (shared with EpisodeRecorder via .latest_joint_state)
        self.latest_joint_state = None
        self.latest_gripper_state = None

        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)
        self.create_subscription(
            JointState, "/robotiq_2f_85_gripper_controller/state", self._gripper_cb, 10
        )

        self.webcam_index = webcam_index
        self.gopro_ip = gopro_ip

    # ---- ROS callbacks -----------------------------------------------

    def _status_cb(self, msg: String):
        self._latest_status = msg.data

    def _joint_cb(self, msg: JointState):
        self.latest_joint_state = msg

    def _gripper_cb(self, msg: JointState):
        self.latest_gripper_state = msg

    # ---- Service helpers ---------------------------------------------

    def _wait_for_service(self, client, timeout: float = 10.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Service {client.srv_name} not available!")
            return False
        return True

    def _call_service(self, client, request, timeout: float = 10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def set_program_file(self, program_name: str) -> bool:
        if not self._wait_for_service(self.set_params_cli):
            return False
        req = SetParameters.Request()
        param = Parameter()
        param.name = "program_file"
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=program_name)
        req.parameters = [param]
        result = self._call_service(self.set_params_cli, req)
        return result is not None and result.results[0].successful

    def load_program(self) -> tuple[bool, str]:
        if not self._wait_for_service(self.load_cli):
            return False, "load_program service unavailable"
        result = self._call_service(self.load_cli, Trigger.Request(), timeout=10.0)
        if result:
            return result.success, result.message
        return False, "Timeout"

    def execute_program(self) -> tuple[bool, str]:
        if not self._wait_for_service(self.execute_cli):
            return False, "execute_program service unavailable"
        result = self._call_service(self.execute_cli, Trigger.Request(), timeout=5.0)
        if result:
            return result.success, result.message
        return False, "Timeout"

    def stop_program(self) -> tuple[bool, str]:
        if not self._wait_for_service(self.stop_cli):
            return False, "stop service unavailable"
        result = self._call_service(self.stop_cli, Trigger.Request(), timeout=5.0)
        if result:
            return result.success, result.message
        return False, "Timeout"

    @property
    def status(self) -> str:
        return self._latest_status

    def spin_once(self, timeout: float = 0.1):
        rclpy.spin_once(self, timeout_sec=timeout)

    def wait_for_completion(self, poll_hz: float = 5.0, timeout: float = 300.0) -> bool:
        """Poll the executor status until it indicates IDLE or error."""
        t0 = time.time()
        prev_status = ""
        while time.time() - t0 < timeout:
            self.spin_once(timeout=1.0 / poll_hz)
            s = self.status.lower()
            if s != prev_status:
                print(f"  Status: {self.status}")
                prev_status = s

            # The executor publishes "Idle" or state name when finished
            if "idle" in s or "complete" in s or "finished" in s or "stopped" in s:
                return True
            if "error" in s:
                print(f"  ⚠ Executor error: {self.status}")
                return False

        print("  ⚠ Timed out waiting for program completion")
        return False


def record_episodes(
    program: str,
    num_episodes: int,
    task: str,
    webcam_index: int | None,
    gopro_ip: str | None,
    recordings_dir: str,
    auto_convert: bool,
    repo_id: str | None,
    fps: int,
    no_prompt: bool,
):
    rclpy.init()
    controller = RecordingController(webcam_index=webcam_index, gopro_ip=gopro_ip)

    # Wait for joint states
    print("Waiting for /joint_states topic ...")
    for _ in range(100):
        controller.spin_once(0.1)
        if controller.latest_joint_state is not None:
            break
    if controller.latest_joint_state is None:
        print("ERROR: No joint states received. Is the UR5 driver running?")
        rclpy.shutdown()
        sys.exit(1)

    # Set program file on executor
    print(f"\n{'='*60}")
    print(f"  Program : {program}")
    print(f"  Task    : {task}")
    print(f"  Episodes: {num_episodes}")
    print(f"{'='*60}\n")

    if not controller.set_program_file(program):
        print("ERROR: Could not set program_file parameter on executor")
        rclpy.shutdown()
        sys.exit(1)

    ok, msg = controller.load_program()
    if not ok:
        print(f"ERROR: Could not load program: {msg}")
        rclpy.shutdown()
        sys.exit(1)
    print(f"Loaded: {msg}")

    recorded_dirs = []

    for ep in range(num_episodes):
        print(f"\n--- Episode {ep + 1}/{num_episodes} ---")

        if not no_prompt and ep > 0:
            input("Press ENTER when ready for next episode (reset scene if needed)...")

        # Create recorder for this episode
        recorder = EpisodeRecorder(
            experiment_name="episode",
            controller=controller,
            recordings_dir=recordings_dir,
            webcam_index=webcam_index,
            gopro_ip=gopro_ip,
        )

        # Start recording
        ok, msg = recorder.start_recording()
        if not ok:
            print(f"WARNING: Recording failed to start: {msg}")
            # Still execute so user can iterate
        else:
            print(f"Recording → {recorder.save_path}")

        # Log program start event
        recorder.log_program_event("execute", program)

        # Reload program for fresh execution each episode
        controller.set_program_file(program)
        controller.load_program()

        # Execute program
        ok, msg = controller.execute_program()
        if not ok:
            print(f"WARNING: execute_program failed: {msg}")

        # Wait for completion
        print("  Waiting for program to finish ...")
        controller.wait_for_completion(timeout=300.0)

        # Log stop
        recorder.log_program_event("stop", program)

        # Stop recording
        ok, msg = recorder.stop_recording()
        print(f"  {msg}")

        if recorder.save_path:
            recorded_dirs.append(str(recorder.save_path))

    controller.destroy_node()
    rclpy.shutdown()

    print(f"\n{'='*60}")
    print(f"  Recorded {len(recorded_dirs)} episodes")
    for d in recorded_dirs:
        print(f"    {d}")
    print(f"{'='*60}")

    # Auto-convert to LeRobot format
    if auto_convert and repo_id:
        print(f"\nConverting to LeRobot dataset: {repo_id}")
        from pathlib import Path

        # Only convert the episodes we just recorded
        include = [Path(d).name for d in recorded_dirs]
        from convert_to_lerobot import convert

        convert(
            recordings_dir=Path(recordings_dir),
            repo_id=repo_id,
            fps=fps,
            include=include,
        )


def main():
    parser = argparse.ArgumentParser(
        description="Record UR5 program episodes for SmolVLA training",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--program", "-p", required=True,
        help="Program file name, e.g. pick_and_place_object.prog",
    )
    parser.add_argument(
        "--episodes", "-n", type=int, default=5,
        help="Number of episodes to record (default: 5)",
    )
    parser.add_argument(
        "--task", "-t", type=str, default=None,
        help="Task description for the dataset (default: derived from program name)",
    )
    parser.add_argument(
        "--webcam", type=int, default=0,
        help="Webcam index for 3rd-person view (default: 0 = auto-detect, -1 to disable)",
    )
    parser.add_argument(
        "--gopro-ip", type=str, default="172.29.170.51",
        help="GoPro IP for wrist camera (default: 172.29.170.51, 'none' to disable)",
    )
    parser.add_argument(
        "--recordings-dir", type=str, default="recordings",
        help="Directory to store raw recordings (default: recordings/)",
    )
    parser.add_argument(
        "--convert", action="store_true",
        help="Auto-convert recordings to LeRobot dataset after recording",
    )
    parser.add_argument(
        "--repo-id", type=str, default=None,
        help="HuggingFace repo ID for the LeRobot dataset (required with --convert)",
    )
    parser.add_argument(
        "--fps", type=int, default=10,
        help="Target FPS for LeRobot dataset (default: 10)",
    )
    parser.add_argument(
        "--no-prompt", action="store_true",
        help="Skip prompts between episodes (auto-continue)",
    )
    args = parser.parse_args()

    if args.convert and not args.repo_id:
        parser.error("--repo-id is required when --convert is specified")

    task = args.task
    if task is None:
        # Derive from program name
        from pathlib import Path

        task = Path(args.program).stem.replace("_", " ")

    webcam = args.webcam if args.webcam >= 0 else None
    gopro_ip = args.gopro_ip if args.gopro_ip and args.gopro_ip.lower() != "none" else None

    record_episodes(
        program=args.program,
        num_episodes=args.episodes,
        task=task,
        webcam_index=webcam,
        gopro_ip=gopro_ip,
        recordings_dir=args.recordings_dir,
        auto_convert=args.convert,
        repo_id=args.repo_id,
        fps=args.fps,
        no_prompt=args.no_prompt,
    )


if __name__ == "__main__":
    main()
