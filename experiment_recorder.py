#!/usr/bin/env python3
"""
Standalone episode recorder for UR5 programs (ML data collection).

Records joint states (HDF5 at configurable Hz), webcam video (MP4), GoPro wrist
video (MP4), and program execution events into a single recording folder.
Designed for capturing individual episodes that are later converted into
LeRobot datasets for policy training (see ``convert_to_lerobot.py``).

This is distinct from ``SessionRecorder`` in ``program_selector_ui.py``, which
records multi-program session timelines for long-horizon experiments.

Used by:
  - ``record_program_episode.py`` (automated CLI recorder)

Output structure per recording::

    recordings/<name>_<timestamp>/
        joint_states.h5        # positions, velocities, gripper, timestamps
        video.mp4              # webcam
        gopro_video.mp4        # GoPro wrist camera (optional)
        metadata.json
        program_events.json
"""

import json
import os
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

# Optional dependencies --------------------------------------------------
try:
    import cv2
    VIDEO_AVAILABLE = True
except ImportError:
    VIDEO_AVAILABLE = False

try:
    import h5py
    HDF5_AVAILABLE = True
except ImportError:
    HDF5_AVAILABLE = False

try:
    from gopro_recorder import GoProRecorder
    GOPRO_AVAILABLE = True
except ImportError:
    GOPRO_AVAILABLE = False


class EpisodeRecorder:
    """Records video and joint states for a robot experiment/episode.

    Parameters
    ----------
    experiment_name : str
        A short label for the recording (used in folder name).
    controller : object or None
        An object exposing ``latest_joint_state`` and ``latest_gripper_state``
        attributes (typically a ROS 2 controller wrapper).  When *None* joint
        data will not be recorded.
    recordings_dir : str or Path
        Parent directory for recording folders (default: ``recordings/``).
    webcam_index : int or None
        OpenCV camera index for the 3rd-person webcam.  Set to *None* to skip.
    gopro_ip : str or None
        IP address of the GoPro Max.  Set to *None* to skip.
    sample_rate_hz : float
        Joint-state / video capture rate.
    """

    def __init__(
        self,
        experiment_name: str = "experiment",
        controller=None,
        recordings_dir: str | Path = "recordings",
        webcam_index: int | None = 0,
        gopro_ip: str | None = None,
        sample_rate_hz: float = 50.0,
    ):
        self.experiment_name = experiment_name
        self.controller = controller
        self.recordings_dir = Path(recordings_dir)
        self.webcam_index = webcam_index
        self.gopro_ip = gopro_ip
        self.sample_rate_hz = sample_rate_hz

        # State
        self.is_recording = False
        self.video_writer = None
        self.video_capture = None
        self.gopro_recorder = None
        self.joint_data: list[dict] = []
        self.gripper_data: list[dict | None] = []
        self.timestamps: list[float] = []
        self.program_events: list[dict] = []
        self.start_time: float | None = None
        self.save_path: Path | None = None
        self._recording_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._joint_names: list[str] | None = None

    # ------------------------------------------------------------------ #
    #  Public API
    # ------------------------------------------------------------------ #

    def start_recording(self) -> tuple[bool, str]:
        """Start capturing video + joint states in a background thread."""
        if self.is_recording:
            return False, "Already recording"

        # Create save directory
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_path = self.recordings_dir / f"{self.experiment_name}_{ts_str}"
        self.save_path.mkdir(parents=True, exist_ok=True)

        # Reset data buffers
        self.joint_data = []
        self.gripper_data = []
        self.timestamps = []
        self.program_events = [{"event": "recording_start", "timestamp": 0.0, "program": None}]

        # ---- Webcam ---------------------------------------------------
        warnings: list[str] = []
        if VIDEO_AVAILABLE and self.webcam_index is not None:
            os.environ["OPENCV_LOG_LEVEL"] = "ERROR"
            # Auto-detect: scan common indices if specified index fails
            if self.webcam_index == 0:
                # Index 0 is the default "auto" value — scan 0-9
                indices = list(range(10))
            else:
                # Explicit index given — try it, then fall back to scan
                indices = [self.webcam_index] + [i for i in range(10) if i != self.webcam_index]
            opened = False
            for idx in indices:
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    self.video_capture = cap
                    opened = True
                    print(f"✓ Webcam opened at /dev/video{idx}")
                    break
                cap.release()
            if not opened:
                msg = "Could not open webcam (tried indices 0-9) — continuing without webcam"
                print(f"⚠ {msg}")
                warnings.append(msg)
                self.video_capture = None

            if self.video_capture is not None:
                fps = 30.0
                w = int(self.video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(self.video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                self.video_writer = cv2.VideoWriter(
                    str(self.save_path / "video.mp4"), fourcc, fps, (w, h)
                )
                if not self.video_writer.isOpened():
                    self.video_capture.release()
                    self.video_capture = None
                    msg = "Could not initialize video writer — continuing without webcam"
                    print(f"⚠ {msg}")
                    warnings.append(msg)

        # ---- GoPro (wrist) -------------------------------------------
        if GOPRO_AVAILABLE and self.gopro_ip is not None:
            try:
                self.gopro_recorder = GoProRecorder(gopro_ip=self.gopro_ip)
                gopro_path = self.save_path / "gopro_video.mp4"
                ok, msg = self.gopro_recorder.start_recording(gopro_path)
                if ok:
                    print(f"✓ {msg}")
                else:
                    print(f"⚠ GoPro: {msg}")
                    self.gopro_recorder = None
            except Exception as e:
                print(f"⚠ GoPro init failed: {e}")
                self.gopro_recorder = None

        # ---- Start background thread ---------------------------------
        self.start_time = time.time()
        self.is_recording = True
        self._stop_event.clear()
        self._recording_thread = threading.Thread(target=self._recording_loop, daemon=True)
        self._recording_thread.start()

        status = f"Recording started: {self.save_path}"
        if warnings:
            status += " (warnings: " + "; ".join(warnings) + ")"
        return True, status

    def log_program_event(self, event_type: str, program_name: str | None = None):
        """Log a program execution event (execute, pause, resume, stop)."""
        if not self.is_recording or self.start_time is None:
            return
        elapsed = time.time() - self.start_time
        event = {"event": event_type, "timestamp": float(elapsed), "program": program_name}
        self.program_events.append(event)
        print(f"[Log Event] {event_type} — {program_name} at {elapsed:.2f}s")

    def stop_recording(self) -> tuple[bool, str]:
        """Stop recording, save all data, and return (success, message)."""
        if not self.is_recording:
            return False, "Not currently recording"

        self.is_recording = False
        self._stop_event.set()

        if self._recording_thread:
            self._recording_thread.join(timeout=3.0)

        # Release webcam resources
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        if self.video_capture:
            self.video_capture.release()
            self.video_capture = None

        # Stop GoPro
        if self.gopro_recorder:
            try:
                ok, msg = self.gopro_recorder.stop_recording()
                print(f"{'✓' if ok else '⚠'} {msg}")
            except Exception as e:
                print(f"⚠ Error stopping GoPro: {e}")
            self.gopro_recorder = None

        duration = time.time() - self.start_time

        # Save HDF5
        self._save_h5(duration)

        # Save metadata JSON
        self._save_metadata(duration)

        # Save program events JSON
        self._save_program_events(duration)

        return True, f"Episode saved to: {self.save_path}"

    # ------------------------------------------------------------------ #
    #  Internal helpers
    # ------------------------------------------------------------------ #

    def _recording_loop(self):
        """Background capture loop — joint states + video at sample_rate_hz."""
        min_interval = 1.0 / self.sample_rate_hz
        last_sample = time.time()
        joint_names_cached = None

        while self.is_recording and not self._stop_event.is_set():
            now = time.time()
            if now - last_sample < min_interval:
                time.sleep(0.001)
                continue
            last_sample = now
            elapsed = now - self.start_time

            # Webcam frame
            if self.video_capture and self.video_capture.isOpened():
                ret, frame = self.video_capture.read()
                if ret and self.video_writer:
                    self.video_writer.write(frame)

            # Joint state
            if self.controller:
                joint_state = getattr(self.controller, "latest_joint_state", None)
                gripper_state = getattr(self.controller, "latest_gripper_state", None)

                if joint_state is not None:
                    if joint_names_cached is None:
                        joint_names_cached = list(joint_state.name)
                        self._joint_names = joint_names_cached

                    self.timestamps.append(elapsed)
                    self.joint_data.append({
                        "positions": np.array(joint_state.position, dtype=np.float32),
                        "velocities": (
                            np.array(joint_state.velocity, dtype=np.float32)
                            if joint_state.velocity else None
                        ),
                        "efforts": (
                            np.array(joint_state.effort, dtype=np.float32)
                            if joint_state.effort else None
                        ),
                    })

                    if gripper_state and gripper_state.position:
                        self.gripper_data.append({
                            "position": float(gripper_state.position[0]),
                            "velocity": float(gripper_state.velocity[0]) if gripper_state.velocity else 0.0,
                            "effort": float(gripper_state.effort[0]) if gripper_state.effort else 0.0,
                        })
                    else:
                        self.gripper_data.append(None)

    # ---- Persistence helpers -----------------------------------------

    def _save_h5(self, duration: float):
        if not HDF5_AVAILABLE or not self.joint_data:
            return
        h5_path = self.save_path / "joint_states.h5"
        try:
            with h5py.File(h5_path, "w") as f:
                f.create_dataset("timestamps", data=np.array(self.timestamps, dtype=np.float32), compression="gzip")

                # Positions — pad to uniform width
                positions_list = [d["positions"] for d in self.joint_data if d.get("positions") is not None]
                if positions_list:
                    max_dim = max(len(p) for p in positions_list)
                    padded = np.zeros((len(positions_list), max_dim), dtype=np.float32)
                    for i, p in enumerate(positions_list):
                        padded[i, : len(p)] = p
                    f.create_dataset("positions", data=padded, compression="gzip")

                # Velocities
                if any(d.get("velocities") is not None for d in self.joint_data):
                    vels = np.array([
                        d["velocities"] if d["velocities"] is not None else np.zeros(6, dtype=np.float32)
                        for d in self.joint_data
                    ], dtype=np.float32)
                    f.create_dataset("velocities", data=vels, compression="gzip")

                # Gripper
                if self.gripper_data and any(g is not None for g in self.gripper_data):
                    g_pos = np.array([g["position"] if g else 0.0 for g in self.gripper_data], dtype=np.float32)
                    f.create_dataset("gripper_position", data=g_pos, compression="gzip")
                    g_vel = np.array([g["velocity"] if g else 0.0 for g in self.gripper_data], dtype=np.float32)
                    f.create_dataset("gripper_velocity", data=g_vel, compression="gzip")
                    g_eff = np.array([g["effort"] if g else 0.0 for g in self.gripper_data], dtype=np.float32)
                    f.create_dataset("gripper_effort", data=g_eff, compression="gzip")

                # Metadata attributes
                f.attrs["experiment_name"] = self.experiment_name
                f.attrs["start_time"] = datetime.fromtimestamp(self.start_time).isoformat()
                f.attrs["duration_seconds"] = float(duration)
                f.attrs["num_samples"] = len(self.joint_data)
                f.attrs["sample_rate_hz"] = self.sample_rate_hz

                if self._joint_names:
                    f.create_dataset("joint_names", data=np.array(self._joint_names, dtype="S64"))
        except Exception as e:
            print(f"[HDF5] Failed to save: {e}")

    def _save_metadata(self, duration: float):
        try:
            metadata = {
                "experiment_name": self.experiment_name,
                "timestamp": datetime.now().isoformat(),
                "duration_seconds": duration,
                "num_joint_samples": len(self.joint_data),
                "sample_rate_hz": self.sample_rate_hz,
                "video_file": "video.mp4",
                "joint_data_file": "joint_states.h5" if HDF5_AVAILABLE else "joint_states.json",
                "program_events_file": "program_events.json",
            }
            if self.gopro_recorder and (self.save_path / "gopro_video.mp4").exists():
                metadata["gopro_video_file"] = "gopro_video.mp4"
            with open(self.save_path / "metadata.json", "w") as f:
                json.dump(metadata, f, indent=2)
        except Exception as e:
            print(f"[Metadata] Failed to save: {e}")

    def _save_program_events(self, duration: float):
        try:
            payload = {
                "experiment_name": self.experiment_name,
                "start_time": datetime.fromtimestamp(self.start_time).isoformat(),
                "duration_seconds": float(duration),
                "events": self.program_events,
            }
            with open(self.save_path / "program_events.json", "w") as f:
                json.dump(payload, f, indent=2)
        except Exception as e:
            print(f"[Events] Failed to save: {e}")
