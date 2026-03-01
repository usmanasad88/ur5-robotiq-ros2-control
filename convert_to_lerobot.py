#!/usr/bin/env python3
"""
Convert UR5 recording folders into a LeRobot dataset for SmolVLA finetuning.

Reads the HDF5 joint data, MP4 video files, and program event logs produced
by the Streamlit UI (EpisodeRecorder) and writes them to a LeRobotDataset
that can be used directly with ``lerobot-train``.

Usage
-----
    # Convert all recordings in the default directory
    python convert_to_lerobot.py \
        --recordings-dir recordings/ \
        --repo-id $HF_USER/ur5_programs \
        --fps 10

    # Convert specific recordings
    python convert_to_lerobot.py \
        --recordings-dir recordings/ \
        --include experiment_20260119_172308 experiment_20260119_170612 \
        --repo-id $HF_USER/ur5_programs \
        --fps 10

    # Push to HuggingFace Hub after conversion
    python convert_to_lerobot.py \
        --recordings-dir recordings/ \
        --repo-id $HF_USER/ur5_programs \
        --fps 10 \
        --push-to-hub
"""

import argparse
import json
import logging
import re
import shutil
import sys
from pathlib import Path

import cv2
import h5py
import numpy as np

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
UR5_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# State / action dimensionality: 6 joints + 1 gripper
STATE_DIM = 7
DEFAULT_FPS = 10
DEFAULT_IMAGE_SIZE = (480, 640)  # (H, W)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def prog_name_to_task(prog_name: str) -> str:
    """Convert a .prog filename to a human-readable task description.

    ``pick_and_place_object.prog`` → ``"pick and place object"``
    """
    name = Path(prog_name).stem  # strip .prog
    name = name.replace("_", " ").strip()
    return name


def load_h5_data(h5_path: Path) -> dict:
    """Load joint-state HDF5 file produced by EpisodeRecorder."""
    data = {}
    with h5py.File(h5_path, "r") as f:
        data["timestamps"] = f["timestamps"][:]
        data["positions"] = f["positions"][:]

        if "velocities" in f:
            data["velocities"] = f["velocities"][:]
        if "gripper_position" in f:
            data["gripper_position"] = f["gripper_position"][:]
        else:
            # Dummy gripper — fully open
            data["gripper_position"] = np.zeros(len(data["timestamps"]), dtype=np.float32)

        if "joint_names" in f:
            data["joint_names"] = [
                n.decode() if isinstance(n, bytes) else n for n in f["joint_names"][:]
            ]
        else:
            data["joint_names"] = UR5_JOINT_NAMES
    return data


def load_program_events(events_path: Path) -> list[dict]:
    """Load program_events.json and return list of events."""
    with open(events_path) as f:
        content = json.load(f)
    return content.get("events", [])


def segment_episodes(events: list[dict], total_duration: float) -> list[dict]:
    """Split a recording into per-program episodes using event timestamps.

    Each ``execute`` → ``stop`` window becomes one episode.  If there is
    only a ``recording_start`` event (no program events), the whole recording
    is treated as a single episode.
    """
    episodes = []
    current_start = None
    current_prog = None

    for ev in events:
        if ev["event"] == "execute":
            current_start = ev["timestamp"]
            current_prog = ev.get("program", "unknown")
        elif ev["event"] == "stop" and current_start is not None:
            episodes.append(
                {
                    "start": current_start,
                    "end": ev["timestamp"],
                    "program": current_prog,
                    "task": prog_name_to_task(current_prog) if current_prog else "robot task",
                }
            )
            current_start = None
            current_prog = None

    # Fallback: if no execute/stop pairs, use the whole recording
    if not episodes:
        episodes.append(
            {
                "start": 0.0,
                "end": total_duration,
                "program": None,
                "task": "robot task",
            }
        )
    return episodes


class VideoFrameReader:
    """Lazy frame reader for an MP4 file.  Reads frames sequentially and
    allows seeking by timestamp.
    """

    def __init__(self, video_path: Path, target_size: tuple[int, int] | None = None):
        self.cap = cv2.VideoCapture(str(video_path))
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open video: {video_path}")
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.duration = self.total_frames / self.fps if self.fps > 0 else 0.0
        self.target_size = target_size  # (H, W) or None

    def get_frame_at_time(self, t: float) -> np.ndarray | None:
        """Return the BGR frame closest to time *t* (seconds)."""
        frame_idx = int(round(t * self.fps))
        frame_idx = max(0, min(frame_idx, self.total_frames - 1))
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = self.cap.read()
        if not ret:
            return None
        # Convert BGR → RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if self.target_size is not None:
            frame = cv2.resize(frame, (self.target_size[1], self.target_size[0]))
        return frame

    def close(self):
        self.cap.release()


def resample_indices(src_timestamps: np.ndarray, target_fps: float, start: float, end: float):
    """Return (target_timestamps, source_indices) arrays.

    For each target timestamp (evenly spaced at *target_fps* within
    [start, end)), find the nearest source sample index.
    """
    n_frames = max(1, int(round((end - start) * target_fps)))
    target_ts = np.linspace(start, end, n_frames, endpoint=False)
    # Nearest-neighbour lookup
    src_indices = np.searchsorted(src_timestamps, target_ts, side="left")
    src_indices = np.clip(src_indices, 0, len(src_timestamps) - 1)
    return target_ts, src_indices


# ---------------------------------------------------------------------------
# Main conversion
# ---------------------------------------------------------------------------
def discover_recordings(recordings_dir: Path, include: list[str] | None = None) -> list[Path]:
    """Return sorted list of valid recording directories."""
    dirs = sorted(p for p in recordings_dir.iterdir() if p.is_dir())
    if include:
        dirs = [d for d in dirs if d.name in include]

    # Filter to those that actually have joint data
    valid = []
    for d in dirs:
        if (d / "joint_states.h5").exists():
            valid.append(d)
        else:
            logger.warning("Skipping %s (no joint_states.h5)", d.name)
    return valid


def convert(
    recordings_dir: Path,
    repo_id: str,
    fps: int = DEFAULT_FPS,
    root: Path | None = None,
    include: list[str] | None = None,
    image_size: tuple[int, int] = DEFAULT_IMAGE_SIZE,
    push_to_hub: bool = False,
):
    """Convert recordings to a LeRobotDataset."""
    # Late import so the script can show --help without lerobot installed
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    rec_dirs = discover_recordings(recordings_dir, include)
    if not rec_dirs:
        logger.error("No valid recordings found in %s", recordings_dir)
        sys.exit(1)

    logger.info("Found %d recording(s) to convert", len(rec_dirs))

    # ------------------------------------------------------------------
    # Probe first recording to determine which cameras are present
    # ------------------------------------------------------------------
    sample_dir = rec_dirs[0]
    has_webcam = (sample_dir / "video.mp4").exists()
    has_gopro = (sample_dir / "gopro_video.mp4").exists()
    # Also check for alternate naming (input/output mp4)
    if not has_webcam and (sample_dir / "input.mp4").exists():
        has_webcam = True
    if not has_gopro and (sample_dir / "output.mp4").exists():
        has_gopro = True

    if not has_webcam and not has_gopro:
        logger.warning("No video files found — dataset will not contain images!")

    # ------------------------------------------------------------------
    # Define features
    # ------------------------------------------------------------------
    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (STATE_DIM,),
            "names": {
                "motors": UR5_JOINT_NAMES + ["gripper"],
            },
        },
        "action": {
            "dtype": "float32",
            "shape": (STATE_DIM,),
            "names": {
                "motors": UR5_JOINT_NAMES + ["gripper"],
            },
        },
    }

    if has_webcam:
        features["observation.images.webcam"] = {
            "dtype": "video",
            "shape": (image_size[0], image_size[1], 3),
            "names": ["height", "width", "channels"],
        }
    if has_gopro:
        features["observation.images.wrist"] = {
            "dtype": "video",
            "shape": (image_size[0], image_size[1], 3),
            "names": ["height", "width", "channels"],
        }

    logger.info("Features: %s", list(features.keys()))

    # ------------------------------------------------------------------
    # Create LeRobotDataset
    # ------------------------------------------------------------------
    dataset_root = root or Path(f"lerobot_datasets/{repo_id}")
    # Remove existing dataset directory to start fresh
    if dataset_root.exists():
        logger.info("Removing existing dataset at %s", dataset_root)
        shutil.rmtree(dataset_root)

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        root=dataset_root,
        robot_type="ur5",
        features=features,
        use_videos=True,
        image_writer_processes=0,
        image_writer_threads=2,
    )

    total_episodes = 0
    total_frames = 0

    # ------------------------------------------------------------------
    # Process each recording
    # ------------------------------------------------------------------
    for rec_dir in rec_dirs:
        logger.info("Processing %s ...", rec_dir.name)

        # Load joint data
        h5_data = load_h5_data(rec_dir / "joint_states.h5")
        timestamps = h5_data["timestamps"]
        positions = h5_data["positions"]  # (N, 6+)
        gripper_pos = h5_data["gripper_position"]  # (N,)

        if len(timestamps) < 2:
            logger.warning("Skipping %s — too few samples (%d)", rec_dir.name, len(timestamps))
            continue

        # Normalise timestamps to start at 0
        ts0 = timestamps[0]
        timestamps = timestamps - ts0
        total_duration = float(timestamps[-1])

        # Load program events for episode segmentation
        events_path = rec_dir / "program_events.json"
        if events_path.exists():
            events = load_program_events(events_path)
        else:
            events = []
        episode_segments = segment_episodes(events, total_duration)

        # Open video readers
        webcam_reader = None
        gopro_reader = None
        if has_webcam:
            for vname in ("video.mp4", "input.mp4"):
                vpath = rec_dir / vname
                if vpath.exists():
                    webcam_reader = VideoFrameReader(vpath, target_size=image_size)
                    break
        if has_gopro:
            for vname in ("gopro_video.mp4", "output.mp4"):
                vpath = rec_dir / vname
                if vpath.exists():
                    gopro_reader = VideoFrameReader(vpath, target_size=image_size)
                    break

        # Process each episode segment
        for seg in episode_segments:
            seg_start, seg_end = seg["start"], seg["end"]
            task_str = seg["task"]

            if seg_end - seg_start < 0.2:
                logger.warning("Skipping very short segment (%.2fs) in %s", seg_end - seg_start, rec_dir.name)
                continue

            # Resample to target FPS
            target_ts, src_idx = resample_indices(timestamps, fps, seg_start, seg_end)
            n_frames = len(target_ts)

            if n_frames < 2:
                logger.warning("Segment too short after resampling (%d frames)", n_frames)
                continue

            # Build state array: [6 joints, 1 gripper]
            joint_cols = min(positions.shape[1], 6)
            states = np.zeros((n_frames, STATE_DIM), dtype=np.float32)
            states[:, :joint_cols] = positions[src_idx, :joint_cols]
            states[:, 6] = gripper_pos[src_idx]

            # Build action array: shifted-by-one states (next state = action)
            actions = np.zeros_like(states)
            actions[:-1] = states[1:]
            actions[-1] = states[-1]  # last action = hold position

            # Add frames
            for i in range(n_frames):
                frame = {
                    "observation.state": states[i],
                    "action": actions[i],
                    "task": task_str,
                    "timestamp": float(target_ts[i] - target_ts[0]),
                }

                # Add camera frames
                vid_time = float(target_ts[i])  # time relative to recording start
                if webcam_reader is not None:
                    img = webcam_reader.get_frame_at_time(vid_time)
                    if img is None:
                        img = np.zeros((*image_size, 3), dtype=np.uint8)
                    frame["observation.images.webcam"] = img

                if gopro_reader is not None:
                    img = gopro_reader.get_frame_at_time(vid_time)
                    if img is None:
                        img = np.zeros((*image_size, 3), dtype=np.uint8)
                    frame["observation.images.wrist"] = img

                dataset.add_frame(frame)

            dataset.save_episode()
            total_episodes += 1
            total_frames += n_frames
            logger.info(
                "  Episode %d: task=%r, frames=%d (%.1fs)",
                total_episodes - 1,
                task_str,
                n_frames,
                seg_end - seg_start,
            )

        # Close video readers
        if webcam_reader:
            webcam_reader.close()
        if gopro_reader:
            gopro_reader.close()

    # ------------------------------------------------------------------
    # Consolidate
    # ------------------------------------------------------------------
    logger.info("Dataset complete: %d episodes, %d frames", total_episodes, total_frames)
    logger.info("Saved to: %s", dataset.root)

    if push_to_hub and total_episodes > 0:
        logger.info("Pushing dataset to HuggingFace Hub as %s ...", repo_id)
        dataset.push_to_hub(
            tags=["ur5", "robotiq", "smolvla"],
        )
        logger.info("Done! Dataset available at https://huggingface.co/datasets/%s", repo_id)

    return dataset


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Convert UR5 recordings to a LeRobot dataset for SmolVLA finetuning.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--recordings-dir",
        type=Path,
        default=Path("recordings"),
        help="Directory containing experiment_* recording folders (default: recordings/)",
    )
    parser.add_argument(
        "--repo-id",
        type=str,
        required=True,
        help="HuggingFace repo ID for the dataset, e.g. myuser/ur5_programs",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=DEFAULT_FPS,
        help=f"Target sampling rate in Hz (default: {DEFAULT_FPS})",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=None,
        help="Local output directory (default: lerobot_datasets/<repo_id>)",
    )
    parser.add_argument(
        "--include",
        nargs="*",
        default=None,
        help="Names of specific recording folders to include (e.g. experiment_20260119_172308)",
    )
    parser.add_argument(
        "--image-height",
        type=int,
        default=DEFAULT_IMAGE_SIZE[0],
        help="Resize images to this height (default: 480)",
    )
    parser.add_argument(
        "--image-width",
        type=int,
        default=DEFAULT_IMAGE_SIZE[1],
        help="Resize images to this width (default: 640)",
    )
    parser.add_argument(
        "--push-to-hub",
        action="store_true",
        help="Push the resulting dataset to the HuggingFace Hub",
    )
    args = parser.parse_args()

    convert(
        recordings_dir=args.recordings_dir,
        repo_id=args.repo_id,
        fps=args.fps,
        root=args.root,
        include=args.include,
        image_size=(args.image_height, args.image_width),
        push_to_hub=args.push_to_hub,
    )


if __name__ == "__main__":
    main()
