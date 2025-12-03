import argparse
import time
import rerun as rr
import torch
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
import json
import os

def load_task_mapping(dataset_root):
    """Load task index to description mapping from tasks.jsonl"""
    tasks_path = os.path.join(dataset_root, "meta", "tasks.jsonl")
    task_map = {}
    if os.path.exists(tasks_path):
        with open(tasks_path, 'r') as f:
            for line in f:
                entry = json.loads(line)
                task_map[entry["task_index"]] = entry["task"]
    return task_map

def main():
    parser = argparse.ArgumentParser(description="Visualize LeRobot dataset in Rerun")
    parser.add_argument("--root", type=str, default="/home/mani/Repos/ur_ws", help="Root directory containing the dataset folder")
    parser.add_argument("--repo-id", type=str, default="generated_dataset", help="Dataset name (folder name inside root)")
    parser.add_argument("--episode", type=int, default=None, help="Episode index to visualize (default: all episodes)")
    parser.add_argument("--fps", type=int, default=10, help="Playback FPS")
    parser.add_argument("--no-wait", action="store_true", help="Don't wait between frames (fast mode)")
    args = parser.parse_args()

    # Initialize Rerun
    rr.init("lerobot_dataset_visualizer", spawn=True)

    dataset_root = f"{args.root}/{args.repo_id}"
    
    # Load task mapping
    task_map = load_task_mapping(dataset_root)
    print(f"Task mapping: {task_map}")

    # Load Dataset
    print(f"Loading dataset {args.repo_id} from {args.root}...")
    
    # Determine which episodes to load
    if args.episode is not None:
        episodes_to_load = [args.episode]
    else:
        # Load all episodes - read from episodes.jsonl
        episodes_path = os.path.join(dataset_root, "meta", "episodes.jsonl")
        episodes_to_load = []
        if os.path.exists(episodes_path):
            with open(episodes_path, 'r') as f:
                for line in f:
                    ep = json.loads(line)
                    episodes_to_load.append(ep["episode_index"])
        print(f"Found {len(episodes_to_load)} episodes: {episodes_to_load}")
    
    dataset = LeRobotDataset(
        args.repo_id, 
        root=dataset_root, 
        episodes=episodes_to_load
    )
    
    print(f"Loaded {len(episodes_to_load)} episode(s). Total frames: {dataset.num_frames}")
    
    # Get feature names
    features = dataset.features
    print("Features:", features.keys())

    # Iterate and log
    current_episode = -1
    for i in range(dataset.num_frames):
        start_time = time.time()
        
        # Get frame data
        frame = dataset[i]
        
        # Check episode change for logging
        ep_idx = frame.get('episode_index', torch.tensor(0))
        if isinstance(ep_idx, torch.Tensor):
            ep_idx = ep_idx.item()
        
        if ep_idx != current_episode:
            current_episode = ep_idx
            print(f"\n--- Episode {current_episode} ---")
        
        # Set time on timelines
        rr.set_time_seconds("sim_time", i / args.fps)
        rr.set_time_sequence("frame_idx", i)
        rr.set_time_sequence("episode", int(ep_idx))

        # Log Images
        for key, value in frame.items():
            if "image" in key:
                if isinstance(value, torch.Tensor):
                    img_np = value.permute(1, 2, 0).numpy()
                    if img_np.dtype == np.float32 or img_np.dtype == np.float64:
                        if img_np.max() <= 1.0:
                            img_np = (img_np * 255).astype(np.uint8)
                        else:
                            img_np = img_np.astype(np.uint8)
                    rr.log(key, rr.Image(img_np))
            
            # Check task fields BEFORE action (since annotation.human.action.task_description contains "action")
            elif "task_index" in key or "task_description" in key:
                # Get task index and convert to text description
                if isinstance(value, torch.Tensor):
                    task_idx = int(value.item())
                else:
                    task_idx = int(value)
                
                task_text = task_map.get(task_idx, f"Task {task_idx}")
                
                # Log as text document with nice formatting
                rr.log("task/description", rr.TextDocument(f"**Task:** {task_text}"))
                rr.log("task/index", rr.Scalars([task_idx]))
            
            elif "state" in key or "action" in key:
                if isinstance(value, torch.Tensor):
                    vec = value.numpy()
                    rr.log(key, rr.Tensor(vec))
                    
                    if vec.ndim > 0:
                        for j, val in enumerate(vec):
                            rr.log(f"{key}/{j}", rr.Scalars([val]))
                    else:
                        rr.log(key, rr.Scalars([vec.item()]))

        # Log episode info
        rr.log("episode/index", rr.TextDocument(f"**Episode:** {ep_idx}"))

        # Sleep to match FPS (unless in fast mode)
        if not args.no_wait:
            process_time = time.time() - start_time
            sleep_time = max(0, (1.0 / args.fps) - process_time)
            time.sleep(sleep_time)

    print("\nDone. Use the timeline slider in Rerun to browse frames.")
    print("Tip: Select 'episode' timeline to filter by episode.")

if __name__ == "__main__":
    main()
