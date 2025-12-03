import os
import json
import glob
import pyarrow as pa
import pyarrow.parquet as pq

DATASET_DIR = "/home/mani/Repos/ur_ws/generated_dataset"
META_DIR = os.path.join(DATASET_DIR, "meta")
DATA_DIR = os.path.join(DATASET_DIR, "data", "chunk-000")

def fix_dataset():
    print("Fixing dataset...")
    
    # 1. Read episodes.jsonl to find all tasks
    episodes_path = os.path.join(META_DIR, "episodes.jsonl")
    if not os.path.exists(episodes_path):
        print("episodes.jsonl not found!")
        return

    episodes = []
    with open(episodes_path, 'r') as f:
        for line in f:
            episodes.append(json.loads(line))

    # Collect unique tasks
    # Assuming 'tasks' in episodes.jsonl is a list of strings currently
    unique_tasks = set()
    for ep in episodes:
        for t in ep['tasks']:
            unique_tasks.add(t)
    
    sorted_tasks = sorted(list(unique_tasks))
    task_to_id = {t: i for i, t in enumerate(sorted_tasks)}
    
    print(f"Found {len(unique_tasks)} unique tasks: {task_to_id}")

    # 2. Create tasks.jsonl and tasks.parquet
    tasks_path = os.path.join(META_DIR, "tasks.jsonl")
    with open(tasks_path, 'w') as f:
        for t, idx in task_to_id.items():
            f.write(json.dumps({"task_index": idx, "task": t}) + "\n")
    print(f"Created {tasks_path}")

    # Create tasks.parquet
    tasks_data = {"task_index": [], "task": []}
    for t, idx in task_to_id.items():
        tasks_data["task_index"].append(idx)
        tasks_data["task"].append(t)
    
    tasks_table = pa.Table.from_pydict(tasks_data)
    pq.write_table(tasks_table, os.path.join(META_DIR, "tasks.parquet"))
    print(f"Created {os.path.join(META_DIR, 'tasks.parquet')}")

    # 3. Update Parquet files (Fix timestamps and reorder columns) - MOVED BEFORE EPISODES METADATA
    parquet_files = glob.glob(os.path.join(DATA_DIR, "*.parquet"))
    print(f"Found {len(parquet_files)} parquet files in {DATA_DIR}")
    
    for pq_file in parquet_files:
        table = pq.read_table(pq_file)
        
        # Add frame_index if missing (do this regardless of task description type)
        if 'frame_index' not in table.column_names:
            if 'index' in table.column_names:
                table = table.append_column('frame_index', table['index'])
                print(f"Added 'frame_index' to {os.path.basename(pq_file)}")
            else:
                print(f"Warning: 'index' column missing in {os.path.basename(pq_file)}, cannot create frame_index")

        # Fix timestamps to be perfectly aligned with 10FPS
        # timestamp = frame_index / 10.0
        if 'frame_index' in table.column_names:
            frame_indices = table['frame_index'].to_pylist()
            # Assuming frame_index starts from 0 for each episode?
            # In generator, frame_index reset to 0 for each episode.
            new_timestamps = [float(idx) / 10.0 for idx in frame_indices]
            
            # Replace timestamp column
            try:
                col_idx = table.column_names.index('timestamp')
                table = table.set_column(col_idx, 'timestamp', pa.array(new_timestamps, type=pa.float64()))
                print(f"Fixed timestamps for {os.path.basename(pq_file)}")
            except ValueError:
                table = table.append_column('timestamp', pa.array(new_timestamps, type=pa.float64()))

        # We need to update 'task_index' column.
        if 'annotation.human.action.task_description' in table.column_names:
            # Check if it's already integer (indices)
            if pa.types.is_integer(table['annotation.human.action.task_description'].type):
                 pass
            else:
                task_descs = table['annotation.human.action.task_description'].to_pylist()
                # Assuming all rows in an episode have the same task
                task_desc = task_descs[0]
                new_task_idx = task_to_id.get(task_desc, 0)
                
                # Create new column for task_index
                new_task_indices = [new_task_idx] * len(table)
                
                # Find index of 'task_index'
                try:
                    col_idx = table.column_names.index('task_index')
                    table = table.set_column(col_idx, 'task_index', pa.array(new_task_indices, type=pa.int64()))
                except ValueError:
                    # Column might not exist, append it
                    table = table.append_column('task_index', pa.array(new_task_indices, type=pa.int64()))
                
                # ALSO update 'annotation.human.action.task_description' to be indices
                try:
                    col_idx = table.column_names.index('annotation.human.action.task_description')
                    table = table.set_column(col_idx, 'annotation.human.action.task_description', pa.array(new_task_indices, type=pa.int64()))
                    print(f"Updated 'annotation.human.action.task_description' to indices for {os.path.basename(pq_file)}")
                except ValueError:
                    print(f"Warning: 'annotation.human.action.task_description' not found in {os.path.basename(pq_file)}")

        # Reorder columns to match expected schema (do this regardless of task description type)
        expected_order = [
            'observation.state', 
            'action', 
            'timestamp', 
            'frame_index', 
            'episode_index', 
            'index', 
            'task_index', 
            'annotation.human.action.task_description'
        ]
        
        # Check if all columns exist
        existing_cols = table.column_names
        final_cols = [c for c in expected_order if c in existing_cols]
        
        # Add any other columns that might be there
        for c in existing_cols:
            if c not in final_cols:
                final_cols.append(c)
        
        table = table.select(final_cols)

        pq.write_table(table, pq_file)
        print(f"Updated {os.path.basename(pq_file)}")

    # 4. Update episodes metadata to v3.0 (parquet) and restructure videos
    # Read episodes.jsonl
    episodes_data = {
        "episode_index": [], 
        "tasks": [], 
        "length": [],
        "videos/observation.images.egoview/chunk_index": [],
        "videos/observation.images.egoview/file_index": [],
        "videos/observation.images.egoview/from_timestamp": [],
        "videos/observation.images.egoview/to_timestamp": []
    }
    total_frames = 0
    
    # Prepare video directory
    video_key = "observation.images.egoview"
    new_video_dir = os.path.join(DATASET_DIR, "videos", video_key, "chunk-000")
    os.makedirs(new_video_dir, exist_ok=True)
    
    # Old video dir
    old_video_dir = os.path.join(DATASET_DIR, "videos", "chunk-000")
    
    for i, ep in enumerate(episodes):
        episodes_data["episode_index"].append(ep["episode_index"])
        episodes_data["tasks"].append(ep["tasks"]) 
        episodes_data["length"].append(ep["length"])
        total_frames += ep["length"]
        
        # Assume chunk 0 for all
        episodes_data["videos/observation.images.egoview/chunk_index"].append(0)
        episodes_data["videos/observation.images.egoview/file_index"].append(i) 
        
        # Get timestamps from parquet file
        # Filename: train-{episode_index:05d}-of-{total:05d}.parquet
        # We need to find it.
        ep_idx = ep["episode_index"]
        pq_files = glob.glob(os.path.join(DATA_DIR, f"train-{ep_idx:05d}-of-*.parquet"))
        if pq_files:
            pq_file = pq_files[0]
            table = pq.read_table(pq_file)
            timestamps = table['timestamp'].to_pylist()
            start_time = timestamps[0]
            end_time = timestamps[-1]
            # Add buffer? Usually it's exact.
            episodes_data["videos/observation.images.egoview/from_timestamp"].append(start_time)
            episodes_data["videos/observation.images.egoview/to_timestamp"].append(end_time)
        else:
            print(f"Warning: Parquet file for episode {ep_idx} not found")
            episodes_data["videos/observation.images.egoview/from_timestamp"].append(0.0)
            episodes_data["videos/observation.images.egoview/to_timestamp"].append(0.0)

        # Move/Rename video
        # ... (existing code)
        old_files = glob.glob(os.path.join(old_video_dir, f"train-{ep_idx:05d}-of-*.mp4"))
        if old_files:
            old_file = old_files[0]
            new_file = os.path.join(new_video_dir, f"file-{i:03d}.mp4") # file-000.mp4
            if not os.path.exists(new_file):
                os.rename(old_file, new_file)
                print(f"Moved {os.path.basename(old_file)} to {video_key}/chunk-000/{os.path.basename(new_file)}")
        else:
            # Check if already moved (if running script multiple times)
            new_file = os.path.join(new_video_dir, f"file-{i:03d}.mp4")
            if not os.path.exists(new_file):
                print(f"Warning: Video for episode {ep_idx} not found in {old_video_dir}")

    # Update info.json with total_frames and features
    info_path = os.path.join(META_DIR, "info.json")
    if os.path.exists(info_path):
        with open(info_path, 'r') as f:
            info = json.load(f)
        info["total_frames"] = total_frames
        info["total_tasks"] = len(unique_tasks)
        
        # Add default features
        default_features = {
            "timestamp": {"dtype": "float32", "shape": [1], "names": None},
            "frame_index": {"dtype": "int64", "shape": [1], "names": None},
            "episode_index": {"dtype": "int64", "shape": [1], "names": None},
            "index": {"dtype": "int64", "shape": [1], "names": None},
            "task_index": {"dtype": "int64", "shape": [1], "names": None},
            "annotation.human.action.task_description": {"dtype": "int64", "shape": [1], "names": None},
        }
        # Merge with existing features
        if "features" not in info:
            info["features"] = {}
        
        for k, v in default_features.items():
            if k not in info["features"]:
                info["features"][k] = v
        
        # Update video_path to v3.0 format
        info["video_path"] = "videos/{video_key}/chunk-{chunk_index:03d}/file-{file_index:03d}.mp4"
        
        with open(info_path, 'w') as f:
            json.dump(info, f, indent=2)
        print(f"Updated info.json with total_frames={total_frames}, total_tasks={len(unique_tasks)} and default features")

    # Create meta/episodes/chunk-000/file-000.parquet
    episodes_dir = os.path.join(META_DIR, "episodes", "chunk-000")
    os.makedirs(episodes_dir, exist_ok=True)
    episodes_table = pa.Table.from_pydict(episodes_data)
    pq.write_table(episodes_table, os.path.join(episodes_dir, "file-000.parquet"))
    print(f"Created {os.path.join(episodes_dir, 'file-000.parquet')}")

if __name__ == "__main__":
    fix_dataset()
