import os
import json
import pandas as pd
import numpy as np
import cv2
import glob

def convert_parquet_to_mp4(dataset_dir):
    data_dir = os.path.join(dataset_dir, "data")
    videos_dir = os.path.join(dataset_dir, "videos")
    
    # Find all parquet files
    parquet_files = glob.glob(os.path.join(data_dir, "chunk-*", "*.parquet"))
    parquet_files.sort()
    
    print(f"Found {len(parquet_files)} parquet files.")
    
    for pq_file in parquet_files:
        print(f"Processing {pq_file}...")
        df = pd.read_parquet(pq_file)
        
        # Extract episode index and chunk from filename/path
        # Path: .../data/chunk-XXX/episode_XXXXXX.parquet
        chunk_name = os.path.basename(os.path.dirname(pq_file))
        filename = os.path.basename(pq_file)
        episode_idx_str = filename.split('_')[1].split('.')[0]
        
        # Create output directory
        # videos/chunk-XXX/observation.images.egoview/
        video_chunk_dir = os.path.join(videos_dir, chunk_name, "observation.images.egoview")
        os.makedirs(video_chunk_dir, exist_ok=True)
        
        video_path = os.path.join(video_chunk_dir, f"episode_{episode_idx_str}.mp4")
        
        # Extract images
        # Assuming images are flattened arrays or lists in 'observation.images.egoview' column
        # Check format
        first_img = df["observation.images.egoview"].iloc[0]
        
        images = []
        for i in range(len(df)):
            img_data = df["observation.images.egoview"].iloc[i]
            # If it's a list or flat array, reshape
            if isinstance(img_data, (list, np.ndarray)):
                img_arr = np.array(img_data, dtype=np.uint8)
                if img_arr.size == 256*256*3:
                    img = img_arr.reshape(256, 256, 3)
                else:
                    # Try to decode if it's bytes?
                    # But my generator saved flattened list.
                    # Let's assume flattened list of 256*256*3
                    img = img_arr.reshape(256, 256, 3)
            else:
                # Maybe bytes?
                img = np.frombuffer(img_data, dtype=np.uint8).reshape(256, 256, 3)
            
            # Convert RGB to BGR for OpenCV
            img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            images.append(img_bgr)
            
        # Write video
        height, width, layers = images[0].shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v') # or avc1
        out = cv2.VideoWriter(video_path, fourcc, 10.0, (width, height))
        
        for img in images:
            out.write(img)
            
        out.release()
        print(f"Saved {video_path}")

if __name__ == "__main__":
    convert_parquet_to_mp4("/home/mani/Repos/ur_ws/generated_dataset")
