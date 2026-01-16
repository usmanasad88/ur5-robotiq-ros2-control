#!/usr/bin/env python3
"""Inspect HDF5 joint states recording."""

import h5py
import numpy as np
from pathlib import Path
import json

def inspect_h5_file(filepath):
    """Load and inspect an HDF5 file."""
    filepath = Path(filepath)
    
    if not filepath.exists():
        print(f"❌ File not found: {filepath}")
        return
    
    print(f"📂 File: {filepath}")
    print(f"📊 File size: {filepath.stat().st_size / 1024:.2f} KB")
    print("\n" + "="*70)
    
    with h5py.File(filepath, 'r') as f:
        # Print attributes
        print("\n📋 Attributes:")
        print("-" * 70)
        if len(f.attrs) > 0:
            for attr_name, attr_value in f.attrs.items():
                print(f"  {attr_name}: {attr_value}")
        else:
            print("  (No attributes found)")
        
        # Print datasets
        print("\n📊 Datasets:")
        print("-" * 70)
        
        if len(f.keys()) == 0:
            print("  (No datasets found)")
        else:
            for dataset_name in f.keys():
                dataset = f[dataset_name]
                print(f"\n  {dataset_name}:")
                print(f"    Shape: {dataset.shape}")
                print(f"    Dtype: {dataset.dtype}")
                print(f"    Compression: {dataset.compression}")
                
                # Show sample data
                if dataset_name == 'joint_names':
                    names = [n.decode() if isinstance(n, bytes) else n for n in dataset[:]]
                    print(f"    Data: {names}")
                elif len(dataset) > 0:
                    if len(dataset.shape) == 1:
                        print(f"    First sample: {dataset[0]}")
                        print(f"    Last sample: {dataset[-1]}")
                        if len(dataset) > 1:
                            print(f"    Min: {np.min(dataset[:]):.6f}, Max: {np.max(dataset[:]):.6f}")
                    else:
                        print(f"    First row: {dataset[0]}")
                        print(f"    Last row: {dataset[-1]}")
        
        # Calculate statistics
        print("\n📈 Recording Statistics:")
        print("-" * 70)
        
        if 'timestamps' in f:
            timestamps = f['timestamps'][:]
            if len(timestamps) > 1:
                duration = timestamps[-1] - timestamps[0]
                num_samples = len(timestamps)
                actual_rate = num_samples / duration if duration > 0 else 0
                print(f"  Duration: {duration:.2f} seconds")
                print(f"  Number of samples: {num_samples}")
                print(f"  Actual sample rate: {actual_rate:.1f} Hz")
                print(f"  Expected sample rate: {f.attrs.get('sample_rate_hz', 'N/A')} Hz")
            else:
                print(f"  Single timestamp recorded: {timestamps[0]:.3f} seconds")
        
        if 'positions' in f:
            positions = f['positions'][:]
            print(f"\n  Position data shape: {positions.shape}")
            if 'joint_names' in f:
                joint_names = [n.decode() if isinstance(n, bytes) else n 
                             for n in f['joint_names'][:]]
                print(f"\n  Position stats by joint:")
                for i, joint_name in enumerate(joint_names):
                    print(f"    {joint_name}: [{np.min(positions[:, i]):.4f}, {np.max(positions[:, i]):.4f}]")
    
    # Also check for metadata.json if it exists
    metadata_path = filepath.parent / "metadata.json"
    if metadata_path.exists():
        print("\n📝 Metadata JSON:")
        print("-" * 70)
        with open(metadata_path, 'r') as f:
            metadata = json.load(f)
            for key, value in metadata.items():
                print(f"  {key}: {value}")

if __name__ == "__main__":
    filepath = "/home/mani/Repos/ur_ws/recordings/experiment_20260115_153902/joint_states.h5"
    inspect_h5_file(filepath)



