#!/usr/bin/env python3
"""
Trajectory Post-Processing Script

This script removes idle time from recorded trajectories by detecting 
when the robot    # Sort and remove duplicates
    indices_to_keep = sorted(set(indices_to_keep))
    
    # Handle task space mismatch (may have fewer poses than joint positions)
    task_pose_count = len(task_poses)
    indices_for_task = [i for i in indices_to_keep if i < task_pose_count]
    
    if len(indices_for_task) < len(indices_to_keep) and verbose:
        print(f"\nNote: Task space has fewer poses ({task_pose_count}) than joint positions ({original_count})")
        print(f"      Keeping {len(indices_for_task)} task space poses")
    
    # Create processed trajectory
    processed_data = trajectory_data.copy()
    processed_data['joint_space'] = {
        'joint_names': trajectory_data['joint_space']['joint_names'],
        'positions': [joint_positions[i] for i in indices_to_keep]
    }
    processed_data['task_space'] = {
        'poses': [task_poses[i] for i in indices_for_task]
    }esn't change significantly.

Usage:
    ./trajectory_postprocess.py <input_file.json> [options]
    
Options:
    --threshold FLOAT    Joint motion threshold in radians (default: 0.001)
    --output PATH        Output file path (default: input_file_processed.json)
    --min-waypoints INT  Minimum waypoints to keep (default: 2)
    --verbose           Print detailed information
"""

import json
import numpy as np
import argparse
import os
from pathlib import Path
from typing import Dict, List, Tuple


def calculate_joint_distance(pos1: List[float], pos2: List[float]) -> float:
    """Calculate Euclidean distance between two joint configurations."""
    return np.linalg.norm(np.array(pos1) - np.array(pos2))


def find_motion_segments(positions: List[List[float]], 
                         threshold: float = 0.001) -> List[Tuple[int, int]]:
    """
    Find segments where robot is moving (position changing beyond threshold).
    
    Returns list of (start_idx, end_idx) tuples for motion segments.
    """
    n_waypoints = len(positions)
    if n_waypoints < 2:
        return [(0, n_waypoints - 1)]
    
    # Mark each waypoint as moving (True) or idle (False)
    is_moving = [False] * n_waypoints
    
    for i in range(1, n_waypoints):
        distance = calculate_joint_distance(positions[i-1], positions[i])
        if distance > threshold:
            is_moving[i] = True
            is_moving[i-1] = True  # Mark previous waypoint as moving too
    
    # Find continuous motion segments
    segments = []
    start_idx = None
    
    for i in range(n_waypoints):
        if is_moving[i] and start_idx is None:
            start_idx = i
        elif not is_moving[i] and start_idx is not None:
            segments.append((start_idx, i - 1))
            start_idx = None
    
    # Handle case where motion continues to end
    if start_idx is not None:
        segments.append((start_idx, n_waypoints - 1))
    
    return segments


def merge_nearby_segments(segments: List[Tuple[int, int]], 
                         max_gap: int = 3) -> List[Tuple[int, int]]:
    """
    Merge motion segments that are separated by small gaps.
    This helps preserve micro-motions.
    """
    if len(segments) <= 1:
        return segments
    
    merged = []
    current_start, current_end = segments[0]
    
    for start, end in segments[1:]:
        if start - current_end <= max_gap:
            # Merge segments
            current_end = end
        else:
            merged.append((current_start, current_end))
            current_start, current_end = start, end
    
    merged.append((current_start, current_end))
    return merged


def remove_idle_time(trajectory_data: Dict, 
                    threshold: float = 0.001,
                    min_waypoints: int = 2,
                    verbose: bool = False) -> Dict:
    """
    Remove idle time from trajectory while preserving all motion.
    
    Args:
        trajectory_data: Original trajectory JSON data
        threshold: Joint motion threshold in radians
        min_waypoints: Minimum number of waypoints to keep
        verbose: Print detailed information
        
    Returns:
        Processed trajectory data with idle time removed
    """
    joint_positions = trajectory_data['joint_space']['positions']
    task_poses = trajectory_data['task_space']['poses']
    
    original_count = len(joint_positions)
    
    if original_count < min_waypoints:
        print(f"Warning: Trajectory has only {original_count} waypoints. No processing applied.")
        return trajectory_data
    
    # Find motion segments
    segments = find_motion_segments(joint_positions, threshold)
    
    if verbose:
        print(f"\nOriginal waypoints: {original_count}")
        print(f"Motion threshold: {threshold} rad")
        print(f"Found {len(segments)} motion segment(s):")
        for i, (start, end) in enumerate(segments):
            print(f"  Segment {i+1}: waypoints {start} to {end} ({end-start+1} points)")
    
    # Merge nearby segments to preserve micro-motions
    merged_segments = merge_nearby_segments(segments, max_gap=3)
    
    if verbose and merged_segments != segments:
        print(f"\nAfter merging nearby segments: {len(merged_segments)} segment(s):")
        for i, (start, end) in enumerate(merged_segments):
            print(f"  Segment {i+1}: waypoints {start} to {end} ({end-start+1} points)")
    
    # Extract indices to keep
    indices_to_keep = []
    for start, end in merged_segments:
        indices_to_keep.extend(range(start, end + 1))
    
    # If no motion detected, keep first and last waypoints
    if len(indices_to_keep) < min_waypoints:
        if verbose:
            print(f"\nWarning: Less than {min_waypoints} waypoints with motion detected.")
            print(f"Keeping first and last waypoints.")
        indices_to_keep = [0, original_count - 1]
    
    # Sort and remove duplicates
    indices_to_keep = sorted(set(indices_to_keep))
    
    # Handle task space mismatch (may have fewer poses than joint positions)
    task_pose_count = len(task_poses)
    indices_for_task = [i for i in indices_to_keep if i < task_pose_count]
    
    if len(indices_for_task) < len(indices_to_keep) and verbose:
        print(f"\nNote: Task space has fewer poses ({task_pose_count}) than joint positions ({original_count})")
        print(f"      Keeping {len(indices_for_task)} task space poses")
    
    # Create processed trajectory
    processed_data = trajectory_data.copy()
    processed_data['joint_space'] = {
        'joint_names': trajectory_data['joint_space']['joint_names'],
        'positions': [joint_positions[i] for i in indices_to_keep]
    }
    processed_data['task_space'] = {
        'poses': [task_poses[i] for i in indices_for_task]
    }
    
    # Update metadata
    processed_data['metadata'] = trajectory_data.get('metadata', {})
    processed_data['metadata']['original_waypoints'] = original_count
    processed_data['metadata']['processed_waypoints'] = len(indices_to_keep)
    processed_data['metadata']['removed_waypoints'] = original_count - len(indices_to_keep)
    processed_data['metadata']['motion_threshold'] = threshold
    processed_data['metadata']['processing'] = 'idle_time_removed'
    
    if verbose:
        print(f"\nProcessed waypoints: {len(indices_to_keep)}")
        print(f"Removed waypoints: {original_count - len(indices_to_keep)} "
              f"({100 * (original_count - len(indices_to_keep)) / original_count:.1f}%)")
    
    return processed_data


def main():
    parser = argparse.ArgumentParser(
        description='Remove idle time from UR5 trajectory recordings',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage with default threshold
  ./trajectory_postprocess.py ~/ur5_trajectories/trajectory_20251119_132731.json
  
  # Use custom threshold for finer motion detection
  ./trajectory_postprocess.py input.json --threshold 0.0005
  
  # Specify output file
  ./trajectory_postprocess.py input.json --output output.json
  
  # Verbose output
  ./trajectory_postprocess.py input.json --verbose
        """
    )
    
    parser.add_argument('input_file', 
                       help='Input trajectory JSON file')
    parser.add_argument('--threshold', 
                       type=float, 
                       default=0.001,
                       help='Joint motion threshold in radians (default: 0.001)')
    parser.add_argument('--output',
                       type=str,
                       default=None,
                       help='Output file path (default: input_file_processed.json)')
    parser.add_argument('--min-waypoints',
                       type=int,
                       default=2,
                       help='Minimum waypoints to keep (default: 2)')
    parser.add_argument('--verbose',
                       action='store_true',
                       help='Print detailed information')
    
    args = parser.parse_args()
    
    # Check input file exists
    input_path = Path(args.input_file)
    if not input_path.exists():
        print(f"Error: Input file not found: {args.input_file}")
        return 1
    
    # Determine output path
    if args.output:
        output_path = Path(args.output)
    else:
        output_path = input_path.parent / f"{input_path.stem}_processed{input_path.suffix}"
    
    # Load trajectory
    if args.verbose:
        print(f"Loading trajectory from: {input_path}")
    
    try:
        with open(input_path, 'r') as f:
            trajectory_data = json.load(f)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON file: {e}")
        return 1
    except Exception as e:
        print(f"Error loading file: {e}")
        return 1
    
    # Process trajectory
    processed_data = remove_idle_time(
        trajectory_data,
        threshold=args.threshold,
        min_waypoints=args.min_waypoints,
        verbose=args.verbose
    )
    
    # Save processed trajectory
    try:
        with open(output_path, 'w') as f:
            json.dump(processed_data, f, indent=2)
        
        if args.verbose:
            print(f"\nProcessed trajectory saved to: {output_path}")
        else:
            print(f"Saved: {output_path}")
            print(f"Waypoints: {processed_data['metadata']['original_waypoints']} → "
                  f"{processed_data['metadata']['processed_waypoints']} "
                  f"({processed_data['metadata']['removed_waypoints']} removed)")
        
        return 0
        
    except Exception as e:
        print(f"Error saving file: {e}")
        return 1


if __name__ == '__main__':
    exit(main())
