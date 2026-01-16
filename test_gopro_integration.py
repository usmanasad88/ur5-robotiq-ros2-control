#!/usr/bin/env python3
"""
Test script for GoPro integration with episode recording.

Usage:
    1. Make sure GoPro is connected and streaming (run connect_gopro.py first)
    2. Run this script to test GoPro recording functionality
"""

import time
from pathlib import Path
from gopro_recorder import GoProRecorder

def main():
    print("=" * 60)
    print("  GoPro Integration Test")
    print("=" * 60)
    print()
    
    # Create test output directory
    test_dir = Path("recordings/gopro_test")
    test_dir.mkdir(parents=True, exist_ok=True)
    
    # Initialize GoPro recorder
    print("📹 Initializing GoPro recorder...")
    gopro = GoProRecorder()
    
    # Check if GoPro is streaming
    print("🔍 Checking if GoPro is streaming...")
    if gopro.is_gopro_streaming():
        print("   ✓ GoPro stream is active!")
    else:
        print("   ✗ GoPro stream not detected")
        print("\n⚠️  Make sure to run 'python connect_gopro.py' first")
        return
    
    # Start recording
    output_file = test_dir / "test_gopro.mp4"
    print(f"\n🔴 Starting GoPro recording to: {output_file}")
    
    success, message = gopro.start_recording(output_file)
    if not success:
        print(f"   ✗ Failed: {message}")
        return
    
    print(f"   ✓ Recording started!")
    
    # Record for 5 seconds
    print("\n⏱️  Recording for 5 seconds...")
    for i in range(5, 0, -1):
        print(f"   {i}...", end='\r', flush=True)
        time.sleep(1)
    print("   Done!        ")
    
    # Stop recording
    print("\n⏹️  Stopping recording...")
    success, message = gopro.stop_recording()
    
    if success:
        print(f"   ✓ Recording stopped!")
        print(f"\n✅ Test completed successfully!")
        print(f"   Output file: {output_file}")
        
        # Check file size
        if output_file.exists():
            size_mb = output_file.stat().st_size / (1024 * 1024)
            print(f"   File size: {size_mb:.2f} MB")
        else:
            print("   ⚠️  Output file not found (may still be processing)")
    else:
        print(f"   ✗ Failed: {message}")
    
    print("\n" + "=" * 60)

if __name__ == "__main__":
    main()
