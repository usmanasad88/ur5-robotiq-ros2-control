#!/usr/bin/env python3
"""
GoPro Max 2 Native Photo Capture
Uses GoPro HTTP API to capture photos natively (not from stream)
"""

import requests
import time
import json
import argparse
import os
from pathlib import Path

# Camera IP (detected from USB interface)
CAMERA_IP = "172.29.170.51"
BASE_URL = f"http://{CAMERA_IP}/gopro"

def check_camera():
    """Check if camera is reachable"""
    try:
        response = requests.get(f"{BASE_URL}/camera/state", timeout=2)
        return response.status_code == 200
    except:
        return False

def stop_stream():
    """Stop video stream if running"""
    try:
        response = requests.get(f"{BASE_URL}/camera/stream/stop", timeout=5)
        time.sleep(1)
        return response.status_code == 200
    except:
        return False

def set_mode_photo():
    """Set camera to photo mode"""
    # Mode: 1=Video, 2=Photo, 3=Timelapse
    try:
        # First check current mode
        state = requests.get(f"{BASE_URL}/camera/state", timeout=5).json()
        current_mode = state.get("settings", {}).get("mode", 0)
        
        if current_mode != 2:
            response = requests.get(f"{BASE_URL}/camera/presets/set_group?id=1001", timeout=5)
            time.sleep(1)
            return True
        return True
    except Exception as e:
        print(f"   ⚠️  Mode switch error: {e}")
        return False

def capture_photo():
    """Trigger camera shutter to capture photo"""
    try:
        response = requests.get(f"{BASE_URL}/camera/shutter/start", timeout=10)
        if response.status_code == 200:
            # Wait for capture to complete (360 photos take longer)
            time.sleep(3)
            return True
        else:
            print(f"   ❌ Shutter failed: {response.status_code}")
            print(f"      Response: {response.text}")
            return False
    except Exception as e:
        print(f"   ❌ Capture error: {e}")
        return False

def get_media_list():
    """Get list of media files on camera"""
    # Retry a few times as media server can be temporarily unavailable
    for attempt in range(3):
        try:
            # Try both endpoints (port 8080 is more reliable)
            response = requests.get(f"http://{CAMERA_IP}:8080/gopro/media/list", timeout=5)
            if response.status_code == 200:
                return response.json()
            
            # Fallback to main port
            response = requests.get(f"{BASE_URL}/media/list", timeout=5)
            if response.status_code == 200:
                return response.json()
            
            # If 503, wait and retry
            if response.status_code == 503 and attempt < 2:
                time.sleep(1)
                continue
            else:
                print(f"   ⚠️  Media list error: {response.status_code}")
                return None
        except Exception as e:
            if attempt < 2:
                time.sleep(1)
                continue
            print(f"   ⚠️  Media list exception: {e}")
            return None
    return None

def download_latest_photo(output_dir="./photos"):
    """Download the most recent photo from camera"""
    try:
        media_list = get_media_list()
        if not media_list:
            return None
        
        # Get latest file
        files = media_list.get("media", [])
        if not files:
            print("   ⚠️  No media files found")
            return None
        
        # Get most recent directory
        latest_dir = files[0]
        dir_name = latest_dir.get("d", "")
        
        # Get latest file in that directory
        dir_files = latest_dir.get("fs", [])
        if not dir_files:
            return None
        
        latest_file = dir_files[0]
        filename = latest_file.get("n", "")
        
        if not filename:
            return None
        
        # Download URL
        download_url = f"http://{CAMERA_IP}:8080/videos/DCIM/{dir_name}/{filename}"
        
        print(f"   📥 Downloading: {filename}")
        
        # Create output directory
        Path(output_dir).mkdir(parents=True, exist_ok=True)
        output_path = os.path.join(output_dir, filename)
        
        # Download file
        response = requests.get(download_url, stream=True, timeout=30)
        if response.status_code == 200:
            with open(output_path, 'wb') as f:
                for chunk in response.iter_content(chunk_size=8192):
                    f.write(chunk)
            
            file_size = os.path.getsize(output_path) / (1024 * 1024)  # MB
            print(f"   ✅ Saved: {output_path}")
            print(f"   Size: {file_size:.2f} MB")
            return output_path
        else:
            print(f"   ❌ Download failed: {response.status_code}")
            return None
            
    except Exception as e:
        print(f"   ❌ Download error: {e}")
        return None

def main():
    parser = argparse.ArgumentParser(
        description="Capture photos natively using GoPro API",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--output-dir",
        default="./photos",
        help="Directory to save downloaded photos (default: ./photos)"
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of photos to capture (default: 1)"
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=3.0,
        help="Interval between captures in seconds (default: 3.0)"
    )
    parser.add_argument(
        "--no-download",
        action="store_true",
        help="Don't download photos (just capture them on camera)"
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("  GoPro Max 2 Native Photo Capture")
    print("=" * 60)
    print()
    
    # Check camera
    print("🔍 Checking camera...")
    if not check_camera():
        print("   ❌ Camera not detected at", CAMERA_IP)
        print("   Make sure camera is connected and powered on")
        return 1
    print("   ✅ Camera detected")
    print()
    
    # Stop stream if running
    print("🛑 Stopping stream (if running)...")
    stop_stream()
    print("   ✅ Stream stopped")
    print()
    
    # Set to photo mode
    print("📷 Setting camera to photo mode...")
    if not set_mode_photo():
        print("   ⚠️  Could not confirm photo mode")
    else:
        print("   ✅ Photo mode active")
    print()
    
    # Capture photos
    for i in range(args.count):
        if args.count > 1:
            print(f"📸 Capturing photo {i+1}/{args.count}...")
        else:
            print("📸 Capturing photo...")
        
        if capture_photo():
            print("   ✅ Photo captured on camera")
            
            if not args.no_download:
                # Wait longer for media server to be ready
                print("   ⏳ Waiting for media server...")
                time.sleep(2)
                downloaded_file = download_latest_photo(args.output_dir)
                if downloaded_file:
                    print(f"   ✅ Downloaded successfully")
        else:
            print("   ❌ Capture failed")
        
        print()
        
        # Wait before next capture
        if i < args.count - 1:
            print(f"⏳ Waiting {args.interval}s before next capture...")
            time.sleep(args.interval)
            print()
    
    print("=" * 60)
    print("  ✅ Capture Complete")
    print("=" * 60)
    if not args.no_download:
        print(f"\nPhotos saved in: {args.output_dir}/")
    
    return 0

if __name__ == "__main__":
    exit(main())
