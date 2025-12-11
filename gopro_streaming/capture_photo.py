#!/usr/bin/env python3
"""
GoPro Max 2 Photo Capture Script

This script captures a single frame from the GoPro camera stream and saves it to disk.
Can be used while streaming is active or independently.

Author: GitHub Copilot
Date: 2025-12-12
"""

import sys
import argparse
import subprocess
import time
from pathlib import Path
from datetime import datetime
import requests


class GoProPhotoCapture:
    """Capture photos from GoPro Max 2."""
    
    CAMERA_IP = "172.29.170.51"
    UDP_PORT = 8554
    
    def __init__(self, output_dir: str = ".", timeout: int = 10):
        """
        Initialize photo capture.
        
        Args:
            output_dir: Directory to save photos
            timeout: Timeout for stream capture in seconds
        """
        self.camera_ip = self.CAMERA_IP
        self.output_dir = Path(output_dir)
        self.timeout = timeout
        self.output_dir.mkdir(parents=True, exist_ok=True)
    
    def check_camera(self) -> bool:
        """Check if camera is accessible."""
        try:
            response = requests.get(
                f"http://{self.camera_ip}/gopro/camera/state",
                timeout=2
            )
            return response.status_code == 200
        except:
            return False
    
    def ensure_stream_active(self) -> bool:
        """Ensure camera stream is active."""
        try:
            # Enable wired USB control
            requests.get(
                f"http://{self.camera_ip}/gopro/camera/control/wired_usb?p=1",
                timeout=5
            )
            time.sleep(0.5)
            
            # Start stream
            response = requests.get(
                f"http://{self.camera_ip}/gopro/camera/stream/start",
                timeout=5
            )
            
            if response.status_code == 200:
                time.sleep(1)  # Give stream time to start
                return True
            
            return False
            
        except Exception as e:
            print(f"   ⚠ Error starting stream: {e}")
            return False
    
    def capture_frame(self, filename: str = None, apply_projection: bool = True) -> Path:
        """
        Capture a single frame from the stream.
        
        Args:
            filename: Output filename (auto-generated if None)
            apply_projection: Whether to apply EAC to equirectangular conversion
            
        Returns:
            Path to saved image
        """
        # Generate filename if not provided
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"gopro_capture_{timestamp}.jpg"
        
        output_path = self.output_dir / filename
        
        # Use UDP with reuse flag to allow multiple connections
        stream_url = f"udp://{self.camera_ip}:{self.UDP_PORT}?reuse=1"
        
        print(f"📸 Capturing frame from {self.camera_ip}...")
        
        # Build FFmpeg command with better error handling
        # Use system ffmpeg if available (newer version)
        ffmpeg_cmd = "/usr/bin/ffmpeg" if Path("/usr/bin/ffmpeg").exists() else "ffmpeg"
        
        cmd = [
            ffmpeg_cmd,
            "-y",  # Overwrite output file
            "-timeout", "10000000",  # 10 second timeout in microseconds
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-probesize", "5000000",  # Larger probe size
            "-analyzeduration", "1000000",  # 1 second analyze
            "-i", stream_url,
        ]
        
        # Add projection filter if requested
        # GoPro Max 2 uses dual fisheye format (two circular fisheye images side by side)
        if apply_projection:
            cmd.extend(["-vf", "v360=input=dfisheye:output=equirect:ih_fov=190:iv_fov=190"])
        
        # Output settings - simplified for better compatibility
        cmd.extend([
            "-frames:v", "1",  # Capture just 1 frame
            "-q:v", "2",  # High quality JPEG
            "-f", "image2",  # Force image format
            str(output_path)
        ])
        
        try:
            # Run FFmpeg with timeout
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self.timeout
            )
            
            if result.returncode == 0 and output_path.exists():
                file_size = output_path.stat().st_size / 1024  # KB
                print(f"   ✅ Saved: {output_path}")
                print(f"   Size: {file_size:.1f} KB")
                return output_path
            else:
                print(f"   ❌ Capture failed")
                # Check for specific error messages
                if "bind failed" in result.stderr or "Address already in use" in result.stderr:
                    print(f"   ⚠️  UDP port in use - viewer is running")
                    print(f"   Please close the viewer window first, then try again:")
                    print(f"      1. Close ./view_stream_raw.sh window")
                    print(f"      2. Run: python capture_photo.py")
                    print(f"      3. Reopen viewer if needed")
                elif result.stderr:
                    # Show last few lines of error
                    error_lines = result.stderr.strip().split('\n')[-3:]
                    for line in error_lines:
                        print(f"   {line}")
                return None
                
        except subprocess.TimeoutExpired:
            print(f"   ❌ Capture timed out after {self.timeout}s")
            print(f"   Make sure the stream is active")
            return None
        except Exception as e:
            print(f"   ❌ Error: {e}")
            return None
    
    def capture_multiple(self, count: int = 5, interval: float = 1.0, 
                        apply_projection: bool = True) -> list:
        """
        Capture multiple frames at intervals.
        
        Args:
            count: Number of frames to capture
            interval: Time between captures in seconds
            apply_projection: Whether to apply projection conversion
            
        Returns:
            List of saved file paths
        """
        print(f"📸 Capturing {count} frames at {interval}s intervals...")
        
        saved_files = []
        for i in range(count):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"gopro_capture_{timestamp}_{i+1:03d}.jpg"
            
            print(f"\n   Frame {i+1}/{count}:")
            output_path = self.capture_frame(filename, apply_projection)
            
            if output_path:
                saved_files.append(output_path)
            
            if i < count - 1:  # Don't wait after last capture
                time.sleep(interval)
        
        return saved_files


def main():
    parser = argparse.ArgumentParser(
        description="Capture photos from GoPro Max 2 stream",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Capture single photo
  python capture_photo.py
  
  # Capture to specific directory
  python capture_photo.py --output-dir ./photos
  
  # Capture without projection conversion (faster)
  python capture_photo.py --no-projection
  
  # Capture multiple photos
  python capture_photo.py --count 5 --interval 2.0
  
  # Specify filename
  python capture_photo.py --filename my_photo.jpg
        """
    )
    
    parser.add_argument(
        "--output-dir",
        type=str,
        default=".",
        help="Directory to save photos (default: current directory)"
    )
    
    parser.add_argument(
        "--filename",
        type=str,
        help="Output filename (auto-generated if not specified)"
    )
    
    parser.add_argument(
        "--no-projection",
        action="store_true",
        help="Skip projection conversion (recommended - use raw stream)"
    )
    
    parser.add_argument(
        "--projection",
        action="store_true",
        help="Apply dual fisheye to equirectangular conversion (for 360° view)"
    )
    
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="Number of frames to capture (default: 1)"
    )
    
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Interval between captures in seconds (default: 1.0)"
    )
    
    parser.add_argument(
        "--timeout",
        type=int,
        default=10,
        help="Timeout for stream capture in seconds (default: 10)"
    )
    
    parser.add_argument(
        "--start-stream",
        action="store_true",
        help="Start camera stream if not already active"
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("  GoPro Max 2 Photo Capture")
    print("=" * 60)
    print()
    
    # Initialize capture
    capturer = GoProPhotoCapture(
        output_dir=args.output_dir,
        timeout=args.timeout
    )
    
    # Check camera
    print("🔍 Checking camera...")
    if not capturer.check_camera():
        print("   ❌ Camera not responding at 172.29.170.51")
        print("   Make sure GoPro is connected and powered on")
        sys.exit(1)
    
    print("   ✅ Camera detected")
    print()
    
    # Start stream if requested
    if args.start_stream:
        print("📹 Starting camera stream...")
        if capturer.ensure_stream_active():
            print("   ✅ Stream active")
        else:
            print("   ⚠ Could not start stream")
            print("   Run 'python connect_gopro.py' in another terminal")
        print()
    
    # Default to NO projection (raw capture) since EAC conversion produces gray images
    apply_projection = args.projection and not args.no_projection
    
    # Capture photos
    if args.count == 1:
        # Single capture
        output_path = capturer.capture_frame(
            filename=args.filename,
            apply_projection=apply_projection
        )
        
        if output_path:
            print()
            print("=" * 60)
            print(f"  ✅ Photo saved: {output_path}")
            print("=" * 60)
        else:
            print()
            print("=" * 60)
            print("  ❌ Capture failed")
            print("=" * 60)
            print()
            print("💡 Troubleshooting:")
            print("   1. Make sure stream is active:")
            print("      python connect_gopro.py --keep-alive")
            print("   2. Or use --start-stream flag:")
            print("      python capture_photo.py --start-stream")
            sys.exit(1)
    else:
        # Multiple captures
        if args.filename:
            print("⚠ Warning: --filename ignored for multiple captures")
            print()
        
        saved_files = capturer.capture_multiple(
            count=args.count,
            interval=args.interval,
            apply_projection=apply_projection
        )
        
        print()
        print("=" * 60)
        print(f"  ✅ Captured {len(saved_files)}/{args.count} photos")
        print("=" * 60)
        
        if saved_files:
            print("\nSaved files:")
            for path in saved_files:
                print(f"  - {path}")
        
        if len(saved_files) < args.count:
            print()
            print("⚠ Some captures failed. Check stream connection.")
            sys.exit(1)
    
    print()


if __name__ == "__main__":
    main()
