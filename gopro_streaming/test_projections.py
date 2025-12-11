#!/usr/bin/env python3
"""
Test different v360 input projection formats to find the correct one for GoPro Max 2.
"""

import subprocess
from pathlib import Path

CAMERA_IP = "172.29.170.51"
UDP_PORT = 8554

# Test different input projection formats
projections = [
    ("equirect", "Equirectangular (already flat 360°)"),
    ("eac", "Equi-Angular Cubemap (current assumption)"),
    ("c3x2", "Cubemap 3x2 layout"),
    ("c6x1", "Cubemap 6x1 layout"),
    ("barrel", "Facebook 360 barrel format"),
    ("dfisheye", "Dual fisheye"),
]

print("=" * 70)
print("  Testing GoPro Max 2 Projection Formats")
print("=" * 70)
print()

for proj_input, description in projections:
    output_file = f"test_{proj_input}.jpg"
    
    print(f"Testing: {proj_input:12} - {description}")
    
    cmd = [
        "/usr/bin/ffmpeg",
        "-y",
        "-timeout", "10000000",
        "-fflags", "nobuffer",
        "-flags", "low_delay",
        "-probesize", "5000000",
        "-analyzeduration", "1000000",
        "-i", f"udp://{CAMERA_IP}:{UDP_PORT}?reuse=1",
        "-vf", f"v360=input={proj_input}:output=equirect",
        "-frames:v", "1",
        "-q:v", "2",
        "-f", "image2",
        output_file
    ]
    
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=15
        )
        
        if result.returncode == 0 and Path(output_file).exists():
            size = Path(output_file).stat().st_size / 1024
            print(f"   ✅ SUCCESS - {size:.1f} KB - Check {output_file}")
        else:
            print(f"   ❌ Failed")
            if "Conversion failed" in result.stderr or "Invalid" in result.stderr:
                print(f"      (Invalid projection format)")
    except subprocess.TimeoutExpired:
        print(f"   ❌ Timeout")
    except Exception as e:
        print(f"   ❌ Error: {e}")
    
    print()

print("=" * 70)
print("Compare the output images to see which projection looks correct!")
print("=" * 70)
