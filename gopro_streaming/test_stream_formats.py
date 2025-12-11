#!/usr/bin/env python3
"""
Test different GoPro stream formats to find the working one.
"""

import subprocess
import time
import requests

CAMERA_IP = "172.29.170.51"

print("Testing GoPro Stream Formats")
print("=" * 60)

# Start the stream
print("\n1. Starting camera stream...")
try:
    response = requests.get(f"http://{CAMERA_IP}/gopro/camera/control/wired_usb?p=1", timeout=5)
    print(f"   Wired USB control: {response.status_code}")
    time.sleep(0.5)
    
    response = requests.get(f"http://{CAMERA_IP}/gopro/camera/stream/start", timeout=5)
    print(f"   Stream start: {response.status_code}")
    time.sleep(2)
except Exception as e:
    print(f"   Error: {e}")

# Test different URLs
test_urls = [
    ("UDP Direct", "udp://172.29.170.51:8554"),
    ("UDP Listen", "udp://@:8554"),
    ("TCP", "tcp://172.29.170.51:8554"),
    ("HTTP MJPEG", "http://172.29.170.51:8080"),
    ("HTTP Stream", "http://172.29.170.51:8554"),
    ("RTP", "rtp://172.29.170.51:8554"),
]

print("\n2. Testing stream URLs...")
for name, url in test_urls:
    print(f"\n   Testing {name}: {url}")
    cmd = [
        "timeout", "3",
        "ffprobe",
        "-v", "error",
        "-print_format", "json",
        "-show_streams",
        url
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=4)
        if result.returncode == 0 and result.stdout.strip():
            print(f"   ✅ SUCCESS! This URL works:")
            print(f"      {url}")
            print(f"      Output: {result.stdout[:200]}")
            break
        else:
            print(f"   ❌ No stream")
    except subprocess.TimeoutExpired:
        print(f"   ❌ Timeout")
    except Exception as e:
        print(f"   ❌ Error: {e}")

print("\n" + "=" * 60)
print("Test complete")
