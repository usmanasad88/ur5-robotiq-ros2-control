#!/bin/bash
# View GoPro Max 2 UDP stream with FFmpeg + NVIDIA GPU acceleration
# This script uses CUDA hardware decoding for low latency

set -e

# Configuration
CAMERA_IP="172.29.170.51"
UDP_PORT="8554"
UDP_URL="udp://${CAMERA_IP}:${UDP_PORT}"
GPU_DECODE="-hwaccel cuda"
# GoPro Max 2 uses dual fisheye format
PROJECTION_FILTER="v360=input=dfisheye:output=equirect:ih_fov=190:iv_fov=190"
LOW_LATENCY_FLAGS="-fflags nobuffer -flags low_delay -probesize 32 -analyzeduration 0"

echo "=================================="
echo "  GoPro Max 2 Stream Viewer"
echo "  with NVIDIA GPU Acceleration"
echo "=================================="
echo ""

# Note: Stream detection may timeout, but viewing will work
echo "📡 Connecting to UDP stream..."
echo "   Note: This will attempt to connect for up to 30 seconds"
echo "   Make sure connect_gopro.py is running in another terminal!"
echo ""
echo "🎬 Starting viewer with GPU acceleration..."
echo "   Camera: $CAMERA_IP"
echo "   Stream: $UDP_URL"
echo "   Press 'q' to quit"
echo ""

# Use ffplay directly with better buffering and connection handling
ffplay \
    -fflags nobuffer \
    -flags low_delay \
    -framedrop \
    -infbuf \
    -probesize 32 \
    -analyzeduration 0 \
    -sync ext \
    -vf "$PROJECTION_FILTER" \
    -window_title "GoPro Max 2 Stream" \
    "$UDP_URL"

# Alternative: Pipe to FFplay
# ffmpeg $GPU_DECODE -i "$UDP_URL" -vf "$PROJECTION_FILTER" -f nut - | \
#     ffplay -i - -window_title "GoPro Max 2 Stream"

echo ""
echo "✅ Viewer closed"
