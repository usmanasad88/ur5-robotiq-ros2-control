#!/bin/bash
# View GoPro Max 2 stream using VLC player
# This method uses FFmpeg for conversion and pipes to VLC

set -e

CAMERA_IP="172.29.170.51"
UDP_URL="udp://${CAMERA_IP}:8554"

echo "=================================="
echo "  GoPro Max 2 Stream → VLC"
echo "  GPU Accelerated Pipeline"
echo "=================================="
echo ""

# Check VLC installation
if ! command -v vlc &> /dev/null; then
    echo "❌ VLC not found. Install with:"
    echo "   sudo apt install vlc"
    exit 1
fi

echo "🎬 Launching stream in VLC..."
echo "   (Converting EAC → Equirectangular with GPU acceleration)"
echo ""

# Use FFmpeg to decode and convert, pipe to VLC
ffmpeg \
    -hwaccel cuda \
    -i "$UDP_URL" \
    -vf "v360=input=eac:output=equirect" \
    -f mpegts \
    -codec:v mpeg2video \
    -b:v 8M \
    - 2>/dev/null | vlc - --play-and-exit --video-title="GoPro Max 2 Stream"

echo "✅ VLC closed"
