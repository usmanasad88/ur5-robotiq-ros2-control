#!/bin/bash
# Simple GoPro viewer without projection conversion (for testing)

CAMERA_IP="172.29.170.51"
UDP_URL="udp://${CAMERA_IP}:8554"

echo "=================================="
echo "  GoPro Max 2 Stream Viewer"
echo "  Raw View (No Projection)"
echo "=================================="
echo ""

echo "📡 Connecting to stream..."
echo "   Camera: $CAMERA_IP"
echo "   Stream: $UDP_URL"
echo ""
echo "💡 Tips:"
echo "   - If stream stops, make sure connect_gopro.py is still running"
echo "   - Press 'q' to quit"
echo "   - If video is distorted, use view_stream.sh for proper projection"
echo ""

# Direct ffplay with minimal processing
ffplay \
    -fflags nobuffer \
    -flags low_delay \
    -framedrop \
    -infbuf \
    -probesize 32 \
    -analyzeduration 0 \
    -sync ext \
    -vf "scale=1280:720" \
    -window_title "GoPro Max 2 - Raw Stream" \
    "$UDP_URL"

echo ""
echo "✅ Viewer closed"
