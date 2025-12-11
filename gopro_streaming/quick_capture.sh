#!/bin/bash
# Quick photo capture - pauses viewer, captures, resumes

echo "📸 Quick Photo Capture"
echo ""

# Check if viewer is running
VIEWER_PID=$(pgrep -f "ffplay.*172.29.170.51:8554")

if [ -n "$VIEWER_PID" ]; then
    echo "⏸️  Pausing viewer (PID: $VIEWER_PID)..."
    kill $VIEWER_PID
    sleep 1
fi

# Capture photo
echo "📸 Capturing..."
python capture_photo.py "$@"
RESULT=$?

# Ask if user wants to resume viewer
if [ -n "$VIEWER_PID" ] && [ $RESULT -eq 0 ]; then
    echo ""
    read -p "Resume viewer? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "▶️  Resuming viewer..."
        ./view_stream_raw.sh &
    fi
fi
