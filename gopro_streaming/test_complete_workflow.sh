#!/bin/bash
# Complete workflow test - starts stream and viewer together

echo "=========================================="
echo "  GoPro Max 2 Complete Streaming Test"
echo "=========================================="
echo ""

# Check if camera is accessible
if ! curl -s --connect-timeout 2 http://172.29.170.51/gopro/camera/state > /dev/null; then
    echo "❌ Camera not responding at 172.29.170.51"
    echo "   Make sure GoPro is connected and powered on"
    exit 1
fi

echo "✅ Camera detected"
echo ""

# Start connection in background
echo "1. Starting camera stream..."
python connect_gopro.py > /tmp/gopro_stream.log 2>&1 &
STREAM_PID=$!

echo "   Stream PID: $STREAM_PID"
echo "   Waiting for stream to initialize..."
sleep 4

# Check if stream process is still running
if ! ps -p $STREAM_PID > /dev/null; then
    echo "   ❌ Stream process died. Log:"
    cat /tmp/gopro_stream.log
    exit 1
fi

echo "   ✅ Stream is running"
echo ""

# Show stream info
grep "UDP Stream URL" /tmp/gopro_stream.log

echo ""
echo "2. Launching viewer..."
echo "   Close the viewer window or press Ctrl+C here to stop"
echo ""

# Launch viewer (raw version for testing)
./view_stream_raw.sh

# Cleanup
echo ""
echo "3. Stopping stream..."
kill $STREAM_PID 2>/dev/null
wait $STREAM_PID 2>/dev/null

echo "✅ Test complete"
