#!/bin/bash
# Quick test script to verify GoPro streaming setup

echo "=========================================="
echo "  GoPro Max 2 Streaming Test"
echo "=========================================="
echo ""

# Check if script exists
if [ ! -f "connect_gopro.py" ]; then
    echo "❌ connect_gopro.py not found"
    echo "   Make sure you're in the gopro_streaming directory"
    exit 1
fi

# Check interface
echo "1. Checking USB interface..."
if ip addr show enx04574796c048 &> /dev/null; then
    IP=$(ip addr show enx04574796c048 | grep "inet " | awk '{print $2}')
    echo "   ✓ Interface found: $IP"
else
    echo "   ❌ Interface enx04574796c048 not found"
    exit 1
fi

# Check camera connectivity
echo ""
echo "2. Testing camera connectivity..."
if timeout 2 curl -s http://172.29.170.51/gopro/camera/state > /dev/null 2>&1; then
    echo "   ✓ Camera responding at 172.29.170.51"
else
    echo "   ❌ Camera not responding"
    echo "   Make sure GoPro is powered ON"
    exit 1
fi

# Test stream start
echo ""
echo "3. Starting stream..."
python connect_gopro.py > /tmp/gopro_test.log 2>&1 &
PID=$!
echo "   Started (PID: $PID)"
sleep 3

# Check if process is still running
if ps -p $PID > /dev/null 2>&1; then
    echo "   ✓ Stream process running"
    
    # Show output
    echo ""
    echo "4. Connection output:"
    cat /tmp/gopro_test.log | grep -E "✓|✅|📺"
    
    # Kill the process
    echo ""
    echo "5. Stopping test stream..."
    kill $PID 2>/dev/null
    wait $PID 2>/dev/null
    echo "   ✓ Stopped"
else
    echo "   ❌ Stream process failed"
    cat /tmp/gopro_test.log
    exit 1
fi

echo ""
echo "=========================================="
echo "  ✅ All Tests Passed!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Start stream: python connect_gopro.py --keep-alive"
echo "  2. View stream: ./view_stream.sh"
echo ""
