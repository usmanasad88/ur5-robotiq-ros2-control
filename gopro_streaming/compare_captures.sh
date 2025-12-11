#!/bin/bash
# Compare different capture modes

echo "=========================================="
echo "  GoPro Max 2 Capture Comparison"
echo "=========================================="
echo ""

# Make sure stream is active
if ! curl -s --connect-timeout 2 http://172.29.170.51/gopro/camera/state > /dev/null; then
    echo "❌ Camera not responding"
    echo "   Run: python connect_gopro.py --keep-alive"
    exit 1
fi

echo "✅ Camera detected"
echo ""

# Close any viewer
pkill -f "ffplay.*172.29.170.51" 2>/dev/null
sleep 1

# Capture raw
echo "1️⃣  Capturing RAW (no projection)..."
python capture_photo.py --filename compare_raw.jpg
echo ""

# Capture with dual fisheye projection
echo "2️⃣  Capturing with DUAL FISHEYE projection..."
python capture_photo.py --projection --filename compare_projected.jpg
echo ""

echo "=========================================="
echo "  ✅ Comparison Complete"
echo "=========================================="
echo ""
echo "Compare these images:"
echo "  compare_raw.jpg       - Raw dual fisheye (two circles)"
echo "  compare_projected.jpg - Converted to 360° equirectangular"
echo ""
echo "The projected version should show a proper 360° panorama!"
