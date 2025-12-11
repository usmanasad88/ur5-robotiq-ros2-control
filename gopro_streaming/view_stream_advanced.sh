#!/bin/bash
# Advanced GoPro Max 2 stream viewer with multiple GPU acceleration options
# Supports different projection modes and quality settings

set -e

# Default configuration
CAMERA_IP="172.29.170.51"
UDP_URL="udp://${CAMERA_IP}:8554"
INPUT_PROJECTION="eac"
OUTPUT_PROJECTION="equirect"
GPU_MODE="cuda"
VIEWER="ffplay"
QUALITY="high"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --projection)
            OUTPUT_PROJECTION="$2"
            shift 2
            ;;
        --gpu)
            GPU_MODE="$2"
            shift 2
            ;;
        --viewer)
            VIEWER="$2"
            shift 2
            ;;
        --quality)
            QUALITY="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --projection [equirect|flat|cylindrical]  Output projection (default: equirect)"
            echo "  --gpu [cuda|nvdec|none]                   GPU acceleration mode (default: cuda)"
            echo "  --viewer [ffplay|vlc|mpv]                 Video player (default: ffplay)"
            echo "  --quality [low|medium|high]               Stream quality (default: high)"
            echo "  --help                                     Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                   # Default settings"
            echo "  $0 --projection flat --viewer mpv   # Flat projection with MPV"
            echo "  $0 --gpu none --quality low         # CPU decoding, low quality"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "=========================================="
echo "  GoPro Max 2 Advanced Stream Viewer"
echo "=========================================="
echo ""
echo "Configuration:"
echo "  Input:       EAC (Equi-Angular Cubemap)"
echo "  Output:      $OUTPUT_PROJECTION"
echo "  GPU Mode:    $GPU_MODE"
echo "  Viewer:      $VIEWER"
echo "  Quality:     $QUALITY"
echo ""

# Build GPU acceleration flags
GPU_FLAGS=""
case $GPU_MODE in
    cuda)
        GPU_FLAGS="-hwaccel cuda -hwaccel_output_format cuda"
        echo "🚀 Using NVIDIA CUDA acceleration"
        ;;
    nvdec)
        GPU_FLAGS="-c:v h264_cuvid"
        echo "🚀 Using NVIDIA NVDEC decoder"
        ;;
    none)
        GPU_FLAGS=""
        echo "⚙️  Using CPU decoding"
        ;;
esac

# Build video filter based on projection
VIDEO_FILTER="v360=input=$INPUT_PROJECTION:output=$OUTPUT_PROJECTION"

# Add quality settings
case $QUALITY in
    low)
        SCALE_FILTER="scale=1280:720"
        ;;
    medium)
        SCALE_FILTER="scale=1920:1080"
        ;;
    high)
        SCALE_FILTER=""
        ;;
esac

if [ -n "$SCALE_FILTER" ]; then
    VIDEO_FILTER="$VIDEO_FILTER,$SCALE_FILTER"
fi

echo "📡 Connecting to UDP stream..."

# Launch viewer based on selection
case $VIEWER in
    ffplay)
        echo "🎬 Launching FFplay..."
        ffmpeg $GPU_FLAGS -i "$UDP_URL" -vf "$VIDEO_FILTER" -f nut - 2>/dev/null | \
            ffplay -i - -window_title "GoPro Max 2 Stream" -autoexit
        ;;
    
    vlc)
        echo "🎬 Launching VLC..."
        if ! command -v vlc &> /dev/null; then
            echo "❌ VLC not found. Install: sudo apt install vlc"
            exit 1
        fi
        ffmpeg $GPU_FLAGS -i "$UDP_URL" -vf "$VIDEO_FILTER" \
            -f mpegts -codec:v mpeg2video -b:v 8M - 2>/dev/null | \
            vlc - --play-and-exit --video-title="GoPro Max 2 Stream"
        ;;
    
    mpv)
        echo "🎬 Launching MPV..."
        if ! command -v mpv &> /dev/null; then
            echo "❌ MPV not found. Install: sudo apt install mpv"
            exit 1
        fi
        ffmpeg $GPU_FLAGS -i "$UDP_URL" -vf "$VIDEO_FILTER" -f nut - 2>/dev/null | \
            mpv - --title="GoPro Max 2 Stream"
        ;;
    
    *)
        echo "❌ Unknown viewer: $VIEWER"
        exit 1
        ;;
esac

echo ""
echo "✅ Stream viewer closed"
