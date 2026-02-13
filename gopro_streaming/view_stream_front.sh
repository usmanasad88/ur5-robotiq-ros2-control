#!/bin/bash
# View GoPro Max 2 stream - full resolution, no stitching
# Shows the raw dual-fisheye frame at native 1408x704 by default.
# Optional dewarp converts the dual-fisheye to equirectangular.
#
# Usage:
#   ./view_stream_front.sh              # Raw full-res stream (default)
#   ./view_stream_front.sh --dewarp     # Dewarped equirectangular view
#   ./view_stream_front.sh --fov 190    # Custom input FOV for dewarp
#   ./view_stream_front.sh --player mpv # Use mpv instead of ffplay

set -e

# ── Configuration ────────────────────────────────────────────────────
CAMERA_IP="172.29.170.51"
UDP_PORT="8554"
UDP_URL="udp://${CAMERA_IP}:${UDP_PORT}"

# GoPro Max2 webcam stream is 1408x704 dual-fisheye (two 704x704 lenses)
STREAM_W=1408
STREAM_H=704
LENS_W=704
LENS_H=704

# Defaults
MODE="raw"        # raw | dewarp
H_FOV=190         # horizontal FOV for fisheye dewarp (degrees)
V_FOV=190         # vertical FOV
OUT_W=1920        # output width  (for dewarp mode)
OUT_H=960         # output height (for dewarp mode)
PLAYER="auto"     # auto | ffplay | mpv

# ── Parse arguments ─────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --dewarp)
            MODE="dewarp"
            shift
            ;;
        --fov)
            H_FOV="$2"
            V_FOV="$2"
            shift 2
            ;;
        --width)
            OUT_W="$2"
            shift 2
            ;;
        --height)
            OUT_H="$2"
            shift 2
            ;;
        --player)
            PLAYER="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Full-resolution GoPro Max2 USB stream viewer."
            echo "Shows the native 1408×704 dual-fisheye by default (no stitching)."
            echo ""
            echo "Options:"
            echo "  --dewarp           Dewarp dual-fisheye → equirectangular"
            echo "  --fov  DEG         Input FOV for dewarp (default: 190)"
            echo "  --width  PX        Output width  for dewarp (default: 1920)"
            echo "  --height PX        Output height for dewarp (default: 960)"
            echo "  --player [ffplay|mpv]  Video player (default: auto-detect)"
            echo "  -h, --help         Show this help"
            echo ""
            echo "Examples:"
            echo "  $0                    # Raw 1408×704 dual-fisheye"
            echo "  $0 --dewarp           # Equirectangular 1920×960"
            echo "  $0 --dewarp --fov 180 # Tighter FOV, less edge distortion"
            echo "  $0 --player ffplay    # Force ffplay"
            exit 0
            ;;
        *)
            echo "Unknown option: $1 (use --help)"
            exit 1
            ;;
    esac
done

# ── Auto-detect player ──────────────────────────────────────────────
if [[ "$PLAYER" == "auto" ]]; then
    if command -v mpv &> /dev/null; then
        PLAYER="mpv"
    elif command -v ffplay &> /dev/null; then
        PLAYER="ffplay"
    else
        echo "❌ No video player found. Install mpv or ffplay."
        exit 1
    fi
fi

# ── Build filter chain ──────────────────────────────────────────────
#
# The webcam stream is 1408×704 with two 704×704 fisheye lenses
# side by side:  [  FRONT 704×704  |  BACK 704×704  ]
#
if [[ "$MODE" == "raw" ]]; then
    # Full-resolution pass-through — no filters, native 1408×704
    VF=""
    TITLE="GoPro Max2 – Raw ${STREAM_W}×${STREAM_H}"
else
    # Dewarp: dual-fisheye → equirectangular
    VF="v360=input=dfisheye:output=equirect:ih_fov=${H_FOV}:iv_fov=${V_FOV}:w=${OUT_W}:h=${OUT_H}"
    TITLE="GoPro Max2 – Equirectangular ${OUT_W}×${OUT_H}"
fi

# ── Print banner ─────────────────────────────────────────────────────
echo "==========================================="
echo "  GoPro Max 2 – Full Resolution Stream"
echo "==========================================="
echo ""
echo "  Mode:       $MODE"
echo "  FOV:        ${H_FOV}° × ${V_FOV}°"
echo "  Output:     ${OUT_W}×${OUT_H}"
echo "  Player:     $PLAYER"
echo "  Stream:     $UDP_URL"
echo ""
echo "  Filter:     $VF"
echo ""
echo "  Press 'q' in the video window to quit."
echo ""

# ── Launch player ────────────────────────────────────────────────────
case $PLAYER in
    ffplay)
        VF_ARGS=()
        if [[ -n "$VF" ]]; then
            VF_ARGS=(-vf "$VF")
        fi
        ffplay \
            -probesize 32 \
            -analyzeduration 0 \
            -fflags +nobuffer+flush_packets \
            -flags low_delay \
            -strict experimental \
            -framedrop \
            -infbuf \
            -sync ext \
            -avioflags direct \
            "${VF_ARGS[@]}" \
            -window_title "$TITLE" \
            "$UDP_URL"
        ;;

    mpv)
        VF_ARGS=()
        if [[ -n "$VF" ]]; then
            VF_ARGS=(--vf="lavfi=[$VF]")
        fi
        mpv \
            --profile=low-latency \
            --no-cache \
            --cache-secs=0 \
            --demuxer-max-bytes=128KiB \
            --demuxer-max-back-bytes=64KiB \
            --demuxer-lavf-o=fflags=nobuffer,flags=low_delay \
            --untimed \
            --no-correct-pts \
            --framedrop=vo \
            --opengl-swapinterval=0 \
            --vd-lavc-threads=1 \
            --title="$TITLE" \
            "${VF_ARGS[@]}" \
            "$UDP_URL"
        ;;

    *)
        echo "❌ Unsupported player: $PLAYER"
        exit 1
        ;;
esac

echo ""
echo "✅ Viewer closed"
