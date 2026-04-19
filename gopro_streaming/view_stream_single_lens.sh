#!/bin/bash
# View a single GoPro Max 2 lens as a normal-FOV rectilinear image.
#
# Pipeline:
#   1408x704 dual-fisheye  →  crop one 704x704 lens  →  fisheye→flat remap
#
# Usage:
#   ./view_stream_single_lens.sh                 # front lens, 90° output FOV
#   ./view_stream_single_lens.sh --lens back     # back lens
#   ./view_stream_single_lens.sh --hfov 75       # tighter FOV (more zoom)
#   ./view_stream_single_lens.sh --hfov 120      # wider FOV
#   ./view_stream_single_lens.sh --size 1920x1080
#
# Prereq: `python connect_gopro.py --keep-alive` running in another terminal.

set -e

CAMERA_IP="172.29.170.51"
UDP_PORT="8554"
UDP_URL="udp://0.0.0.0:${UDP_PORT}"

LENS="front"        # front | back
IN_FOV=190          # GoPro Max fisheye input FOV
OUT_HFOV=90         # rectilinear output horizontal FOV
OUT_VFOV=60
OUT_W=1280
OUT_H=720

while [[ $# -gt 0 ]]; do
    case $1 in
        --lens)  LENS="$2"; shift 2 ;;
        --hfov)  OUT_HFOV="$2"; shift 2 ;;
        --vfov)  OUT_VFOV="$2"; shift 2 ;;
        --size)  OUT_W="${2%x*}"; OUT_H="${2#*x}"; shift 2 ;;
        -h|--help)
            sed -n '2,19p' "$0"
            exit 0 ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

# Crop: left half for front, right half for back.
if [[ "$LENS" == "front" ]]; then
    CROP="crop=704:704:0:0"
else
    CROP="crop=704:704:704:0"
fi

FILTER="${CROP},v360=input=fisheye:output=flat:ih_fov=${IN_FOV}:iv_fov=${IN_FOV}:h_fov=${OUT_HFOV}:v_fov=${OUT_VFOV}:w=${OUT_W}:h=${OUT_H}"

echo "==========================================="
echo "  GoPro Max 2 – Single Lens (${LENS})"
echo "  Output: ${OUT_W}x${OUT_H}  FOV: ${OUT_HFOV}°×${OUT_VFOV}°"
echo "==========================================="

exec ffplay \
    -hide_banner \
    -fflags nobuffer \
    -flags low_delay \
    -framedrop \
    -infbuf \
    -probesize 32 \
    -analyzeduration 0 \
    -sync ext \
    -vf "$FILTER" \
    -window_title "GoPro Max 2 – ${LENS} lens" \
    "$UDP_URL"
