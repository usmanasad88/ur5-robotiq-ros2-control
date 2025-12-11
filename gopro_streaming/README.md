# GoPro Max 2 USB Streaming Workflow

Real-time camera streaming pipeline for GoPro Max 2 using USB connection with NVIDIA GPU acceleration.

## Overview

This workflow enables low-latency streaming from GoPro Max 2 via USB, with EAC (Equi-Angular Cubemap) to Equirectangular projection conversion using NVIDIA RTX 4070 Ti Super GPU acceleration.

## Directory Structure

```
gopro_streaming/
├── README.md                    # This file
├── QUICKSTART.md                # Quick start guide
├── setup_environment.sh         # Environment setup script
├── connect_gopro.py             # GoPro USB connection script
├── view_stream.sh               # Basic stream viewer (FFplay)
├── view_stream_vlc.sh           # VLC-based viewer
└── view_stream_advanced.sh      # Advanced viewer with options
```

## System Requirements

- **OS:** Ubuntu 22.04 (or compatible Linux)
- **GPU:** NVIDIA RTX 4070 Ti Super (or any CUDA-capable GPU)
- **Camera:** GoPro Max 2 connected via USB
- **Python:** Conda environment (`ur5_python`)
- **Dependencies:** open-gopro SDK, ffmpeg with CUDA support

## Quick Start

### 1. Environment Setup

```bash
cd /home/mani/Repos/ur_ws/gopro_streaming
chmod +x *.sh
./setup_environment.sh
```

This will:
- Activate `ur5_python` conda environment
- Install `open-gopro` Python SDK
- Install/verify `ffmpeg` with CUDA support
- Check NVIDIA GPU availability

### 2. Connect GoPro and Start Stream

```bash
# Auto-detect GoPro and start webcam mode
python connect_gopro.py

# Or specify IP address if known
python connect_gopro.py --ip 172.20.110.51

# Keep connection alive
python connect_gopro.py --keep-alive
```

### 3. View the Stream

```bash
# Option 1: Basic FFplay viewer (recommended)
./view_stream.sh

# Option 2: VLC player
./view_stream_vlc.sh

# Option 3: Advanced viewer with options
./view_stream_advanced.sh --projection equirect --viewer ffplay
```

## Detailed Usage

### Connection Script (`connect_gopro.py`)

```bash
# Basic usage - auto-detect camera
python connect_gopro.py

# Specify IP address
python connect_gopro.py --ip 172.20.110.51

# Custom timeout
python connect_gopro.py --timeout 10

# Stop webcam mode
python connect_gopro.py --stop

# Keep connection alive with heartbeat
python connect_gopro.py --keep-alive
```

**Expected Output:**
```
============================================================
  GoPro Max 2 USB Webcam Controller
============================================================

🔌 Checking USB network interfaces...
   ✓ USB network interfaces found:
     inet 172.20.110.50/24 ...

🔍 Scanning for GoPro camera on USB network...
   Trying 172.20.110.51... ✓ Found!

✅ Connected to GoPro at 172.20.110.51

⚙️  Enabling wired USB control mode...
   ✓ USB control enabled

📹 Starting GoPro webcam/live stream mode...
   Trying endpoint: /gopro/webcam/start... ✓

✅ GoPro webcam mode started!
   Camera IP: 172.20.110.51
   UDP Stream: udp://@:8554
```

### Stream Viewers

#### Basic Viewer (`view_stream.sh`)

Uses FFmpeg with CUDA hardware decoding and FFplay for display:

```bash
./view_stream.sh
```

Features:
- NVIDIA GPU-accelerated H.264 decoding
- EAC → Equirectangular conversion
- Low latency display
- Automatic stream detection

#### VLC Viewer (`view_stream_vlc.sh`)

Pipes FFmpeg output to VLC player:

```bash
./view_stream_vlc.sh
```

Features:
- Familiar VLC interface
- GPU-accelerated pipeline
- MPEG-TS format for streaming

#### Advanced Viewer (`view_stream_advanced.sh`)

Customizable viewer with multiple options:

```bash
# Default settings
./view_stream_advanced.sh

# Flat projection with MPV
./view_stream_advanced.sh --projection flat --viewer mpv

# CPU decoding (no GPU)
./view_stream_advanced.sh --gpu none

# Low quality for reduced bandwidth
./view_stream_advanced.sh --quality low

# Full options
./view_stream_advanced.sh --projection equirect --gpu cuda --viewer ffplay --quality high
```

**Options:**
- `--projection [equirect|flat|cylindrical]` - Output projection type
- `--gpu [cuda|nvdec|none]` - GPU acceleration mode
- `--viewer [ffplay|vlc|mpv]` - Video player
- `--quality [low|medium|high]` - Stream quality/resolution

## Manual FFmpeg Commands

If you prefer running FFmpeg manually:

### Basic Command (GPU Accelerated)

```bash
ffmpeg -hwaccel cuda -i udp://@:8554 \
    -vf 'v360=input=eac:output=equirect' \
    -f sdl 'GoPro Max 2 Stream'
```

### With NVDEC Decoder

```bash
ffmpeg -c:v h264_cuvid -i udp://@:8554 \
    -vf 'v360=input=eac:output=equirect' \
    -f sdl 'GoPro Max 2 Stream'
```

### Pipe to FFplay

```bash
ffmpeg -hwaccel cuda -i udp://@:8554 \
    -vf 'v360=input=eac:output=equirect' \
    -f nut - | ffplay -i -
```

### Pipe to VLC

```bash
ffmpeg -hwaccel cuda -i udp://@:8554 \
    -vf 'v360=input=eac:output=equirect' \
    -f mpegts -codec:v mpeg2video -b:v 8M - | \
    vlc -
```

### CPU Decoding (Fallback)

```bash
ffmpeg -i udp://@:8554 \
    -vf 'v360=input=eac:output=equirect' \
    -f sdl 'GoPro Max 2 Stream'
```

## Projection Formats

The GoPro Max 2 uses **EAC (Equi-Angular Cubemap)** format. The `v360` filter converts this to viewable formats:

| Format | Description | Use Case |
|--------|-------------|----------|
| `equirect` | Equirectangular (360° panorama) | Full 360° viewing |
| `flat` | Rectilinear (flat) | Normal perspective view |
| `cylindrical` | Cylindrical projection | Wide horizontal view |
| `stereographic` | Stereographic projection | Fish-eye effect |

Example:
```bash
# Flat perspective view
ffmpeg -hwaccel cuda -i udp://@:8554 \
    -vf 'v360=input=eac:output=flat' \
    -f sdl 'GoPro Stream'
```

## Troubleshooting

### Camera Not Detected

**Symptoms:** `Could not detect GoPro camera`

**Solutions:**
1. Check USB connection and camera power
2. Verify USB network interface:
   ```bash
   ip addr show | grep 172.2
   ```
3. Try pinging camera manually:
   ```bash
   ping 172.20.110.51
   ping 172.21.110.51
   ping 172.22.110.51
   ```
4. Check if interface is up:
   ```bash
   sudo ip link set usb0 up  # Replace usb0 with your interface
   ```

### No Stream Available

**Symptoms:** `No stream detected on UDP port 8554`

**Solutions:**
1. Ensure `connect_gopro.py` ran successfully
2. Check if webcam mode is active on camera display
3. Verify UDP port is open:
   ```bash
   sudo netstat -ulnp | grep 8554
   ```
4. Try connecting with `--keep-alive` flag:
   ```bash
   python connect_gopro.py --keep-alive
   ```

### GPU Acceleration Not Working

**Symptoms:** High CPU usage, low FPS, or errors about CUDA

**Solutions:**
1. Check NVIDIA driver:
   ```bash
   nvidia-smi
   ```
2. Verify FFmpeg CUDA support:
   ```bash
   ffmpeg -hwaccels
   ```
   Should show `cuda` in the list.
3. Rebuild FFmpeg with CUDA support:
   ```bash
   conda install -c conda-forge ffmpeg
   ```
4. Fall back to CPU decoding:
   ```bash
   ./view_stream_advanced.sh --gpu none
   ```

### Video Appears Distorted

**Symptoms:** Image looks warped or incorrect

**Solutions:**
1. Verify projection format:
   ```bash
   ./view_stream_advanced.sh --projection equirect
   ```
2. Check if camera is in correct mode (should be in webcam/preview mode)
3. Try different projection formats:
   ```bash
   ./view_stream_advanced.sh --projection flat
   ```

### High Latency

**Symptoms:** Noticeable delay between camera and display

**Solutions:**
1. Use FFplay instead of VLC:
   ```bash
   ./view_stream.sh
   ```
2. Enable GPU acceleration:
   ```bash
   ./view_stream_advanced.sh --gpu cuda
   ```
3. Reduce quality:
   ```bash
   ./view_stream_advanced.sh --quality low
   ```
4. Add low-latency flags to manual FFmpeg command:
   ```bash
   ffmpeg -hwaccel cuda -fflags nobuffer -flags low_delay \
       -i udp://@:8554 -vf 'v360=input=eac:output=equirect' \
       -f sdl 'GoPro Stream'
   ```

## Network Interface Details

GoPro cameras use USB Ethernet (RNDIS) when connected via USB:

- **Default IP Range:** `172.20.110.x` - `172.25.110.x`
- **Camera IP:** Typically `172.20.110.51`
- **Host IP:** Typically `172.20.110.50`
- **Subnet:** `/24` (255.255.255.0)

To manually configure the interface:
```bash
# Find interface name
ip addr show | grep 172.2

# Set interface up
sudo ip link set <interface> up

# Add IP if needed
sudo ip addr add 172.20.110.50/24 dev <interface>
```

## Performance Tips

1. **Use GPU acceleration** - Provides 5-10x speedup over CPU decoding
2. **Close unnecessary applications** - Free up GPU memory
3. **Use FFplay over VLC** - Lower latency
4. **Reduce quality if needed** - Use `--quality low` for bandwidth-limited scenarios
5. **Keep connection alive** - Use `--keep-alive` to maintain stable stream

## Advanced Configuration

### Custom UDP Port

If using a different port:
```bash
# In connect_gopro.py, modify UDP_PORT
UDP_PORT = 8555

# In viewer scripts, update UDP_URL
UDP_URL="udp://@:8555"
```

### Recording Stream

```bash
# Record to file while viewing
ffmpeg -hwaccel cuda -i udp://@:8554 \
    -vf 'v360=input=eac:output=equirect' \
    -c:v libx264 -preset fast -crf 18 \
    output.mp4
```

### Streaming to Network

```bash
# Re-stream to RTMP server
ffmpeg -hwaccel cuda -i udp://@:8554 \
    -vf 'v360=input=eac:output=equirect' \
    -c:v libx264 -preset veryfast -b:v 4M \
    -f flv rtmp://your-server/live/stream
```

## API Reference

### GoPro HTTP Endpoints

The `connect_gopro.py` script uses these HTTP endpoints:

- `GET /gopro/camera/state` - Get camera status
- `GET /gopro/camera/control/wired_usb?p=1` - Enable USB control
- `GET /gopro/webcam/start` - Start webcam mode
- `GET /gopro/webcam/stop` - Stop webcam mode
- `GET /gopro/camera/keep_alive` - Maintain connection

### v360 Filter Parameters

FFmpeg `v360` filter syntax:
```
v360=input=<input_projection>:output=<output_projection>[:options]
```

Common options:
- `input=eac` - Equi-Angular Cubemap (GoPro Max format)
- `output=equirect` - Equirectangular projection
- `output=flat` - Rectilinear projection
- `ih_fov=90` - Input horizontal field of view (degrees)
- `iv_fov=90` - Input vertical field of view (degrees)

## Integration with ROS 2

To integrate with the UR5 workspace:

```bash
# Source ROS 2 workspace
source /home/mani/Repos/ur_ws/install/setup.bash

# Run GoPro stream in background
python connect_gopro.py --keep-alive &

# Launch your ROS 2 nodes
ros2 launch your_package your_launch.py
```

## License

Part of the `ur5-robotiq-ros2-control` workspace.

## Support

For issues or questions:
1. Check this README's troubleshooting section
2. Verify all dependencies are installed
3. Check GoPro camera firmware is up to date
4. Review GoPro USB connection documentation

## References

- [Open GoPro SDK](https://github.com/gopro/OpenGoPro)
- [FFmpeg v360 Filter](https://ffmpeg.org/ffmpeg-filters.html#v360)
- [NVIDIA CUDA FFmpeg](https://developer.nvidia.com/ffmpeg)
