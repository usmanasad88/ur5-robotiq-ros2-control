# GoPro Max 2 Streaming - Quick Start Guide

Get up and running with GoPro Max 2 USB streaming in 5 minutes.

## Prerequisites

- GoPro Max 2 camera
- USB cable connected to your Linux machine
- NVIDIA GPU (RTX 4070 Ti Super or compatible)
- Ubuntu 22.04

## Step 1: Navigate to Directory

```bash
cd /home/mani/Repos/ur_ws/gopro_streaming
```

## Step 2: Make Scripts Executable

```bash
chmod +x *.sh
```

## Step 3: Setup Environment (One-time)

```bash
./setup_environment.sh
```

This installs all required dependencies.

## Step 4: Activate Conda Environment

Every time you start a new terminal session:

```bash
conda activate ur5_python
```

## Step 5: Connect GoPro

Make sure your GoPro Max 2 is:
- Powered ON
- Connected via USB cable
- Not in any other mode (let it boot to home screen)

Then run:

```bash
python connect_gopro.py
```

**Expected output:**
```
============================================================
  GoPro Max 2 USB Webcam Controller
============================================================

🔍 Scanning for GoPro camera on USB network...
   Trying 172.20.110.51... ✓ Found!

✅ Connected to GoPro at 172.20.110.51

📹 Starting GoPro webcam/live stream mode...
   ✓ Webcam started

✅ GoPro webcam mode started!
   Camera IP: 172.20.110.51
   UDP Stream: udp://@:8554
```

## Step 6: View the Stream

Open a **new terminal**, activate the environment, and run:

```bash
conda activate ur5_python
cd /home/mani/Repos/ur_ws/gopro_streaming
./view_stream.sh
```

You should see the camera feed in a window!

## Quick Commands Reference

```bash
# Terminal 1: Connect to camera
conda activate ur5_python
cd /home/mani/Repos/ur_ws/gopro_streaming
python connect_gopro.py --keep-alive

# Terminal 2: View stream
conda activate ur5_python
cd /home/mani/Repos/ur_ws/gopro_streaming
./view_stream.sh
```

## Troubleshooting Quick Fixes

### "Could not detect GoPro camera" or "GoPro powers off after USB connection"

**This is the most common issue!** Your GoPro is in CHARGING mode, not NETWORK mode.

**Solution:**

1. **On the GoPro camera screen:**
   - Swipe down to access Preferences
   - Go to: Preferences → Connections → Wired Connections (or USB Connection)
   - Change from "Charge" or "MTP" to "GoPro Connect"
   - Or enable "Quik App" mode

2. **Try the diagnostic tool:**
   ```bash
   python diagnose_gopro.py
   ```

3. **Manual IP detection:**
   ```bash
   # Find your USB network
   ip addr show | grep 172.
   
   # If you see 172.29.170.55, try pinging:
   ping 172.29.170.51
   ping 172.29.170.1
   
   # Then connect with detected IP:
   python connect_gopro.py --ip 172.29.170.51
   ```

4. **If nothing works, try WiFi instead:**
   - GoPro Max 2 may prefer wireless for live streaming
   - Enable WiFi on camera and connect to its network

### "No stream detected on UDP port 8554"

Make sure `connect_gopro.py` is still running. If you closed it, run again with:

```bash
python connect_gopro.py --keep-alive
```

### Video looks distorted

The camera might not be in the correct mode. Try:

```bash
python connect_gopro.py --stop
# Wait 2 seconds
python connect_gopro.py
```

### High CPU usage / Low FPS

Make sure GPU acceleration is working:

```bash
# Check NVIDIA driver
nvidia-smi

# Verify FFmpeg has CUDA
ffmpeg -hwaccels | grep cuda
```

## Alternative Viewers

```bash
# Use VLC player
./view_stream_vlc.sh

# Use advanced viewer with options
./view_stream_advanced.sh --projection flat --viewer mpv
```

## Stopping Everything

```bash
# Stop the viewer: Press 'q' or Ctrl+C in viewer window

# Stop the connection: Ctrl+C in connect_gopro.py terminal

# Or explicitly stop webcam mode:
python connect_gopro.py --stop
```

## Next Steps

- Read [README.md](README.md) for detailed documentation
- Try different projection modes
- Adjust stream quality settings
- Record streams to file

## One-Liner Workflow

After initial setup, you can use this single command to start everything:

```bash
conda activate ur5_python && \
cd /home/mani/Repos/ur_ws/gopro_streaming && \
python connect_gopro.py --keep-alive &
sleep 3 && \
./view_stream.sh
```

## Need Help?

Check the full [README.md](README.md) for:
- Detailed troubleshooting
- Manual FFmpeg commands
- Advanced configuration
- API reference
- Performance tips
