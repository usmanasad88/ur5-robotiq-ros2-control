#!/usr/bin/env python3
"""
GoPro video recorder integration for episode recording.

Records from the GoPro Max 2 UDP preview stream to an MP4 file via ffmpeg.
Handles stream activation/deactivation via the GoPro HTTP API automatically.
"""

import subprocess
import time
import os
from pathlib import Path
from typing import Optional, Tuple

import requests


# GoPro HTTP API endpoints (port 8080)
_EP_STATE = "/gopro/camera/state"
_EP_WIRED_USB = "/gopro/camera/control/wired_usb?p=1"
_EP_STREAM_START = "/gopro/camera/stream/start"
_EP_STREAM_STOP = "/gopro/camera/stream/stop"


class GoProRecorder:
    """GoPro recorder using UDP preview stream and ffmpeg.

    Automatically activates the preview stream via the GoPro HTTP API
    before recording, matching the protocol used by the AURA
    ``GoProStreamSource``.
    """

    def __init__(self, gopro_ip: str = "172.29.170.51", udp_port: int = 8554):
        self.ffmpeg_process = None
        self.output_file = None
        self.is_recording = False
        self.gopro_ip = gopro_ip
        self.udp_port = udp_port
        self._stream_started_by_us = False

        # UDP URL with ffmpeg-friendly buffer settings
        self.stream_url = (
            f"udp://{gopro_ip}:{udp_port}"
            "?overrun_nonfatal=1&fifo_size=50000000"
        )

    # ------------------------------------------------------------------
    # GoPro HTTP helpers
    # ------------------------------------------------------------------

    def _http_get(self, endpoint: str, timeout: int = 5, silent: bool = False) -> bool:
        """Send GET to the GoPro HTTP API.  Returns True on HTTP 200."""
        url = f"http://{self.gopro_ip}:8080{endpoint}"
        try:
            resp = requests.get(url, timeout=timeout)
            ok = resp.status_code == 200
            if not ok and not silent:
                print(f"⚠ GoPro API: {url} returned HTTP {resp.status_code}")
            return ok
        except Exception as exc:
            if not silent:
                print(f"⚠ GoPro API: {url} failed: {exc}")
            return False

    def _ensure_stream_active(self) -> bool:
        """Ping the camera, enable wired USB, and start the preview stream."""
        # 1. Check camera is reachable
        if not self._http_get(_EP_STATE, silent=True):
            print("⚠ GoPro not responding — is it connected and powered on?")
            return False

        # 2. Enable wired USB control
        self._http_get(_EP_WIRED_USB, silent=True)
        time.sleep(0.3)

        # 3. Start preview stream
        if not self._http_get(_EP_STREAM_START):
            print("⚠ Failed to start GoPro preview stream")
            return False

        self._stream_started_by_us = True
        time.sleep(0.5)  # let the stream stabilise
        return True
    
    def start_recording(self, output_path: Path) -> Tuple[bool, str]:
        """Start recording from the GoPro UDP preview stream.

        Activates the stream via the HTTP API if needed, then launches
        an ffmpeg subprocess to write the HEVC stream directly to MP4.
        """
        if self.is_recording:
            return False, "Already recording"

        # Activate GoPro preview stream
        if not self._ensure_stream_active():
            return False, "GoPro stream not available (camera unreachable or stream failed to start)"

        self.output_file = output_path
        
        # FFmpeg command to capture UDP stream
        cmd = [
            'ffmpeg',
            '-i', self.stream_url,
            '-c:v', 'copy',  # Copy video codec (no re-encoding)
            '-c:a', 'aac',   # Encode audio as AAC
            '-f', 'mp4',
            '-y',  # Overwrite output file
            str(output_path)
        ]
        
        try:
            # Start ffmpeg in background
            self.ffmpeg_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            # Give it a moment to start
            time.sleep(0.5)
            
            # Check if process is still running
            if self.ffmpeg_process.poll() is None:
                self.is_recording = True
                return True, f"GoPro recording started: {output_path}"
            else:
                return False, "FFmpeg failed to start (is GoPro streaming?)"
                
        except FileNotFoundError:
            return False, "FFmpeg not found - install with: sudo apt install ffmpeg"
        except Exception as e:
            return False, f"Failed to start recording: {e}"
    
    def stop_recording(self) -> Tuple[bool, str]:
        """Stop recording and finalize the MP4 file."""
        if not self.is_recording or not self.ffmpeg_process:
            return False, "Not currently recording"
        
        try:
            # Send quit signal to ffmpeg (graceful shutdown)
            self.ffmpeg_process.terminate()
            
            # Wait for process to finish (with timeout)
            self.ffmpeg_process.wait(timeout=5)
            
            self.is_recording = False
            self.ffmpeg_process = None

            # Stop the preview stream if we started it
            if self._stream_started_by_us:
                self._http_get(_EP_STREAM_STOP, silent=True)
                self._stream_started_by_us = False
            
            # Check if file was created
            if self.output_file and self.output_file.exists():
                size_mb = self.output_file.stat().st_size / (1024 * 1024)
                return True, f"GoPro recording saved: {self.output_file} ({size_mb:.1f} MB)"
            else:
                return False, "Recording file was not created"
                
        except subprocess.TimeoutExpired:
            # Force kill if it doesn't stop gracefully
            self.ffmpeg_process.kill()
            self.ffmpeg_process.wait()
            return True, "Recording stopped (forced)"
        except Exception as e:
            return False, f"Error stopping recording: {e}"
    
    def is_gopro_streaming(self) -> bool:
        """Check if the GoPro is reachable via its HTTP API."""
        return self._http_get(_EP_STATE, timeout=2, silent=True)
