#!/usr/bin/env python3
"""
GoPro video recorder integration for episode recording.

Simplified wrapper around GoPro streaming for recording episodes.
"""

import subprocess
import time
import os
from pathlib import Path
from typing import Optional, Tuple

class GoProRecorder:
    """Simple GoPro recorder using UDP stream and ffmpeg."""
    
    def __init__(self, gopro_ip: str = "172.29.170.51"):
        self.ffmpeg_process = None
        self.output_file = None
        self.is_recording = False
        self.gopro_ip = gopro_ip
        
        # GoPro UDP stream URL (assumes GoPro is already in webcam mode)
        self.stream_url = f"udp://{gopro_ip}:8554?overrun_nonfatal=1&fifo_size=50000000"
    
    def start_recording(self, output_path: Path) -> Tuple[bool, str]:
        """
        Start recording from GoPro UDP stream.
        
        Args:
            output_path: Path object pointing to output MP4 file
            
        Returns:
            (success, message)
        """
        if self.is_recording:
            return False, "Already recording"
        
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
        """
        Stop recording and finalize video file.
        
        Returns:
            (success, message)
        """
        if not self.is_recording or not self.ffmpeg_process:
            return False, "Not currently recording"
        
        try:
            # Send quit signal to ffmpeg (graceful shutdown)
            self.ffmpeg_process.terminate()
            
            # Wait for process to finish (with timeout)
            self.ffmpeg_process.wait(timeout=5)
            
            self.is_recording = False
            self.ffmpeg_process = None
            
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
        """
        Check if GoPro is currently streaming on UDP port.
        
        Returns:
            True if stream appears to be active
        """
        try:
            # Try to connect to GoPro HTTP API to check if it's alive
            import socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((self.gopro_ip, 8080))
            sock.close()
            if result == 0:
                return True
            
            # Fallback: check if UDP port 8554 is in use
            result = subprocess.run(
                ['netstat', '-anu'],
                capture_output=True,
                text=True,
                timeout=2
            )
            return ':8554' in result.stdout
        except Exception:
            return False
