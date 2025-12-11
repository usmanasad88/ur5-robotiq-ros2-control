#!/usr/bin/env python3
"""
GoPro Max 2 USB Connection and Webcam/Live Stream Activation Script

This script:
1. Detects GoPro Max 2 connected via USB (USB Ethernet/RNDIS interface)
2. Uses open-gopro library to connect and enable wired USB webcam mode
3. Outputs the UDP stream URL for use with ffmpeg

Author: GitHub Copilot
Date: 2025-12-12
"""

import sys
import time
import argparse
import subprocess
import re
import ipaddress
from typing import Optional, List

# Import requests (always needed)
import requests

# Try to import open-gopro
try:
    from open_gopro import WiredGoPro, Params
    from open_gopro.constants import WebcamError, WebcamStatus
    GOPRO_LIBRARY_AVAILABLE = True
except ImportError:
    GOPRO_LIBRARY_AVAILABLE = False


class GoProUSBController:
    """Controller for GoPro Max 2 over USB connection."""
    
    # GoPro USB interface name pattern
    USB_INTERFACE_PATTERN = r"enx[0-9a-f]{12}"
    TARGET_INTERFACE = "enx04574796c048"  # Specific interface from user
    
    # Common camera IP patterns (usually .51, .50, or .1 in the subnet)
    CAMERA_IP_CANDIDATES = [51, 50, 1, 2]
    
    # UDP stream configuration
    UDP_PORT = 8554
    UDP_HOST = "0.0.0.0"
    
    def __init__(self, interface: Optional[str] = None, ip: Optional[str] = None, timeout: int = 5):
        """
        Initialize GoPro USB controller.
        
        Args:
            interface: Specific USB network interface (e.g., 'enx04574796c048')
            ip: Specific IP address to use (if known)
            timeout: Connection timeout in seconds
        """
        self.interface = interface or self.TARGET_INTERFACE
        self.ip = ip
        self.timeout = timeout
        self.base_url = None
        self.gopro = None
    
    def find_usb_interface(self) -> Optional[str]:
        """
        Find GoPro USB network interface (enxXXXXXXXXXXXX pattern).
        
        Returns:
            Interface name or None if not found
        """
        try:
            result = subprocess.run(
                ["ip", "link", "show"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            # First try to find our specific interface
            if self.TARGET_INTERFACE in result.stdout:
                return self.TARGET_INTERFACE
            
            # Otherwise look for any interface matching GoPro USB pattern
            for line in result.stdout.split('\n'):
                match = re.search(self.USB_INTERFACE_PATTERN, line)
                if match:
                    return match.group(0)
            
            return None
            
        except Exception as e:
            print(f"⚠ Error finding USB interface: {e}")
            return None
    
    def get_interface_subnet(self, interface: str) -> Optional[ipaddress.IPv4Network]:
        """
        Get the IPv4 subnet for a network interface.
        
        Args:
            interface: Network interface name
            
        Returns:
            IPv4Network object or None
        """
        try:
            result = subprocess.run(
                ["ip", "addr", "show", interface],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            # Parse IP address with CIDR notation (e.g., 172.29.170.55/24)
            match = re.search(r'inet\s+(\d+\.\d+\.\d+\.\d+)/(\d+)', result.stdout)
            if match:
                ip_str = match.group(1)
                prefix_len = match.group(2)
                interface_ip = ipaddress.IPv4Interface(f"{ip_str}/{prefix_len}")
                return interface_ip.network
            
            return None
            
        except Exception as e:
            print(f"⚠ Error getting subnet for {interface}: {e}")
            return None
    
    def detect_camera_ip(self) -> Optional[str]:
        """
        Detect GoPro camera IP address from USB interface.
        
        Returns:
            Camera IP address or None if not found
        """
        print("🔍 Detecting GoPro camera IP address...")
        
        # If IP is already specified, validate it
        if self.ip:
            print(f"   Using specified IP: {self.ip}")
            if self._test_camera_connection(self.ip):
                return self.ip
            else:
                print(f"   ✗ Specified IP {self.ip} not responding")
                return None
        
        # Find USB interface
        if not self.interface:
            self.interface = self.find_usb_interface()
        
        if not self.interface:
            print("   ❌ No GoPro USB interface found")
            print("      Expected interface: enx04574796c048")
            print("      Run 'ip link' to check available interfaces")
            return None
        
        print(f"   ✓ Found USB interface: {self.interface}")
        
        # Get subnet from interface
        subnet = self.get_interface_subnet(self.interface)
        if not subnet:
            print(f"   ❌ Could not determine subnet for {self.interface}")
            return None
        
        print(f"   ✓ Interface subnet: {subnet}")
        
        # Generate candidate IPs based on common GoPro patterns
        candidate_ips = []
        base_ip = str(subnet.network_address)
        parts = base_ip.split('.')
        
        for last_octet in self.CAMERA_IP_CANDIDATES:
            candidate_ip = f"{parts[0]}.{parts[1]}.{parts[2]}.{last_octet}"
            candidate_ips.append(candidate_ip)
        
        print(f"   Scanning candidate IPs...")
        
        # Test each candidate
        for candidate_ip in candidate_ips:
            print(f"      {candidate_ip}...", end=" ")
            if self._test_camera_connection(candidate_ip):
                print("✓ FOUND!")
                self.ip = candidate_ip
                self.base_url = f"http://{candidate_ip}"
                return candidate_ip
            else:
                print("✗")
        
        print("\n   ❌ Could not detect camera on any candidate IP")
        return None
    
    def _test_camera_connection(self, ip: str) -> bool:
        """
        Test if GoPro camera is accessible at given IP.
        
        Args:
            ip: IP address to test
            
        Returns:
            True if camera responds, False otherwise
        """
        try:
            response = requests.get(
                f"http://{ip}/gopro/camera/state",
                timeout=2
            )
            return response.status_code == 200
        except:
            return False
    
    def connect_with_opengopro(self) -> bool:
        """
        Connect to GoPro using open-gopro library.
        
        Returns:
            True if successful, False otherwise
        """
        if not GOPRO_LIBRARY_AVAILABLE:
            print("⚠️  open-gopro library not available, using fallback method")
            return False
        
        try:
            print("📱 Connecting to GoPro with open-gopro library...")
            
            # Connect to wired GoPro
            self.gopro = WiredGoPro(target=self.ip)
            self.gopro.open()
            
            # Get camera info
            print(f"   ✓ Connected to: {self.gopro.identifier}")
            print(f"   Firmware: {self.gopro.version}")
            
            return True
            
        except Exception as e:
            print(f"   ⚠ Connection failed: {e}")
            return False
    
    def start_webcam_opengopro(self) -> bool:
        """
        Start webcam mode using open-gopro library.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.gopro:
            return False
        
        try:
            print("📹 Starting wired USB webcam mode...")
            
            # Enable wired control
            self.gopro.http_command.wired_usb_control(control=Params.Toggle.ENABLE)
            print("   ✓ Wired USB control enabled")
            time.sleep(0.5)
            
            # Start webcam
            self.gopro.http_command.webcam_start()
            print("   ✓ Webcam mode started")
            
            # Get webcam status
            status = self.gopro.http_command.webcam_status()
            print(f"   Status: {status}")
            
            return True
            
        except Exception as e:
            print(f"   ⚠ Failed to start webcam: {e}")
            return False
    
    def start_webcam_http(self) -> bool:
        """
        Start webcam mode using direct HTTP API (fallback).
        
        Returns:
            True if successful, False otherwise
        """
        print("📹 Starting wired USB webcam mode (HTTP API)...")
        
        try:
            # Enable wired USB control
            response = requests.get(
                f"{self.base_url}/gopro/camera/control/wired_usb?p=1",
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                print("   ✓ Wired USB control enabled")
                time.sleep(0.5)
            else:
                print(f"   ⚠ Wired USB control status: {response.status_code}")
            
            # Start webcam/stream
            endpoints = [
                "/gopro/camera/stream/start",  # Works on GoPro Max 2
                "/gopro/webcam/start",
                "/gopro/webcam/preview",
            ]
            
            success = False
            for endpoint in endpoints:
                try:
                    print(f"   Trying {endpoint}...", end=" ")
                    response = requests.get(
                        f"{self.base_url}{endpoint}",
                        timeout=self.timeout
                    )
                    
                    if response.status_code == 200:
                        print(f"✓")
                        success = True
                        break
                    else:
                        print(f"✗ (HTTP {response.status_code})")
                except Exception as e:
                    print(f"✗ ({e})")
                    continue
            
            if success:
                print(f"   ✓ Stream started successfully")
            
            return success
            
        except Exception as e:
            print(f"   ⚠ Error: {e}")
            return False
    
    def start_webcam(self) -> bool:
        """
        Start webcam mode using best available method.
        
        Returns:
            True if successful, False otherwise
        """
        if GOPRO_LIBRARY_AVAILABLE and self.gopro:
            return self.start_webcam_opengopro()
        else:
            return self.start_webcam_http()
    
    def stop_webcam(self) -> bool:
        """Stop webcam mode."""
        print("⏹️  Stopping webcam mode...")
        
        try:
            if GOPRO_LIBRARY_AVAILABLE and self.gopro:
                self.gopro.http_command.webcam_stop()
                self.gopro.close()
            else:
                requests.get(f"{self.base_url}/gopro/webcam/stop", timeout=self.timeout)
            
            print("   ✓ Stopped")
            return True
            
        except Exception as e:
            print(f"   ⚠ Error: {e}")
            return False
    
    def keep_alive(self):
        """Send keep-alive signal to maintain connection."""
        try:
            if GOPRO_LIBRARY_AVAILABLE and self.gopro:
                # open-gopro handles keep-alive automatically
                pass
            else:
                requests.get(f"{self.base_url}/gopro/camera/keep_alive", timeout=2)
        except:
            pass
    
    def get_stream_url(self) -> str:
        """
        Get the UDP stream URL.
        
        Returns:
            UDP URL string
        """
        # GoPro streams to the camera IP, not localhost
        return f"udp://{self.ip}:{self.UDP_PORT}"


def check_interface_status(interface: str = "enx04574796c048"):
    """Check if the target interface is up and has an IP."""
    print(f"\n🔌 Checking interface status: {interface}")
    try:
        result = subprocess.run(
            ["ip", "addr", "show", interface],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode != 0:
            print(f"   ❌ Interface {interface} not found")
            print("      Run 'ip link' to see available interfaces")
            return False
        
        # Check if interface is UP
        if "state UP" in result.stdout:
            print("   ✓ Interface is UP")
        else:
            print("   ⚠ Interface may be DOWN")
        
        # Check for IPv4 address
        if "inet " in result.stdout:
            match = re.search(r'inet\s+(\d+\.\d+\.\d+\.\d+)/(\d+)', result.stdout)
            if match:
                print(f"   ✓ IP Address: {match.group(1)}/{match.group(2)}")
        else:
            print("   ❌ No IPv4 address assigned")
            return False
        
        return True
        
    except Exception as e:
        print(f"   ⚠ Error checking interface: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Connect to GoPro Max 2 via USB and start webcam mode",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-detect camera on interface enx04574796c048
  python connect_gopro.py
  
  # Specify different interface
  python connect_gopro.py --interface enx1234567890ab
  
  # Specify IP address directly
  python connect_gopro.py --ip 172.29.170.51
  
  # Stop webcam mode
  python connect_gopro.py --stop
  
  # Keep connection alive
  python connect_gopro.py --keep-alive
        """
    )
    
    parser.add_argument(
        "--interface",
        type=str,
        help="USB network interface name (default: enx04574796c048)"
    )
    
    parser.add_argument(
        "--ip",
        type=str,
        help="Specific GoPro IP address (auto-detect if not specified)"
    )
    
    parser.add_argument(
        "--timeout",
        type=int,
        default=5,
        help="Connection timeout in seconds (default: 5)"
    )
    
    parser.add_argument(
        "--stop",
        action="store_true",
        help="Stop webcam mode and exit"
    )
    
    parser.add_argument(
        "--keep-alive",
        action="store_true",
        help="Keep connection alive after starting stream"
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("  GoPro Max 2 USB Webcam Controller")
    print("  Using open-gopro library" if GOPRO_LIBRARY_AVAILABLE else "  Using HTTP API (fallback)")
    print("=" * 60)
    
    # Check interface status
    target_interface = args.interface or "enx04574796c048"
    if not check_interface_status(target_interface):
        print("\n💡 Interface troubleshooting:")
        print("   1. Make sure GoPro is connected via USB")
        print("   2. Check if interface appears: ip link")
        print("   3. Bring interface up: sudo ip link set <interface> up")
        sys.exit(1)
    
    # Initialize controller
    controller = GoProUSBController(
        interface=args.interface,
        ip=args.ip,
        timeout=args.timeout
    )
    
    # Detect camera IP
    camera_ip = controller.detect_camera_ip()
    if not camera_ip:
        print("\n❌ Failed to detect GoPro camera")
        print("\n💡 Troubleshooting tips:")
        print("   1. Make sure GoPro is powered ON and stays on")
        print("   2. If it powers off after 'USB Connected', manually enable USB mode:")
        print("      - Swipe down from top")
        print("      - Tap Preferences > Connections > Wired Connections")
        print("      - Enable 'GoPro Connect'")
        print("   3. Try specifying IP manually: python connect_gopro.py --ip 172.29.170.51")
        print("   4. Check interface: ip addr show enx04574796c048")
        sys.exit(1)
    
    print(f"\n✅ GoPro camera detected at {camera_ip}")
    
    # Stop mode if requested
    if args.stop:
        controller.stop_webcam()
        sys.exit(0)
    
    # Connect using open-gopro if available
    if GOPRO_LIBRARY_AVAILABLE:
        if not controller.connect_with_opengopro():
            print("   Falling back to HTTP API...")
    
    # Start webcam mode
    success = controller.start_webcam()
    
    if not success:
        print("\n⚠️  Webcam mode could not be started")
        print("\n💡 Possible solutions:")
        print("   1. Manually enable webcam mode on camera:")
        print("      - Swipe down > Preferences > Connections")
        print("      - Enable 'USB' or 'Wired Webcam' mode")
        print("   2. Update GoPro firmware to latest version")
        print("   3. Try rebooting the camera")
        sys.exit(1)
    
    # Get and display stream URL
    stream_url = controller.get_stream_url()
    
    print("\n" + "=" * 60)
    print("  ✅ Webcam Mode Active!")
    print("=" * 60)
    print(f"\n📺 UDP Stream URL: {stream_url}")
    print(f"   Camera IP: {controller.ip}")
    print("\n🎬 View the stream with FFmpeg (GPU accelerated):")
    print(f"\n   ffmpeg -hwaccel cuda -fflags nobuffer -flags low_delay \\")
    print(f"     -i {stream_url} \\")
    print("     -vf 'v360=input=dfisheye:output=equirect:ih_fov=190:iv_fov=190' \\")
    print("     -f sdl 'GoPro Max 2 Stream'")
    print("\n   Or simply run: ./view_stream.sh")
    
    # Keep alive loop
    if args.keep_alive:
        print("\n🔄 Keeping connection alive (press Ctrl+C to stop)...")
        try:
            while True:
                time.sleep(5)
                controller.keep_alive()
                print("   ♥ heartbeat", end="\r")
        except KeyboardInterrupt:
            print("\n\n⏹️  Stopping...")
            controller.stop_webcam()
    
    print("\n✅ Done!")


if __name__ == "__main__":
    main()
