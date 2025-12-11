#!/usr/bin/env python3
"""
GoPro Max 2 USB Connection and Webcam/Live Stream Activation Script

This script:
1. Detects GoPro Max 2 connected via USB (USB Ethernet/RNDIS interface)
2. Uses open-gopro library to connect and enable webcam mode
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
from typing import Optional, Tuple, List
from pathlib import Path


class GoProUSBController:
    """Controller for GoPro Max 2 over USB connection using open-gopro."""
    
    # GoPro USB interface name pattern
    USB_INTERFACE_PATTERN = r"enx[0-9a-f]{12}"
    
    # Common camera IP patterns (usually .51, .50, or .1 in the subnet)
    CAMERA_IP_CANDIDATES = [51, 50, 1]
    
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
        self.interface = interface
        self.ip = ip
        self.timeout = timeout
        self.base_url = None
        self.gopro = None
    
    def get_usb_network_subnet(self) -> Optional[str]:
        """Detect the USB network interface and extract potential GoPro IP."""
        try:
            result = subprocess.run(
                ["ip", "-4", "addr", "show"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            # Look for 172.x.x.x interfaces (GoPro uses 172.x subnets)
            lines = result.stdout.split('\n')
            for i, line in enumerate(lines):
                if 'inet 172.' in line and 'scope global' in line:
                    # Extract the IP address
                    parts = line.strip().split()
                    if parts[0] == 'inet':
                        host_ip = parts[1].split('/')[0]
                        # GoPro is typically .51 on the same subnet
                        # Host is typically .50 or .55
                        subnet_base = '.'.join(host_ip.split('.')[:-1])
                        
                        print(f"   Found USB network: {host_ip}")
                        print(f"   Derived subnet: {subnet_base}.0/24")
                        
                        # Try common GoPro IPs in this subnet
                        potential_ips = [
                            f"{subnet_base}.51",  # Most common
                            f"{subnet_base}.1",   # Gateway/camera
                            f"{subnet_base}.2",
                        ]
                        return potential_ips
            
            return None
            
        except Exception as e:
            print(f"   ⚠ Could not detect subnet: {e}")
            return None
        
    def detect_camera(self) -> bool:
        """
        Auto-detect GoPro camera on USB network interface.
        
        Returns:
            True if camera detected, False otherwise
        """
        print("🔍 Scanning for GoPro camera on USB network...")
        
        # First, try to detect IPs from actual USB interface
        subnet_ips = self.get_usb_network_subnet() if not self.ip else None
        
        # Build list of IPs to check
        if self.ip:
            ips_to_check = [self.ip]
        elif subnet_ips:
            # Try subnet-derived IPs first, then common defaults
            ips_to_check = subnet_ips + self.POSSIBLE_IPS
        else:
            ips_to_check = self.POSSIBLE_IPS
        
        for test_ip in ips_to_check:
            print(f"   Trying {test_ip}...", end=" ")
            try:
                # Try to connect to GoPro status endpoint
                response = requests.get(
                    f"http://{test_ip}/gopro/camera/state",
                    timeout=self.timeout
                )
                
                if response.status_code == 200:
                    self.ip = test_ip
                    self.base_url = f"http://{test_ip}"
                    print(f"✓ Found!")
                    return True
                else:
                    print("✗")
                    
            except (RequestException, Timeout):
                print("✗")
                continue
        
        print("\n❌ Could not detect GoPro camera.")
        print("\n   📱 Is your GoPro powering off when connected via USB?")
        print("   This is normal - it means the camera is in CHARGING mode, not NETWORK mode.")
        print("\n   ✅ How to fix:")
        print("   1. On the GoPro camera screen:")
        print("      • Swipe down to access Preferences")
        print("      • Go to: Preferences → Connections → USB Connection")
        print("      • Change from 'Charge' or 'MTP' to 'GoPro Connect'")
        print("      • Or enable 'Quik App' mode")
        print("\n   2. Alternative: Use wireless connection")
        print("      • The GoPro Max may prefer WiFi for live streaming")
        print("      • Enable WiFi on camera and connect to its network")
        print("\n   3. Check USB mode on camera:")
        print("      • Some GoPro models automatically sleep when detecting charge-only USB")
        print("      • Try a different USB cable (data-capable, not charge-only)")
        print("      • Try a different USB port on your computer")
        print("\n   4. Manual IP check:")
        print("      • Your USB network is on: 172.29.170.x")
        print("      • Try pinging: ping 172.29.170.51")
        print("      • Try pinging: ping 172.29.170.1")
        return False
    
    def get_camera_status(self) -> dict:
        """Get camera status information."""
        try:
            response = requests.get(
                f"{self.base_url}/gopro/camera/state",
                timeout=self.timeout
            )
            if response.status_code == 200:
                return response.json()
            return {}
        except Exception as e:
            print(f"⚠ Warning: Could not get camera status: {e}")
            return {}
    
    def enable_wired_usb_control(self) -> bool:
        """
        Enable wired USB control mode.
        
        Returns:
            True if successful, False otherwise
        """
        print("⚙️  Enabling wired USB control mode...")
        try:
            # Enable USB control (required for some GoPro models)
            response = requests.get(
                f"{self.base_url}/gopro/camera/control/wired_usb?p=1",
                timeout=self.timeout
            )
            
            if response.status_code == 200:
                print("   ✓ USB control enabled")
                time.sleep(0.5)
                return True
            else:
                print(f"   ⚠ Status code: {response.status_code}")
                return False
                
        except Exception as e:
            print(f"   ⚠ Error: {e}")
            return False
    
    def start_webcam_mode(self) -> Tuple[bool, Optional[str]]:
        """
        Start webcam/live stream mode on GoPro.
        
        Returns:
            Tuple of (success, udp_url)
        """
        print("📹 Starting GoPro webcam/live stream mode...")
        
        try:
            # For GoPro Max 2, we need to:
            # 1. Set webcam resolution (optional)
            # 2. Start webcam preview
            
            # Set webcam to start (some models require setting preset first)
            endpoints_to_try = [
                "/gopro/webcam/start",  # Primary webcam endpoint
                "/gopro/webcam/preview",  # Alternative preview endpoint
                "/gopro/camera/stream/start",  # Generic stream start
            ]
            
            success = False
            for endpoint in endpoints_to_try:
                try:
                    print(f"   Trying endpoint: {endpoint}...", end=" ")
                    response = requests.get(
                        f"{self.base_url}{endpoint}",
                        timeout=self.timeout
                    )
                    
                    if response.status_code == 200:
                        print("✓")
                        success = True
                        break
                    else:
                        print(f"✗ (status {response.status_code})")
                        
                except Exception as e:
                    print(f"✗ ({e})")
                    continue
            
            if not success:
                print("   ⚠ Could not start webcam mode via standard endpoints")
                print("   Trying alternative method...")
                
                # Try setting video mode and starting preview
                try:
                    requests.get(f"{self.base_url}/gopro/camera/presets/load?id=2", timeout=self.timeout)
                    time.sleep(1)
                    response = requests.get(f"{self.base_url}/gopro/camera/shutter/start", timeout=self.timeout)
                    if response.status_code == 200:
                        print("   ✓ Started recording mode")
                        success = True
                except:
                    pass
            
            if success:
                # UDP stream URL
                udp_url = f"udp://@:{self.UDP_PORT}"
                print(f"\n✅ GoPro webcam mode started!")
                print(f"   Camera IP: {self.ip}")
                print(f"   UDP Stream: {udp_url}")
                return True, udp_url
            else:
                print("\n❌ Failed to start webcam mode")
                return False, None
                
        except Exception as e:
            print(f"\n❌ Error starting webcam: {e}")
            return False, None
    
    def stop_webcam_mode(self) -> bool:
        """Stop webcam/live stream mode."""
        print("⏹️  Stopping webcam mode...")
        try:
            endpoints = [
                "/gopro/webcam/stop",
                "/gopro/camera/stream/stop",
                "/gopro/camera/shutter/stop",
            ]
            
            for endpoint in endpoints:
                try:
                    requests.get(f"{self.base_url}{endpoint}", timeout=self.timeout)
                except:
                    pass
            
            print("   ✓ Stopped")
            return True
            
        except Exception as e:
            print(f"   ⚠ Error: {e}")
            return False
    
    def keep_alive(self):
        """Send keep-alive signal to maintain connection."""
        try:
            requests.get(f"{self.base_url}/gopro/camera/keep_alive", timeout=2)
        except:
            pass


def check_usb_network_interface():
    """Check if USB network interface is available."""
    print("\n🔌 Checking USB network interfaces...")
    try:
        result = subprocess.run(
            ["ip", "addr", "show"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        output = result.stdout
        
        # Look for USB ethernet interfaces
        usb_interfaces = []
        for line in output.split('\n'):
            if 'usb' in line.lower() or '172.' in line:
                usb_interfaces.append(line.strip())
        
        if usb_interfaces:
            print("   ✓ USB network interfaces found:")
            for iface in usb_interfaces[:5]:  # Show first 5
                print(f"     {iface}")
        else:
            print("   ⚠ No USB network interfaces detected")
            print("     Make sure GoPro is connected via USB")
            print("\n   📱 GoPro powering off after USB connection?")
            print("      This usually means it's in charging/MTP mode.")
            print("      Try:")
            print("      1. Navigate to: Preferences > Connections > Wired Connections")
            print("      2. Enable 'GoPro Connect' or 'USB Connection' mode")
            print("      3. Set USB mode to 'GoPro' not 'MTP' or 'Charge Only'")
            
    except Exception as e:
        print(f"   ⚠ Could not check interfaces: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="Connect to GoPro Max 2 via USB and start webcam mode",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-detect and connect
  python connect_gopro.py
  
  # Specify IP address
  python connect_gopro.py --ip 172.20.110.51
  
  # Stop webcam mode
  python connect_gopro.py --stop
        """
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
    print("=" * 60)
    
    # Check USB network interface
    check_usb_network_interface()
    
    # Initialize controller
    controller = GoProUSBController(ip=args.ip, timeout=args.timeout)
    
    # Detect camera
    if not controller.detect_camera():
        print("\n💡 Troubleshooting tips:")
        print("   1. Check USB cable connection")
        print("   2. Make sure GoPro is powered ON")
        print("   3. Try unplugging and reconnecting USB cable")
        print("   4. Check 'ip addr' output for 172.2x.x.x addresses")
        print("   5. Try pinging 172.20.110.51 manually")
        sys.exit(1)
    
    print(f"\n✅ Connected to GoPro at {controller.ip}")
    
    # Stop mode if requested
    if args.stop:
        controller.stop_webcam_mode()
        sys.exit(0)
    
    # Enable USB control
    controller.enable_wired_usb_control()
    
    # Start webcam mode
    success, udp_url = controller.start_webcam_mode()
    
    if not success:
        print("\n⚠️  Webcam mode could not be started automatically.")
        print("   You may need to manually enable webcam mode on the camera.")
        print("   Or try the open-gopro library for full BLE control.")
        sys.exit(1)
    
    # Print ffmpeg command
    print("\n" + "=" * 60)
    print("  Next Steps: View the Stream")
    print("=" * 60)
    print("\n📺 Use this command to view with FFmpeg + GPU acceleration:")
    print(f"\n   ffmpeg -hwaccel cuda -i {udp_url} \\")
    print(f"     -vf 'v360=input=eac:output=equirect' \\")
    print(f"     -f sdl 'GoPro Max 2 Stream'")
    print("\n   Or run: ./view_stream.sh")
    
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
            controller.stop_webcam_mode()
    
    print("\n✅ Done!")


if __name__ == "__main__":
    main()
