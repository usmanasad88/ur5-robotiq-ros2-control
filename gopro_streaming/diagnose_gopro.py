#!/usr/bin/env python3
"""
GoPro USB Connection Diagnostics

This script helps diagnose connection issues with GoPro cameras over USB.
"""

import subprocess
import sys
import socket
import requests
from requests.exceptions import Timeout, RequestException


def print_section(title):
    """Print a section header."""
    print("\n" + "=" * 60)
    print(f"  {title}")
    print("=" * 60)


def check_usb_interfaces():
    """Check for USB network interfaces."""
    print_section("USB Network Interfaces")
    
    try:
        result = subprocess.run(
            ["ip", "-4", "addr", "show"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        interfaces = []
        current_iface = None
        
        for line in result.stdout.split('\n'):
            if line and not line[0].isspace():
                # New interface
                parts = line.split(':')
                if len(parts) >= 2:
                    current_iface = {
                        'name': parts[1].strip(),
                        'ips': []
                    }
            elif 'inet ' in line and current_iface:
                # IP address line
                parts = line.strip().split()
                if parts[0] == 'inet':
                    ip_with_mask = parts[1]
                    ip = ip_with_mask.split('/')[0]
                    current_iface['ips'].append(ip)
                    
                    # Check if this looks like a GoPro network
                    if ip.startswith('172.'):
                        interfaces.append(current_iface)
                        current_iface = None
        
        if interfaces:
            print("✅ Found USB network interfaces:\n")
            for iface in interfaces:
                print(f"   Interface: {iface['name']}")
                for ip in iface['ips']:
                    print(f"   Host IP:   {ip}")
                    subnet = '.'.join(ip.split('.')[:-1])
                    print(f"   Subnet:    {subnet}.0/24")
                    print(f"   Likely GoPro IPs: {subnet}.51, {subnet}.1, {subnet}.2")
                print()
        else:
            print("❌ No USB network interfaces with 172.x.x.x addresses found")
            print("\n   Possible reasons:")
            print("   • GoPro is in charging mode (not network mode)")
            print("   • USB cable is charge-only (not data-capable)")
            print("   • GoPro USB network mode is not enabled in camera settings")
            
        return interfaces
        
    except Exception as e:
        print(f"❌ Error checking interfaces: {e}")
        return []


def scan_subnet(subnet_base):
    """Scan a subnet for potential GoPro IPs."""
    print_section(f"Scanning Subnet: {subnet_base}.0/24")
    
    test_ips = [
        f"{subnet_base}.51",  # Most common GoPro IP
        f"{subnet_base}.1",   # Gateway/first IP
        f"{subnet_base}.2",
        f"{subnet_base}.50",  # Alternative host
    ]
    
    found_devices = []
    
    for ip in test_ips:
        print(f"   Testing {ip}...", end=" ")
        sys.stdout.flush()
        
        # Try ping first
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", ip],
                capture_output=True,
                timeout=2
            )
            
            if result.returncode == 0:
                print("✓ PING OK", end=" ")
                
                # Try HTTP connection
                try:
                    response = requests.get(
                        f"http://{ip}/gopro/camera/state",
                        timeout=2
                    )
                    
                    if response.status_code == 200:
                        print("✓ GoPro API FOUND!")
                        found_devices.append({
                            'ip': ip,
                            'type': 'gopro',
                            'status': 'active'
                        })
                    else:
                        print(f"✓ HTTP {response.status_code}")
                        found_devices.append({
                            'ip': ip,
                            'type': 'unknown',
                            'status': response.status_code
                        })
                        
                except (Timeout, RequestException):
                    print("(No GoPro API)")
                    found_devices.append({
                        'ip': ip,
                        'type': 'device',
                        'status': 'no_api'
                    })
            else:
                print("✗")
                
        except subprocess.TimeoutExpired:
            print("✗ timeout")
        except Exception as e:
            print(f"✗ {e}")
    
    return found_devices


def test_gopro_endpoints(ip):
    """Test various GoPro API endpoints."""
    print_section(f"Testing GoPro API: {ip}")
    
    endpoints = [
        ("/gopro/camera/state", "Camera State"),
        ("/gopro/version", "Version Info"),
        ("/gopro/camera/keep_alive", "Keep Alive"),
        ("/gopro/webcam/status", "Webcam Status"),
        ("/gopro/camera/stream/start", "Stream Start"),
    ]
    
    for endpoint, description in endpoints:
        url = f"http://{ip}{endpoint}"
        print(f"   {description:20s} {endpoint:30s} ", end="")
        sys.stdout.flush()
        
        try:
            response = requests.get(url, timeout=3)
            if response.status_code == 200:
                print(f"✓ OK")
            else:
                print(f"✗ HTTP {response.status_code}")
        except Timeout:
            print("✗ Timeout")
        except RequestException as e:
            print(f"✗ {type(e).__name__}")


def check_gopro_mode():
    """Provide guidance on GoPro USB modes."""
    print_section("GoPro USB Mode Settings")
    
    print("""
📱 GoPro Max 2 USB Connection Modes:

The GoPro Max 2 has different USB modes:

1. CHARGING MODE (Default)
   • Camera shows "USB Connected" then powers off
   • This is what you're experiencing
   • Computer sees USB network, but camera doesn't respond to API
   
2. GoPro CONNECT MODE (Needed for streaming)
   • Camera stays on and responsive
   • Network API endpoints work
   • Can enable webcam/live streaming
   
✅ How to enable GoPro Connect mode:

   Option A: Via Camera Menu
   ─────────────────────────────────
   1. On GoPro: Swipe down from top
   2. Tap the tools/preferences icon
   3. Navigate to: Connections → Wired Connections
   4. Enable "GoPro Connect" or set USB mode to "GoPro"
   5. Connect USB cable
   
   Option B: Try Different Approach
   ─────────────────────────────────
   1. Enable WiFi on GoPro (may work better for Max 2)
   2. Use GoPro Quik app to enable remote control
   3. Some Max models prefer wireless for live features
   
   Option C: Check USB Cable
   ─────────────────────────────────
   1. Some USB cables are charge-only
   2. Try a different cable (one used for data transfer)
   3. Try a different USB port on your computer
   
⚠️  Note: GoPro Max 2 may have limited USB networking support
    compared to newer Hero models. WiFi might be more reliable
    for live streaming features.
""")


def main():
    print("=" * 60)
    print("  GoPro USB Connection Diagnostics")
    print("=" * 60)
    
    # Check interfaces
    interfaces = check_usb_interfaces()
    
    if not interfaces:
        check_gopro_mode()
        return
    
    # Scan each subnet found
    all_devices = []
    for iface in interfaces:
        for ip in iface['ips']:
            subnet = '.'.join(ip.split('.')[:-1])
            devices = scan_subnet(subnet)
            all_devices.extend(devices)
    
    # Test any found GoPro devices
    gopro_devices = [d for d in all_devices if d['type'] == 'gopro']
    
    if gopro_devices:
        print_section("Summary")
        print("\n✅ Found GoPro camera(s):\n")
        for device in gopro_devices:
            print(f"   {device['ip']} - {device['status']}")
            test_gopro_endpoints(device['ip'])
    else:
        print_section("Summary")
        print("\n❌ No GoPro cameras detected on USB network")
        
        if all_devices:
            print("\n   However, found these responding devices:")
            for device in all_devices:
                print(f"   {device['ip']} - {device['type']}")
        
        check_gopro_mode()
    
    print("\n" + "=" * 60)
    print("  Diagnosis Complete")
    print("=" * 60)


if __name__ == "__main__":
    main()
