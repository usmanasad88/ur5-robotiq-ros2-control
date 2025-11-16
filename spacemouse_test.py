#!/usr/bin/env python3
"""
SpaceMouse Pro Input Reader
Reads and prints inputs from a 3Dconnexion SpaceMouse Pro device.
"""

import time
import sys

try:
    import pyspacemouse
except ImportError:
    print("Error: pyspacemouse library not found!")
    print("Please install it using: conda activate ur5_python && pip install pyspacemouse")
    sys.exit(1)


def print_state(state):
    """Print the current state of the SpaceMouse in a readable format."""
    print("\n" + "="*60)
    print("SpaceMouse Pro State:")
    print("="*60)
    
    # Translation values (x, y, z)
    print(f"Translation:")
    print(f"  X: {state.x:8.3f}  (left/right)")
    print(f"  Y: {state.y:8.3f}  (forward/backward)")
    print(f"  Z: {state.z:8.3f}  (up/down)")
    
    # Rotation values (roll, pitch, yaw)
    print(f"\nRotation:")
    print(f"  Roll:  {state.roll:8.3f}  (rotate around X)")
    print(f"  Pitch: {state.pitch:8.3f}  (rotate around Y)")
    print(f"  Yaw:   {state.yaw:8.3f}  (rotate around Z)")
    
    # Button states
    print(f"\nButtons:")
    print(f"  Button states: {state.buttons}")
    
    # Time since last update
    print(f"\nTime: {state.t:.3f}s")
    print("="*60)


def main():
    """Main function to read and print SpaceMouse inputs."""
    print("Starting SpaceMouse Pro Input Reader...")
    print("Press Ctrl+C to exit\n")
    
    # Try to open the SpaceMouse device
    # Using set_nonblocking_loop=False and callbacks=0 to prevent interfering with cursor
    success = pyspacemouse.open(set_nonblocking_loop=False, DeviceNumber=0)
    
    if not success:
        print("Error: Could not open SpaceMouse device!")
        print("\nTroubleshooting:")
        print("1. Make sure the SpaceMouse is connected via USB")
        print("2. Check if the device is detected: lsusb | grep 3Dconnexion")
        print("3. You might need udev rules for non-root access")
        print("   Create /etc/udev/rules.d/90-spacemouse.rules with:")
        print('   SUBSYSTEM=="usb", ATTRS{idVendor}=="256f", MODE="0666"')
        print("4. Reload udev rules: sudo udevadm control --reload-rules")
        print("5. If cursor is still moving, try stopping the 3Dconnexion driver:")
        print("   sudo systemctl stop 3dxsrv")
        sys.exit(1)
    
    print("SpaceMouse device opened successfully!")
    print("Note: If your cursor still moves, you may need to stop the 3Dconnexion system driver.")
    print("Move the SpaceMouse or press buttons to see output...\n")
    
    try:
        last_state = None
        while True:
            # Read the current state
            state = pyspacemouse.read()
            
            if state is not None:
                # Print state when any value changes
                if last_state is None or (
                    state.x != last_state.x or 
                    state.y != last_state.y or 
                    state.z != last_state.z or
                    state.roll != last_state.roll or
                    state.pitch != last_state.pitch or
                    state.yaw != last_state.yaw or
                    state.buttons != last_state.buttons
                ):
                    print_state(state)
                    last_state = state
            
            # Small delay to prevent CPU hogging
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        # Close the device
        pyspacemouse.close()
        print("SpaceMouse device closed.")


if __name__ == "__main__":
    main()
