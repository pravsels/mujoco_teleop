#!/usr/bin/env python3
# find_port.py

import time
from serial.tools import list_ports

def list_serial_ports():
    """Return a set of all serial port device names."""
    return {port.device for port in list_ports.comports()}

def find_gello_port():
    print("This script will help you identify the correct USB port for your GELLO.")
    
    # Get current ports
    initial_ports = list_serial_ports()
    
    print(f"\nCurrently detected ports: {sorted(list(initial_ports)) if initial_ports else 'None'}")
    input("1) UNPLUG your GELLO USB cable, then press Enter → ")
    time.sleep(0.5)
    
    ports_after_unplug = list_serial_ports()
    removed = list(initial_ports - ports_after_unplug)
    
    if len(removed) == 1:
        port = removed[0]
        print(f"\n✅ SUCCESS! Detected GELLO on port: {port}")
        print(f"You can now use this port in your commands (e.g., --port {port})")
        input("\n2) RE-PLUG the USB cable now, then press Enter to finish → ")
        return port
    elif len(removed) == 0:
        print("\n❌ ERROR: No port change detected. Did you unplug the right cable?")
        return None
    else:
        print(f"\n❌ ERROR: Multiple ports changed ({removed}). Please try again with only one device.")
        return None

if __name__ == "__main__":
    try:
        find_gello_port()
    except KeyboardInterrupt:
        print("\nOperation cancelled.")
