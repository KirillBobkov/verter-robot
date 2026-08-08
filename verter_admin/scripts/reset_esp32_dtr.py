#!/usr/bin/env python3
"""
Reset ESP32 via DTR signal before micro-ROS agent start.

Usage: python3 reset_esp32_dtr.py /dev/ttyACM0
"""

import sys
import time
import serial

def reset_esp32_dtr(port: str):
    """Toggle DTR line to reset ESP32."""
    try:
        with serial.Serial(port, baudrate=921600, timeout=1) as ser:
            # Ensure DTR is initially low
            ser.dtr = False
            time.sleep(0.1)

            # Toggle DTR high -> low to trigger reset
            ser.dtr = True
            time.sleep(0.1)
            ser.dtr = False
            time.sleep(0.5)  # Wait for ESP32 to boot

            print(f"Reset ESP32 on {port} via DTR")
            return True
    except Exception as e:
        print(f"Failed to reset ESP32 on {port}: {e}", file=sys.stderr)
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: reset_esp32_dtr.py <port>")
        sys.exit(1)

    success = reset_esp32_dtr(sys.argv[1])
    sys.exit(0 if success else 1)
