import argparse
import os
import time
import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    args, _ = parser.parse_known_args()
    with serial.Serial(args.port, baudrate=921600, timeout=1) as ser:
        ser.dtr = False
        time.sleep(0.1)
        ser.dtr = True
        time.sleep(0.1)
        ser.dtr = False
        time.sleep(0.5)
    os.execvp(
        'ros2', ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', args.port, '-b', '921600']
    )
