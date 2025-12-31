#!/usr/bin/env python3
"""
Backward-compatible entry point.

`setup.py` exposes a console script `ultrasonic_to_laserscan_node`.
Historically this node name was used for the Range->LaserScan converter.

Current implementation lives in `range_to_laserscan.py`, so we just re-export `main`.
"""

from .range_to_laserscan import main


if __name__ == '__main__':
    main()


