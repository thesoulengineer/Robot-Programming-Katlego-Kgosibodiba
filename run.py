#!/usr/bin/env python3
"""
Run script for PID Waypoint Navigator.
This ensures proper Python path setup.
"""

import sys
import os

# Add src directory to path
src_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src')
sys.path.insert(0, src_path)

# Run main
from main import main

if __name__ == "__main__":
    main()
