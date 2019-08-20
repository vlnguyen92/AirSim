import setup_path
import cv2
import math
import os
import airsim
import time
import numpy as np

import argparse
from .orbit_navigator import OrbitNavigator

if __name__ == '__main__':
    navigator = OrbitNavigator()

    while True:
        pos = (-10, 10, -10)
        import pdb
        pdb.set_trace()
        navigator.move_to(position=pos, speed=5)
        navigator.take_snapshots()
