import setup_path
import airsim
import time
import numpy as np


class TestFlight:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

        self.home = self.client.getMultirotorState().kinematics_estimated.position
        self.takeoff = False
        self._check_stable()

    def _check_stable(self):
        start = time.time()
        count = 0

        while count < 100:
            pos = self.home
            if abs(pos.z_val - self.home.z_val) > 1:
                count = 0
                self.home = pos
                if time.time() - start > 10:
                    print("Drone position is drifting, we are waiting for it to settle down...")
                    start = time
            else:
                count += 1

    def takeoff(self):
        pass
