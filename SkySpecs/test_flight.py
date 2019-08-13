import setup_path
import cv2
import math
import os
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
        self.snapshot_index = 0
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
        print("Arming drone")
        start = self.client.getMultirotorState().kinematics_estimated.position
        landed = self.client.getMultirotorState().landed_state
        if not self.takeoff and landed == airsim.LandedState.Landed:
            self.takeoff = True
            print("taking off...")
            self.client.takeoffAsync().join()
            start = self.client.getMultirotorState().kinematics_estimated.position
            z = -self.altitude + self.home.z_val
        else:
            print("already flying so we will orbit at current altitude {}".format(start.z_val))
            z = start.z_val # use current altitude then

    def orbit(self,
              radius=2,
              altitude=10,
              speed=2,
              iterations=1,
              center=[1,0],
              snapshots=None):

        # TODO: change start and z to correct value
        start = self.client.getMultirotorState().kinematics_estimated.position
        z = start.z_val  # use current altitude then

        center = self.client.getMultirotorState().kinematics_estimated.position
        ###

        print("climbing to position: {},{},{}".format(start.x_val, start.y_val, z))
        self.client.moveToPositionAsync(start.x_val, start.y_val, z, speed).join()
        self.z = z

        print("ramping up to speed...")
        count = 0
        self.start_angle = None
        self.next_snapshot = None

        # ramp up time
        ramptime = radius / 10
        self.start_time = time.time()

        snapshot_index = 0

        while count < iterations:
            if snapshots > 0 and not (snapshot_index < snapshots):
                break
            # ramp up to full speed in smooth increments so we don't start too aggressively.
            now = time.time()
            speed = speed
            diff = now - self.start_time
            if diff < ramptime:
                speed = speed * diff / ramptime
            elif ramptime > 0:
                print("reached full speed...")
                ramptime = 0

            lookahead_angle = speed / radius

            # compute current angle
            pos = self.client.getMultirotorState().kinematics_estimated.position
            dx = pos.x_val - center.x_val
            dy = pos.y_val - center.y_val
            actual_radius = math.sqrt((dx * dx) + (dy * dy))
            angle_to_center = math.atan2(dy, dx)

            camera_heading = (angle_to_center - math.pi) * 180 / math.pi

            # compute lookahead
            lookahead_x = self.center.x_val + self.radius * math.cos(angle_to_center + lookahead_angle)
            lookahead_y = self.center.y_val + self.radius * math.sin(angle_to_center + lookahead_angle)

            vx = lookahead_x - pos.x_val
            vy = lookahead_y - pos.y_val

            if self.track_orbits(angle_to_center * 180 / math.pi):
                count += 1
                print("completed {} orbits".format(count))

            self.camera_heading = camera_heading
            self.client.moveByVelocityZAsync(vx, vy, z, 1, airsim.DrivetrainType.MaxDegreeOfFreedom,
                                             airsim.YawMode(False, camera_heading))

        self.client.moveToPositionAsync(start.x_val, start.y_val, z, 2).join()

        if self.takeoff:
            # if we did the takeoff then also do the landing.
            if z < self.home.z_val:
                print("descending")
                self.client.moveToPositionAsync(start.x_val, start.y_val, self.home.z_val - 5, 2).join()

            print("landing...")
            self.client.landAsync().join()

            print("disarming.")
            self.client.armDisarm(False)

    def take_snapshot(self):
        # first hold our current position so drone doesn't try and keep flying while we take the picture.
        pos = self.client.getMultirotorState().kinematics_estimated.position

        self.client.moveToPositionAsync(pos.x_val, pos.y_val, self.z, 0.5, 10, airsim.DrivetrainType.MaxDegreeOfFreedom,
            airsim.YawMode(False, self.camera_heading)).join()

        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.DepthVis),
            airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True),  # depth in perspective projection
            airsim.ImageRequest("1", airsim.ImageType.Scene),  # scene vision image in png format
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, True),
            airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)
        ])

        for idx, response in enumerate(responses):
            tmp_dir = 'snapshots'

            try:
                os.makedirs(tmp_dir)
            except OSError:
                if not os.path.isdir(tmp_dir):
                    raise

            filename = os.path.join(tmp_dir, str(idx))

            if response.pixels_as_float:
                print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
                airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
            elif response.compress:  # png format
                print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
            else:  # uncompressed array
                print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)  # get numpy array
                img_rgb = img1d.reshape(response.height, response.width,
                                        3)  # reshape array to 4 channel image array H X W X 3
                cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb)  # write to png

        self.snapshot_index += 1

    def track_orbits(self, angle):
        # tracking # of completed orbits is surprisingly tricky to get right in order to handle random wobbles
        # about the starting point.  So we watch for complete 1/2 orbits to avoid that problem.
        if angle < 0:
            angle += 360

        if self.start_angle is None:
            self.start_angle = angle
            if self.snapshot_delta:
                self.next_snapshot = angle + self.snapshot_delta
            self.previous_angle = angle
            self.shifted = False
            self.previous_sign = None
            self.previous_diff = None
            self.quarter = False
            return False

        # now we just have to watch for a smooth crossing from negative diff to positive diff
        if self.previous_angle is None:
            self.previous_angle = angle
            return False

        # ignore the click over from 360 back to 0
        if self.previous_angle > 350 and angle < 10:
            if self.snapshot_delta and self.next_snapshot >= 360:
                self.next_snapshot -= 360
            return False

        diff = self.previous_angle - angle
        crossing = False
        self.previous_angle = angle
        self.snapshot_delta = None

        if self.snapshot_delta and angle > self.next_snapshot:
            print("Taking snapshot at angle {}".format(angle))
            self.take_snapshot()
            self.next_snapshot += self.snapshot_delta

        diff = abs(angle - self.start_angle)
        if diff > 45:
            self.quarter = True

        if self.quarter and self.previous_diff is not None and diff != self.previous_diff:
            # watch direction this diff is moving if it switches from shrinking to growing
            # then we passed the starting point.
            direction = self._sign(self.previous_diff - diff)
            if self.previous_sign is None:
                self.previous_sign = direction
            elif self.previous_sign > 0 and direction < 0:
                if diff < 45:
                    self.quarter = False
                    if self.snapshots <= self.snapshot_index + 1:
                        crossing = True
            self.previous_sign = direction
        self.previous_diff = diff

        return crossing

    def _sign(self, s):
        if s < 0:
            return -1
        return 1
