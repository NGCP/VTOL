'''
First thoughts on SLAM cv
'''
import ssl
import os
from time import time, sleep
from math import cos, sin, radians
# from threading import Thread
import json
import urllib.request
import numpy as np
import cv2
from simple_pid import PID
from dronekit import connect, VehicleMode
from shapely.geometry import Polygon, Point
from dotenv import load_dotenv
from vtol import VTOL

load_dotenv()

def setup_vehicle(configs):
    '''sets up gps denied vehicle'''
    con_str = "tcp:127.0.0.1:5763"
    veh = connect(con_str, wait_ready=True, vehicle_class=GpsDeniedVtol)
    veh.configs = configs
    # veh.setup_coms()
    return veh


class GpsDeniedVtol(VTOL): #pylint: disable=too-many-instance-attributes
    '''encapsualtes variables and tools used for GPS denied navigation'''

    # Quick configs
    total_tilt = 25
    target_position = (0, 0)
    reached_dest = False

    locating = True

    #CV sim
    frame_size = (600, 280)
    misses = 0
    position_guess = (0, 0)

    def get_image(self):
        '''gets image at vehicle's current lat lng position'''
        lat = self.location.global_relative_frame.lat
        lng = self.location.global_relative_frame.lon
        #Gets image of current lat long coorindates from google static maps
        url = 'https://maps.googleapis.com/maps/api/staticmap?center=' + \
            '{},{}&zoom=20&size=600x300&maptype=satellite&key={}' \
            .format(lat, lng, os.getenv('google_maps_api_key'))
        context = ssl._create_unverified_context()              # pylint: disable=protected-access
        resp = urllib.request.urlopen(url, context=context)
        image = np.asarray(bytearray(resp.read()), dtype="uint8")
        decoded = cv2.imdecode(image, cv2.IMREAD_GRAYSCALE)
        cropped = decoded[:280, :]
        return cropped


    def set_position(self):
        '''Uses PID and position guesses to set the quad's position'''
        assert self.target_position is not None

        #set attitude to 0
        self.set_attitude(yaw_angle=0, duration=5)
        while True:
            print("Targeting", self.target_position)
            targ = []
            targ = self.target_position
            xpid = PID(self.configs.pos.P, self.configs.pos.I, self.configs.pos.D, setpoint=targ[0])
            xpid.sample_time = 0.33
            ypid = PID(self.configs.pos.P, self.configs.pos.I, self.configs.pos.D, setpoint=targ[1])
            ypid.sample_time = 0.33
            till_stop = 15
            while True:
                if targ != self.target_position:
                    #if dest changes rest PID algs
                    break
                if till_stop == 0:
                    self.reached_dest = True
                #Get control from each PID
                x_control = xpid(self.position_guess[0])
                y_control = ypid(self.position_guess[1])
                remaining_x = targ[0] - self.position_guess[0]
                remaining_y = targ[1] - self.position_guess[1]
                remaining_dst_sq = (remaining_x ** 2 + remaining_y ** 2) ** .5
                if remaining_dst_sq < 20:
                    till_stop -= 1
                else:
                    till_stop = 15
                #Correct for tilts higher than total_tilt
                total_control_sq = (y_control ** 2 + x_control ** 2) ** .5
                if total_control_sq > self.total_tilt:
                    x_control = self.total_tilt * remaining_x / remaining_dst_sq
                    y_control = self.total_tilt * remaining_y / remaining_dst_sq

                self.set_attitude(roll_angle=x_control, \
                            pitch_angle=y_control, \
                            duration=.33, \
                            yaw_angle=0)


    def cv_stitch(self):
        '''Gathers and organizes feature points. Reports position guesses'''
        point_img = np.zeros([5000, 5000])

        min_match_count = 10
        # Initiate SIFT detector

        orb = cv2.ORB_create(nfeatures=1000, scoreType=cv2.ORB_FAST_SCORE)

        flann_index_lsh = 6
        index_params = dict(
            algorithm=flann_index_lsh,
            table_number=6,  # 12
            key_size=12,  # 20
            multi_probe_level=1,
        )  # 2

        search_params = dict(checks=5)
        flann = cv2.FlannBasedMatcher(index_params, search_params)

        base_image = self.get_image()

        map_kp, map_des = orb.detectAndCompute(base_image, None)

        dim = base_image.shape
        low = .1
        high = 1 - low
        image_bounds = [(dim[1] * low, dim[0] * low), (dim[1] * high, dim[0] * low), \
            (dim[1] * high, dim[0] * high), (dim[1] * low, dim[0] * high)]
        seen_poly = Polygon()

        while self.locating:
            # fetch key points from google image
            img = self.get_image()
            cur_kp, cur_des = orb.detectAndCompute(img, None)
            # gathers map keys within frame size
            if self.misses < 3:
                keys_within_region = [i for i, v in enumerate(map_kp) if \
                    v.pt[0] >= self.position_guess[0] - self.frame_size[0] / 2 and \
                    v.pt[0] <= self.position_guess[0] + self.frame_size[0] * 1.5 and \
                    v.pt[1] >= self.position_guess[1] - self.frame_size[1] / 2 and \
                    v.pt[1] <= self.position_guess[1] + self.frame_size[1] * 1.5]
                #Use map des within the bounding box
                relevant_map_des = np.array([map_des[i] for i in keys_within_region])
                relevant_map_kp = [map_kp[i] for i in keys_within_region]
            else:
                #Use all map des
                relevant_map_des = map_des
                relevant_map_kp = map_kp
            matches = flann.knnMatch(relevant_map_des, cur_des, k=2)
            # store all the good matches as per Lowe's ratio test.
            good = []
            try:
                for match_a, match_b in matches:
                    if match_a.distance < 0.7 * match_b.distance:
                        good.append(match_a)
            except ValueError:
                print('err')

            if len(good) > min_match_count:
                self.misses = 0
                 #pylint: disable=too-many-function-args
                src_pts = np.float32([cur_kp[m.trainIdx].pt for m in good]) \
                    .reshape(-1, 1, 2)
                #pylint: disable=too-many-function-args
                dst = np.float32([relevant_map_kp[m.queryIdx].pt for m in good]) \
                    .reshape(-1, 1, 2)

                m_matrix, _ = cv2.findHomography(src_pts, dst, cv2.LMEDS, 5.0)

                im_points = cv2.drawKeypoints(img, cur_kp, None)
                cv2.imshow('test', im_points)
                cv2.waitKey(5)

                # translate points by m matrix
                for key in cur_kp:
                    key.pt = tuple(np.matmul(m_matrix, key.pt + (1,))[:2])

                # estimae new position
                new_guess = np.matmul(m_matrix, [img.shape[1] / 2, img.shape[0] / 2, 1.0])[:2]

                # find points outside of already scanned region and within bounds
                bounds = Polygon(list(map(lambda bound: np.matmul(m_matrix, bound + \
                    (1.0,))[:2], image_bounds)))
                new_pts = [i for i, v in enumerate(cur_kp) if bounds.contains(Point(v.pt)) \
                    and not seen_poly.contains(Point(v.pt))]

                print("Found ", len(new_pts), new_guess)
                self.position_guess = new_guess

                if seen_poly.exterior is None or seen_poly.exterior.distance(Point(new_guess)) \
                        < 100 and new_pts:
                    print('added new')
                    new_des = [cur_des[i] for i in new_pts] #pylint: disable=unused-variable
                    new_kp = [cur_kp[i] for i in new_pts] #pylint: disable=unused-variable

                    for key in new_kp:
                        cv2.circle(point_img, (int(key.pt[0]), int(key.pt[1])), \
                            10, 255, thickness=10)

                    cv2.imshow('points', cv2.resize(point_img, (500, 500)))

                    seen_poly = seen_poly.union(Polygon(bounds))

                    # map_kp.extend(new_kp)
                    # map_des = np.append(map_des, new_des, axis=0)

                map_kp = cur_kp
                map_des = cur_des
            else:
                print("Not enough matches are found - %d/%d" % (len(good), min_match_count))
                self.misses += 1
            cv2.waitKey(5)


    def send_attitude_target(self, \
        roll_angle=0.0, \
        pitch_angle=0.0, \
        yaw_angle=None, \
        yaw_rate=0.0, \
        use_yaw_rate=False, \
        thrust=0.5 \
    ):
        '''
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                    When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        '''
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw_angle = self.attitude.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            radians(yaw_rate),  # Body yaw rate in radian/second
            thrust,  # Thrust
        )
        self.send_mavlink(msg)

    def set_mode(self, mode):
        '''sets the vehicle's mode'''
        self.mode = VehicleMode(mode)
        print('set mode to {}'.format(mode))

    def set_attitude(self, \
        roll_angle=0.0, \
        pitch_angle=0.0, \
        yaw_angle=None, \
        yaw_rate=0.0, \
        use_yaw_rate=False, \
        thrust=0.5, \
        duration=0 \
    ):
        '''
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        '''
        self.send_attitude_target(
            roll_angle, pitch_angle, yaw_angle, yaw_rate, use_yaw_rate, thrust
        )
        start = time()
        while time() - start < duration:
            self.send_attitude_target(
                roll_angle, pitch_angle, yaw_angle, yaw_rate, use_yaw_rate, thrust
            )
            sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0, 0, 0, True, thrust)


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    '''
    Convert degrees to quaternions
    '''
    t0_val = cos(radians(yaw * 0.5))
    t1_val = sin(radians(yaw * 0.5))
    t2_val = cos(radians(roll * 0.5))
    t3_val = sin(radians(roll * 0.5))
    t4_val = cos(radians(pitch * 0.5))
    t5_val = sin(radians(pitch * 0.5))

    w_val = t0_val * t2_val * t4_val + t1_val * t3_val * t5_val
    x_val = t0_val * t3_val * t4_val - t1_val * t2_val * t5_val
    y_val = t0_val * t2_val * t5_val + t1_val * t3_val * t4_val
    z_val = t1_val * t2_val * t4_val - t0_val * t3_val * t5_val

    return [w_val, x_val, y_val, z_val]

if __name__ == '__main__':
    with open('gps_denied_config.json', 'r') as data:
        VEH = setup_vehicle(json.load(data))
        VEH.takeoff()
        VEH.cv_stitch()
