from dronekit import *
from dronekit_sitl import start_default
import cv2
from math import atan, atan2, sqrt, acos, pi, fabs
import numpy as np
import urllib.request
import ssl
from os import listdir, environ
from math import cos, sin, radians
from time import time, sleep
from pickle import dump, load, HIGHEST_PROTOCOL
from simple_pid import PID
from threading import Thread
import re
import subprocess
from scipy.spatial import ConvexHull, convex_hull_plot_2d, Delaunay

import sys

sys.path.append(".")
sys.path.append("..")
from conversions import geodetic2ecef, ecef2enu, ecef2ned

# Quick configs
ALTITUDE = 10
SOLO = False
CV_SIMULATION = False
LOITER = True
SCALE = 0.5
MAP_HEADING = 50
STABILIZE_TIME = 3
MOVE_DURATION = 1
TOTAL_TILT = 25
TARGET_POSITION = None
REACHED_DEST = False

#Motion PID
P = .10
I = .02
D = .01

#Pos PID
pos_p = .13
pos_i = .00005
pos_d = .00001

#CV sim
FRAME_SIZE = (600, 280)
MISSES = 0
UL_pos = (0,0)

# how many pixels the drone can be off from the target before being in acceptance state
EPSILON = 1000

X_VELOCITY = 0
Y_VELOCITY = 0

def main():
    global TARGET_POSITION
    global REACHED_DEST
    if SOLO:
        connection_string = "udpin:0.0.0.0:14550"
        
    else:
        # Port 5763 must be forwarded on vagrant
        connection_string = "tcp:127.0.0.1:5760"
    vehicle = connect(connection_string)

    @vehicle.on_message('GLOBAL_POSITION_INT')
    def listener(self, name, message):
        global X_VELOCITY
        global Y_VELOCITY
        X_VELOCITY = message.vx
        Y_VELOCITY = message.vy

    takeoff(vehicle, ALTITUDE)

    TARGET_POSITION = (400, 0)

    position_thread = Thread(target=set_position, kwargs={'vehicle': vehicle})
    position_thread.daemon = True

    cv_thread = Thread(target=cv_stitch, kwargs={'vehicle': vehicle})
    cv_thread.daemon = True

    cv_thread.start()
    position_thread.start()
    while not REACHED_DEST:
        sleep(1)
    REACHED_DEST = False
    TARGET_POSITION = (400, 400)
    while not REACHED_DEST:
        sleep(1)
    REACHED_DEST = False
    TARGET_POSITION = (0, 400)
    while not REACHED_DEST:
        sleep(1)
    REACHED_DEST = False
    # TARGET_POSITION = (-35.3603678, 149.1678384)
    # sleep(40)
    # takeoff currently uses GPS

def getImage(lat, lng):
    url = "https://maps.googleapis.com/maps/api/staticmap?center={},{}&zoom=20&size=600x300&maptype=satellite&key=".format(lat,lng)
    context = ssl._create_unverified_context()
    resp = urllib.request.urlopen(url,context=context)
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    decoded = cv2.imdecode(image, cv2.IMREAD_GRAYSCALE)
    cropped = decoded[:280, :]
    return cropped

def set_position(vehicle = None):
    global REACHED_DEST
    global TARGET_POSITION
    assert TARGET_POSITION is not None

    #set attitude to 0
    set_attitude(vehicle, yaw_angle=0, duration=5)
    while True:
        print("Targeting", TARGET_POSITION)
        current_target = TARGET_POSITION
        x_pid = PID(pos_p, pos_i, pos_d, setpoint=current_target[0])
        x_pid.sample_time = 0.33
        y_pid = PID(pos_p, pos_i, pos_d, setpoint=current_target[1])
        y_pid.sample_time = 0.33
        till_stop = 15
        while True:
            if(current_target != TARGET_POSITION):
                #if dest changes rest PID algs
                break
            if(till_stop == 0):
                REACHED_DEST = True
            #Get control from each PID
            x_control = x_pid(UL_pos[0])
            y_control = y_pid(UL_pos[1])
            remaining_x = current_target[0] - UL_pos[0]
            remaining_y = current_target[1] - UL_pos[1]
            remaining_dst_sq = (remaining_x ** 2 + remaining_y ** 2) ** .5
            if(remaining_dst_sq < 20):
                till_stop -= 1
            else:
                till_stop = 15
            #Correct for tilts higher than TOTAL_TILT
            total_control_sq = (y_control ** 2 + x_control ** 2) ** .5
            if total_control_sq > TOTAL_TILT:
                x_control = TOTAL_TILT * remaining_x / remaining_dst_sq
                y_control = TOTAL_TILT * remaining_y / remaining_dst_sq

            set_attitude(vehicle, 
                roll_angle=x_control, 
                pitch_angle= y_control, 
                duration=.33, 
                yaw_angle=0)

def set_velocity(vehicle, vx = 0, vy = 0, duration = 0):
    xpid = PID(P, I, D, setpoint=vx * 100)
    xpid.sample_time = 0.33
    ypid = PID(P, I, D, setpoint=vy * 100)
    ypid.sample_time = 0.33
    for x in range(0, int(duration * 10)):
        # compute new ouput from the PID according to the systems current value
        xcontrol = xpid(X_VELOCITY)
        ycontrol = ypid(Y_VELOCITY)
        set_attitude(vehicle, pitch_angle= -1 * xcontrol, roll_angle=ycontrol, duration=.3)
        # feed the PID output to the system and get its current value



def cv_stitch(vehicle):
    global MISSES
    global UL_pos

    min_match_count = 10
    PIXELS_PER_METER = 8.25
    # Initiate SIFT detector

    ORB = cv2.ORB_create(nfeatures=1000, scoreType=cv2.ORB_FAST_SCORE)

    flann_index_lsh = 6
    index_params = dict(
        algorithm=flann_index_lsh,
        table_number=6,  # 12
        key_size=12,  # 20
        multi_probe_level=1,
    )  # 2
    search_params = dict(checks=5)
    FLANN = cv2.FlannBasedMatcher(index_params, search_params)

    BASE = getImage(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
    ORIGIN = (BASE.shape[1] / 2, BASE.shape[0] / 2)
    map_kp, map_des = ORB.detectAndCompute(BASE, None)

    point_poly = Delaunay(list(map(lambda key: key.pt, map_kp)), incremental=True)

    while True: 
        img = getImage(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        cur_kp, cur_des = ORB.detectAndCompute(img, None)
        
        keys_within_region = [i for i,v in enumerate(map_kp) if 
                    v.pt[0] >= UL_pos[0] - FRAME_SIZE[0] / 2 and 
                    v.pt[0] <= UL_pos[0] + FRAME_SIZE[0] * 1.5 and 
                    v.pt[1] >= UL_pos[1] - FRAME_SIZE[1] / 2 and 
                    v.pt[1] <= UL_pos[1] + FRAME_SIZE[1] * 1.5]
        if MISSES < 3:
            #Use map des within the bounding box
            relevant_map_des = np.array([map_des[i] for i in keys_within_region])
            relevant_map_kp = [map_kp[i] for i in keys_within_region]
        else:
            #Use all map des
            relevant_map_des = map_des
            relevant_map_kp = map_kp
        matches = FLANN.knnMatch(relevant_map_des, cur_des, k=2)
        # store all the good matches as per Lowe's ratio test.
        good = []
        try:
            for m, n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)
        except ValueError:
            print('err')

        if len(good) > min_match_count:
            MISSES = 0
            src_pts = np.float32([cur_kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([relevant_map_kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.LMEDS, 5.0)
            matchesMask = mask.ravel().tolist()
            for key in cur_kp:
                key.pt = tuple(np.matmul(M, [key.pt[0], key.pt[1], 1])[:2])
            outside_region_indecies = [i for i,v in enumerate(cur_kp) if not point_poly.find_simplex(v.pt) >= 0]
            UL_pos = np.matmul(M, [0.0, 0.0, 1.0])[:2]

            new_des = [cur_des[i] for i in outside_region_indecies]
            new_kp = [cur_kp[i] for i in outside_region_indecies]

            if(len(outside_region_indecies) > 0):
                # print("appending new descriptors and keys", len(outside_region_indecies))
                map_kp.extend(new_kp)
                map_des = np.append(map_des, new_des, axis=0)
                point_poly.add_points(list(map(lambda key: key.pt, new_kp)))
        else:
            print("Not enough matches are found - %d/%d" % (len(good), min_match_count))
            matchesMask = None
            MISSES += 1

        print(UL_pos)

# Commands drone to take off by arming vehicle and flying to altitude
def takeoff(vehicle, altitude):
    print("Pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize")
        sleep(1)

    print("Arming motors")
    # Vehicle should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting to arm vehicle")
        sleep(1)

    print("Taking off")
    vehicle.simple_takeoff(altitude)  # take off to altitude

    # Wait until vehicle reaches minimum altitude
    while vehicle.location.global_relative_frame.alt < altitude * 0.95:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        sleep(1)

    print("Reached target altitude")



def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = cos(radians(yaw * 0.5))
    t1 = sin(radians(yaw * 0.5))
    t2 = cos(radians(roll * 0.5))
    t3 = sin(radians(roll * 0.5))
    t4 = cos(radians(pitch * 0.5))
    t5 = sin(radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


def send_attitude_target(
    vehicle,
    roll_angle=0.0,
    pitch_angle=0.0,
    yaw_angle=None,
    yaw_rate=0.0,
    use_yaw_rate=False,
    thrust=0.5,
):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
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
    vehicle.send_mavlink(msg)


def set_attitude(
    vehicle,
    roll_angle=0.0,
    pitch_angle=0.0,
    yaw_angle=None,
    yaw_rate=0.0,
    use_yaw_rate=False,
    thrust=0.5,
    duration=0,
):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(
        vehicle, roll_angle, pitch_angle, yaw_angle, yaw_rate, use_yaw_rate, thrust
    )
    start = time()
    while time() - start < duration:
        send_attitude_target(
            vehicle, roll_angle, pitch_angle, yaw_angle, yaw_rate, use_yaw_rate, thrust
        )
        sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(vehicle, 0, 0, 0, 0, True, thrust)


def read_from_file():
    with open("map2.bin", "rb") as read:
        data = load(read)
        keys_reconstructed = map(
            lambda x: cv2.KeyPoint(
                x["pt"][0],
                x["pt"][1],
                x["size"],
                x["angle"],
                x["response"],
                x["octave"],
                x["class_id"],
            ),
            data["keys"],
        )
    return (keys_reconstructed, data["descs"], data["length"], data["width"])


if __name__ == "__main__":
    main()
