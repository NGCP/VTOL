'''
GPS denied enviornment for movement and CV
'''

from math import atan2, sqrt, acos, pi, cos, sin, radians, tan
from time import time, sleep
from os import listdir, environ
from threading import Thread
from cProfile import Profile
from pickle import load
import pstats
from io import StringIO
import re
import subprocess
import sys
import cv2
from dronekit import connect, LocationGlobalRelative, VehicleMode
import numpy as np
from simple_pid import PID


sys.path.append(".")
sys.path.append("..") 
from conversions import geodetic2ecef, ecef2ned # pylint: disable=wrong-import-position

# Quick configs
ALTITUDE = 10
SOLO = False
CV_SIMULATION = False
LOITER = True
SCALE = 0.5
MAP_HEADING = 50
STABILIZE_TIME = 3
MOVE_DURATION = 1
MAX_VELOCITY = 7
P = .13
I = .01
D = .005
# how many pixels the drone can be off from the target before being in acceptance state
EPSILON = 1000

X_VELOCITY = 0
Y_VELOCITY = 0

def main():
    '''Initializes and runs gps denied program'''
    if SOLO:
        connection_string = "udpin:0.0.0.0:14550"
    else:
        # Port 5763 must be forwarded on vagrant
        connection_string = "tcp:127.0.0.1:5763"
    vehicle = connect(connection_string)

    #Assigns listener to update X_VELOCITY and Y_VELOCITY on vehicle events
    @vehicle.on_message('GLOBAL_POSITION_INT')
    def listener(_, __, message): # pylint: disable=unused-variable
        '''updates x vel and y vel values on GLOBAL_POSITION_INT'''
        global X_VELOCITY   # pylint: disable=global-statement
        global Y_VELOCITY   # pylint: disable=global-statement
        X_VELOCITY = message.vx
        Y_VELOCITY = message.vy

    map_origin = LocationGlobalRelative(35.328403, -120.752401, ALTITUDE)
    target_location = LocationGlobalRelative(35.328219, -120.752315, ALTITUDE)
    keys, descs, length, width = read_from_file()

    # takeoff currently uses GPS
    takeoff(vehicle, ALTITUDE)

    camera = init_camera()
    gps_denied_move(
        vehicle,
        camera,
        target_location,
        keys,
        descs,
        map_origin,
        MAP_HEADING,
        length,
        width,
    )

    land(vehicle)


def gps_denied_move(
        vehicle,
        camera,
        location,
        map_keys,
        map_descs,
        map_origin,
        map_heading,
        map_length,
        map_width,
):
    '''Move using GPS denied functionaly'''
    # convert location to pixels
    target_pixel_location = gps_to_pixel(
        location, map_origin, map_length, map_width, map_heading
    )
    print("target")
    print(target_pixel_location)

    result = calculate_pixel_location(camera, map_keys, map_descs)
    # Wait for a match
    while not result:
        print("Can't find template match: " + str(IMG_COUNTER - 1))
        result = calculate_pixel_location(camera, map_keys, map_descs)
        sleep(2)
    current_pixel_location, current_orientation = result
    print(current_pixel_location)
    print(current_orientation)
    print(IMG_COUNTER - 1)
    current_orientation += 180
    while (
            euclidean_distance(
                current_pixel_location[0],
                current_pixel_location[1],
                target_pixel_location[0],
                target_pixel_location[1],
            ) >= EPSILON
    ):
        # Need to multiply y coordinates by -1 because of the image coordinate system
        counter_clockwise = (
            atan2(
                (-1 * target_pixel_location[1] - (-1 * current_pixel_location[1])),
                (target_pixel_location[0] - current_pixel_location[0]),
            )
            * 180
            / pi
        )
        clockwise = -1 * counter_clockwise
        relative_to_y = 90 + clockwise
        delta_direction = relative_to_y - current_orientation
        if delta_direction < -180:
            delta_direction += 360
        print("delta direction: ", str(delta_direction))

        vehicle.mode = VehicleMode("GUIDED_NOGPS")
        move(vehicle, delta_direction, MOVE_DURATION)

        # Wait a time interval for the image to stabilize
        if LOITER:
            vehicle.mode = VehicleMode("GUIDED")
        sleep(STABILIZE_TIME)

        result = calculate_pixel_location(camera, map_keys, map_descs)
        misses = 0
        # Wait for a match
        while not result:
            print("Can't find template match: " + str(IMG_COUNTER - 1))
            result = calculate_pixel_location(camera, map_keys, map_descs)
            sleep(2)
            misses += 1
            if misses == 5:
                print("Failed to find a template match")
                move(vehicle, delta_direction, MOVE_DURATION / 2)
                break
        current_pixel_location, current_orientation = result
        print(current_pixel_location)
        print(current_orientation)
        current_orientation += 180
        print(IMG_COUNTER - 1)


def gps_to_pixel(location, map_origin, map_length, map_width, heading=0):
    '''Converts gps dcoordinates to pixel coordinates
        Returns tuple of location (tuple) and orientation
        Assumes map is oriented North'''
    # TODO look into this code with test cases. For now return hardcoded value
    return (333, 11193)
    altitude = location.alt     # pylint: disable=unreachable
    print("len {} and width {}".format(map_length, map_width))

    horizonatal_resolution = 3280 * SCALE
    horizontal_angle = 62.2 * SCALE

    vertical_resolution = 2464 * SCALE
    vertical_angle = 48.8 * SCALE

    alt_pixel_x_lon = horizonatal_resolution / tan(radians(horizontal_angle))
    alt_pixel_y_lat = vertical_resolution / tan(radians(vertical_angle))

    origin_pixel = (map_length // 2, map_width // 2)

    met_1640 = altitude / alt_pixel_x_lon
    met_1232 = altitude / alt_pixel_y_lat
    origin_lat, origin_lon = map_origin.lat, map_origin.lon
    poi_lat, poi_lon = location.lat, location.lon
    c_n, c_e, c_d = geodetic2ecef(poi_lat, poi_lon, altitude)
    delta_lat, delta_lon, _ = ecef2ned(c_n, c_e, c_d, origin_lat, origin_lon, altitude)
    delta_lat_pixel = delta_lat / met_1232
    delta_lon_pixel = delta_lon / met_1640

    angled_lon = (delta_lon_pixel * (cos(radians(heading)))) + (
        delta_lat_pixel * (sin(radians(heading)))
    )
    angled_lat = (delta_lat_pixel * (cos(radians(heading)))) - (
        delta_lon_pixel * (sin(radians(heading)))
    )

    origin_x, origin_y = origin_pixel
    print("xOrigin {}, yOrigin {}".format(origin_x, origin_y))
    poi_x, poi_y = origin_x + angled_lon, origin_y + angled_lat
    print("xPixel {}, yPixel {}".format(poi_x, poi_y))
    return poi_x, poi_y


ORB = cv2.ORB_create(nfeatures=1000, scoreType=cv2.ORB_FAST_SCORE)


def calculate_pixel_location(camera, map_keys, map_descs):
    '''Cauculates a pixel location on a map'''
    #Take picture using solo
    img = take_picture(camera)
    # img = cv2.resize(img, (0, 0), fx = SCALE, fy = SCALE)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    keys, descs = ORB.detectAndCompute(img, None)

    flann_index_lsh = 6
    index_params = dict(
        algorithm=flann_index_lsh,
        table_number=6,  # 12
        key_size=12,  # 20
        multi_probe_level=1,
    )  # 2
    search_params = dict(checks=5)
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(descs, map_descs, k=2)

    # Lowe's test to filter matches
    good_matches = []
    for _, (match, num) in enumerate(matches):
        if match.distance < 0.7 * num.distance:
            good_matches.append(match)
    src = [keys[match.queryIdx].pt for match in good_matches]
    src_pts = np.float32(src.reshape(-1, 1, 2)) #pylint: disable=too-many-function-args
    dst = [map_keys[match.trainIdx].pt for match in good_matches]
    dst_pts = np.float32(dst).reshape(-1, 1, 2) #pylint: disable=too-many-function-args

    try:
        m_homography, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        height, width = img.shape
        pts = np.float32( #pylint: disable=too-many-function-args
            [[0, 0], [0, height - 1], [width - 1, height - 1], [width - 1, 0]])\
            .reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, m_homography)
        map_points = cv2.perspectiveTransform(
            np.float32([[width / 2.0, height / 2.0]]).reshape(-1, 1, 2), m_homography #pylint: disable=too-many-function-args
        )[0][0]

        # Vector of the upper edge
        vec = dst[3][0] - dst[0][0]
        # Angle of upper edge to x axis in degrees
        angle = (
            acos(np.dot(vec, np.array([1, 0])) / (sqrt(vec[0] ** 2 + vec[1] ** 2)))
            * 180
            / pi
        )
        return ((map_points[0], map_points[1]), angle)

    except cv2.error:
        # If there aren't enough matches to template match, return None
        return None


IMG_COUNTER = 0


def sorted_alphanumeric(data):
    '''Sorts array data, alpha numeric'''
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split("([0-9]+)", key)]
    return sorted(data, key=alphanum_key)


def cv_simulation():
    '''Initializes up the cv simulation'''
    global IMG_COUNTER # pylint: disable=global-statement
    files = sorted_alphanumeric(listdir("./solopics"))
    path = "./solopics/" + files[IMG_COUNTER % len(files)]
    img = cv2.imread(path, 1)
    crop_img = img[78:630, 270:1071]
    # crop_img = img
    IMG_COUNTER += 1
    return crop_img


def connect_solo_wifi():
    '''execute the shell script (does not terminate)'''
    if sys.platform == "win32":
        subprocess.check_output(["nc", "10.1.1.1", "5502"], shell=True)
    else:
        subprocess.check_output(["nc", "10.1.1.1", "5502"])


def init_camera():
    '''Initializes the camera'''
    if CV_SIMULATION:
        return None
    # initialize camera for 3DR Solo
    solo_connect_thread = Thread(target=connect_solo_wifi)
    solo_connect_thread.daemon = True
    solo_connect_thread.start()
    sleep(1)
    environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "protocol_whitelist;file,rtp,udp"
    cam = cv2.VideoCapture("./sololink.sdp")
    return cam


def take_picture(camera):
    '''Takes a picture using the cimulated camera or the connected camera'''
    if CV_SIMULATION:
        return cv_simulation()
    try:
        global IMG_COUNTER # pylint: disable=global-statement
        count = camera.get(cv2.CAP_PROP_FRAME_COUNT)
        camera.set(cv2.CAP_PROP_POS_FRAMES, count - 1)
        camera.grab()
        _, img = camera.retrieve()
        cv2.imwrite("solopics/" + str(IMG_COUNTER) + ".png", img)
        crop_img = img[78:630, 270:1071]
        IMG_COUNTER += 1
        return crop_img
    # except KeyboardInterrupt:
    #     raise
    except TypeError:
        # Try taking the picture again
        sleep(1)
        take_picture(camera)


def euclidean_distance(x_initial, y_initial, x_final, y_final):
    '''Calculates distance between two points'''
    return sqrt((x_initial - x_final) ** 2 + (y_initial - y_final) ** 2)


def move(vehicle, theta_deg, duration):
    '''moves the vehicle in the direction thetat for duration seconds'''
    theta_deg = theta_deg / 180.0 * pi
    print("moving", theta_deg, sin(theta_deg), cos(theta_deg))
    set_velocity(vehicle, x_velocity=cos(theta_deg) * MAX_VELOCITY,
                 y_velocity=sin(theta_deg) * MAX_VELOCITY, duration=duration)


def set_velocity(vehicle, x_velocity=0, y_velocity=0, duration=0):
    '''Sets the velocity of the vehicle RELATIVE TO HEADING'''
    x_pid = PID(P, I, D, setpoint=x_velocity * 133)
    x_pid.sample_time = 0.1
    y_pid = PID(P, I, D, setpoint=y_velocity * 133)
    y_pid.sample_time = 0.1
    for _ in range(0, duration * 10):
        # compute new ouput from the PID according to the systems current value
        xcontrol = x_pid(X_VELOCITY)
        ycontrol = y_pid(Y_VELOCITY)
        set_attitude(vehicle, pitch_angle=-1 * xcontrol, roll_angle=ycontrol, duration=.1)
        # feed the PID output to the system and get its current value


def takeoff(vehicle, altitude):
    '''Commands drone to take off by arming vehicle and flying to altitude'''
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


def land(vehicle):
    '''Commands vehicle to land'''
    print("Returning to launch")
    vehicle.mode = VehicleMode("RTL")

    print("Closing vehicle object")
    vehicle.close()


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


def send_attitude_target(
        vehicle,
        roll_angle=0.0,
        pitch_angle=0.0,
        yaw_angle=None,
        yaw_rate=0.0,
        use_yaw_rate=False,
        thrust=0.5,
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
    '''
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    '''
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
    '''Reads map from a map2.bin file'''
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
    PR = Profile()
    PR.enable()
    main()
    PR.disable()
    STREAM = StringIO()
    SORT_BY = "cumulative"
    PS = pstats.Stats(PR, stream=STREAM).sort_stats(SORT_BY)
    PS.print_stats()
    print(STREAM.getvalue())
