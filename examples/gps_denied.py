from dronekit import *
from dronekit_sitl import start_default
import cv2
from math import atan, atan2, sqrt, acos, pi
import numpy as np
from os import listdir, environ
import cProfile, pstats, StringIO
from math import cos, sin, radians
from time import time, sleep
from pickle import dump, load, HIGHEST_PROTOCOL
from threading import Thread
import re
import subprocess

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
MOVE_DURATION = 2
TOTAL_TILT = 25
# how many pixels the drone can be off from the target before being in acceptance state
EPSILON = 500


def main():
    if SOLO:
        connection_string = "udpin:0.0.0.0:14550"
    else:
        # Port 5763 must be forwarded on vagrant
        connection_string = "tcp:127.0.0.1:5763"
    vehicle = connect(connection_string)

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
    # convert location to pixels
    target_pixel_location = gpsToPixels(
        location, map_origin, map_length, map_width, map_heading
    )
    print("target")
    print(target_pixel_location)

    result = calculatePixelLocation(camera, map_keys, map_descs)
    # Wait for a match
    while not result:
        print("Can't find template match: " + str(img_counter - 1))
        result = calculatePixelLocation(camera, map_keys, map_descs)
        sleep(2)
    current_pixel_location, current_orientation = result
    print(current_pixel_location)
    print(current_orientation)
    print(img_counter - 1)
    current_orientation += 180
    while (
        euclidean_distance(
            current_pixel_location[0],
            current_pixel_location[1],
            target_pixel_location[0],
            target_pixel_location[1],
        )
        >= EPSILON
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

        result = calculatePixelLocation(camera, map_keys, map_descs)
        misses = 0
        # Wait for a match
        while not result:
            print("Can't find template match: " + str(img_counter - 1))
            result = calculatePixelLocation(camera, map_keys, map_descs)
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
        print(img_counter - 1)


# returns tuple of location (tuple) and orientation
# Assumes map is oriented North
def gpsToPixels(location, map_origin, map_length, map_width, HEADING=0):
    # TODO look into this code with test cases. For now return hardcoded value
    return (333, 11193)
    altitude = location.alt
    print("len {} and width {}".format(map_length, map_width))

    HORIZONTAL_RESOLUTION = 3280 * SCALE
    HORIZONTAL_ANGLE = 62.2 * SCALE

    VERTICAL_RESOLUTION = 2464 * SCALE
    VERTICAL_ANGLE = 48.8 * SCALE

    ALT_PIXEL_X_Lon = HORIZONTAL_RESOLUTION / math.tan(math.radians(HORIZONTAL_ANGLE))
    ALT_PIXEL_Y_Lat = VERTICAL_RESOLUTION / math.tan(math.radians(VERTICAL_ANGLE))

    originPixel = (map_length // 2, map_width // 2)

    met_1640 = altitude / ALT_PIXEL_X_Lon
    met_1232 = altitude / ALT_PIXEL_Y_Lat
    originLat, originLon = map_origin.lat, map_origin.lon
    POI_Lat, POI_Lon = location.lat, location.lon
    cn, ce, cd = geodetic2ecef(POI_Lat, POI_Lon, altitude)
    delta_Lat, delta_Lon, d = ecef2ned(cn, ce, cd, originLat, originLon, altitude)
    delta_lat_pixel = delta_Lat / met_1232
    delta_lon_pixel = delta_Lon / met_1640

    angled_lon = (delta_lon_pixel * (math.cos(math.radians(HEADING)))) + (
        delta_lat_pixel * (math.sin(math.radians(HEADING)))
    )
    angled_lat = (delta_lat_pixel * (math.cos(math.radians(HEADING)))) - (
        delta_lon_pixel * (math.sin(math.radians(HEADING)))
    )

    originX, originY = originPixel
    print("xOrigin {}, yOrigin {}".format(originX, originY))
    POIX, POIY = originX + angled_lon, originY + angled_lat
    print("xPixel {}, yPixel {}".format(POIX, POIY))
    return POIX, POIY


orb = cv2.ORB_create(nfeatures=1000, scoreType=cv2.ORB_FAST_SCORE)


def calculatePixelLocation(camera, map_keys, map_descs):
    # Take picture using solo
    img = take_picture(camera)
    # img = cv2.resize(img, (0, 0), fx = SCALE, fy = SCALE)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    keys, descs = orb.detectAndCompute(img, None)

    FLANN_INDEX_LSH = 6
    index_params = dict(
        algorithm=FLANN_INDEX_LSH,
        table_number=6,  # 12
        key_size=12,  # 20
        multi_probe_level=1,
    )  # 2
    search_params = dict(checks=5)
    flann = cv2.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(descs, map_descs, k=2)

    # Lowe's test to filter matches
    good_matches = []
    for i, (m, n) in enumerate(matches):
        if m.distance < 0.7 * n.distance:
            good_matches.append(m)

    src_pts = np.float32([keys[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([map_keys[m.trainIdx].pt for m in good_matches]).reshape(
        -1, 1, 2
    )

    try:
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        h, w = img.shape
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(
            -1, 1, 2
        )
        dst = cv2.perspectiveTransform(pts, M)
        mp = cv2.perspectiveTransform(
            np.float32([[w / 2.0, h / 2.0]]).reshape(-1, 1, 2), M
        )[0][0]

        # Vector of the upper edge
        vec = dst[3][0] - dst[0][0]
        # Angle of upper edge to x axis in degrees
        angle = (
            acos(np.dot(vec, np.array([1, 0])) / (sqrt(vec[0] ** 2 + vec[1] ** 2)))
            * 180
            / pi
        )
        return ((mp[0], mp[1]), angle)

    except cv2.error:
        # If there aren't enough matches to template match, return None
        return None


img_counter = 0


def sorted_alphanumeric(data):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split("([0-9]+)", key)]
    return sorted(data, key=alphanum_key)


def sorted_alphanumeric(data):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [convert(c) for c in re.split("([0-9]+)", key)]
    return sorted(data, key=alphanum_key)


def cv_simulation():
    global img_counter
    files = sorted_alphanumeric(listdir("./solopics"))
    path = "./solopics/" + files[img_counter % len(files)]
    img = cv2.imread(path, 1)
    crop_img = img[78:630, 270:1071]
    # crop_img = img
    img_counter += 1
    return crop_img


def connect_solo_wifi():
    # execute the shell script (does not terminate)
    if sys.platform == "win32":
        subprocess.check_output(["nc", "10.1.1.1", "5502"], shell=True)
    else:
        subprocess.check_output(["nc", "10.1.1.1", "5502"])


def init_camera():
    if CV_SIMULATION:
        return None
    else:
        # initialize camera for 3DR Solo
        solo_connect_thread = Thread(target=connect_solo_wifi)
        solo_connect_thread.daemon = True
        solo_connect_thread.start()
        sleep(1)
        environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "protocol_whitelist;file,rtp,udp"
        cam = cv2.VideoCapture("./sololink.sdp")
        return cam


def take_picture(camera):
    if CV_SIMULATION:
        return cv_simulation()
    else:
        global img_counter
        count = camera.get(cv2.CAP_PROP_FRAME_COUNT)
        camera.set(cv2.CAP_PROP_POS_FRAMES, count - 1)
        camera.grab()
        _, img = camera.retrieve()
        cv2.imwrite("solopics/" + str(img_counter) + ".png", img)
        crop_img = img[78:630, 270:1071]
        img_counter += 1
        return crop_img


def euclidean_distance(x1, y1, x2, y2):
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def change_yaw(vehicle, thetaDeg):
    set_attitude(vehicle, yaw_angle=thetaDeg + vehicle.heading, duration=3)


def move_forward(vehicle, duration):
    set_attitude(
        vehicle, pitch_angle=-16, yaw_rate=0, use_yaw_rate=True, duration=duration
    )


def move(vehicle, thetaDeg, duration):
    thetaDeg = thetaDeg / 180.0 * pi
    print("moving", thetaDeg, sin(thetaDeg), cos(thetaDeg))
    set_attitude(
        vehicle,
        pitch_angle=-1 * cos(thetaDeg) * TOTAL_TILT,
        roll_angle=sin(thetaDeg) * TOTAL_TILT,
        yaw_rate=0,
        use_yaw_rate=True,
        duration=duration,
    )


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


# Commands vehicle to land
def land(vehicle):
    print("Returning to launch")
    vehicle.mode = VehicleMode("RTL")

    print("Closing vehicle object")
    vehicle.close()


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
    pr = cProfile.Profile()
    pr.enable()
    main()
    pr.disable()
    s = StringIO.StringIO()
    sortby = "cumulative"
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats()
    print(s.getvalue())
