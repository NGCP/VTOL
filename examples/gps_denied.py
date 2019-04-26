from dronekit import *
from dronekit_sitl import start_default
import cv2
from math import atan, sqrt, pi, acos
import numpy as np
from os import listdir
import cProfile, pstats, StringIO
from math import cos, sin, radians
from time import time, sleep
from pickle import dump, load, HIGHEST_PROTOCOL

def main():
    sitl = start_default(lat=35.328423, lon=-120.752505) # EFR location
    vehicle = connect(sitl.connection_string())
    vehicle = None

    map_origin = LocationGlobalRelative(1.0, 1.0, 30)
    keys, descs = read_from_file()
    # takeoff currently uses GPS
    takeoff(vehicle, 30)
    vehicle.mode = VehicleMode("GUIDED")

    gps_denied_move(vehicle, LocationGlobalRelative(1.005, 1.0, 30), keys, descs, map_origin)

    land(vehicle)

# how many pixels the drone can be off from the target before being in acceptance state
EPSILON = 100

def gps_denied_move(vehicle, location, map_keys, map_descs, map_origin):
    # convert location to pixels
    target_pixel_location = gpsToPixels(location, map_origin)

    current_pixel_location, current_orientation = calculatePixelLocation(map_keys, map_descs)
    print(current_pixel_location)
    print(current_orientation)
    while (euclidean_distance(current_pixel_location[0], current_pixel_location[1],
                              target_pixel_location[0], target_pixel_location[1]) >= EPSILON):
        # trigonometry to calculate how much yaw must change
        delta_direction = atan((target_pixel_location[1] - current_pixel_location[1]) /
                               (target_pixel_location[0] - current_pixel_location[0])) - current_orientation
        print(delta_direction)
        change_yaw(vehicle, delta_direction)
        move_forward(vehicle, 5)
        # Wait a second for the image to stabilize
        sleep(1)
        current_pixel_location, current_orientation = calculatePixelLocation(map_keys, map_descs)
        print(current_pixel_location)
        print(current_orientation)

# returns tuple of location (tuple) and orientation
def gpsToPixels(location, map_origin):
    # TODO Samay Nathani
    return (906, 1210)

CV_SIMULATION = True
imgCounter = 0
orb = cv2.ORB_create(nfeatures=1000, scoreType=cv2.ORB_FAST_SCORE)


def calculatePixelLocation(map_keys, map_descs):
    # TODO create test case in Paint.net
    if (CV_SIMULATION):
        img = cv_simulation()
    else:
        # TODO take picture with raspicam
        pass
    
    img = cv2.resize(img, (0, 0), fx = 0.5, fy = 0.5)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    keys, descs = orb.detectAndCompute(img, None)

    FLANN_INDEX_LSH = 6
    index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2
    search_params = dict(checks=5)
    flann = cv2.FlannBasedMatcher(index_params,search_params)

    matches = flann.knnMatch(descs, map_descs, k=2)

    # Lowe's test to filter matches
    good_matches = []
    for i,(m,n) in enumerate(matches):
        if m.distance < 0.7 * n.distance:
            good_matches.append(m)

    src_pts = np.float32([ keys[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
    dst_pts = np.float32([ map_keys[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)

    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    h,w = img.shape
    pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
    dst = cv2.perspectiveTransform(pts,M)
    mp = cv2.perspectiveTransform(np.float32([[w/2.0, h/2.0]]).reshape(-1,1,2), M)[0][0]
    
    # Vector of the upper edge
    vec = dst[3][0] - dst[0][0]
    # Angle of upper edge to x axis in degrees
    angle = acos(np.dot(vec, np.array([1, 0])) / (sqrt(vec[0]**2 + vec[1]**2))) * 180 / pi
    return ((mp[0], mp[1]), angle)

def cv_simulation():
    global imgCounter
    files = listdir("./simulation_images")
    path = "./simulation_images/" + str(imgCounter) + ".png"
    img = cv2.imread(path)
    imgCounter = (imgCounter + 1) % len(files)
    return img

def euclidean_distance(x1, y1, x2, y2):
    return sqrt((x1 - x2)**2 + (y1 - y2)**2)

def change_yaw(vehicle, theta):
    set_attitude(vehicle, yaw_angle=theta + vehicle.heading, duration=3)

def move_forward(vehicle, durration):
    set_attitude(vehicle, pitch_angle=-12, yaw_rate=0, use_yaw_rate=True, duration=durration)

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


def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
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

def send_attitude_target(vehicle, roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
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
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(vehicle, roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(vehicle, roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, use_yaw_rate,
                         thrust)
    start = time()
    while time() - start < duration:
        send_attitude_target(vehicle, roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, use_yaw_rate,
                             thrust)
        sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(vehicle, 0, 0,
                         0, 0, True,
                         thrust)


def read_from_file():
    with open('map.bin', 'rb') as read:
        data = load(read)
        keys_reconstructed = map(lambda x: cv2.KeyPoint(x['pt'][0], x['pt'][1], x['size'], x['angle'], x['response'], x['octave'], x['class_id']), data['keys'])
    return (keys_reconstructed, data['descs'])


if __name__ == "__main__":
    pr = cProfile.Profile()
    pr.enable()
    main()
    pr.disable()
    s = StringIO.StringIO()
    sortby = 'cumulative'
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats()
    print s.getvalue()