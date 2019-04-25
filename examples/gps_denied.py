from dronekit import *
import dronekit_sitl
import cv2
import math
import numpy as np
from os import listdir
import cProfile, pstats, StringIO

def main():
    sitl = dronekit_sitl.start_default(lat=35.328423, lon=-120.752505) # EFR location
    vehicle = connect(sitl.connection_string())
    vehicle = None

    map_origin = LocationGlobalRelative(1.0, 1.0, 30)
    keys, descs = initializeMap()

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
        delta_direction = math.atan((target_pixel_location[1] - current_pixel_location[1]) /
                               (target_pixel_location[0] - current_pixel_location[0])) - current_orientation
        print(delta_direction)
        change_yaw(delta_direction)
        move_forward()
        # Wait a second for the image to stabilize
        time.sleep(1)
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

def initializeMap():
    areaMap = cv2.imread("images/Map1.jpg")
    areaMap = cv2.resize(areaMap, (0, 0), fx = 0.5, fy = 0.5)
    areaMap = cv2.cvtColor(areaMap, cv2.COLOR_BGR2GRAY)

    # find the keypoints and descriptors with ORB
    kp, des = orb.detectAndCompute(areaMap, None)

    return (kp, des)

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
    angle = math.acos(np.dot(vec, np.array([1, 0])) / (math.sqrt(vec[0]**2 + vec[1]**2))) * 180 / math.pi
    return ((mp[0], mp[1]), angle)

def cv_simulation():
    global imgCounter
    files = listdir("./simulation_images")
    path = "./simulation_images/" + str(imgCounter) + ".png"
    img = cv2.imread(path)
    imgCounter = (imgCounter + 1) % len(files)
    return img

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def change_yaw(theta):
    # TODO
    time.sleep(1)

def move_forward():
    # TODO
    time.sleep(1)

# Commands drone to take off by arming vehicle and flying to altitude
def takeoff(vehicle, altitude):
    print("Pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize")
        time.sleep(1)

    print("Arming motors")
    # Vehicle should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting to arm vehicle")
        time.sleep(1)

    print("Taking off")
    vehicle.simple_takeoff(altitude)  # take off to altitude

    # Wait until vehicle reaches minimum altitude
    while vehicle.location.global_relative_frame.alt < altitude * 0.95:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("Reached target altitude")

# Commands vehicle to land
def land(vehicle):
    print("Returning to launch")
    vehicle.mode = VehicleMode("RTL")

    print("Closing vehicle object")
    vehicle.close()

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