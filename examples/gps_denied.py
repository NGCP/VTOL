from dronekit import *
import dronekit_sitl
import cv2
import autonomy
from autonomy import takeoff, land
import math

def main():
    sitl = dronekit_sitl.start_default(lat=1, lon=1)
    vehicle = connect(sitl.connection_string())

    map = cv2.imread("map.png")
    map_origin = (1.0, 1.0)
    map_altitude = 30

    # takeoff currently uses GPS
    takeoff(vehicle)
    vehicle.mode = VehicleMode("GUIDED_NOGPS")

    gps_denied_move(vehicle, LocationGlobalRelative(1.005, 1.0, 30))

# how many pixels the drone can be off from the target before being in acceptance state
EPSILON = 20

def gps_denied_move(vehicle, location, map, map_origin, map_altitude):
    # convert location to pixels
    target_pixel_location = gpsToPixels(location, map_origin, map_altitude)

    (current_pixel_location, current_orientation) = calculatePixelLocation(map, map_origin, map_altitude)
    while (euclidean_distance(current_pixel_location[0], current_pixel_location[1]
                              target_pixel_location[0], target_pixel_location[1]) < EPSILON):
        # trigonometry to calculate how much yaw must change
        delta_direction = atan((target_pixel_location[1] - current_pixel_location[1]) /
                               (target_pixel_location[0] - current_pixel_location[0])) - current_orientation
        change_yaw(delta_direction)
        move_forward()
        (current_pixel_location, current_orientation) = calculatePixelLocation(map, map_origin, map_altitude)

# returns tuple of location (tuple) and orientation
def gpsToPixels(location, map_origin, map_altitude):
    pass

def calculatePixelLocation():
    pass

def euclidean_distance(x1, y1, x2, y2):
    pass

def change_yaw():
    pass

def move_forward():
    pass

if __name__ == "__main__":
    main()