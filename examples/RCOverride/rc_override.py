'''
Module for Navigation with RC-override
move_to_target() to move to a target with rc_override

'''

import math
import time

import dronekit

from pid import PID


def arm(vehicle):
    """Arms vehicle. Taken from dronekit example"""

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = dronekit.VehicleMode("MANUAL")
    vehicle.armed = True


def move_to_target(vehicle, target_waypoint, dt=0.5):
    """
    Moves vehicle to the target coordinate. Must be interrupted if 'vehicle'
    only needs to move in the direction of target coordinate. 

    Parameters
    ----------
    vehicle : dronekit.Vehicle
        Vehicle object from dronekit module.
    target : dronekit.LocationGlobal
        A global location object from dronekit module.
    dt : float
        Interval to send rc override and check if target is reached.
    
    Returns
    -------
    reached_target : bool
        True if target is reached.

    """

    reached_target = False
    while not reached_target:
        time.sleep(dt)
        pid_controller = PID()
        correct_thrust(vehicle)
        correct_yaw(vehicle, target_waypoint, pid_controller)

        reached_target = check_reached_target(vehicle, target_waypoint)

    return reached_target


def correct_thrust(vehicle, target_relative_altitude=50):
    """Corrects thrust so vehicle stays near target relative altitude."""
    if vehicle.location.global_relative_frame.alt <= target_relative_altitude:
        change_thrust(vehicle, 1700)
    else:
        change_thrust(vehicle, 1500)


def correct_yaw(vehicle, target_waypoint, pid_controller=PID()):
    """Corrects the yaw so that vehicle heads towards target_waypoint"""
    calculated_yaw = get_calculated_yaw(vehicle, target_waypoint, pid_controller)
    change_yaw(vehicle, calculated_yaw) 


def get_calculated_yaw(vehicle, target_waypoint, pid_controller):
    """Gets the yaw to correct trajectory.

    1000 is the value for counter-clockwise yaw (Full left stick)
    1500 is the value for hold current yaw (Mid stick).
    2000 is the value for clockwise yaw (Full right stick).
    """
    neutral_yaw = 1500
    max_yaw_increment = 500

    # calculate error btw heading and line to destination
    current_waypoint = get_current_location(vehicle)
    destination_angle_from_north = to_positive_angle(get_angle(current_waypoint, target_waypoint))
    heading_angle_from_north = get_heading_angle(vehicle)

    angle_difference = get_smallest_angle_diff(heading_angle_from_north, destination_angle_from_north)
    error_percent = pid_controller.output(angle_difference) / 360
    increment = math.erf(error_percent) * max_yaw_increment

    calculated_yaw = round(neutral_yaw + increment)

    return calculated_yaw


def get_current_location(vehicle):
    """Gets current lat, lon from GPS.

    Need to replace implementation of this function for true GPS denied.
    """

    return vehicle.location.global_frame


def get_angle(current_waypoint, target_waypoint):
    """Gets angle vehicle needs to face."""
    lat_dist = target_waypoint.lat - current_waypoint.lat
    lon_dist = target_waypoint.lon - current_waypoint.lon

    return math.degrees(math.atan2(lon_dist, lat_dist))


def get_heading_angle(vehicle):
    """Gets heading of vehicle."""
    return vehicle.heading


def get_distance(p0, p1):
    """Gets distance between two points."""
    return math.sqrt((p0.lat - p1.lat)**2 + (p0.lon - p1.lon)**2)


def to_positive_angle(angle):
    """Gets the positive angle."""
    return angle % 360


def get_smallest_angle_diff(source, target):
    """Gets the smallest difference between two angles.
    
    Gives a signed angle.
    """

    result = target - source
    result = (result + 180) % 360 - 180

    return result


def circle_currrent_location(vehicle):
    """Circle location in GPS-denied.
    
    Don't use this function. Use LOITER instead.
    """

    change_thrust(vehicle, 1050)
    vehicle.mode = dronekit.VehicleMode("CIRCLE")


def check_reached_target(vehicle, target_waypoint, margin_of_error=0.0001):
    """Checks if vehicle has reached target.
       
    Margin of error is in terms of degrees.
    1 degree = 69 mi = 111045 m.
    """

    current_waypoint = get_current_location(vehicle)
    distance = get_distance(target_waypoint, current_waypoint)
    
    if abs(distance) < margin_of_error:
        return True
    
    return False


def change_yaw(vehicle, yaw):
    """Sends RC override for yaw. Assumes yaw uses RC channel 4."""
    print("RC override yaw:", yaw)
    vehicle.channels.overrides['4'] = yaw


def change_thrust(vehicle, thrust):
    """Sends RC override for thrust. Assumes thrust uses RC channel 3."""
    print("RC override thrust: ", thrust)
    vehicle.channels.overrides['3'] = thrust


# Quadrant 1
TARGET_WAYPOINT = dronekit.LocationGlobal(-35.356833, 149.162703, None)

# Quadrant 2
#TARGET_WAYPOINT = dronekit.LocationGlobal(-35.359124, 149.168475, None)

# Quadrant 3
#TARGET_WAYPOINT = dronekit.LocationGlobal(-35.371669, 149.169827, None)

# Quadrant 4
#TARGET_WAYPOINT = dronekit.LocationGlobal(-35.371415, 149.162192, None)

def main():
    vehicle = dronekit.connect('tcp:127.0.0.1:5763', wait_ready=True)
    arm(vehicle)
    move_to_target(vehicle, TARGET_WAYPOINT)
    vehicle.mode = dronekit.VehicleMode("LOITER")
    
    # Sleeping so all MAVLINK messages are correctly sent
    time.sleep(3)

if __name__ == '__main__':
    main()
