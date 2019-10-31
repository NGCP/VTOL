'''main executable for setting up VTOL'''
import json
import time
from dronekit import LocationGlobalRelative, VehicleMode
from vtol import setup_vehicle
from util import get_distance_metres

def main(configs):

    '''Configure vtol and ready for mission'''
    # pylint: disable=unused-variable
    vehicle = setup_vehicle(configs)

    vehicle.takeoff()

    destination = LocationGlobalRelative(configs["lat"], configs["long"], configs["alt"])

    vehicle.simple_goto(destination, 20)

    while (get_distance_metres(vehicle.location.global_relative_frame, destination) > 1) :
        print("Distance remaining:", get_distance_metres(vehicle.location.global_relative_frame, destination))
        time.sleep(1)
    
    print("Target reached")
    
    # Pick-up function for ping pong balls

    vehicle.land()

    print("Landed")


if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))