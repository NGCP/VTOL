'''main executable for setting up VTOL'''
import json
import time
from dronekit import LocationGlobalRelative, LocationGlobal, VehicleMode
from vtol import setup_vehicle
from util import get_distance_metres

def main(configs):
    '''Configure vtol and ready for mission'''
    vehicle = setup_vehicle(configs)
    
    vehicle.takeoff()

    home = vehicle.location.global_relative_frame

    destination = LocationGlobalRelative(configs["dest"]["lat"], configs["dest"]["long"], configs["dest"]["alt"])

    vehicle.go_to(destination)
    
    # Pick-up function for ping pong balls

    vehicle.land()

    vehicle.takeoff()

    vehicle.go_to(home)

    vehicle.land()

if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))
