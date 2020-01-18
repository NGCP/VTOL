'''main executable for setting up VTOL'''
import json
from util import setup_vehicle
from vtol import VTOL

def main(configs):
    '''Configure vtol and ready for mission'''
    setup_vehicle(configs, VTOL)
    while True:
        pass
    # vehicle.takeoff()

    # vehicle.set_altitude(10)

    # home = vehicle.location.global_relative_frame

    # # Pick-up function for ping pong balls

    # vehicle.land()

    # vehicle.takeoff()

    # vehicle.go_to(home)

    # vehicle.land()

if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))
