'''main executable for setting up VTOL'''
import json
import time
from dronekit import LocationGlobalRelative
from vtol import setup_vehicle
def main(configs):
    '''Configure vtol and ready for mission'''
    # pylint: disable=unused-variable
    vehicle = setup_vehicle(configs)

    print(vehicle.location.global_relative_frame)

    vehicle.takeoff()
    
    point1 = LocationGlobalRelative(35.3284237, -120.7530046, 9.94)

    vehicle.simple_goto(point1, 20)

    while (vehicle.location.global_relative_frame != point1) :
        print(vehicle.location.global_relative_frame)
        time.sleep(1)
    
    print("Target reached")
    
    # Pick-up function for ping pong balls

    vehicle.land()


if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))