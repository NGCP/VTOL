'''main executable for setting up VTOL'''
import json
from util import setup_vehicle
from quadplane import QuadPlane

def main(configs):
    '''Configure vtol and ready for mission'''
    vehicle = setup_vehicle(configs, QuadPlane)

    vehicle.takeoff()
    vehicle.set_attitude(pitch_angle=-5, duration=10, yaw_angle=180)

    # vehicle.land()

    # home = vehicle.location.global_relative_frame

    # Pick-up function for ping pong balls

    # vehicle.land()

if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))
