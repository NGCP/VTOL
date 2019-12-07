'''main executable for setting up VTOL'''
import json
from util import setup_vehicle
from quadplane import QuadPlane

def main(configs):
    '''Configure vtol and ready for mission'''
    vehicle = setup_vehicle(configs, QuadPlane)

    vehicle.parameters['Q_GUIDED_MODE'] = 1
    vehicle.takeoff()
    vehicle.set_altitude(10)
    # vehicle.send_ned_velocity(-5, 3, 0, 5)
    # vehicle.set_attitude(pitch_angle=20, duration=5)

    # vehicle.land()

    # home = vehicle.location.global_relative_frame

    # Pick-up function for ping pong balls

    # vehicle.land()

if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))
