'''main executable for setting up VTOL'''
import json
from vtol import setup_vehicle

def main(configs):
    '''Configure vtol and ready for mission'''
    # pylint: disable=unused-variable
    vehicle = setup_vehicle(configs)
    vehicle.takeoff()
    # vehicle.send_ned_velocity(-5, 3, 0, 5)
    vehicle.set_attitude(pitch_angle=20, duration=5)

if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))
