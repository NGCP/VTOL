'''main executable for setting up VTOL'''
import json
from src.util import setup_vehicle
from src.quad import QUAD


def main(configs):
    '''Configure vtol and ready for mission'''
    vehicle = setup_vehicle(configs, QUAD)
    vehicle.takeoff()
    vehicle.__class__ = QUAD
    # vehicle.set_altitude(30)
    vehicle.move_meters(x_dist=10, y_dist=-10)

if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))
