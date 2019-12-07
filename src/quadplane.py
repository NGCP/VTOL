'''Automous tools for VTOL'''
import json
from vtol import VTOL
from util import setup_vehicle

class QuadPlane(VTOL):
    '''State deffinition for QuadPlane'''
    land_mode = 'QLAND'


if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        VEHICLE = setup_vehicle(json.load(data), QuadPlane)
        VEHICLE.takeoff()
