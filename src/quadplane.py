'''Automous tools for VTOL'''
import json
from vtol import VTOL
from util import setup_vehicle

class QuadPlane(VTOL):
    '''State deffinition for QuadPlane'''
    land_mode = 'QLAND'

    def setup(self):
        '''vtol specific steps needed before flight'''
        print('setting params')
        self.parameters['Q_GUIDED_MODE'] = 1

if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        VEHICLE = setup_vehicle(json.load(data), QuadPlane)
        VEHICLE.takeoff()
