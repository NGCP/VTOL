'''main executable for setting up VTOL'''
import json
from vtol import setup_vehicle
def main(configs):
    '''Configure vtol and ready for mission'''
    # pylint: disable=unused-variable
    vehicle = setup_vehicle(configs)

if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))
        