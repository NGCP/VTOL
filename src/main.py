'''main executable for setting up VTOL'''
import json
from time import sleep
from util import setup_vehicle
from quad import QUAD


def main(configs):
    '''Configure vtol and ready for mission'''
    vehicle = setup_vehicle(configs, QUAD)
    # vehicle.takeoff()

    # vehicle.set_mode("QHOVER")
    # while True:
        # sleep(.5)
        # cur = pid.output(setpoint - vehicle.altitude())
        # zero = 1530
        # print(min(max(1300, zero + cur), 2000))
        # vehicle.channels.overrides = {
        #     '3': 1500, # min(max(1300, cur * 6), 2000),
        #     '2': 1500,
        #     '1': 1490
        # }
        # vehicle.channels.overrides['3'] = 1500
        # print('overriding')


if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))
