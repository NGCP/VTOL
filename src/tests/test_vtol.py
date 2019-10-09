'''test for vtol.py'''
import json
from vtol import setup_vehicle
import pytest


with open('configs.json', 'r') as data:
    CONFIGS = json.load(data)
    VEHICLE = setup_vehicle(CONFIGS)
    print("HERE")


def test_takeoff():
    '''tests vehicle's takeoff operation'''
    global VEHICLE
    VEHICLE.takeoff(10)
    alt = VEHICLE.location.global_relative_frame.alt
    assert alt > 9 and alt < 11, "vehicle did not reach target alt"
