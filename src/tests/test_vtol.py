'''test for vtol.py'''
import pytest
from vtol import setup_vehicle

CONFIGS = {
    'vehicle_simulated': True,
    'vehicle_type': 'Quadcopter',
    'vehicle_id': 1,
    'coms_simulated': True,
    'altitude': 5,
    'comm_sim_file': 'comm_sim_example.json'
}

with open('configs.json', 'r') as data:
    VEHICLE = setup_vehicle(CONFIGS)
    print("HERE")


def test_takeoff():
    '''quadcopter dronekit-sitl takeoff'''
    global VEHICLE # pylint: disable=global-statement
    VEHICLE.takeoff()
    alt = VEHICLE.location.global_relative_frame.alt
    assert 4 < alt < 6, "vehicle did not reach target alt"


def test_land():
    '''quadcopter dronekit-sitl land'''
    global VEHICLE # pylint: disable=global-statement
    VEHICLE.rtl()
    alt = VEHICLE.location.global_relative_frame.alt
    assert -1 < alt < 1, "vehicle did not land"

def test_change_status():
    '''set status sets the vehicles status correctly and throws with invalid status'''
    global VEHICLE # pylint: disable=global-statement
    VEHICLE.change_status('paused')
    assert VEHICLE.status == 'paused'

    with pytest.raises(Exception):
        VEHICLE.change_status('invalid')
