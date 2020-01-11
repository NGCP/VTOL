'''test for vtol.py'''
from dronekit_sitl import start_default
from util import setup_vehicle
from quad import QUAD

CONFIGS = {
    'vehicle_simulated': True,
    'vehicle_id': 1,
    'coms_simulated': True,
    'altitude': 5,
    'comm_sim_file': 'comm_sim_example.json',
    'air_speed': 20,
    'simulation': {
        'defaultPort': 5760,
        'shelveName': 'shelfStore.store'
    }
}

CON_STR = start_default().connection_string()

with open('configs.json', 'r') as data:
    VEHICLE = setup_vehicle(CONFIGS, QUAD)

def test_takeoff():
    '''quadcopter dronekit-sitl takeoff'''
    global VEHICLE # pylint: disable=global-statement
    VEHICLE.takeoff()
    alt = VEHICLE.location.global_relative_frame.alt
    assert 4 < alt < 6, "vehicle did not reach target alt"


def test_land():
    '''quadcopter dronekit-sitl land'''
    global VEHICLE # pylint: disable=global-statement
    VEHICLE.land()
    alt = VEHICLE.location.global_relative_frame.alt
    assert -1 < alt < 1, "vehicle did not land"
