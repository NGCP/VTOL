'''main executable for setting up VTOL'''
import json
import os
from dronekit_sitl import SITL
from util import setup_vehicle
from quad import QUAD

def main(configs):
    '''Configure vtol and ready for mission'''
    args = {}
    binary = os.getenv("SITL_BINARY")
    do_download = True
    if binary is not None:
        do_download = False
        args["path"] = binary
        defaults = os.getenv("SITL_DEFAULTS_FILEPATH")
        if defaults is not None:
            args["defaults_filepath"] = defaults
    sitl = SITL(**args)
    if do_download:
        sitl.download('copter', '3.3', verbose=True)

    sitl.launch(['--model', 'quad', ], await_ready=True, restart=False)
    vehicle = setup_vehicle(configs, QUAD)
    print(vehicle.location.global_relative_frame.alt)

if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))
