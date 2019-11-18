'''Automous tools for VTOL'''
import json
import shelve
import sys
from dronekit import connect, APIException
from vtol import VTOL

def setup_vehicle(configs):
    '''Sets up self as a quadplane vehicle'''
    if configs["vehicle_simulated"]:
        veh = scan_ports(configs)
    else:
        # connect to pixhawk via MicroUSB
        # if we switch back to using the telem2 port, use "/dev/serial0"
        con_str = "/dev/ttyACM0"
        veh = connect(con_str, baud=configs["baud_rate"], wait_ready=True, vehicle_class=VTOL)
    veh.configs = configs
    veh.airspeed = configs['air_speed']
    return veh

def scan_ports(configs):
    '''scans TCP ports to find open simulator'''
    itteration = 0
    shelf = shelve.open(configs['simulation']['shelveName'])
    try:
        port = shelf['port']
        print('shelf found')
    except KeyError:
        port = configs['simulation']['defaultPort']
        itteration += 1
        print('not found')
    while True:
        try:
            con_str = "tcp:127.0.0.1:{}".format(port)
            print("Attempting to connect to {}".format(con_str))
            veh = connect(con_str, wait_ready=True, vehicle_class=VTOL)
            shelf['port'] = port
            shelf.close()
            break
        except (OSError, APIException):
            port = configs['simulation']['defaultPort'] + itteration
            itteration += 1
            if itteration == 8:
                print('make sure your simulator is running')
                sys.exit(-1)
    return veh


class QuadPlane(VTOL):
    '''State deffinition for QuadPlane'''



if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        VEHICLE = setup_vehicle(json.load(data))
        VEHICLE.takeoff()
