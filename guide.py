from dronekit import connect, VehicleMode, LocationGlobalRelative
from threading import Thread
# from digi.xbee.devices import RemoteXBeeDevice
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import math
import json
import dronekit_sitl
import sys
import time
from autonomy import start_dummy_server, comm_simulation, land, takeoff
from conversions import *

start_mission = False
destination = None
continue_path = False
stop_mission = False

def parse_configs(argv):
    """Parses the .json file given as the first command line argument.
    If no .json file is specified, defaults to "configs.json".
    If the specified .json file is found, returns a dict of the contents.
    Otherwise, raises a FileNotFoundError."""
    try:
        if (len(argv) == 1):
            read_file = open("configs.json", "r")
        else:
            read_file = open(argv[1], "r")

        return json.load(read_file)

    except IOError:
        raise


def xbee_callback(message):
    global start_mission
    global destination
    global continue_path
    global stop_mission

    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data.decode("utf8"))
    print("Received data from %s: %s" % (address, msg))

    # try:
    msg_type = msg["type"]

    if msg_type == "start":
        start_mission = True
        destination = LocationGlobalRelative(
            msg["destination"]["lat"], msg["destination"]["lon"])

    elif msg_type == "continue":
        continue_path = True
    elif msg_type == "stop":
        stop_mission = True
    else:
        print("uknown type" + msg_type)
    # except KeyError as e:
    #     # bad_msg(address, e.args[0])
    #     print("bad mesage")
    #     print(e)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def guide(configs, radio):
    global xbee
    global continue_path
    xbee = radio

    # Add the callback.
    configs = configs['guide_config']
    # xbee.add_data_received_callback(xbee_callback)

    print("Waiting for start message")
    while(not start_mission):
        pass
    print ("Start simulator (SITL)")
    sitl = dronekit_sitl.start_default(
        lat=configs['start_lat'], lon=configs['start_lon'])
    connection_string = sitl.connection_string()

    # Connect to the Vehicle.
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)

    vehicle.groundspeed = configs['ground_speed']
    home = vehicle.location.global_relative_frame
    (home_x, home_y, _) = geodetic2ecef(home.lat, home.lon, configs['altitude'])
    (dest_x, dest_y, _)  = geodetic2ecef(destination.lat, destination.lon, configs['altitude'])
    theta = math.atan((dest_y - home_y) /
                      (dest_x - home_x))
    dy = math.cos(theta) * configs['distance_meters']
    dx = math.sin(theta) * configs['distance_meters']

    takeoff(VehicleMode("GUIDED"), vehicle, configs['altitude'])

    while get_distance_metres(vehicle.location.global_relative_frame, destination) > 1:
        current = vehicle.location.global_relative_frame
        (current_x, current_y, current_z) = geodetic2ecef(current.lat, current.lon, current.alt)
        (lat, lon, alt) = ecef2geodetic(current_x+dx, current_y+dy, current_z)
        loc = LocationGlobalRelative(lat, lon, alt)
        vehicle.simple_goto(loc)
        while(get_distance_metres(vehicle.location.global_relative_frame, loc) > 1):
            print("flying :)")
            time.sleep(.05)
        while not continue_path:
            print("awaiting continue path")
            time.sleep(1)
        continue_path = False
        if stop_mission:
            land()
            break
    vehicle.groundspeed = 0
    while not continue_path:
        print("awaiting continue path")
        time.sleep(1)
    print("waiting")

    # Land vehicle procedure ??
    vehicle.close()

    # Shut down simulator
    if(configs['vehicle_simulated']):
        sitl.stop()
    print("Completed")

# Reads through comm simulation file from configs and calls xbee_callback to simulate radio messages.


configs = parse_configs(sys.argv)
if(configs['comms_simulated']['file_simulated']):
    comm_sim = Thread(target=comm_simulation, args=(
        configs["comms_simulated"]["comm_sim_file"],))
    comm_sim.start()
elif(configs['comms_simulated']['http_simulated']):
    comm_sim = Thread(target=start_dummy_server, args=(configs['comms_simulated']['http_port'], xbee_callback))
    comm_sim.daemon=True
    comm_sim.start()


guide(configs, comm_sim)
