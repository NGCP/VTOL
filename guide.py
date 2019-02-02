from dronekit import connect, VehicleMode, LocationGlobalRelative
from threading import Thread
# from digi.xbee.devices import RemoteXBeeDevice
import math
import json
import dronekit_sitl
import sys
import time

start_mission = False
destination = None
continue_path = False

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
    configs = configs.get('guide_config')
    print(configs)
    # xbee.add_data_received_callback(xbee_callback)

    print ("Start simulator (SITL)")
    sitl = dronekit_sitl.start_default(
        lat=configs.get('start_lat'), lon=configs.get('start_lon'))
    connection_string = sitl.connection_string()

    # Connect to the Vehicle.
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)

    # Get some vehicle attributes (state)
    print("Get some vehicle attribute values:")
    print(" GPS: %s" % vehicle.gps_0)
    print(" Battery: %s" % vehicle.battery)
    print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print(" Is Armable?: %s" % vehicle.is_armable)
    print(" System status: %s" % vehicle.system_status.state)
    print(" Mode: %s" % vehicle.mode.name)    # settable)

    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("waiting for coordinates")
    while not start_mission:
        time.sleep(.5)

    print("Taking off")
    # revisit this (Talking with Pomona about incements based on ration vs distance)
    # Take off to target altitude
    target_altitude = configs.get('altitude')
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

    vehicle.groundspeed = configs.get('ground_speed')
    home = vehicle.location.global_relative_frame
    theta = math.atan((home.lon - destination.lon) /
                      (home.lat - destination.lat))
    d_lat = math.cos(theta) * configs.get('distance_meters') / 111111.0
    d_lon = math.sin(theta) * configs.get('distance_meters') / 111111.0

    print(d_lat, d_lon)

    while get_distance_metres(vehicle.location.global_relative_frame, destination) > 1:
        lat = vehicle.location.global_relative_frame.lat + d_lat/111111
        lon = vehicle.location.global_relative_frame.lon + d_lon/111111

        loc = LocationGlobalRelative(
            lat, lon, vehicle.location.global_relative_frame.alt)
        print(loc)
        vehicle.simple_goto(loc)
        while(get_distance_metres(vehicle.location.global_relative_frame, loc) > 1):
            print(get_distance_metres(vehicle.location.global_relative_frame, loc) > 1)
            time.sleep(1)
        while not continue_path:
            print("awaiting continue path")
            time.sleep(1)
        continue_path = False

    vehicle.groundspeed = 0
    while not continue_path:
        print("awaiting continue path")
        time.sleep(1)
    print("waiting")

    # Land vehicle procedure ??
    vehicle.close()

    # Shut down simulator
    if(configs.get('vehicle_simulated')):
        sitl.stop()
    print("Completed")

# Reads through comm simulation file from configs and calls xbee_callback to simulate radio messages.


# Dummy message class for comm simulation thread to be compatible with xbee_callback function
class DummyMessage:
    def __init__(self, data=None):
        self.data = data  # UTF-8 encoded JSON message
        self.remote_device = DummyRemoteDevice()


# Dummy remote device object for use in DummyMessage, has fake address 0
class DummyRemoteDevice:
    def __init__(self):
        pass

    def get_64bit_addr(self):
        return "comms simulation"


def comm_simulation(comm_file):
    try:
        f = open(comm_file, "r")
    except FileNotFoundError:
        raise

    line = f.readline().strip()
    prev_time = float(line[:line.index("~")])
    while line != "":
        delim = line.index("~")
        curr_time = float(line[:delim])
        time.sleep(curr_time - prev_time)

        # Send message to xbee_callback
        xbee_callback(DummyMessage(line[delim + 1:]))

        line = f.readline().strip()
        prev_time = curr_time


configs = parse_configs(sys.argv)
comm_sim = Thread(target=comm_simulation, args=(
    configs["comms_simulated"]["comm_sim_file"],))
comm_sim.start()
while(not start_mission):
    pass

guide(configs, comm_sim)
