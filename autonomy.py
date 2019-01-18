import json
import math
from conversions import *
import time
from threading import Thread
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

# Globals, updated by xBee callback function
start_mission = False  # takeoff
pause_mission = False  # vehicle will hover
stop_mission = False  # return to start and land
search_area = None  # search area object, populated by callback on start
xbee = None  # XBee radio object


# Represents a search area for quick scan aerial vehicles
class SearchArea:
    def __init__(self, center, rad1, rad2):
        self.center = center  # tuple containing coor. of center point
        self.rad1 = rad1  # first search radius from center
        self.rad2 = rad2  # second search radius from center

    def __str__(self):
        return "SearchArea(" + \
               str(self.center) + ", " + \
               str(self.rad1) + ", " + \
               str(self.rad2) + ")"


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


# Callback function for messages from GCS, parses JSON message and sets globals
def xbee_callback(message):
    global start_mission
    global pause_mission
    global stop_mission
    global search_area

    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data)
    print("Received data from %s: %s" % (address, msg))

    try:
        msg_type = msg["type"]

        if msg_type == "start":
            acknowledge(address, msg_type)
            area = msg["searchArea"]
            search_area = SearchArea(area["center"], area["rad1"], area["rad2"])
            start_mission = True

        elif msg_type == "pause":
            pause_mission = True
            acknowledge(address, msg_type)

        elif msg_type == "resume":
            pause_mission = False
            acknowledge(address, msg_type)

        elif msg_type == "stop":
            stop_mission = True
            acknowledge(address, msg_type)

        else:
            bad_msg(address, "Unknown message type: \'" + msg_type + "\'")

    # KeyError if message was missing an expected key
    except KeyError as e:
        bad_msg(address, "Missing \'" + e.args[0] + "\' key")


# Commands drone to take off by arming vehicle and flying to altitude
def takeoff(flight_mode, vehicle, altitude):
    print("Pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize")
        time.sleep(1)

    print("Arming motors")
    # Vehicle should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
       print("Waiting to arm vehicle")
       time.sleep(1)

    print("Taking off")
    vehicle.simple_takeoff(altitude) # take off to altitude

    # Wait until vehicle reaches minimum altitude
    while vehicle.location.global_relative_frame.alt < altitude * 0.95:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("Reached target altitude")


# Commands vehicle to land
def land(vehicle):
    print "Returning to launch"
    vehicle.mode = VehicleMode("RTL")

    print "Closing vehicle object"
    vehicle.close()


# Sends message received acknowledgement to GCS
# :param address: address of GCS
def acknowledge(address, received_type):
    ack = {
        "type": "ack",
        "received": received_type
    }
    # xbee is None if comms is simulated
    if xbee:
        # Instantiate a remote XBee device object to send data.
        send_xbee = RemoteXBeeDevice(xbee, address)
        xbee.send_data(send_xbee, json.dumps(ack))


# Sends "bad message" to GCS if message received was poorly formatted/unreadable
# and describes error from parsing original message.
# :param address: address of GCS
# :param problem: string describing error from parsing original message
def bad_msg(address, problem):
    msg = {
        "type": "badMessage",
        "error": problem
    }
    # xbee is None if comms is simulated
    if xbee:
        # Instantiate a remote XBee device object to send data.
        send_xbee = RemoteXBeeDevice(xbee, address)
        xbee.send_data(send_xbee, json.dumps(msg))
    else:
        print("Error:", problem)


# Reads through comm simulation file from configs and calls xbee_callback to simulate radio messages.
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


# Generate waypoints for VTOL
def generateWaypoints(configs, search_area):
    print("Begin generating waypoints")
    
    waypointsNED = []
    waypointsLLA = []
    
    origin = search_area.center
    radius = search_area.rad2
    
    # pre-definied in configs file
    altitude = configs["altitude"]
    lat = origin[0]
    lon = origin[1]
    centerX, centerY, centerZ = geodetic2ecef(lat, lon, altitude)
    n, e, d = ecef2ned(centerX, centerY, centerZ, lat, lon, altitude)
    waypointsLLA.append([lat, lon, altitude])
    waypointsNED.append([n, e, d])
    
    fovH = math.radians(62.2)     # raspicam horizontal FOV
    fovV = math.radians(48.8)     # raspicam vertical FOV
    boxH = 2 * altitude / math.tan(fovH / 2)     # height of bounding box
    boxV = 2 * altitude / math.tan(fovV / 2)     # width of bounding box
    
    b = (0.75 * boxH) / (2 * math.pi)   # distance between coils with 25% overlap
    rotation = -(math.pi / 2)   # number of radians spiral is rotated
    maxTheta = radius / b   # after using formula r = b * theta
    
    theta = 0
    while theta <= maxTheta:
        # distance away from center
        away = b * theta
        
        # distance around center
        around = theta + rotation
        
        # around & away -> x & y
        x = centerX + math.cos(around) * away
        y = centerY + math.sin(around) * away
        
        # convert ECEF to NED and LLA
        n, e, d = ecef2ned(x, y, centerZ, lat, lon, altitude)
        newLat, newLon, newAlt = ned2geodetic(n, e, d, lat, lon, altitude)
        waypointsNED.append([n, e, d])
        waypointsLLA.append([newLat, newLon, newAlt])
        
        # generate a waypoint every pi/8 radian
        theta += configs["d_theta_rad"]
    
    return (waypointsNED, waypointsLLA)


def adds_mission(vehicle, lla_waypoint_list):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """    

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
    lat = 0
    lon = 1
     
    # Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    # This command is there when vehicle is in AUTO mode, where it takes off through command list
    # In guided mode, the actual takeoff function is needed, in which case this command is ignored
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))


    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    for point in lla_waypoint_list:
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point[0], point[1], 12))

    # adds dummy end point - this endpoint is the same as the last waypoint and lets us know we have reached destination and that
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lla_waypoint_list[-1][0], lla_waypoint_list[-1][1], 12))
    print("Upload new commands to vehicle")
    cmds.upload()


# Main autonomous flight thread
# :param configs: dict from configs file
# :param radio: XBee radio object
def autonomy(configs, radio):
    global xbee
    xbee = radio
    comm_sim = None

    # If comms is simulated, start comm simulation thread
    if xbee is None:
        comm_sim = Thread(target=comm_simulation, args=(configs["comms_simulated"]["comm_sim_file"],))
        comm_sim.start()
    # Otherwise, import Xbee Python Library and set up callback
    else:
        from digi.xbee.devices import RemoteXBeeDevice, XBee64BitAddress
        xbee.add_data_received_callback(xbee_callback)

    # Generate waypoints after start_mission = True
    while not start_mission:
        pass

    # Generate waypoints
    waypoints = generateWaypoints(configs, search_area)
    num_points = len(waypoints[1])

    # Start SITL if vehicle is being simulated
    if (configs["vehicle_simulated"] is True):
        import dronekit_sitl
        sitl = dronekit_sitl.start_default(lat=1, lon=1)
        connection_string = sitl.connection_string()
    else:
        connection_string = "/dev/serial0"
        sitl = None

    # Connect to vehicle
    vehicle = connect(connection_string, wait_ready=True)

    # Send mission to vehicle
    adds_mission(vehicle, waypoints[1])

    # Takeoff
    takeoff(configs["flight_mode"], vehicle, configs["altitude"])

    vehicle.mode = VehicleMode(configs["flight_mode"])

    # Fly about spiral pattern
    if (configs["flight_mode"] == "AUTO"):
        while vehicle.commands.next != vehicle.commands.count:
            # TODO monitor pause, resume, stop variables
            print(vehicle.location.global_frame)
            time.sleep(1)
    else:
        raise Exception("Guided mode not supported")

    # TODO: land()
    
    # Wait for comm simulation thread to end
    if comm_sim:
        comm_sim.join()
