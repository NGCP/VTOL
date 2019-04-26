import json
import math
import time
from conversions import *
from threading import Thread
from pymavlink import mavutil
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
from detailed_search import detailed_search

# first import gives access to global variables in "autonomy" namespace
# second import is for functions
import autonomy
from autonomy import comm_simulation, acknowledge, bad_msg, takeoff, land, update_thread, \
    change_status, setup_xbee, start_auto_mission, setup_vehicle

search_area = None  # search area object, populated by callback on start


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


# Callback function for messages from GCS, parses JSON message and sets globals
def xbee_callback(message, autonomyToCV):
    global search_area

    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data.decode("utf8"))
    print("Received data from %s: %s" % (address, msg))

    try:
        msg_type = msg["type"]

        if msg_type == "addMission":
            area = msg["missionInfo"]["searchArea"]
            search_area = SearchArea(area["center"], area["rad1"], area["rad2"])
            autonomy.start_mission = True
            acknowledge(address, msg["id"])

        elif msg_type == "pause":
            autonomy.pause_mission = True
            acknowledge(address, msg["id"])

        elif msg_type == "resume":
            autonomy.pause_mission = False
            acknowledge(address, msg["id"])

        elif msg_type == "stop":
            autonomy.stop_mission = True
            acknowledge(address, msg["id"])

        elif msg_type == "ack":
            autonomy.ack_id = msg["ackid"]

        else:
            bad_msg(address, "Unknown message type: \'" + msg_type + "\'", autonomyToCV)

    # KeyError if message was missing an expected key
    except KeyError as e:
        bad_msg(address, "Missing \'" + e.args[0] + "\' key", autonomyToCV)


# Generate NED and LLA waypoints in spiral pattern
def generate_waypoints(configs, search_area):
    print("Begin generating waypoints")

    waypointsNED = []
    waypointsLLA = []

    origin = search_area.center
    radius = search_area.rad2

    # pre-defined in configs file
    altitude = configs["altitude"]
    lat = origin[0]
    lon = origin[1]
    centerX, centerY, centerZ = geodetic2ecef(lat, lon, altitude)
    n, e, d = ecef2ned(centerX, centerY, centerZ, lat, lon, altitude)
    waypointsLLA.append(LocationGlobalRelative(lat, lon, altitude))
    waypointsNED.append([n, e, d])

    fovH = math.radians(62.2)  # raspicam horizontal FOV
    boxH = 2 * altitude / math.tan(fovH / 2)  # height of bounding box

    b = (0.75 * boxH) / (2 * math.pi)  # distance between coils with 25% overlap
    rotation = -(math.pi / 2)  # number of radians spiral is rotated
    maxTheta = radius / b  # after using formula r = b * theta

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
        waypointsLLA.append(LocationGlobalRelative(newLat, newLon, altitude))

        # generate a waypoint every pi/8 radian
        theta += configs["d_theta_rad"]

    return (waypointsNED, waypointsLLA)

def quick_scan_adds_mission(configs, vehicle, lla_waypoint_list):
    """
    Adds a takeoff command and four waypoint commands to the current mission.
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state
    (you must have called download at least once in the session and after clearing the mission)
    """
    # Declare shorthands
    altitude = configs["altitude"]
    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear()

    print(" Define/add new commands.")

    # Due to a bug presumed to be the fault of DroneKit, the first command is ignored. Thus we have two takeoff commands
    if (configs["vehicle_type"] == "VTOL"):
        # Separate MAVlink message for a VTOL takeoff. This takes off to altitude and transitions
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_VTOL_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))

    elif (configs["vehicle_type"] == "Quadcopter"):
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, altitude))

    # Set the target speed of vehicle
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0,
                     0, configs["air_speed"], -1, 0, 0, 0, 0))

    # Add waypoints to the auto mission
    if (configs["vehicle_type"] == "VTOL"):
        for point in lla_waypoint_list:
            # Planes need a waypoint tolerance
            cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                            0, configs["waypoint_tolerance"], 0, 0, point.lat, point.lon, point.alt))
            # Adds dummy end point - this endpoint is the same as the last waypoint and lets us know we have reached destination.
            cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                             0, configs["waypoint_tolerance"], 0, 0, lla_waypoint_list[-1].lat, lla_waypoint_list[-1].lon, lla_waypoint_list[-1].alt))
    elif (configs["vehicle_type"] == "Quadcopter"):
        for point in lla_waypoint_list:
            cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                             0, 0, 0, 0, point.lat, point.lon, point.alt))
            # Adds dummy end point - this endpoint is the same as the last waypoint and lets us know we have reached destination.
            cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                             0, 0, 0, 0, lla_waypoint_list[-1].lat, lla_waypoint_list[-1].lon, lla_waypoint_list[-1].alt))
                
    print("Upload new commands to vehicle")
    cmds.upload()


# Main autonomous flight thread
# :param configs: dict from configs file
def quick_scan_autonomy(configs, autonomyToCV, gcs_timestamp, connection_timestamp):
    print("\n######################## STARTING QUICK SCAN AUTONOMY ########################")
    autonomy.configs = configs

    # If comms is simulated, start comm simulation thread
    if configs["quick_scan_specific"]["comms_simulated"]["toggled_on"]:
        comm_sim = Thread(target=comm_simulation, args=(configs["quick_scan_specific"]["comms_simulated"]["comm_sim_file"], xbee_callback,))
        comm_sim.start()
    # Otherwise, set up XBee device and add callback
    else:
        comm_sim = None
        autonomy.xbee = setup_xbee()

        #store xbee to autonomyToCV
        autonomyToCV.xbeeMutex.acquire()
        autonomyToCV.xbee = autonomy.xbee
        autonomyToCV.xbeeMutex.release()

        autonomy.gcs_timestamp = gcs_timestamp
        autonomy.connection_timestamp = connection_timestamp


        autonomyToCV.xbeeMutex.acquire()
        autonomy.xbee.add_data_received_callback(xbee_callback)
        autonomyToCV.xbeeMutex.release()

    # Generate waypoints after start_mission = True
    while not autonomy.start_mission:
        pass

    # Generate waypoints
    waypoints = generate_waypoints(configs, search_area)

    # Connect to vehicle
    vehicle = setup_vehicle(configs)
    autonomyToCV.vehicleMutex.acquire()
    autonomyToCV.vehicle = vehicle
    autonomyToCV.vehicleMutex.release()

    # Starts the update thread
    update = Thread(target=update_thread, args=(vehicle, configs["mission_control_MAC"], autonomyToCV))
    update.start()

    # Send mission to vehicle
    quick_scan_adds_mission(configs, vehicle, waypoints[1])

    # Start the mission
    start_auto_mission(configs, vehicle)

    # Change vehicle status to running
    change_status("running")

    # Fly about spiral pattern
    set_autonomytocv_start(autonomyToCV, True)
    while vehicle.commands.next != vehicle.commands.count:
        print(vehicle.location.global_relative_frame)
        time.sleep(1)
        # Holds the copter in place if receives pause
        if autonomy.pause_mission:
            if (configs["vehicle_type"] == "VTOL"):
                vehicle.mode = VehicleMode("QHOVER")
            elif (configs["vehicle_type"] == "Quadcopter"):
                vehicle.mode = VehicleMode("ALT_HOLD")
        # Lands the vehicle if receives stop mission
        elif autonomy.stop_mission:
            set_autonomytocv_stop(autonomyToCV, True)
            land(configs, vehicle)
    
            return

    set_autonomytocv_stop(autonomyToCV, True)

    # Switch to detailed search if role switching is enabled
    if configs["quick_scan_specific"]["role_switching"]:
        autonomy.mission_completed = True
        update.join()
        detailed_search(vehicle)
    else:
        land(configs, vehicle)

        # Ready for a new mission
        autonomy.mission_completed = True

        # Wait for update thread to end
        update.join()

    # Wait for comm simulation thread to end
    if comm_sim:
        comm_sim.join()
    else:
        autonomyToCV.xbeeMutex.acquire()
        autonomy.xbee.close()
        autonomyToCV.xbeeMutex.release()

def set_autonomytocv_stop(autonomyToCV, stop):
    autonomyToCV.stopMutex.acquire()
    autonomyToCV.stop = stop
    autonomyToCV.stopMutex.release()

def set_autonomytocv_start(autonomyToCV, start):
    autonomyToCV.startMutex.acquire()
    autonomyToCV.start = start
    autonomyToCV.startMutex.release()

