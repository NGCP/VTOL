import queue
import json
from threading import Thread
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
import math
from conversions import ecef2ned, geodetic2ecef, ned2geodetic
import time
from pymavlink import mavutil

# first import gives access to global variables in "autonomy" namespace
# second import is for functions
import autonomy
from autonomy import comm_simulation, acknowledge, bad_msg, takeoff, land, setup_xbee, \
    change_status, update_thread, start_auto_mission, setup_vehicle

# queue of points of interest
POI_queue = queue.Queue()


# Callback function for messages from GCS, parses JSON message and sets globals
def xbee_callback(message, autonomyToCV):
    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data)
    print("Received data from %s: %s" % (address, msg))

    try:
        msg_type = msg["type"]

        if msg_type == "addMission":
            msg_lat = msg['missionInfo']['lat']
            msg_lon = msg['missionInfo']['lng']

            # convert mission coordinates to dronekit object, and add to POI queue
            POI_queue.put(LocationGlobalRelative(msg_lat, msg_lon, None))
            acknowledge(address, msg["id"], autonomyToCV)

        elif msg_type == "pause":
            autonomy.PAUSE_MISSION = True
            acknowledge(address, msg["id"], autonomyToCV)

        elif msg_type == "resume":
            autonomy.PAUSE_MISSION = False
            acknowledge(address, msg["id"], autonomyToCV)

        elif msg_type == "stop":
            autonomy.STOP_MISSION = True
            acknowledge(address, msg["id"], autonomyToCV)

        elif msg_type == "ack":
            autonomy.ACK_ID = msg["ackid"]

        else:
            bad_msg(address, "Unknown message type: \'" + msg_type + "\'", autonomyToCV)

    # KeyError if message was missing an expected key
    except KeyError as e:
        bad_msg(address, "Missing \'" + e.args[0] + "\' key", autonomyToCV)


def orbit_poi(vehicle, poi, configs):
    waypoints = []  # waypoints in LLA
    altitude = configs["altitude"]
    poi_scan_altitude = configs["detailed_search_specific"]["poi_scan_altitude"]
    waypoint_tolerance = configs["waypoint_tolerance"]
    radius = configs["radius"]  # radius of circle travelled
    orbit_number = configs["orbit_number"]  # how many times we repeat orbit
    x, y, z = geodetic2ecef(poi.lat, poi.lon, poi_scan_altitude)  # LLA -> ECEF
    n, e, d = ecef2ned(x, y, z, poi.lat, poi.lon, poi_scan_altitude)  # ECEF -> NED
    cmds = vehicle.commands
    cmds.clear()

    # add circle of waypoints around the poi
    c = math.sqrt(2) / 2
    point_list = [[1, 0], [c, c], [0, 1], [-c, c], [-1, 0], [-c, -c], [0, -1], [c, -c], [1, 0]]
    for orbit in range(orbit_number):
        for point in point_list:
            a = (radius * point[0]) + n
            b = (radius * point[1]) + e
            lat, lon, alt = ned2geodetic(a, b, d, poi.lat, poi.lon, poi_scan_altitude)  # NED -> LLA
            waypoints.append(LocationGlobalRelative(lat, lon, alt))

    # Go to center of POI
    if (configs["vehicle_type"] == "HEX"):
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0, waypoint_tolerance, 0, 0, poi.lat, poi.lon, poi_scan_altitude))

    elif (configs["vehicle_type"] == "Hexcopter"):
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0, 0, 0, 0, poi.lat, poi.lon, poi_scan_altitude))

    # Transition to hexcopter if applicable
    if (configs["vehicle_type"] == "HEX"):
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_HEX_TRANSITION,
                    0, 0,
                    mavutil.mavlink.MAV_HEX_STATE_MC, 0, 0, 0, 0, 0, 0))

    # Circular waypoints
    if (configs["vehicle_type"] == "HEX"):
        for point in waypoints:
            cmds.add(
                Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                        0,
                        waypoint_tolerance, 0, 0, 0, point.lat, point.lon, point.alt))
    elif (configs["vehicle_type"] == "Hexcopter"):
        for point in waypoints:
            cmds.add(
                Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0,
                        0,
                        0, 0, 0, 0, point.lat, point.lon, point.alt))

    # Transition to forward flight if applicable
    if (configs["vehicle_type"] == "HEX"):
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_HEX_TRANSITION,
                    0, 0,
                    mavutil.mavlink.MAV_HEX_STATE_MC, 0, 0, 0, 0, 0, 0))

    # Add dummy endpoint
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                0, 0, 0, 0, poi.lat, poi.lon, poi_scan_altitude))

    print("Upload new commands to vehicle")
    cmds.upload()


def detailed_search_adds_mission(configs, vehicle):
    """
    Only adds a takeoff command for AUTO mission
    """
    # Declare shorthands
    altitude = configs["altitude"]
    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear()

    # Due to a bug presumed to be the fault of DroneKit, the first command is ignored. Thus we have two takeoff commands
    if (configs["vehicle_type"] == "HEX"):
        # Separate MAVlink message for a HEX takeoff. This takes off to altitude and transitions
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_HEX_TAKEOFF, 0,
                    0, 0, 0, 0, 0, 0, 0, altitude))
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_HEX_TAKEOFF, 0,
                    0, 0, 0, 0, 0, 0, 0, altitude))
    elif (configs["vehicle_type"] == "Hexcopter"):
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,
                    0, 0, 0, 0, 0, 0, altitude))
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,
                    0, 0, 0, 0, 0, 0, altitude))

    # Set the target speed of vehicle
    cmds.add(
        Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 0,
                0, configs["air_speed"], -1, 0, 0, 0, 0))

    cmds.upload()


def detailed_search_autonomy(configs, autonomyToCV, GCS_TIMESTAMP, CONNECTION_TIMESTAMP, vehicle=None):
    print("\n######################## STARTING DETAILED SEARCH AUTONOMY ########################")
    autonomy.configs = configs

    # If comms is simulated, start comm simulation thread
    if configs["detailed_search_specific"]["comms_simulated"]["toggled_on"]:
        comm_sim = Thread(target=comm_simulation, args=(
        configs["detailed_search_specific"]["comms_simulated"]["comm_sim_file"], xbee_callback, autonomyToCV))
        comm_sim.start()
    # Otherwise, set up XBee device and add callback
    else:
        comm_sim = None
        autonomy.xbee = setup_xbee()
        autonomy.GCS_TIMESTAMP = GCS_TIMESTAMP
        autonomy.CONNECTION_TIMESTAMP = CONNECTION_TIMESTAMP
        autonomy.xbee.add_data_received_callback(xbee_callback)

    # If detailed search was not role-switched from quick scan, connect to new vehicle and takeoff
    if not vehicle:
        vehicle = setup_vehicle(configs)

        # Starts the update thread
        update = Thread(target=update_thread, args=(vehicle, configs["mission_control_MAC"], autonomyToCV))
        update.start()

        # Add the takeoff command and start the takeoff mission
        detailed_search_adds_mission(configs, vehicle)
        start_auto_mission(configs, vehicle)

        # Wait until vehicle reaches minimum altitude
        while vehicle.location.global_relative_frame.alt < configs["altitude"] * 0.8:
            print("Altitude: ", vehicle.location.global_relative_frame.alt)
            time.sleep(1)
    else:
        # Starts the update thread
        update = Thread(target=update_thread, args=(vehicle, configs["mission_control_MAC"]))
        update.start()

    # Change vehicle status to running
    change_status("running")
    vehicle.mode = VehicleMode("GUIDED")

    # Continuously fly to POIs
    while not autonomy.STOP_MISSION:
        if not POI_queue.empty() and not autonomy.PAUSE_MISSION:
            poi = POI_queue.get()

            orbit_poi(vehicle, poi, configs)
            # Change flight mode to AUTO to start auto mission
            vehicle.commands.next = 0
            vehicle.mode = VehicleMode("AUTO")

            # print location while orbiting
            while vehicle.commands.next != vehicle.commands.count:
                if vehicle.commands.next > 1:
                    # TODO start CV scanning
                    pass

                print(vehicle.location.global_relative_frame)

                time.sleep(1)
            # TODO stop CV scanning

            # Change flight mode back
            vehicle.mode = VehicleMode("GUIDED")
        else:
            change_status("waiting")

            # Holds the copter in place if receives pause
        if autonomy.PAUSE_MISSION:
            if (configs["vehicle_type"] == "HEX"):
                vehicle.mode = VehicleMode("QHOVER")
            elif (configs["vehicle_type"] == "Hexcopter"):
                vehicle.mode = VehicleMode("ALT_HOLD")
            change_status("paused")

    land(configs, vehicle)

    # Sets vehicle status to "ready"
    change_status("ready")
    autonomy.mission_completed = True

    update.join()

    # Wait for comm simulation thread to end
    if comm_sim:
        comm_sim.join()
    else:
        autonomy.xbee.close()
