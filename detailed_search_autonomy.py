import queue
import json
from threading import Thread
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative
import math
import conversions
import time
from pymavlink import mavutil

# first import gives access to global variables in "autonomy" namespace
# second import is for functions
import autonomy
from autonomy import comm_simulation, acknowledge, bad_msg, takeoff, land, setup_xbee, change_status, update_thread

# queue of points of interest
POI_queue = queue.Queue()


# Callback function for messages from GCS, parses JSON message and sets globals
def xbee_callback(message):
    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data)
    print("Received data from %s: %s" % (address, msg))

    try:
        msg_type = msg["type"]

        if msg_type == "addMission":
            msg_lat = msg['missionInfo']['lat']
            msg_lon = msg['missionInfo']['lon']

            # convert mission coordinates to dronekit object, and add to POI queue
            POI_queue.put(LocationGlobalRelative(msg_lat, msg_lon, None))
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

        elif msg_type == "acknowledge":
            # TODO check the ID
            pass

        else:
            bad_msg(address, "Unknown message type: \'" + msg_type + "\'")

    # KeyError if message was missing an expected key
    except KeyError as e:
        bad_msg(address, "Missing \'" + e.args[0] + "\' key")


def orbit_poi(vehicle, poi, configs):
    waypoints = []  # waypoints in LLA
    original_altitude = configs["altitude"]  # given altitude
    radius = configs["radius"]  # radius of circle travelled
    orbit_number = configs["orbit_number"]  # how many times we repeat orbit
    x, y, z = conversions.geodetic2ecef(poi.lat, poi.lon, original_altitude)  # LLA -> ECEF
    cmds = vehicle.commands
    cmds.clear()

    # add circle of waypoints around the poi
    c = math.sqrt(2) / 2
    point_list = [[1, 0], [c, c], [0, 1], [-c, c], [-1, 0], [-c, -c], [0, -1], [c, -c], [1, 0]]
    for orbit in range(orbit_number):
        for point in point_list:
            a = (radius * point[0]) + x
            b = (radius * point[1]) + y
            lat, lon, alt = conversions.ecef2geodetic(a, b, z)
            waypoints.append(LocationGlobalRelative(lat, lon, original_altitude))

    # Define the MAV_CMD_NAV_WAYPOINT locations and add the commands
    for point in waypoints:
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0, 0, 0, 0, 0, point.lat, point.lon, point.alt))
        print(point.lat, point.lon, point.alt)

    print("Upload new commands to vehicle")
    cmds.upload()


def detailed_search_autonomy(configs, autonomyToCV, gcs_timestamp, connection_timestamp, vehicle=None):
    print("\n######################## STARTING DETAILED SEARCH AUTONOMY ########################")
    autonomy.configs = configs

    # If comms is simulated, start comm simulation thread
    if configs["detailed_search_specific"]["comms_simulated"]["toggled_on"]:
        comm_sim = Thread(target=comm_simulation, args=(configs["detailed_search_specific"]["comms_simulated"]["comm_sim_file"], xbee_callback,))
        comm_sim.start()
    # Otherwise, set up XBee device and add callback
    else:
        comm_sim = None
        autonomy.xbee = setup_xbee()
        autonomy.gcs_timestamp = gcs_timestamp
        autonomy.connection_timestamp = connection_timestamp
        autonomy.xbee.add_data_received_callback(xbee_callback)

    # If detailed search was not role-switched from quick scan, connect to new vehicle and takeoff
    if not vehicle:
        # Start SITL if vehicle is being simulated
        if (configs["vehicle_simulated"]):
            import dronekit_sitl
            sitl = dronekit_sitl.start_default(lat=35.328423, lon=-120.752505)
            connection_string = sitl.connection_string()
        else:
            if (configs["3dr_solo"]):
                connection_string = "udpin:0.0.0.0:14550"
            else:
                connection_string = "/dev/serial0"

        # Connect to vehicle
        vehicle = connect(connection_string, wait_ready=True)

        # Starts the update thread
        update = Thread(target=update_thread, args=(vehicle, configs["mission_control_MAC"]))
        update.start()

        takeoff(vehicle, configs["altitude"])
    else:
        # Starts the update thread
        update = Thread(target=update_thread, args=(vehicle, configs["mission_control_MAC"]))
        update.start()
    
    # Change vehicle status to running
    change_status("running")

    # Continuously fly to POIs
    while not autonomy.stop_mission:
        if not POI_queue.empty() and not autonomy.pause_mission:
            poi = POI_queue.get()
            poi.alt = configs["altitude"]
            vehicle.simple_goto(poi)

            print("At POI, now orbiting")
            # TODO start CV scanning
            orbit_poi(vehicle, poi, configs)
            # Change flight mode to AUTO to start auto mission
            vehicle.mode = VehicleMode("AUTO")
            # print location while orbiting
            while vehicle.commands.next != vehicle.commands.count:
                print(vehicle.location.global_frame)
                print(vehicle.commands.next)
                print(vehicle.commands.count)
                time.sleep(1)
            # TODO stop CV scanning

            # Change flight mode back
            vehicle.mode = VehicleMode("GUIDED")
        else:
            change_status("waiting")    

        # Holds the copter in place if receives pause
        if autonomy.pause_mission:
            vehicle.mode = VehicleMode("ALT_HOLD")
            change_status("paused")
        # Otherwise continue
        else:
            vehicle.mode = VehicleMode("GUIDED")

    land(vehicle)

    # Sets vehicle status to "ready"
    change_status("ready")
    autonomy.mission_completed = True

    update.join()

    # Wait for comm simulation thread to end
    if comm_sim:
        comm_sim.join()
    else:
        autonomy.xbee.close()