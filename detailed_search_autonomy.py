import queue
import json
from threading import Thread
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative

# first import gives access to global variables in "autonomy" namespace
# second import is for functions
import autonomy
from autonomy import comm_simulation, acknowledge, bad_msg, takeoff, land, setup_xbee

# queue of points of interest
POI_queue = queue.Queue()


# Callback function for messages from GCS, parses JSON message and sets globals
def xbee_callback(message):
    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data)
    print("Received data from %s: %s" % (address, msg))

    try:
        msg_type = msg["type"]

        if msg_type == "start":
            autonomy.start_mission = True
            acknowledge(address, msg_type)

        elif msg_type == "addMission":
            msg_lat = msg['missionInfo']['lat']
            msg_lon = msg['missionInfo']['lon']

            # convert mission coordinates to dronekit object, and add to POI queue
            POI_queue.put(LocationGlobalRelative(msg_lat, msg_lon, None))
            acknowledge(address, msg_type)

        elif msg_type == "pause":
            autonomy.pause_mission = True
            acknowledge(address, msg_type)

        elif msg_type == "resume":
            autonomy.pause_mission = False
            acknowledge(address, msg_type)

        elif msg_type == "stop":
            autonomy.stop_mission = True
            acknowledge(address, msg_type)

        else:
            bad_msg(address, "Unknown message type: \'" + msg_type + "\'")

    # KeyError if message was missing an expected key
    except KeyError as e:
        bad_msg(address, "Missing \'" + e.args[0] + "\' key")


def orbit_poi(vehicle, poi, configs):
    # TODO see orbit poi issue
    pass


def detailed_search_autonomy(configs, vehicle=None):
    print("\n######################## STARTING DETAILED SEARCH ########################")
    global POI_queue
    comm_sim = None

    # If comms is simulated, start comm simulation thread
    if configs["detailed_search_specific"]["comms_simulated"]["toggled_on"]:
        comm_sim = Thread(target=comm_simulation, args=(configs["detailed_search_specific"]["comms_simulated"]["comm_sim_file"], xbee_callback,))
        comm_sim.start()

    # Otherwise, set up XBee device and add callback
    else:
        autonomy.xbee = setup_xbee()
        autonomy.xbee.add_data_received_callback(xbee_callback)

    # Takeoff when the mission is started
    while not autonomy.start_mission:
        pass

    # If detailed search was not role-switched from quick scan, connect to new vehicle
    if not vehicle:
        # Start SITL if vehicle is being simulated
        if (configs["vehicle_simulated"]):
            import dronekit_sitl
            sitl = dronekit_sitl.start_default(lat=1, lon=1)
            connection_string = sitl.connection_string()
        else:
            connection_string = "/dev/serial0"

        # Connect to vehicle
        vehicle = connect(connection_string, wait_ready=True)
        takeoff(vehicle, configs["altitude"])
        vehicle.mode = VehicleMode("GUIDED")

    # Continuously fly to POIs
    while not autonomy.stop_mission:
        if not POI_queue.empty() and not autonomy.pause_mission:
            poi = POI_queue.get()
            # TODO start CV scanning
            orbit_poi(vehicle, poi, configs)
            # TODO stop CV scanning

        # Holds the copter in place if receives pause
        if autonomy.pause_mission:
            vehicle.mode = VehicleMode("ALT_HOLD")
        # Otherwise continue
        else:
            vehicle.mode = VehicleMode("GUIDED")

    land(vehicle)

    # Wait for comm simulation thread to end
    if comm_sim:
        comm_sim.join()
