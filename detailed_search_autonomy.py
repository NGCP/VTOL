import queue
from threading import Thread

# first import gives access to global variables in "autonomy" namespace
# second import is for functions
import autonomy
from autonomy import *

# queue of points of interests
POI_queue = queue.Queue()


# Callback function for messages from GCS, parses JSON message and sets globals
def xbee_callback(message):
    global start_mission
    global pause_mission
    global stop_mission

    print(message)

    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data)
    print("Received data from %s: %s" % (address, msg))

    try:
        msg_type = msg["type"]

        if msg_type == "start":
            start_mission = True
            acknowledge(address, msg_type)

        elif msg_type == "addMission":
            # handle for detailed_search
            pass

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


def detailed_search_autonomy(configs):
    comm_sim = None

    # If comms is simulated, start comm simulation thread
    if configs["comms_simulated"]["toggled_on"]:
        comm_sim = Thread(target=comm_simulation, args=(configs["comms_simulated"]["comm_sim_file"], xbee_callback,))
        comm_sim.start()

    # Set up XBee device if communications not simulated
    else:
        autonomy.xbee = setup_xbee()
        print("XBee radio set up!")

    # Generate waypoints after start_mission = True
    while not start_mission:
        pass

    # Wait for comm simulation thread to end
    if comm_sim:
        comm_sim.join()