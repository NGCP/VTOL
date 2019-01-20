import queue

# first import gives access to global variables in "autonomy" namespace
# second import is for functions
import autonomy
from autonomy import comm_simulation, acknowledge, bad_msg, adds_mission, takeoff, land

POI_queue = queue.Queue()

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
            start_mission = True

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

def detailed_search_autonomy():
    #TODO
    pass