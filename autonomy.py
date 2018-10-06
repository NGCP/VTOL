import json

# Globals, updated by xBee callback function
start_mission = False # takeoff
pause_mission = False # vehicle will hover
stop_mission = False # return to start and land
search_area = None # search area object, populated by callback on start
xbee = None # xBee radio object


# Represents a search area for quick scan aerial vehicles, each vehicle gets
# a "slice" of area with center point, radius, and start and end angle
class SearchArea:
    def __init__(self, center, radius, theta0, theta1):
        self.center = center # tuple containing coor. of center point
        self.radius = radius # search radius from center, i.e. margin of error
        self.theta0 = theta0 # first bounding angle
        self.theta1 = theta1 # second bounding angle


# Callback function for messages from GCS, parses JSON and sets globals
def xbee_callback(message):
    global start_mission
    global pause_mission
    global stop_mission
    global search_area

    address = message.remote_device.get_64bit_addr()
    data = message.data.decode("utf8")
    print("Received data from %s: %s" % (address, data))

    try:
        msg_type = message["type"]

        if msg_type == "start":
            # set search area
            area = message["searchArea"]
            search_area = SearchArea(area["center"], area["radius"], area["theta0"], area["theta1"])

            start_mission = True
            acknowledge(address)

        elif msg_type == "pause":
            pause_mission = True
            acknowledge(address)

        elif msg_type == "stop":
            stop_mission = True
            acknowledge(address)

    except KeyError as e:
        bad_msg(address, e.args[0])


# Sends message received acknowledgement to GCS
# :param address: address of GCS
def acknowledge(address):
    ack = {
        type: 'ack'
    }
    xbee.send_data(address, json.dumps(ack))


# Sends "bad message" to GCS if message received was poorly formatted/unreadable,
# also includes attribute attempted to retrieve
# :param address: address of GCS
# :param missing: attribute from message that caused KeyError
def bad_msg(address, missing):
    msg = {
        type: 'badMessage',
        missing: missing
    }
    xbee.send_data(address, json.dumps(msg))


# Main autonomous flight thread
# :param configs: dict from configs file
# :param radio: xBee radio object
def autonomy(configs, radio):
    global xbee
    xbee = radio

    # Add the callback.
    xbee.add_data_received_callback(xbee_callback)