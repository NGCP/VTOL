import json
from digi.xbee.devices import RemoteXBeeDevice

# Globals, updated by xBee callback function
start_mission = False  # takeoff
pause_mission = False  # vehicle will hover
stop_mission = False  # return to start and land
search_area = None  # search area object, populated by callback on start
xbee = None  # XBee radio object


vehicle_position = None
heading = None
error_message = None
vehicle_status = "ready"    # Default status


class UpdateThread(Thread):
    """
    Generates and sends an update in JSON format to Mission Control
    """
    def __init__(self, event, configs):
        """
        Constructs UpdateThread object with the threading.Thread base class constructor

        :param event: threading.Event object that terminates thread when internal flag is set()
        :param configs: dict from configs file
        """
        Thread.__init__(self)
        self.stopped = event
        self.configs = configs


    def run(self):
        """
        Instantiates remote XBee device object to transmit updates every 0.5 seconds.
        Sends update while Event object's internal flag is defaulted to false.
        """
        remote_device = RemoteXBeeDevice(xbee,
            XBee64BitAddress.from_hex_string(configs["mission_control_MAC"]))
        update_json = ""

        while not self.stopped.wait(0.5):
            update_json = self.generate_update()
            xbee.send_data(remote_device,update_json)


    def generate_update(self):
        """
        Generate update based on global variables and configs dict.
        vehicle_status, vehicle_position, heading, error_message assumed to be global variables.

        :return: json update string
        """
        update_parameters = {"type": "update", "vehicleType": self.configs["vehicle_type"],
                             "status": vehicle_status}

        # TODO: implement battery as parameter

        # Remove if vehicle_position is calculated before this method is called
        if vehicle_position:
            update_parameters["lat"] = vehicle_position[0]
            update_parameters["lon"] = vehicle_position[1]

        if heading:
            update_parameters["heading"] = heading

        if error_message:
            update_parameters["errorMessage"] = error_message

        update_json = json.dumps(update_parameters)

        return update_json


    def send_update(self, remote_device, update_json):
        xbee.send_data(remote_device, update_json)


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
def xbee_callback(message):
    global start_mission
    global pause_mission
    global stop_mission
    global search_area

    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data.decode("utf8"))
    print("Received data from %s: %s" % (address, msg))

    try:
        msg_type = msg["type"]

        if msg_type == "start":
            start_mission = True
            acknowledge(address, msg_type)
            area = msg["searchArea"]
            search_area = SearchArea(area["center"], area["rad1"], area["rad2"])

        elif msg_type == "pause":
            pause_mission = True
            acknowledge(address, msg_type)

        elif msg_type == "resume":
            pause_mission = False
            acknowledge(address, msg_type)

        elif msg_type == "stop":
            stop_mission = True
            acknowledge(address, msg_type)

    except KeyError as e:
        bad_msg(address, e.args[0])


# Sends message received acknowledgement to GCS
# :param address: address of GCS
def acknowledge(address, received_type):
    ack = {
        "type": "ack",
        "received": received_type
    }
    # Instantiate a remote XBee device object to send data.
    send_xbee = RemoteXBeeDevice(xbee, address)
    xbee.send_data(send_xbee, json.dumps(ack))


# Sends "bad message" to GCS if message received was poorly formatted/unreadable,
# also includes attribute attempted to retrieve
# :param address: address of GCS
# :param missing: attribute from message that caused KeyError
def bad_msg(address, missing):
    msg = {
        "type": "badMessage",
        "missing": missing
    }
    # Instantiate a remote XBee device object to send data.
    send_xbee = RemoteXBeeDevice(xbee, address)
    xbee.send_data(send_xbee, json.dumps(msg))


def setup_update_thread(configs):
    """
    Instantiates an Event object as stop condition for thread and an UpdateThread object
    Starts Mission Control update thread.

    :param configs: dict from configs file
    :return: -1 if simulated test. None otherwise.
    """
    stop_flag = Event()

    if configs["coms_simulated"] == "false":
        update_thread = UpdateThread(stop_flag, configs)
        update_thread.start()
        
    else:
        print("This is a simulated test. Mission control updates are not being sent.")
        return -1


# In case thread needs to be terminated for testing
def stop_update_thread(stop_flag):
    stop_flag.set()


# Main autonomous flight thread
# :param configs: dict from configs file
# :param radio: XBee radio object
def autonomy(configs, radio):
    global xbee
    xbee = radio

    # Add the callback.
    xbee.add_data_received_callback(xbee_callback)

    setup_update_thread(configs)
