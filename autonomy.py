import json, subprocess, time, sys
from serial import SerialException
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice

# Globals, updated by xBee callback function
start_mission = False  # takeoff
pause_mission = False  # vehicle will hover
stop_mission = False  # return to start and land
search_area = None  # search area object, populated by callback on start
xbee = None  # XBee radio object

# Represents a search area for quick scan aerial vehicles
class SearchArea:
    def __init__(self, center, rad1, rad2):
        self.center = center # tuple containing coor. of center point
        self.rad1 = rad1 # first search radius from center
        self.rad2 = rad2 # second search radius from center

    def __str__(self):
        return "SearchArea(" + \
            str(self.center) +  ", " + \
            str(self.rad1) + ", " + \
            str(self.rad2) + ")"
            
# Instantiates XBee device object
def setup_xbee():
    # Continues looking until connection is found
    while (1):
        try:
            port_name = ""

            if sys.platform == "darwin":
                port_name = mac_xbee_port_name()
            elif sys.platform == "linux" or sys.platform == "linux2":
                port_name = "/dev/ttyUSB0"
            # TODO: figure out windows port name
            elif sys.platform == "win32":
                port_name = "COMS1"

            # Instantiate XBee device object.
            xbee = XBeeDevice(port_name, 57600)
            xbee.open()
            return xbee

        # FileNotFoundError and SerialException are thrown on Linux machines
        # if error with opening XBee device, mac_xbee_port_name() will throw
        # a ValueError if port name could not be found on a Mac
        except (FileNotFoundError, SerialException, ValueError) as e:
            print(e)
            print("Connect the XBee radio!")
            time.sleep(5)


# Looks in /dev directory for connected XBee serial port name on a macOS.
def mac_xbee_port_name():
    try:
        # System call to get port name of connected XBee radio
        port_name = subprocess.run(["ls", "/dev/"], stdout=subprocess.PIPE, encoding="utf-8").stdout
        i = port_name.index("tty.usbserial-")  # index in dev directory of port name
        return "/dev/" + port_name[i: i + 22]  # 22 is length of "tty.usbserial-" + 8-char port name

    except ValueError:
        raise ValueError("Value Error: \'tty.usbserial-\' not found in /dev")

# Callback function for messages from GCS, parses JSON message and sets globals
def quick_scan_callback(message):
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

def detailed_search_callback(message):
    #TODO implement detailed_search_callback here
    return


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


# Main autonomous flight thread
# :param configs: dict from configs file
# :param radio: xBee radio object
def quick_scan_autonomy(configs, radio):
    global xbee
    xbee = radio

    # Add the callback.
    xbee.add_data_received_callback(quick_scan_callback)

def detailed_search_autonomy(configs, radio):
    global xbee
    xbee = radio

    # Add the callback.
    xbee.add_data_received_callback(detailed_search_callback)
