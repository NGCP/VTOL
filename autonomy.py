import json
import sys
import subprocess
import time
import cv2
import numpy
from dronekit import VehicleMode
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress

# Globals, updated by XBee callback function
start_mission = False  # takeoff
pause_mission = False  # vehicle will hover
stop_mission = False  # return to start and land
xbee = None  # XBee radio object

# Global status, updated by various functions
status = "ready"
heading = None
mission_completed = False


# Dummy message class for comm simulation thread to be compatible with xbee_callback function
class DummyMessage:
    def __init__(self, data=None):
        self.data = data  # UTF-8 encoded JSON message
        self.remote_device = DummyRemoteDevice()


# Dummy remote device object for use in DummyMessage
class DummyRemoteDevice:
    def __init__(self):
        pass

    def get_64bit_addr(self):
        return "comms simulation"


# Instantiates XBee device object
def setup_xbee():
    global xbee

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

        # If error in setting up XBee, try again
        except Exception as e:
            print(e)
            print("Connect the XBee radio!")
            time.sleep(5)


# Looks in /dev directory for connected XBee serial port name on a macOS.
def mac_xbee_port_name():
    try:
        # System call to get port name of connected XBee radio
        port_name = subprocess.check_output(["ls", "/dev/"])

        i = port_name.index("tty.usbserial-")  # index in dev directory of port name
        return "/dev/" + port_name[i: i + 22]  # 22 is length of "tty.usbserial-" + 8-char port name

    except ValueError:
        raise ValueError("Value Error: \'tty.usbserial-\' not found in /dev")

# Takes in a cv2 image object and sends it over the XBee radio to a given MAC address
def send_image(image, address):
    # converts the cv2 image object into a byte array so it can be sent over the radio
    image = bytearray(cv2.imencode('.jpg', image)[1])
    remote_device = RemoteXBeeDevice(xbee, XBee64BitAddress.from_hex_string(address))
    
    # Sends packets of 256 bytes in a loop until the whole image is sent
    # Maximum payload size is 256 bytes
    while len(image) > 256:
        xbee.send_data(remote_device, image[:256])
        image = image[256:]
    xbee.send_data(remote_device, image)

    # Sends the final message to tell the image has finished sending
    xbee.send_data(remote_device, "image finished")

def receive_image():
    image_buf = bytearray()
    xbee.flush_queues()

    # Polls for messages until the full image is received
    while True:
        xbee_message = xbee.read_data()
        if xbee_message is not None:
            message = xbee_message.data
            # Checks if the data is the ending message
            if message == "image finished":
                print("Image fully received")
                break
            # Concatenates the data to the current image buffer
            else:
                image_buf += message
    # Decodes the image buffer back into the cv2 image object
    decoded_image = numpy.frombuffer(image_buf, numpy.uint8)
    decoded_image = cv2.imdecode(decoded_image, cv2.IMREAD_COLOR)
    return decoded_image

# Reads through comm simulation file from configs and calls xbee_callback to simulate radio messages.
def comm_simulation(comm_file, xbee_callback):
   pass 


# Commands drone to take off by arming vehicle and flying to altitude
def takeoff(vehicle, altitude):
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
    vehicle.simple_takeoff(altitude)  # take off to altitude

    # Wait until vehicle reaches minimum altitude
    while vehicle.location.global_relative_frame.alt < altitude * 0.95:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)

    print("Reached target altitude")


# Commands vehicle to land
def land(vehicle):
    print("Returning to launch")
    vehicle.mode = VehicleMode("RTL")

    print("Closing vehicle object")
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
def comm_simulation(comm_file, xbee_callback):
    f = open(comm_file, "r")

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


# :param new_status: new vehicle status to change to (refer to GCS formatting)
def change_status(new_status):
    global status
    if new_status != "ready" and new_status != "running" and new_status != "waiting" and new_status != "paused" \
        and new_status != "error":
        raise Exception("Error: Unsupported status for vehicle")
    else:
        status = new_status


def include_heading():
    global heading
    heading = True


# :param vehicle: vehicle object that represents drone
# :param vehicle_type: vehicle type from configs file
def update_thread(vehicle, vehicle_type, address):
    print("Starting update thread\n")
    while not mission_completed:
        location = vehicle.location.global_frame
        battery_level = vehicle.battery.level / 100.0  # To comply with format of 0 - 1
        update_message = {
            "type": "update",
            "vehicleType": vehicle_type,
            "lat": location.lat,
            "lon": location.lon,
            "status": status,
            "battery": battery_level
        }

        if heading:
            update_message["heading"] = vehicle.heading

        if xbee:
            # Instantiate a remote XBee device object to send data.
            send_xbee = RemoteXBeeDevice(xbee, address)
            xbee.send_data(send_xbee, json.dumps(update_message))
        time.sleep(1)

    change_status("ready")
