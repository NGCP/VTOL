import json
import math
import sys
import subprocess
from conversions import *
import time
from threading import Thread
from serial import SerialException
from dronekit import connect, Command, VehicleMode, LocationGlobalRelative

# Globals, updated by xBee callback function
start_mission = False  # takeoff
pause_mission = False  # vehicle will hover
stop_mission = False  # return to start and land
xbee = None  # XBee radio object

# Dummy message class for comm simulation thread to be compatible with xbee_callback function
class DummyMessage:
    def __init__(self, data=None):
        self.data = data  # UTF-8 encoded JSON message
        self.remote_device = DummyRemoteDevice()


# Dummy remote device object for use in DummyMessage, has fake address 0
class DummyRemoteDevice:
    def __init__(self):
        pass

    def get_64bit_addr(self):
        return "comms simulation"


# Instantiates XBee device object
def setup_xbee():
    # Continues looking until connection is found
    from digi.xbee.devices import XBeeDevice
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
        process = subprocess.Popen(["ls", "/dev/"], stdout=subprocess.PIPE, encoding="utf-8")
        port_name, err = process.communicate()
        i = port_name.index("tty.usbserial-")  # index in dev directory of port name
        return "/dev/" + port_name[i: i + 22]  # 22 is length of "tty.usbserial-" + 8-char port name

    except ValueError:
        raise ValueError("Value Error: \'tty.usbserial-\' not found in /dev")


# Commands drone to take off by arming vehicle and flying to altitude
def takeoff(flight_mode, vehicle, altitude):
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
    vehicle.simple_takeoff(altitude) # take off to altitude

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
        from digi.xbee.devices import RemoteXBeeDevice, XBee64BitAddress
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
        from digi.xbee.devices import RemoteXBeeDevice, XBee64BitAddress
        # Instantiate a remote XBee device object to send data.
        send_xbee = RemoteXBeeDevice(xbee, address)
        xbee.send_data(send_xbee, json.dumps(msg))
    else:
        print("Error:", problem)


# Reads through comm simulation file from configs and calls xbee_callback to simulate radio messages.
def comm_simulation(comm_file, xbee_callback):
    try:
        f = open(comm_file, "r")
    except FileNotFoundError:
        raise

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

