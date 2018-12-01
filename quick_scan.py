import json
import sys
import subprocess
import time
from threading import Thread, Lock
from digi.xbee.devices import XBeeDevice
from serial import SerialException
from autonomy import autonomy
from cv import cv

class AutonomyToCV:
    def __init__(self):
        self.startMutex = Lock()
        self.start = False

        self.latMutex = Lock()
        self.lat = 0.0

        self.lonMutex = Lock()
        self.lon = 0.0

        self.altMutex = Lock()
        self.alt = 0.0

        self.northMutex = Lock()
        self.north = 0.0

        self.eastMutex = Lock()
        self.east = 0.0


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


def parse_configs(argv):
    """Parses the .json file given as the first command line argument.
    If no .json file is specified, defaults to "configs.json".
    If the specified .json file is found, returns a dict of the contents.
    Otherwise, raises a FileNotFoundError."""
    try:
        if (len(argv) == 1):
            read_file = open("configs.json", "r")
        else:
            read_file = open(argv[1], "r")

        return json.load(read_file)

    except FileNotFoundError:
        raise

def quick_scan():
    # Parse configs file
    configs = parse_configs(sys.argv)

    # Setup XBee device if communications not simulated
    if configs["comms_simulated"] == False:
        xbee = setup_xbee()
    else:
        xbee = None

    # Start autonomy thread
    autonomyToCV = AutonomyToCV()
    autonomy_thread = Thread(target = autonomy, args = (configs, xbee, autonomyToCV))
    cv_thread = Thread(target = cv, args = (configs, autonomyToCV))
    
    autonomy_thread.start()
    cv_thread.start()

    autonomy_thread.join()
    cv_thread.join()


if __name__ == "__main__":
    quick_scan()
