import sys
from quick_scan_autonomy import quick_scan_autonomy
from autonomy import setup_xbee
from util import parse_configs


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


def quick_scan(filename = ""):
    # If the optional argument is set, override command line argument
    if filename != "":
        configs = parse_configs(["", filename]);

    # Parse filename given in command line
    else:
        configs = parse_configs(sys.argv)

    if configs["comms_simulated"]["toggled_on"]:
        xbee = None
    # Set up XBee device if communications not simulated
    else:
        xbee = setup_xbee()

    # Start autonomy thread
    quick_scan_autonomy(configs, xbee)

if __name__ == "__main__":
    quick_scan()
