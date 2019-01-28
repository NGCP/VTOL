import sys
import subprocess
import time
from threading import Thread, Lock
from serial import SerialException
from quick_scan_autonomy import quick_scan_autonomy
from quick_scan_cv import quick_scan_cv
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


def quick_scan():
    # Parse configs file
    configs = parse_configs(sys.argv)

    if configs["comms_simulated"]["toggled_on"]:
        xbee = None
    # Set up XBee device if communications not simulated
    else:
        xbee = setup_xbee()

    # Start autonomy and CV threads
    autonomyToCV = AutonomyToCV()
    autonomy_thread = Thread(target = quick_scan_autonomy, args = (configs, xbee, autonomyToCV))
    autonomy_thread.daemon = True
    autonomy_thread.start()

    cv_thread = Thread(target = quick_scan_cv, args = (configs, autonomyToCV))
    cv_thread.daemon = True
    cv_thread.start()

    # Wait for the threads to finish
    autonomy_thread.join()
    cv_thread.join()


if __name__ == "__main__":
    quick_scan()
