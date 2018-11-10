import json, sys
from digi.xbee.devices import XBeeDevice
from serial import SerialException
from autonomy import quick_scan_autonomy, setup_xbee
from utilities import parse_configs

def quick_scan():
    # Parse configs file
    configs = parse_configs(sys.argv)

    # Setup XBee device if communications not simulated
    if configs["comms_simulated"] == False:
        xbee = setup_xbee()
    else:
        xbee = None

    # Start autonomy thread
    quick_scan_autonomy(configs, xbee)


if __name__ == "__main__":
    quick_scan()
