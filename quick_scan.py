import json, sys
from digi.xbee.devices import XBeeDevice
from autonomy import quick_scan_autonomy, setup_xbee
from utilities import parse_json

def quick_scan():
    # Parse configs file
    configs = parse_json(sys.argv)

    # Setup xBee device
    xbee = setup_xbee(configs)

    # Start autonomy thread
    quick_scan_autonomy(configs, xbee)


if __name__ == "__main__":
    quick_scan()
