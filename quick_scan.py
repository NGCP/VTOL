import sys
from quick_scan_autonomy import quick_scan_autonomy
from autonomy import setup_xbee
from util import parse_configs

def quick_scan():
    # Parse configs file
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
