import json, sys
from digi.xbee.devices import XBeeDevice
from autonomy import detailed_search_autonomy, setup_xbee
from utilities import parse_configs


def detailed_search():

    # Parse configs file
    configs = parse_configs(sys.argv)

    # Setup xBee device
    if configs["comms_simulated"] == False:
        xbee = setup_xbee()
    else:
        xbee = None

    # Start autonomy thread
    detailed_search_autonomy(configs, xbee)

    # Initialize global queue for POIs
    global POI_queue


if __name__ == "__main__":
    detailed_search()
