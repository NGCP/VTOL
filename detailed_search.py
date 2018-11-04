import json, sys
from digi.xbee.devices import XBeeDevice
from autonomy import detailed_search_autonomy, setup_xbee
from utilities import parse_json


def detailed_search():
    # Parse configs file
    configs = parse_json(sys.argv)

    # Setup xBee device
    xbee = setup_xbee(configs)

    # Start autonomy thread
    detailed_search_autonomy(configs, xbee)


if __name__ == "__main__":
    detailed_search()
