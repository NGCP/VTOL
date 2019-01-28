import sys
from detailed_search_autonomy import detailed_search_autonomy
from autonomy import setup_xbee
from util import parse_configs

def detailed_search():
    # Parse configs file
    configs = parse_configs(sys.argv)

    if configs["comms_simulated"]["toggled_on"]:
        xbee = None
	# Set up XBee device if communications not simulated
    else:
        xbee = setup_xbee()

    # Start autonomy thread
    detailed_search_autonomy(configs, xbee)

if __name__ == "__main__":
    detailed_search()
