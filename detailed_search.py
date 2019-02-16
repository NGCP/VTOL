import sys
import autonomy
from detailed_search_autonomy import detailed_search_autonomy
from util import parse_configs


def detailed_search():
    # Parse configs file
    configs = parse_configs(sys.argv)

    # Start autonomy thread
    detailed_search_autonomy(configs)

    # Close XBee device
    if autonomy.xbee:
        autonomy.xbee.close()


if __name__ == "__main__":
    detailed_search()
