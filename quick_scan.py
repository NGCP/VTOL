import sys
from quick_scan_autonomy import quick_scan_autonomy
from util import parse_configs


def quick_scan():
    # Parse configs file
    configs = parse_configs(sys.argv)

    # Start autonomy thread
    quick_scan_autonomy(configs)


if __name__ == "__main__":
    quick_scan()
