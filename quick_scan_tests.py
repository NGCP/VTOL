from quick_scan import *
from autonomy import *
# from cv import *


# check lack of exceptions
def test_no_exceptions():
    print("Test exceptions in autonomy")
    configs = parse_configs(sys.argv)
    if configs["comms_simulated"]["toggled_on"]:
        xbee = None
    else:
        from digi.xbee.devices import XBeeDevice
        xbee = setup_xbee()
    autonomy(configs, xbee)

    print("Test exceptions in quick_scan")
    quick_scan()
    # cv()


# check if program reaches completion
def test_reach_completion():
    quick_scan()


if __name__ == "__main__":
    print("##### Test for no exceptions #####")
    test_no_exceptions()

    print("\n##### Test that program completes #####")
    test_reach_completion()
