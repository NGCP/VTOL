import json, sys
from digi.xbee.devices import XBeeDevice
from autonomy import autonomy


# Instantiates xBee device object
def setup_xbee(configs):
    xbee = None

    # Only set-up xBee radio if not simulated
    if configs["comms_simulated"] == False:
        os = configs["os"]
        port_name = ""

        if os == "Mac":
            port_name = configs["xbee_port_name"]
        elif os == "Linux":
            port_name = "/dev/ttyUSB0"
        # TODO: figure out windows port name
        elif os == "Windows":
            port_name = "COMS1"

        # Instantiate XBee device object.
        xbee = XBeeDevice(port_name, 57600)
        xbee.open()

    return xbee


def parse_json(argv):
    """Parses the .json file given as the first command line argument.
    If no .json file is specified, defaults to "configs.json".
    If the specified .json file is found, returns a dict of the contents.
    Otherwise, raises a FileNotFoundError."""
    try:
        if (len(argv) == 1):
            read_file = open("configs.json", "r")
        else:
            read_file =  open(argv[1], "r")

        return json.load(read_file)

    except FileNotFoundError:
        raise


def quick_scan():
    # Parse configs file
    configs = parse_json(sys.argv)

    # Setup xBee device
    xbee = setup_xbee(configs)

    # Start autonomy thread
    autonomy(configs, xbee)


if __name__ == "__main__":
    quick_scan()
