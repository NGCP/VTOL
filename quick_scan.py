import json, sys, subprocess, time
from digi.xbee.devices import XBeeDevice
from autonomy import autonomy


# Instantiates XBee device object
def setup_xbee(configs):
    xbee = None

    # Only set-up XBee radio if not simulated
    if configs["comms_simulated"] == False:
        os = configs["os"]
        port_name = ""

        if os == "Mac":
            # System call to get port name of connected XBee radio
            port_name = subprocess.run(["ls", "/dev/"], stdout=subprocess.PIPE, encoding="utf-8").stdout
            try:
                i = port_name.index("tty.usbserial-") # index in dev directory of port name
                port_name = "/dev/" + port_name[i: i + 22]
            except ValueError:
                print("Connect the XBee radio!")
                return
        elif os == "Linux":
            port_name = "/dev/ttyUSB0"
        # TODO: figure out windows port name
        elif os == "Windows":
            port_name = "COMS1"

        print(port_name)

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

    # Setup XBee device
    xbee = setup_xbee(configs)
    while xbee is None:
        time.sleep(5)
        xbee = setup_xbee(configs)

    # Start autonomy thread
    autonomy(configs, xbee)


if __name__ == "__main__":
    quick_scan()
