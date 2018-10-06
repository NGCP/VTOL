# Instantiates xBee device object
def setup_xbee(configs):
    xbee = None

    # Only set-up xBee radio if not simulated
    if configs["comms_simulated"] == False:
        os = configs["os"]
        port_name = ""

        if os == "Mac":
            port_name = "tty.usbserial-DA01R50T"
        elif os == "Linux":
            port_name = "/dev/ttyUSB0"
        # TODO: figure out windows port name
        elif os == "Windows":
            port_name = "COMS1"

        # Instantiate XBee device object.
        xbee = XBeeDevice(port_name, 57600)
        xbee.open()

    return xbee
