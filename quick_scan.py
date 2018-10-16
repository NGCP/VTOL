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
            port_name = "/dev/tty.usbserial-DJ00HY2G"
        elif os == "Linux":
            port_name = "/dev/ttyUSB0"
        # TODO: figure out windows port name
        elif os == "Windows":
            port_name = "COMS1"

        # Instantiate XBee device object.
        xbee = XBeeDevice(port_name, 57600)
        xbee.open()

    return xbee


def quick_scan():

    configs = {
        "vehicle_type": "Quadcopter",
        "vehicle_name": "A",
        "vehicle_simulated": True,
        "comms_simulated": False,
        "cv_simulated":  {
            "toggled_on": True,
            "directory": "some/path/for/now/i/will/add/in/later/when/testing"
        },
        "os": "Mac"
    }

    xbee = setup_xbee(configs)
    autonomy(configs, xbee)


if __name__ == "__main__":
    quick_scan()