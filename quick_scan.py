import json
import sys
import subprocess
import time
from digi.xbee.devices import XBeeDevice
from serial import SerialException
from autonomy import autonomy
import pymap3d as pm


# Instantiates XBee device object
def setup_xbee():
    # Continues looking until connection is found
    while (1):
        try:
            port_name = ""

            if sys.platform == "darwin":
                port_name = mac_xbee_port_name()
            elif sys.platform == "linux" or sys.platform == "linux2":
                port_name = "/dev/ttyUSB0"
            # TODO: figure out windows port name
            elif sys.platform == "win32":
                port_name = "COMS1"

            # Instantiate XBee device object.
            xbee = XBeeDevice(port_name, 57600)
            xbee.open()
            return xbee

        # FileNotFoundError and SerialException are thrown on Linux machines
        # if error with opening XBee device, mac_xbee_port_name() will throw
        # a ValueError if port name could not be found on a Mac
        except (FileNotFoundError, SerialException, ValueError) as e:
            print(e)
            print("Connect the XBee radio!")
            time.sleep(5)


# Looks in /dev directory for connected XBee serial port name on a macOS.
def mac_xbee_port_name():
    try:
        # System call to get port name of connected XBee radio
        port_name = subprocess.run(["ls", "/dev/"], stdout=subprocess.PIPE, encoding="utf-8").stdout
        i = port_name.index("tty.usbserial-")  # index in dev directory of port name
        return "/dev/" + port_name[i: i + 22]  # 22 is length of "tty.usbserial-" + 8-char port name

    except ValueError:
        raise ValueError("Value Error: \'tty.usbserial-\' not found in /dev")


def parse_configs(argv):
    """Parses the .json file given as the first command line argument.
    If no .json file is specified, defaults to "configs.json".
    If the specified .json file is found, returns a dict of the contents.
    Otherwise, raises a FileNotFoundError."""
    try:
        if (len(argv) == 1):
            read_file = open("configs.json", "r")
        else:
            read_file = open(argv[1], "r")

        return json.load(read_file)

    except FileNotFoundError:
        raise


# Generate waypoints for VTOL
def generateWaypoints(origin, radius):
    print("Begin generating waypoints")

    configs = parse_configs(sys.argv)

    waypointsNED = []
    waypointsLLA = []

    altitude = vehicle.location.global_frame.alt
    direction = 1
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    centerX, centerY, centerZ = pm.geodetic2ecef(lat, lon, altitude)

    fovH = math.radians(62.2)     # raspicam horizontal FOV
    fovV = math.radians(48.8)     # raspicam vertical FOV
    boxH = 2 * altitude / math.tan(fovH / 2)     # height of bounding box
    boxV = 2 * altitude / math.tan(fovV / 2)     # width of bounding box

    coils = (configs["coil_lower"] + configs["coil_upper"]) / 2  # number of complete rotations - hardcoded at the moment
    chord = boxV * 0.75     # distance between points, with 25% overlap
    rotation = -(math.pi / 2)   # number of radians spiral is rotated
    maxTheta = coils * 2 * math.pi  # theta for last coil
    awayStep = radius / maxTheta    # how far to step away from center

    n, e, d = pm.ecef2ned(centerX, centerY, centerZ, lat, lon, altitude)
    waypointsLLA.append([lat, lon, altitude])
    waypointsNED.append([n, e, d])

    theta = chord / awayStep

    while theta <= maxTheta:
        # distance away from center
        away = awayStep * theta

        # distance around center
        around = direction * theta + rotation

        # around & away -> x & y
        x = centerX + math.cos(around) * away
        y = centerY + math.sin(around) * away

        # convert ECEF to NED and LLA
        n, e, d = pm.ecef2ned(x, y, z, lat, lon, altitude)
        newLat, newLon, newAlt = pm.ned2geodetic(n, e, d, lat, lon, altitude);

        waypointsNED.append([n, e, d])
        waypointsLLA.append([newLat, newLon, newAlt])

        theta += chord / away

    return (waypointsNED, waypointsLLA)


def quick_scan():
    # Parse configs file
    configs = parse_configs(sys.argv)

    # Setup XBee device if communications not simulated
    if configs["comms_simulated"] == False:
        xbee = setup_xbee()
    else:
        xbee = None

    # Start autonomy thread
    autonomy(configs, xbee)


if __name__ == "__main__":
    quick_scan()
