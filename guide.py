from dronekit import connect, VehicleMode, LocationGlobalRelative
import math
import dronekit_sitl
import time


def xbee_callback(message):
    global start_mission
    global destination
    global continue_path

    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data.decode("utf8"))
    print("Received data from %s: %s" % (address, msg))

    try:
        msg_type = msg["type"]

        if msg_type == "start":
            start_mission = True
            acknowledge(address, msg_type)
            destination = msg["destination"]

        elif msg_type == "continue":
            coontinue_path = True
            acknowledge(address, msg_type)

    except KeyError as e:
        bad_msg(address, e.args[0])


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def guide():
    print "Start simulator (SITL)"
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

    # Import DroneKit-Python

    # Connect to the Vehicle.
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)

    # Get some vehicle attributes (state)
    print "Get some vehicle attribute values:"
    print " GPS: %s" % vehicle.gps_0
    print " Battery: %s" % vehicle.battery
    print " Last Heartbeat: %s" % vehicle.last_heartbeat
    print " Is Armable?: %s" % vehicle.is_armable
    print " System status: %s" % vehicle.system_status.state
    print " Mode: %s" % vehicle.mode.name    # settable

    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "waiting for coordinates"
    while not start_mission:
        time.sleep(.5)

    print "Taking off"
    alt = 20  # Talk to Pomona about this
    # revisit this (Talking with Pomona about incements based on ration vs distance)
    increments = 4
    pauses = 1
    groundspeed = 5
    continue_path = False

    vehicle.simple_takeoff(alt)  # Take off to target altitude

    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= alt*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

    vehicle.groundspeed = groundspeed
    initial = vehicle.location.global_relative_frame

    d_lat = (initial.lat - destination.lat) / increments
    d_lon = (initial.lon - destination.lon) / increments
    d_alt = (initial.alt - destination.alt) / increments

    while pauses <= increments:
        lat = vehicle.location.global_relative_frame.lat + d_lat
        lon = vehicle.location.global_relative_frame.lon + d_lon
        alt = vehicle.location.global_relative_frame.alt + d_alt

        loc = LocationGlobalRelative(lat, lon, alt)
        print(vehicle.location.global_relative_frame)
        vehicle.simple_goto(loc)
        while True:
            # Break and return from function just below target altitude.
            if get_distance_metres(
                    loc, vehicle.location.global_relative_frame) < .5:
                print "Reached ", pauses, " / ", increments
                break
            print(vehicle.location.global_relative_frame)
            time.sleep(1)
        print " waiting"

        while not continue_path:
            print(vehicle.location.global_relative_frame)
            time.sleep(1)
        pauses += 1

    # Land vehicle procedure ??
    vehicle.close()

    # Shut down simulator
    sitl.stop()
    print("Completed")
