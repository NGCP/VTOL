import json
import sys
import subprocess
import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice

# Globals, updated by XBee callback function
start_mission = False  # takeoff
pause_mission = False  # vehicle will hover
stop_mission = False  # return to start and land
msg_id = 0  # unique ID increments for each message sent
ack_id = None
xbee = None  # XBee radio object
outfile = None

# Timestamps to keep track of the time field in messages to GCS
gcs_timestamp = 0
connection_timestamp = 0

# Global config dictionary
configs = None

# Global status, updated by various functions
status = "ready"
heading = None
mission_completed = False

# Writes to all file objects
class Tee(object):
    def __init__(self, *files):
        self.files = files

    def write(self, obj):
        for f in self.files:
            f.write(obj)

    def flush(self) :
        for f in self.files:
            f.flush()


# Dummy message class for comm simulation thread to be compatible with xbee_callback function
class DummyMessage:
    def __init__(self, data=None):
        self.data = data  # UTF-8 encoded JSON message
        self.remote_device = DummyRemoteDevice()


# Dummy remote device object for use in DummyMessage
class DummyRemoteDevice:
    def __init__(self):
        pass

    def get_64bit_addr(self):
        return "comms simulation"

def setup_vehicle(configs):
    # Start SITL if vehicle is being simulated
    if (configs["vehicle_simulated"]):
        if (configs["vehicle_type"] == "VTOL"):
            # If running a simulated VTOL on vagrant, connect to it via TCP
            # Port 5763 must be forwarded on vagrant
            connection_string = "tcp:127.0.0.1:5763"
        elif (configs["vehicle_type"] == "Quadcopter"):
            import dronekit_sitl
            sitl = dronekit_sitl.start_default(lat=35.328423, lon=-120.752505)
            connection_string = sitl.connection_string()
    else:
        if (configs["3dr_solo"]):
            connection_string = "udpin:0.0.0.0:14550"
        else:
            connection_string = "/dev/serial0"

    if (configs["vehicle_simulated"]):
        return connect(connection_string, wait_ready=True)
    else:
        return connect(connection_string, baud=configs["baud_rate"], wait_ready=True)

# Instantiates XBee device object
def setup_xbee():
    global xbee

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

        # If error in setting up XBee, try again
        except Exception as e:
            print(e)
            print("Connect the XBee radio!")
            time.sleep(5)


# Looks in /dev directory for connected XBee serial port name on a macOS.
def mac_xbee_port_name():
    try:
        # System call to get port name of connected XBee radio
        port_name = subprocess.check_output(["ls", "/dev/"])

        i = port_name.index("tty.usbserial-")  # index in dev directory of port name
        return "/dev/" + port_name[i: i + 22]  # 22 is length of "tty.usbserial-" + 8-char port name

    except ValueError:
        raise ValueError("Value Error: \'tty.usbserial-\' not found in /dev")


# Arms and starts an AUTO mission loaded onto the vehicle
def start_auto_mission(configs, vehicle):
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    vehicle.commands.next = 0
    vehicle.mode = VehicleMode("AUTO")

    if (configs["vehicle_type"] == "Quadcopter"):
        msg = vehicle.message_factory.command_long_encode(
            0, 0,    # target_system, target_component
            mavutil.mavlink.MAV_CMD_MISSION_START, #command
            0, #confirmation
            0, 0, 0, 0, 0, 0, 0)    # param 1 ~ 7 not used
        # send command to vehicle
        vehicle.send_mavlink(msg)

    vehicle.commands.next = 0


# Commands drone to take off by arming vehicle and flying to altitude
def takeoff(vehicle, altitude):
    print("Pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize")
        time.sleep(1)

    print("Arming motors")
    # Vehicle should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting to arm vehicle")
        time.sleep(1)

    print("Taking off")
    vehicle.simple_takeoff(altitude)  # take off to altitude

    # Wait until vehicle reaches minimum altitude
    while vehicle.location.global_relative_frame.alt < altitude * 0.95:
        print("Altitude: " + str(vehicle.location.global_relative_frame.alt))
        time.sleep(1)

    print("Reached target altitude")


# Commands vehicle to land
def land(configs, vehicle):
    print("Returning to launch")
    if (configs["vehicle_type"] == "VTOL"):
        vehicle.mode = VehicleMode("QRTL")
    elif (configs["vehicle_type"] == "Quadcopter"):
        vehicle.mode = VehicleMode("RTL")

    # Wait until vehicle reaches ground
    while not vehicle.location.global_relative_frame.alt < 1.0:
        print("Altitude: " + str(vehicle.location.global_relative_frame.alt))
        time.sleep(1)
    
    time.sleep(10)
    vehicle.close()


# Sends message received acknowledgement to GCS
# :param address: address of GCS
def acknowledge(address, ackid, autonomyToCV):
    ack = {
        "type": "ack",
        "time": round(time.clock() - connection_timestamp) + gcs_timestamp,
        "sid": configs['vehicle_id'],
        "tid": 0, # The ID of GCS
        "id": new_msg_id(),
        "ackid": ackid
    }
    # xbee is None if comms is simulated
    if xbee:
        # Instantiate a remote XBee device object to send data.
        send_xbee = RemoteXBeeDevice(xbee, address)
        packed_data = packb(json.dumps(ack), use_bin_type = True)
        autonomyToCv.xbeeMutex.acquire()
        xbee.send_data(send_xbee, packed_data)
        autonomyToCV.xbeeMutex.release()


# Sends "bad message" to GCS if message received was poorly formatted/unreadable
# and describes error from parsing original message.
# :param address: address of GCS
# :param problem: string describing error from parsing original message
def bad_msg(address, problem, autonomyToCV):
    msg = {
        "type": "badMessage",
        "time": round(time.clock() - connection_timestamp) + gcs_timestamp,
        "sid": configs['vehicle_id'],
        "tid": 0, # The ID of GCS
        "id": new_msg_id(),

        "error": problem
    }
    # xbee is None if comms is simulated
    if xbee:
        # Instantiate a remote XBee device object to send data.
        send_xbee = RemoteXBeeDevice(xbee, address)
        packed_data = packb(json.dumps(msg, use_bin_type = True))
        autonomyToCV.xbeeMutex.acquire()
        xbee.send_data(send_xbee, packed_Data)
        autonomyToCV.xbeeMutex.release()
    else:
        print("Error:", problem)


# Increments global msg_id and returns unique id for new message
def new_msg_id():
    global msg_id
    msg_id += 1
    return msg_id


def send_msg(address, msg):
    # Instantiate a remote XBee device object to send data.
    send_xbee = RemoteXBeeDevice(xbee, address)
    xbee.send_data(send_xbee, json.dumps(msg))


# Reads through comm simulation file from configs and calls xbee_callback to simulate radio messages.
def comm_simulation(comm_file, xbee_callback, autonomyToCV):
    comms = json.load(open(comm_file, "r"))  # reads the json file
    prev_time = 0
    for instr in comms:  # gets time and message from each json object (instruction)
        curr_time = instr["time"]
        time.sleep(curr_time - prev_time)  # waits for the next instruction
        # Send message to xbee_callback
        xbee_callback(DummyMessage(json.dumps(instr["message"])), autonomyToCV)
        prev_time = curr_time


# :param new_status: new vehicle status to change to (refer to GCS formatting)
def change_status(new_status):
    global status
    if new_status != "ready" and new_status != "running" and new_status != "waiting" and new_status != "paused" \
        and new_status != "error":
        raise Exception("Error: Unsupported status for vehicle")
    else:
        status = new_status


def include_heading():
    global heading
    heading = True


# :param vehicle: vehicle object that represents drone
# :param vehicle_type: vehicle type from configs file
def update_thread(vehicle, address, autonomyToCV):
    print("Starting update thread\n")

    while not mission_completed:
        location = vehicle.location.global_frame
        # Comply with format of 0 - 1 and check that battery level is not null
        battery_level = vehicle.battery.level / 100.0 if vehicle.battery.level else 0.0
        update_message = {
            "type": "update",
            "time": round(time.clock() - connection_timestamp) + gcs_timestamp,
            "sid": configs["vehicle_id"],
            "tid": 0, # the ID of the GCS is 0
            "id": new_msg_id(),

            "vehicleType": "VTOL",
            "lat": location.lat,
            "lon": location.lon,
            "status": status,
            # TODO heading
            "battery": battery_level
        }

        if heading:
            update_message["heading"] = vehicle.heading

        if xbee:
            # Instantiate a remote XBee device object to send data.
            autonomyToCV.xbeeMutex.acquire()
            send_till_ack(address, update_message, msg_id)
            autonomyToCV.xbeeMutex.release()

        time.sleep(1)

    change_status("ready")


# Continuously sends message to given address until acknowledgement message is recieved with the corresponding ackid.
def send_till_ack(address, msg, msg_id):
    # Instantiate a remote XBee device object to send data.
    send_xbee = RemoteXBeeDevice(xbee, address)
    packed_data = packb(json.dumps(ack), use_bin_type = True)
    while ack_id != msg_id:
        xbee.send_data(send_xbee, packed_data)
        time.sleep(1)
