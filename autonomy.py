'''Automous tools for VTOL'''
import json
import sys
import subprocess
import time
from dronekit import connect, VehicleMode, Vehicle
from pymavlink import mavutil
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
import dronekit_sitl
import msgpack

def mac_xbee_port_name():
    '''Looks in /dev directory for connected XBee serial port name on a macOS.'''
    try:
        # System call to get port name of connected XBee radio
        port_name = subprocess.check_output(["ls", "/dev/"])

        # index in dev directory of port name
        i = port_name.index("tty.usbserial-")
        # 22 is length of "tty.usbserial-" + 8-char port name
        return "/dev/" + port_name[i: i + 22]

    except ValueError:
        raise ValueError("Value Error: \'tty.usbserial-\' not found in /dev")

class Tee():
    '''Writes to all file objects'''
    def __init__(self, *files):
        self.files = files

    def write(self, obj):
        '''writes files'''
        for file in self.files:
            file.write(obj)

    def flush(self):
        '''flushes files'''
        for file in self.files:
            file.flush()

class DummyMessage:
    '''Dummy msg class for comm simulation thread to be compatible with xbee_callback function'''
    def __init__(self, data=None):
        self.data = data  # UTF-8 encoded JSON message
        self.remote_device = DummyRemoteDevice()


class DummyRemoteDevice:
    '''Dummy remote device object for use in DummyMessage'''
    def __init__(self):
        self.get_64bit_addr = lambda _: 'comms simulation'

def setup_vehicle(configs):
    '''Sets up self as a vehicle'''
    #Start SITL if vehicle is being simulated
    if configs["vehicle_simulated"]:
        if configs["vehicle_type"] == "VTOL":
            # If running a simulated VTOL on vagrant, connect to it via TCP
            # Port 5763 must be forwarded on vagrant
            con_str = "tcp:127.0.0.1:5763"
        elif configs["vehicle_type"] == "Quadcopter":
            sitl = dronekit_sitl.start_default(lat=35.328423, lon=-120.752505)
            con_str = sitl.connection_string()
    else:
        if configs["3dr_solo"]:
            con_str = "udpin:0.0.0.0:14550"
        else:
            # connect to pixhawk via MicroUSB
            # if we switch back to using the telem2 port, use "/dev/serial0"
            con_str = "/dev/ttyACM0"

    if configs["vehicle_simulated"]:
        veh = connect(con_str, wait_ready=True, vehicle_class=VTOL)
    else:
        veh = connect(con_str, baud=configs["baud_rate"], wait_ready=True, vehicle_class=VTOL)
    veh.configs = configs

class VTOL(Vehicle):
    ''' VTOL basic state isolated'''
    def __init__(self, *args): #pylint: disable=useless-super-delegation
        super(VTOL, self).__init__(*args)

    # State, updated by XBee callback function
    configs = None
    start_mission = False  # takeoff
    pause_mission = False  # vehicle will hover
    stop_mission = False  # return to start and land
    msg_id = 0  # unique ID increments for each message sent
    ack_id = None
    xbee = None  # XBee radio object

    # Timestamps to keep track of the time field in messages to GCS
    GCS_TIMESTAMP = 0
    CONNECTION_TIMESTAMP = 0

    # Global status, updated by various functions
    status = "ready"
    heading = None
    MISSION_COMPLETED = False

    def setup_xbee(self):
        '''Instantiates XBee device object'''
        # Continues looking until connection is found
        while 1:
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
                self.xbee = XBeeDevice(port_name, 57600)
                self.xbee.open()

            # If error in setting up XBee, try again
            except TimeoutError as ex:
                print(ex)
                print("Connect the XBee radio!")
                time.sleep(5)

    def recieve(self):
        '''Instantiate XBee device'''
        try:
            self.xbee.flush_queues()
            print("Waiting for data...\n")
            while True:
                xbee_message = self.xbee.read_data()
                if xbee_message is not None:
                    print("Received '%s' from %s" % (xbee_message.data.decode(), \
                        xbee_message.remote_self.xbee.get_64bit_addr()))
        finally:
            if self.xbee is not None and self.xbee.is_open():
                self.xbee.close()


    def send(self, mac_address, data):
        '''sends data over xbee'''
        remote_device = RemoteXBeeDevice(self.xbee, XBee64BitAddress.from_hex_string(mac_address))

        if remote_device is None:
            print("Invalid MAC Address")
            return

        print("Sending '%s' to %s" % (data, remote_device.get_64bit_addr()))
        self.xbee.send_data(remote_device, data)
        print("Success")


    def start_auto_mission(self, configs):
        '''Arms and starts an AUTO mission loaded onto the vehicle'''
        while not self.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        self.mode = VehicleMode("GUIDED")
        self.armed = True

        while not self.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        self.commands.next = 0
        self.mode = VehicleMode("AUTO")

        if configs["vehicle_type"] == "Quadcopter":
            msg = self.message_factory.command_long_encode(
                0, 0,    # target_system, target_component
                mavutil.mavlink.MAV_CMD_MISSION_START, #command
                0, #confirmation
                0, 0, 0, 0, 0, 0, 0)    # param 1 ~ 7 not used
            # send command to vehicle
            self.send_mavlink(msg)

        self.commands.next = 0


    def takeoff(self, altitude):
        '''Commands drone to take off by arming vehicle and flying to altitude'''
        print("Pre-arm checks")
        while not self.is_armable:
            print("Waiting for vehicle to initialize")
            time.sleep(1)

        print("Arming motors")
        # Vehicle should arm in GUIDED mode
        self.mode = VehicleMode("GUIDED")
        self.armed = True

        while not self.armed:
            print("Waiting to arm vehicle")
            time.sleep(1)

        print("Taking off")
        self.simple_takeoff(altitude)  # take off to altitude

        # Wait until vehicle reaches minimum altitude
        while self.location.global_relative_frame.alt < altitude * 0.95:
            print("Altitude: " + str(self.location.global_relative_frame.alt))
            time.sleep(1)

        print("Reached target altitude")


    def land(self, configs):
        '''Commands vehicle to land'''
        print("Returning to launch")
        if configs["vehicle_type"] == "VTOL":
            self.mode = VehicleMode("QRTL")
        elif configs["vehicle_type"] == "Quadcopter":
            self.mode = VehicleMode("RTL")

        # Wait until vehicle reaches ground
        while not self.location.global_relative_frame.alt < 1.0:
            print("Altitude: " + str(self.location.global_relative_frame.alt))
            time.sleep(1)
        time.sleep(10)
        self.close()


    def acknowledge(self, address, ack_id, autonomy_to_cv):
        '''Sends message received acknowledgement to GCS
        param address: address of GCS'''
        ack = {
            "type": "ack",
            "time": round(time.clock() - self.CONNECTION_TIMESTAMP) + self.GCS_TIMESTAMP,
            "sid": self.configs['vehicle_id'],
            "tid": 0, # The ID of GCS
            "id": self.new_msg_id(),
            "ackid": ack_id
        }
        # xbee is None if comms is simulated
        if self.xbee:
            # Instantiate a remote XBee device object to send data.
            send_xbee = RemoteXBeeDevice(self.xbee, XBee64BitAddress.from_hex_string(address))
            packed_data = msgpack.packb(ack)
            autonomy_to_cv.xbeeMutex.acquire()
            self.xbee.send_data(send_xbee, packed_data)
            autonomy_to_cv.xbeeMutex.release()

    def bad_msg(self, address, problem, autonomy_to_cv):
        '''Sends "bad message" to GCS if message received was poorly formatted/unreadable
        and describes error from parsing original message.
        :param address: address of GCS
        :param problem: string describing error from parsing original message'''
        msg = {
            "type": "badMessage",
            "time": round(time.clock() - self.CONNECTION_TIMESTAMP) + self.GCS_TIMESTAMP,
            "sid": self.configs['vehicle_id'],
            "tid": 0, # The ID of GCS
            "id": self.new_msg_id(),

            "error": problem
        }
        # xbee is None if comms is simulated
        if self.xbee:
            # Instantiate a remote XBee device object to send data.
            send_xbee = RemoteXBeeDevice(self.xbee, XBee64BitAddress.from_hex_string(address))
            packed_data = msgpack.packb(msg)
            autonomy_to_cv.xbeeMutex.acquire()
            self.xbee.send_data(send_xbee, packed_data)
            autonomy_to_cv.xbeeMutex.release()
        else:
            print("Error:", problem)

    def new_msg_id(self):
        '''Increments msg_id and returns unique id for new message'''
        self.msg_id += 1
        return self.msg_id


    def send_msg(self, address, msg):
        '''Instantiate a remote XBee device object to send data.'''
        send_xbee = RemoteXBeeDevice(self.xbee, XBee64BitAddress.from_hex_string(address))
        self.xbee.send_data(send_xbee, json.dumps(msg))

    @classmethod
    def comm_simulation(cls, comm_file, xbee_callback, autonomy_to_cv):
        '''Reads through comm simulation file from configs and calls
        xbee_callback to simulate radio messages.'''
        comms = json.load(open(comm_file, "r"))  # reads the json file
        prev_time = 0
        for instr in comms:  # gets time and message from each json object (instruction)
            curr_time = instr["time"]
            time.sleep(curr_time - prev_time)  # waits for the next instruction
            # Send message to xbee_callback
            xbee_callback(DummyMessage(json.dumps(instr["message"])), autonomy_to_cv)
            prev_time = curr_time


    def change_status(self, new_status):
        ''':param new_status: new vehicle status to change to (refer to GCS formatting)'''
        if new_status not in ("ready", "running", "waiting", "paused", "error"):
            raise Exception("Error: Unsupported status for vehicle")
        self.status = new_status

    def include_heading(self):
        '''Includes heading in messages'''
        self.heading = True


    def update_thread(self, address, autonomy_to_cv):
        ''':param vehicle: vehicle object that represents drone
        :param vehicle_type: vehicle type from configs file'''
        print("Starting update thread\n")

        while not self.MISSION_COMPLETED:
            location = self.location.global_frame
            # Comply with format of 0 - 1 and check that battery level is not null
            battery_level = self.battery.level / 100.0 if self.battery.level else 0.0
            update_message = {
                "type": "update",
                "time": round(time.clock() - self.CONNECTION_TIMESTAMP) + self.GCS_TIMESTAMP,
                "sid": self.configs["vehicle_id"],
                "tid": 0, # the ID of the GCS is 0
                "id": self.new_msg_id(),

                "vehicleType": "VTOL",
                "lat": location.lat,
                "lon": location.lon,
                "status": self.status,
                # TODO heading
                "battery": battery_level
            }

            if self.heading:
                update_message["heading"] = self.heading

            if self.xbee:
                # Instantiate a remote XBee device object to send data.
                autonomy_to_cv.xbeeMutex.acquire()
                self.send_till_ack(address, update_message, self.msg_id)
                autonomy_to_cv.xbeeMutex.release()
            time.sleep(1)
        self.change_status("ready")


    def send_till_ack(self, address, msg, msg_id):
        '''Continuously sends message to given address until acknowledgement
        message is recieved with the corresponding ackid.'''
        # Instantiate a remote XBee device object to send data.
        send_xbee = RemoteXBeeDevice(self.xbee, XBee64BitAddress.from_hex_string(address))
        packed_data = bytearray(msgpack.packb(msg))
        while self.ack_id != msg_id:
            self.xbee.send_data(send_xbee, packed_data)
            time.sleep(1)
