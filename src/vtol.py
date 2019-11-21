'''Automous tools for VTOL'''
import time
import json
from dronekit import connect, VehicleMode, Vehicle, LocationGlobalRelative
from pymavlink import mavutil
import dronekit_sitl
from coms import Coms
from util import get_distance_metres

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
    veh.setup_coms()
    return veh


class VTOL(Vehicle):
    ''' VTOL basic state isolated'''
    def __init__(self, *args): #pylint: disable=useless-super-delegation
        super(VTOL, self).__init__(*args)

    # State, updated by XBee callback function
    configs = None
    start_mission = False  # takeoff
    pause_mission = False  # vehicle will hover
    stop_mission = False  # return to start and land

    # Global status, updated by various functions
    status = "ready"
    heading = None
    MISSION_COMPLETED = False
    coms = None

    # pylint: disable=no-self-use
    def coms_callback(self, message):
        '''callback for radio messages'''
        parsed_message = json.loads(message.data)
        #tuple of commands that can be executed
        valid_commands = ("takeoff", "RTL")
        #gives us the specific command we want the drone to executre
        command = parsed_message['type']

        print('Recieved message type:', type(parsed_message['type']))

        #checking for valid command
        if command not in valid_commands:
            raise Exception("Error: Unsupported status for vehicle")

        #executes takeoff command to drone
        if command == 'takeoff':
            self.takeoff()
        #executes land command to drone
        elif command == 'land':
            self.land()

        # TODO respond to xbee messagge
        data = json.loads(message.data)
        print(data['type'])

    def setup_coms(self):
        '''sets up communication radios'''
        # TODO set up coms and callback
        print('Initializing Coms')
        self.coms = Coms(self.configs, self.coms_callback)


    def start_auto_mission(self):
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

        if self.configs["vehicle_type"] == "Quadcopter":
            msg = self.message_factory.command_long_encode(
                0, 0,    # target_system, target_component
                mavutil.mavlink.MAV_CMD_MISSION_START, #command
                0, #confirmation
                0, 0, 0, 0, 0, 0, 0)    # param 1 ~ 7 not used
            # send command to vehicle
            self.send_mavlink(msg)

        self.commands.next = 0


    def takeoff(self):
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

        altitude = self.configs['altitude']
        self.simple_takeoff(altitude)  # take off to altitude

        # Wait until vehicle reaches minimum altitude
        while self.location.global_relative_frame.alt < altitude * 0.95:
            print("Altitude: " + str(self.location.global_relative_frame.alt))
            time.sleep(1)

        print("Reached target altitude")

    def go_to(self, point):
        '''Commands drone to fly to a specified point perform a simple_goto '''

        self.simple_goto(point, self.configs["air_speed"])

        while True:
            distance = get_distance_metres(self.location.global_relative_frame, point)
            if distance > self.configs['waypoint_tolerance']:
                print("Distance remaining:", distance)
                time.sleep(1)
            else:
                break
        print("Target reached")

    def land(self):
        '''Commands vehicle to land'''
        self.mode = VehicleMode("LAND")

        print("Landing...")

        while self.location.global_relative_frame.alt > 0:
            print("Altitude: " + str(self.location.global_relative_frame.alt))
            time.sleep(1)

        print("Landed")

        print("Sleeping...")
        time.sleep(5)

    def set_altitude(self, alt):
        '''Sets altitude of quadcopter using an "alt" parameter'''
        print("Setting altitude:")
        destination = LocationGlobalRelative(self.location.global_relative_frame.lat, \
            self.location.global_relative_frame.lon, alt)
        self.go_to(destination)
        print("Altitude reached")

    def change_status(self, new_status):
        ''':param new_status: new vehicle status to change to (refer to GCS formatting)'''
        if new_status not in ("ready", "running", "waiting", "paused", "error"):
            raise Exception("Error: Unsupported status for vehicle")
        self.status = new_status

    def include_heading(self):
        '''Includes heading in messages'''
        self.heading = True


    def update_thread(self, address):
        ''':param vehicle: vehicle object that represents drone
        :param vehicle_type: vehicle type from configs file'''
        print("Starting update thread\n")

        while not self.MISSION_COMPLETED:
            location = self.location.global_frame
            # Comply with format of 0 - 1 and check that battery level is not null
            battery_level = self.battery.level / 100.0 if self.battery.level else 0.0
            update_message = {
                "type": "update",
                "time": round(time.clock() - self.coms.con_timestamp) + self.coms.gcs_timestamp,
                "sid": self.configs["vehicle_id"],
                "tid": 0, # the ID of the GCS is 0
                "id": self.coms.new_msg_id(),

                "vehicleType": "VTOL",
                "lat": location.lat,
                "lon": location.lon,
                "status": self.status,
                # TODO heading
                "battery": battery_level
            }

            if self.heading:
                update_message["heading"] = self.heading

            self.coms.send_till_ack(address, update_message, update_message['id'])
            time.sleep(1)
        self.change_status("ready")
