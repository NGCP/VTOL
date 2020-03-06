'''Automous tools for QUAD'''
import time
from math import radians
from dronekit import VehicleMode, Vehicle, LocationGlobalRelative
from src.coms import Coms
from src.util import get_distance_metres, to_quaternion
from src.flight_objects import Attitude
from examples.RCOverride.pid import PID

class QUAD(Vehicle):
    ''' QUAD basic state isolated'''

    def __init__(self, *args): #pylint: disable=useless-super-delegation
        super(QUAD, self).__init__(*args)

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
        self.simple_takeoff(altitude)

        # Wait until vehicle reaches minimum altitude
        while self.location.global_relative_frame.alt < altitude * 0.95:
            print("Altitude: " + str(self.location.global_relative_frame.alt))
            time.sleep(1)

        print("Reached target altitude")

    def setup(self):
        '''QUAD specific steps needed before flight'''
        print('Initializing Coms')
        self.tolerance = self.configs['tolerance']
        self.coms = Coms(self.configs, self.coms_callback)

    def set_mode(self, mode):
        '''sets vehicle's mode'''
        self.mode = VehicleMode(mode)

    def altitude(self):
        '''gets location'''
        return self.location.global_relative_frame.alt

    # State, updated by XBee callback function
    configs = None
    start_mission = False  # takeoff
    pause_mission = False  # vehicle will hover
    stop_mission = False  # return to start and land
    tolerance = 0

    # Global status, updated by various functions
    status = "ready"
    MISSION_COMPLETED = False
    coms = None
    # pylint: disable=no-self-use
    def coms_callback(self, command):
        '''callback for radio messages'''

        #tuple of commands that can be executed
        valid_commands = ("takeoff", "land", "go_to", "set_altitude")
        #gives us the specific command we want the drone to executre

        #checking for valid command
        if command["Type"] not in valid_commands:
            raise Exception("Error: Unsupported status for vehicle")

        #executes takeoff command to drone
        if command["Type"] == 'takeoff':
            self.takeoff()
        #executes land command to drone
        elif command["Type"] == 'land':
            self.land()
        elif command["Type"] == 'go_to':
            self.go_to(LocationGlobalRelative(command["Body"]["Lat"], \
                command["Body"]["Lon"], command["Body"]["Alt"]))
        elif command["Type"] == 'set_altitude':
            self.set_altitude(command["Body"]["Alt"])


    def go_to(self, point):
        '''Commands drone to fly to a specified point perform a simple_goto '''

        self.simple_goto(point)
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
        self.set_mode("LAND")

        print("Landing...")

        while self.location.global_relative_frame.alt > 0:
            print("Altitude: " + str(self.location.global_relative_frame.alt))
            time.sleep(1)

        print("Landed")

        print("Sleeping...")
        time.sleep(5)

    def move_meters(self, x_dist=0, y_dist=0):
        '''moves meters in x y frame'''
        print("Moving {} by {}".format(x_dist, y_dist))
        x_pid = PID(.5, .0, 0, .5)
        y_pid = PID(1, .0, 0, .5)
        stage = 0
        hits = 0
        att = Attitude(pitch=0, yaw=0, roll=0, thrust=.5)
        while True:
            if stage == 0:
                self.set_mode('GUIDED')
                start = self.location.global_relative_frame
                stage += 1
            if stage == 1:
                d_y, d_x, _ = get_distance_metres(start, self.location.global_relative_frame)
                d_y -= y_dist
                d_x -= x_dist
                att.set_roll(-x_pid.output(d_x)) #set hard min and max
                att.set_pitch(y_pid.output(d_y))
                if abs(d_x) < self.tolerance and abs(d_y) < self.tolerance:
                    hits += 1
                    print("HIT")
                    if hits == 5:
                        stage += 1
                        att.set_roll(0)
                        att.set_pitch(0)
                else:
                    hits = 0
            else:
                print("Altitude reached")
                return

            self.set_attitude(att, duration=.5)
            print(d_y, d_x)

    def set_altitude(self, alt_target):
        '''Sets altitude of quadcopter using an "alt" parameter'''
        print("Setting altitude to {}".format(alt_target))
        pid = PID(.095, .0, 0, .5)
        stage = 0
        hits = 0
        att = Attitude(pitch=0, yaw=0, roll=0, thrust=.5)
        while True:
            if stage == 0:
                self.set_mode('GUIDED')
                stage += 1
            if stage == 1:
                delta = alt_target - self.altitude()
                att.set_thrust(.5 + pid.output(delta)) #set hard min and max
                if abs(delta) < self.tolerance:
                    hits += 1
                    print("HIT")
                    if hits == 5:
                        stage += 1
                        att.set_thrust(.5)
                else:
                    hits = 0
            else:
                print("Altitude reached")
                return

            self.set_attitude(att, duration=.5)
            print(delta)


    def change_status(self, new_status):
        ''':param new_status: new vehicle status to change to (refer to GCS formatting)'''
        if new_status not in ("ready", "running", "waiting", "paused", "error"):
            raise Exception("Error: Unsupported status for vehicle")
        self.status = new_status


    def send_attitude_target(
            self,
            roll_angle=0.0,
            pitch_angle=0.0,
            yaw_angle=None,
            yaw_rate=0.0,
            use_yaw_rate=False,
            thrust=0.5,
    ):
        '''
        use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                    When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        '''
        if yaw_angle is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw_angle = self.attitude.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            radians(yaw_rate),  # Body yaw rate in radian/second
            thrust,  # Thrust
        )
        self.send_mavlink(msg)


    def set_attitude(self, attitude, duration=0):
        '''
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        '''
        roll_angle = attitude.roll
        pitch_angle = attitude.pitch
        yaw_angle = attitude.yaw
        thrust = attitude.thrust
        use_yaw_rate = False

        self.send_attitude_target(
            roll_angle, pitch_angle, yaw_angle, 0, use_yaw_rate, thrust
        )
        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(
                roll_angle, pitch_angle, yaw_angle, 0, use_yaw_rate, thrust
            )
            time.sleep(0.1)
        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0, 0, 0, True, thrust)


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

                "vehicleType": "QUAD",
                "lat": location.lat,
                "lon": location.lon,
                "status": self.status,
                # TODO heading
                "battery": battery_level
            }

            self.coms.send_till_ack(address, update_message, update_message['id'])
            time.sleep(1)
        self.change_status("ready")
