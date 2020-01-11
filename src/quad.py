'''Automous tools for VTOL'''
import time
import json
from dronekit import VehicleMode, Vehicle
from coms import Coms
from util import get_distance_metres


class QUAD(Vehicle):
    ''' VTOL basic state isolated'''
    coms = None

    def __init__(self, *args): #pylint: disable=useless-super-delegation
        super(QUAD, self).__init__(*args)

    def setup(self):
        '''vtol specific steps needed before flight'''

    def coms_callback(self, message):
        '''callback for radio messages'''
        parsed_message = json.loads(message.data)
        #gives us the specific command we want the drone to executre
        command = parsed_message['type']

        # START HERE

    def setup_coms(self):
        '''sets up communication radios'''
        print('Initializing Coms')
        self.coms = Coms(self.configs, self.coms_callback)


    def takeoff(self):
        '''Commands drone to take off by arming vehicle and flying to altitude'''
        print("Pre-arm checks")
        while not self.is_armable:
            print("Waiting for vehicle to initialize")
            time.sleep(1)

        print("Arming motors")
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
