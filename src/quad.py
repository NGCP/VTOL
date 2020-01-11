'''Automous tools for VTOL'''
import time
import json
from dronekit import VehicleMode, Vehicle, LocationGlobalRelative
from coms import Coms
from util import get_distance_metres


class QUAD(Vehicle):
    ''' VTOL basic state isolated'''
    coms = None

    def __init__(self, configs, *args): #pylint: disable=useless-super-delegation
        super(QUAD, self).__init__(*args)
        self.configs = configs

    def setup(self):
        '''initializes coms'''
        self.coms = Coms(self.configs, self.coms_callback)

    #pylint: disable=no-self-use
    def coms_callback(self, message):
        '''callback for radio messages'''
        parsed_message = json.loads(message.data)
        #gives us the specific command we want the drone to executre
        command = parsed_message['type']
        print("recieved message {}".format(command))

        # START HERE

        # Respond to takeoff command

        # Respond to go_to command

        # Respond to land command


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


    def go_to(self, lat, lon):
        '''Commands drone to fly to a specified point perform a simple_goto '''
        point = LocationGlobalRelative(lat, lon, self.configs['altitude'])
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
        # Implement land
        # See above methods as examples
        # https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
        # https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
