'''main executable for setting up quadplane'''
import json
import time
from dronekit import VehicleMode, LocationGlobal
from gps_denied_v2 import setup_vehicle, GpsDeniedVtol

def main(configs):
    '''Configure vtol and ready for mission'''
    vehicle = setup_vehicle(configs)
    vehicle.__class__ = GpsDeniedVtol

    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)


    print("taking off")
    vehicle.takeoff(mode="GUIDED")
    print("tilting forward")
    dest = LocationGlobal(37.616729, -122.381685, alt=50)
    vehicle.simple_goto(dest, airspeed=50)
    time.sleep(5)
    vehicle.set_mode("QLOITER")

    time.sleep(15)
    vehicle.set_mode("GUIDED")
    vehicle.airspeed = 5 #m/s
    vehicle.groundspeed = 7.5 #m/s


    vehicle.simple_goto(dest)
    vehicle.simple_goto(dest, groundspeed=5)
    print('going to')
    time.sleep(10)

print()
if __name__ == '__main__':
    with open('configs.json', 'r') as data:
        main(json.load(data))
        