'''Util methods for VTOL'''
import json
import datetime
import subprocess
import sys
import math

def parse_configs(argv):
    """Parses the .json file given as the first command line argument.
    If no .json file is specified, defaults to "configs.json".
    If the specified .json file is found, returns a dict of the contents.
    """
    if len(argv) == 1:
        read_file = open("configs.json", "r")
    else:
        read_file = open(argv[1], "r")

    return json.load(read_file)


def new_output_file():
    '''Creates a new directory for the current date if not created already
    and creates an output file for all console output in the directory.'''
    curr_time = str(datetime.datetime.today()).split()
    date = curr_time[0]
    time = curr_time[1]

    # makes logs folder if not existing already
    try:
        if sys.platform == "darwin" or sys.platform == "linux" or sys.platform == "linux2":
            subprocess.check_output(["ls", "logs/"], shell=False, stderr=subprocess.STDOUT)
        elif sys.platform == "win32":
            subprocess.check_output(["cd", "logs/"], shell=True, stderr=subprocess.STDOUT)
        else:
            raise Exception("Operating system not recognized")
    except subprocess.CalledProcessError:
        if sys.platform == "darwin" or sys.platform == "linux" or sys.platform == "linux2":
            subprocess.call(["mkdir", "logs/"], shell=False)
        elif sys.platform == "win32":
            # No ls on Windows cmd
            subprocess.call(["mkdir", "logs/"], shell=True)
        else:
            raise Exception("Operating system not recognized")

    # makes folder for the current date in logs folder if not existing already
    try:
        if sys.platform == "darwin" or sys.platform == "linux" or sys.platform == "linux2":
            subprocess.check_output(["ls", "logs/" + date], shell=False, stderr=subprocess.STDOUT)
        elif sys.platform == "win32":
            # No ls on Windows cmd
            subprocess.check_output(["cd", "logs/" + date], shell=True, stderr=subprocess.STDOUT)
        else:
            raise Exception("Operating system not recognized")
    except subprocess.CalledProcessError:
        if sys.platform == "darwin" or sys.platform == "linux" or sys.platform == "linux2":
            subprocess.call(["mkdir", "logs/" + date], shell=False)
        elif sys.platform == "win32":
            # Windows cmd handles mkdir path in a strange way
            subprocess.call(["mkdir", "logs\\" + date], shell=True)
        else:
            raise Exception("Operating system not recognized")

    # open file for current time
    return open("logs/" + date + "/" + time.replace(":", ".") + ".txt", "w")

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