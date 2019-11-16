'''Util methods for VTOL'''
import json
import datetime
import subprocess
import sys
from math import cos, sin, radians

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    '''
    Convert degrees to quaternions
    '''
    t0_val = cos(radians(yaw * 0.5))
    t1_val = sin(radians(yaw * 0.5))
    t2_val = cos(radians(roll * 0.5))
    t3_val = sin(radians(roll * 0.5))
    t4_val = cos(radians(pitch * 0.5))
    t5_val = sin(radians(pitch * 0.5))

    w_val = t0_val * t2_val * t4_val + t1_val * t3_val * t5_val
    x_val = t0_val * t3_val * t4_val - t1_val * t2_val * t5_val
    y_val = t0_val * t2_val * t5_val + t1_val * t3_val * t4_val
    z_val = t1_val * t2_val * t4_val - t0_val * t3_val * t5_val

    return [w_val, x_val, y_val, z_val]

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
