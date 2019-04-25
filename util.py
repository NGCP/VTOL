import json
import datetime
import subprocess

def parse_configs(argv):
    """Parses the .json file given as the first command line argument.
    If no .json file is specified, defaults to "configs.json".
    If the specified .json file is found, returns a dict of the contents.
    """
    if (len(argv) == 1):
        read_file = open("configs.json", "r")
    else:
        read_file = open(argv[1], "r")

    return json.load(read_file)


# Creates a new directory for the current date if not created already
# and creates an output file for all console output in the directory.
def new_output_file():
    curr_time = str(datetime.datetime.today()).split()
    date = curr_time[0]
    time = curr_time[1]

    # makes logs folder if not existing already
    try:
        subprocess.check_output(["ls", "logs/"], stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError:
        subprocess.call(["mkdir", "logs/"])

    # makes folder for the current date in logs folder if not existing already
    try:
        subprocess.check_output(["ls", "logs/" + date], stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError:
        subprocess.call(["mkdir", "logs/" + date])

    # open file for current time
    return open("logs/" + date + "/" + time.replace(":", ".") + ".txt", "w")
