import json


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
