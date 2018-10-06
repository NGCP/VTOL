import json, sys

def parse_json():
   """Parses the .json file given as the first command line argument.
   If no .json file is specified, defaults to "configs.json".
   If the specified .json file is found, returns a dict of the contents.
   Otherwise, raises a FileNotFoundError."""
   try:
      if (len(sys.argv) == 1):
         read_file = open("configs.json", "r")
      else:
         read_file =  open(sys.argv[1], "r")
      return json.load(read_file)
   except FileNotFoundError:
      raise