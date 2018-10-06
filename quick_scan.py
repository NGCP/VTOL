import json

def parse_json():
   try:
      with open("configs.json", "r") as read_file:
         return json.load(read_file)
   except FileNotFoundError:
      raise