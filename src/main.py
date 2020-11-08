"""main executable for setting up HEX"""
import json
from util import setup_vehicle
from hex import HEX


def main(configs):
    """Configure hexcopter and ready for mission"""
    hexa = setup_vehicle(configs, HEX)
    hexa.takeoff()
    hexa.go_to(27.28, 153.01)
    hexa.land()


if __name__ == "__main__":
    with open("configs.json", "r") as data:
        main(json.load(data))
