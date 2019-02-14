import sys
from threading import Thread, Lock
from detailed_search_autonomy import detailed_search_autonomy
from detailed_search_cv import detailed_search_cv
from util import parse_configs


class AutonomyToCV:
    def __init__(self):
        self.startMutex = Lock()
        self.start = False

        self.latMutex = Lock()
        self.lat = 0.0

        self.lonMutex = Lock()
        self.lon = 0.0

        self.altMutex = Lock()
        self.alt = 0.0

        self.northMutex = Lock()
        self.north = 0.0

        self.eastMutex = Lock()
        self.east = 0.0


def detailed_search():
    # Parse configs file
    configs = parse_configs(sys.argv)

    # Start autonomy and CV threads
    autonomyToCV = AutonomyToCV()
    autonomy_thread = Thread(target = detailed_search_autonomy,
                             args = (configs, autonomyToCV))
    autonomy_thread.daemon = True
    autonomy_thread.start()

    cv_thread = Thread(target = detailed_search_cv,
                       args = (configs, autonomyToCV))
    cv_thread.daemon = True
    cv_thread.start()

    # Wait for the threads to finish
    autonomy_thread.join()
    cv_thread.join()

    # detailed_search_autonomy(configs)



if __name__ == "__main__":
    detailed_search()
