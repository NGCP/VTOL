import sys
from threading import Thread, Lock
import autonomy
from detailed_search_autonomy import detailed_search_autonomy
from detailed_search_cv import detailed_search_cv
from util import parse_configs

class DetailedSearchAutonomyToCV:
    def __init__(self):
        self.startMutex = Lock()
        self.start = False

        self.stopMutex = Lock()
        self.stop = False

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

        self.xbeeMutex = Lock()
        self.xbee = None

        self.acknowledgementMutex = Lock()
        self.acknowledgementMutex = False

def detailed_search(vehicle = None):
    # Parse configs file
    configs = parse_configs(sys.argv)

    # Start autonomy and CV threads
    autonomyToCV = DetailedSearchAutonomyToCV()
    autonomy_thread = Thread(target = detailed_search_autonomy,
                             args = (configs, autonomyToCV, vehicle))
    autonomy_thread.daemon = True
    autonomy_thread.start()

    cv_thread = Thread(target = detailed_search_cv, args = (configs, autonomyToCV))
    cv_thread.daemon = True
    cv_thread.start()

    # Wait for the threads to finish
    autonomy_thread.join()
    cv_thread.join()

    # Close XBee device
    if autonomy.xbee:
        autonomy.xbee.close()


if __name__ == "__main__":
    detailed_search()
