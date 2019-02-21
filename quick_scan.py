import sys
import autonomy
from threading import Thread, Lock
from quick_scan_autonomy import quick_scan_autonomy
from quick_scan_cv import quick_scan_cv
from util import parse_configs


class QuickScanAutonomyToCV:
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


def quick_scan():
    # Parse configs file
    configs = parse_configs(sys.argv)

    # Start autonomy and CV threads
    autonomyToCV = QuickScanAutonomyToCV()
    autonomy_thread = Thread(target=quick_scan_autonomy, args=(configs, autonomyToCV))
    autonomy_thread.daemon = True
    autonomy_thread.start()

    cv_thread = Thread(target=quick_scan_cv, args=(configs, autonomyToCV))
    cv_thread.daemon = True
    cv_thread.start()

    # Wait for the threads to finish
    autonomy_thread.join()
    cv_thread.join()

    # Close XBee device
    if autonomy.xbee:
        autonomy.xbee.close()


if __name__ == "__main__":
    quick_scan()
