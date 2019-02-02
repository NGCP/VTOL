import sys
from threading import Thread, Lock
from quick_scan_autonomy import quick_scan_autonomy
from quick_scan_cv import quick_scan_cv
from util import parse_configs

def quick_scan(filename = ""):
    # If the optional argument is set, override command line argument
    if filename != "":
        configs = parse_configs(["", filename]);

    # Parse filename given in command line
    else:
        configs = parse_configs(sys.argv)

    # Start autonomy and CV threads
    autonomyToCV = AutonomyToCV()
    autonomy_thread = Thread(target = quick_scan_autonomy, args = (configs, autonomyToCV))
    autonomy_thread.daemon = True
    autonomy_thread.start()

    cv_thread = Thread(target = quick_scan_cv, args = (configs, autonomyToCV))
    cv_thread.daemon = True
    cv_thread.start()

    # Wait for the threads to finish
    autonomy_thread.join()
    cv_thread.join()


if __name__ == "__main__":
    quick_scan()
    