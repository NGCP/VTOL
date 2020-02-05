'''Runs simulator using python'''

import os
from time import sleep
from dronekit_sitl import SITL
SIM = {}
BIN = os.getenv("SITL_BIN")
DO_DOWNLOAD = True
if BIN is not None:
    DO_DOWNLOAD = False
    SIM["path"] = BIN
    DEFAULTS = os.getenv("SITL_DEFAULTS_FILEPATH")
    if DEFAULTS is not None:
        SIM["defaults_filepath"] = DEFAULTS
SIM = SITL(**SIM)
if DO_DOWNLOAD:
    SIM.download('copter', '3.3', verbose=False)

SIM.launch(['--model', 'quad', ], await_ready=True, restart=False)
print("Simulator lanched on {}".format(SIM.connection_string()))

while True:
    sleep(5)