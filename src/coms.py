'''methods neccecary for coms'''
import time
from threading import Thread
import json

class Coms():
    '''compartenmentalizes coms functionality and scope'''
    configs = None
    con_timestamp = 0
    gcs_timestamp = 0
    msg_id = 0  # unique ID increments for each message sent
    ack_id = None
    xbee = None  # XBee radio object
    xbee_callback = None

    def __init__(self, configs, xbee_callback):
        '''initializes coms object'''
        self.configs = configs
        self.xbee_callback = xbee_callback

        sim_file = configs["comm_sim_file"]
        comm_sim = Thread(target=self.comm_simulation, args=(sim_file,))
        comm_sim.start()


    def comm_simulation(self, comm_file):
        '''Reads through comm simulation file from configs and calls
        xbee_callback to simulate radio messages.'''
        with open(comm_file, "r") as com_data:
            comms = json.load(com_data)  # reads the json file
            prev_time = 0
            for instr in comms:  # gets time and message from each json object (instruction)
                curr_time = instr["time"]
                time.sleep(curr_time - prev_time)  # waits for the next instruction
                # Send message to xbee_callback
                self.xbee_callback(DummyMessage(json.dumps(instr["message"])))
                prev_time = curr_time


class DummyMessage:
    '''Dummy msg class for comm simulation thread to be compatible with xbee_callback function'''
    def __init__(self, data=None):
        self.data = data  # UTF-8 encoded JSON message
        self.remote_device = DummyRemoteDevice()

class DummyRemoteDevice:
    '''Dummy remote device object for use in DummyMessage'''
    def __init__(self):
        self.get_64bit_addr = lambda _: 'comms simulation'
