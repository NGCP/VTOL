from threading import Thread
import time
import json

def comm_simulation(config_dict):
    with open(config_dict, 'r') as f:
        commands = json.load(f)
    if (not commands["comms_simulated"]):
        return -1
        
           
    start = time.time()
    for i in commands["comms_simulated"]:
        current = time.time() - start
        val = float(i) 
        while(val > current):
            time.sleep(1)
            current = time.time() - start
        t = Thread(target = callback, 
                args = (commands["comms_simulated"][i]))
        t.start()

