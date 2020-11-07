'''
The GCS allocates roles to vehicles. The purpose of this program is to connect to the GCS and then
read a start message from the GCS, which contains a certain job_type. It then runs the
corresponding HEX program (quick scan, detailed search, guide) based on the start message.
'''
import sys
import json
import time
from util import parse_configs, new_output_file
from autonomy import HEX
import autonomy

XBEE = None
GCS_TIMESTAMP = 0
CONNECTION_TIMESTAMP = 0

def xbee_callback(message):
    '''callback for xbee'''
    global GCS_TIMESTAMP
    global CONNECTION_TIMESTAMP

    address = message.remote_device.get_64bit_addr()
    msg = json.loads(message.data)
    print("Received data from %s: %s" % (address, msg))

    try:
        msg_type = msg["type"]

        if msg_type == "connectionAck":
            GCS_TIMESTAMP = msg['clocktime']
            CONNECTION_TIMESTAMP = time.time()
            autonomy.ACK_ID = msg["ackid"]

        elif msg_type == "start":
            # detach this callback so that the corresponding role can use its own callback
            XBEE.del_data_received_callback(xbee_callback)
            XBEE.close()

            job_type = msg['jobType']
            # TODO: Execute job
            bad_msg(address, "Unknown jobType: \'" + job_type + "\'")

        else:
            bad_msg(address, "Unknown message type: \'" + msg_type + "\'")

    # KeyError if message was missing an expected key
    except KeyError as e:
        bad_msg(address, "Missing \'" + e.args[0] + "\' key")


def main():
    configs = parse_configs(sys.argv)

    # create output file for all console output
    autonomy.outfile = new_output_file()
    tee = autonomy.Tee(sys.stdout, autonomy.outfile)
    sys.stdout = tee
    sys.stderr = tee

    # no comms simulation; that wouldn't be useful as this program is supposed to interact w/ GCS
    global XBEE
    XBEE = setup_xbee()

    # send connection message
    connection_message = {
        "type": "connect",
        "time": 0, # This field is currently not used
        "sid": configs['vehicle_id'],
        "tid": 0, # The ID of GCS
        "id": 0, # The ID of this message

        "jobsAvailable": ["quickScan", "detailedSearch", "guide"]
    }

    # wait to receive the connection ack and start message
    XBEE.add_data_received_callback(xbee_callback)

    send_till_ack(configs["mission_control_MAC"], connection_message, 0)

    if not autonomy.outfile.closed:
        autonomy.outfile.close()


if __name__ == "__main__":
    main()
