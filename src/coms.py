'''methods neccecary for coms'''
import time
import msgpack
from digi.xbee.devices import RemoteXBeeDevice, XBee64BitAddress

def bad_msg(self, address, problem, autonomy_to_cv):
    '''Sends "bad message" to GCS if message received was poorly formatted/unreadable
    and describes error from parsing original message.
    :param address: address of GCS
    :param problem: string describing error from parsing original message'''
    msg = {
        "type": "badMessage",
        "time": round(time.clock() - self.CONNECTION_TIMESTAMP) + self.GCS_TIMESTAMP,
        "sid": self.configs['vehicle_id'],
        "tid": 0, # The ID of GCS
        "id": self.new_msg_id(),

        "error": problem
    }
    # xbee is None if comms is simulated
    if self.xbee:
        # Instantiate a remote XBee device object to send data.
        send_xbee = RemoteXBeeDevice(self.xbee, XBee64BitAddress.from_hex_string(address))
        packed_data = msgpack.packb(msg)
        autonomy_to_cv.xbeeMutex.acquire()
        self.xbee.send_data(send_xbee, packed_data)
        autonomy_to_cv.xbeeMutex.release()
    else:
        print("Error:", problem)
