'''methods neccecary for coms'''
import time
from threading import Lock, Thread
import sys
import subprocess
import json
import logging
from functools import partial
from http.server import BaseHTTPRequestHandler, HTTPServer
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice, XBee64BitAddress
import msgpack

class ComsMutex:
    '''maintains order between threads'''
    def __init__(self):
        self.start_mutex = Lock()
        self.start = False

        self.stop_mutex = Lock()
        self.stop = False

        self.xbee_mutex = Lock()
        self.xbee = None

        self.acknowledgement_mutex = Lock()
        self.acknowledgement_mutex = False


def mac_xbee_port_name():
    '''Looks in /dev directory for connected XBee serial port name on a macOS.'''
    try:
        # System call to get port name of connected XBee radio
        port_name = subprocess.check_output(["ls", "/dev/"])

        # index in dev directory of port name
        i = port_name.index("tty.usbserial-")
        # 22 is length of "tty.usbserial-" + 8-char port name
        return "/dev/" + port_name[i: i + 22]

    except ValueError:
        raise ValueError("Value Error: \'tty.usbserial-\' not found in /dev")


class DummyMessage:
    '''Dummy msg class for comm simulation thread to be compatible with xbee_callback function'''
    def __init__(self, data=None):
        self.data = data  # UTF-8 encoded JSON message
        self.remote_device = DummyRemoteDevice()


class DummyRemoteDevice:
    '''Dummy remote device object for use in DummyMessage'''
    def __init__(self):
        self.get_64bit_addr = lambda _: 'comms simulation'

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
        self.mutex = ComsMutex()

        if configs['coms_simulated'] is True:
            comm_sim = Thread(target=self.comm_simulation)
            comm_sim.start()
        else:
            try:
                port_name = ""
                if sys.platform == "darwin":
                    port_name = mac_xbee_port_name()
                elif sys.platform == "linux" or sys.platform == "linux2":
                    port_name = "/dev/ttyUSB0"
                # TODO: figure out windows port name
                elif sys.platform == "win32":
                    port_name = "COMS1"

                # Instantiate XBee device object.
                self.xbee = XBeeDevice(port_name, 57600)
                self.xbee.open()

            # If error in setting up XBee, try again
            except TimeoutError as ex:
                print(ex)
                print("Connect the XBee radio!")
                time.sleep(5)


    def send_till_ack(self, address, msg, msg_id):
        '''Continuously sends message to given address until acknowledgement
        message is recieved with the corresponding ackid.'''
        # Instantiate a remote XBee device object to send data.
        packed_data = bytearray(msgpack.packb(msg))
        while self.ack_id != msg_id:
            self.send(address, packed_data)
            time.sleep(1)


    def acknowledge(self, address, ack_id):
        '''Sends message received acknowledgement to GCS
        param address: address of GCS'''
        ack = {
            "type": "ack",
            "time": round(time.clock() - self.con_timestamp) + self.gcs_timestamp,
            "sid": self.configs['vehicle_id'],
            "tid": 0, # The ID of GCS
            "id": self.new_msg_id(),
            "ackid": ack_id
        }
        self.send(address, ack)


    def new_msg_id(self):
        '''Increments msg_id and returns unique id for new message'''
        self.msg_id += 1
        return self.msg_id


    def recieve(self):
        '''Instantiate XBee device'''
        try:
            self.xbee.flush_queues()
            print("Waiting for data...\n")
            while True:
                xbee_message = self.xbee.read_data()
                if xbee_message is not None:
                    print("Received '%s' from %s" % (xbee_message.data.decode(), \
                        xbee_message.remote_self.xbee.get_64bit_addr()))
        finally:
            if self.xbee is not None and self.xbee.is_open():
                self.xbee.close()


    def send(self, mac_address, data):
        '''sends data over xbee'''
        remote_device = RemoteXBeeDevice(self.xbee, XBee64BitAddress.from_hex_string(mac_address))

        if remote_device is None:
            print("Invalid MAC Address")
            return

        print("Sending '%s' to %s" % (data, remote_device.get_64bit_addr()))
        self.mutex.xbee_mutex.acquire()
        self.xbee.send_data(remote_device, data)
        self.mutex.xbee_mutex.release()
        print("Success")


    def bad_msg(self, address, problem):
        '''Sends "bad message" to GCS if message received was poorly formatted/unreadable
        and describes error from parsing original message.
        :param address: address of GCS
        :param problem: string describing error from parsing original message'''
        msg = {
            "type": "badMessage",
            "time": round(time.clock() - self.con_timestamp) + self.gcs_timestamp,
            "sid": self.configs['vehicle_id'],
            "tid": 0, # The ID of GCS
            "id": self.new_msg_id(),

            "error": problem
        }
        self.send(address, msg)

    # TODO: needs to be updated
    def comm_simulation(self):
        '''Connects to HTTP server at a specific port'''
        logging.basicConfig(level=logging.INFO)
        server_address = ('', 8080)
        partial_server = partial(ComsServer, self.xbee_callback)
        httpd = HTTPServer(server_address, partial_server)
        logging.info('Starting httpd...\n')
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
        httpd.server_close()
        logging.info('Stopping httpd...\n')

class ComsServer(BaseHTTPRequestHandler, Coms):
    '''Sets response'''
    def __init__(self, callback, *args, **kwargs):
        self.callback = callback
        # BaseHTTPRequestHandler calls do_GET **inside** __init__ !!!
        # So we have to call super().__init__ after setting attributes.
        super().__init__(*args, **kwargs)

    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    # pylint: disable=invalid-name
    def do_POST(self):
        '''Handles POST requests'''
        content_length = int(self.headers['Content-Length']) # <--- Gets the size of data
        post_data = self.rfile.read(content_length) # <--- Gets the data itself
        logging.info("POST request,\nPath: %s\nHeaders:\n%s\n\nBody:\n%s\n", \
            str(self.path), str(self.headers), post_data.decode('utf-8'))

        self._set_response()

        post_data = post_data.decode()
        data = json.loads(post_data)
        cmd = data["Type"]
        # self.wfile.close() to send to Postman before the command is called
        if cmd == "takeoff":
            self.wfile.write("Taking off\n".encode())
            write = Thread(target=self.callback, args=(data,))
            write.start()
        elif cmd == "go_to":
            self.wfile.write("Heading to point\n".encode())
            write = Thread(target=self.callback, args=(data,))
            write.start()
        elif cmd == "land":
            self.wfile.write("Landing\n".encode())
            write = Thread(target=self.callback, args=(data,))
            write.start()
        elif cmd == "set_altitude":
            self.wfile.write("Setting altitude\n".encode())
            write = Thread(target=self.callback, args=(data,))
            write.start()
