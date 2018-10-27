from digi.xbee.devices import XBeeDevice
from sys import platform     

def main():
    device  = None
    port_name = ""

    if platform == "darwin":
        # TODO: figure out mac port name
        port_name = "/dev/tty.usbserial-DA01QW1R"
    elif platform == "linux" or platform == "linux2":
        port_name = "/dev/ttyUSB0"
    elif platform == "win32":
        port_name = "COMS1"

    device = XBeeDevice(port_name, 57600)

    try:
        device.open()
        device.flush_queues()
        print("Waiting for data...\n")
        while True:
            xbee_message = device.read_data()
            if xbee_message is not None:
                print("Received '%s' from %s" % 
                        (xbee_message.data.decode(), xbee_message.remote_device.get_64bit_addr()))
    finally:
        if device is not None and device.is_open():
            device.close()

if __name__ == '__main__':
    main()
