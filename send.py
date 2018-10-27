from digi.xbee.devices import XBeeDevice
from digi.xbee.devices import RemoteXBeeDevice
from digi.xbee.models.status import NetworkDiscoveryStatus
from digi.xbee.models.address import XBee64BitAddress
import time
import serial

BAUD_RATE = 57600
PORT = "/dev/ttyUSB0"
DATA_TO_SEND = "Hello World!"

def main():
    device = XBeeDevice(PORT, BAUD_RATE)
    try:
        device.open()

        xbee_network = device.get_network()

        #Insert the MAC Address of desired radio here
        remote_device = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string("0013A20040F8064D"))
        if remote_device is None:
            print("Invalid MAC Address")
            exit(1)

        print("Sending '%s' to %s" % (DATA_TO_SEND, remote_device.get_64bit_addr()))
        device.send_data(remote_device, DATA_TO_SEND)
        print("Success")

    finally:
        if device is not None and device.is_open():
            device.close()


if __name__ == '__main__':
    main()
