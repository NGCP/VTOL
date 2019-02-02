import sys
from digi.xbee.devices import RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress

sys.path.append('..')
from VTOL.autonomy import setup_xbee

# Insert the MAC Address of desired radio here
MAC_ADDR = "0013A200409BD79C"
DATA_TO_SEND = "Hello XBee!"


def main():
    # Instantiate XBee device
    device = setup_xbee()

    try:
        remote_device = RemoteXBeeDevice(device, XBee64BitAddress.from_hex_string(MAC_ADDR))

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
