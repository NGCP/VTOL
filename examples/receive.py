import sys

sys.path.append('..')
from VTOL.autonomy import setup_xbee


def main():
    # Instantiate XBee device
    device = setup_xbee()

    try:
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
