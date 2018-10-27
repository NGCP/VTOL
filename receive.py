from digi.xbee.devices import XBeeDevice

#Assuming radio is in the first USB port.
PORT = "/dev/ttyUSB0"
BAUD_RATE = 57600

def main():
    device = XBeeDevice(PORT, BAUD_RATE)
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
