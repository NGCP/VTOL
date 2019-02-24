import sys
import cv2
import json

sys.path.append('..')
from autonomy import setup_xbee
from autonomy import receive_image

def main():
    # Instantiate XBee device
    xbee = setup_xbee()
    try:
        xbee.flush_queues()
        print("Waiting for data...\n")
        while True:
            xbee_message = xbee.read_data()
            if xbee_message is not None:
                address = str(xbee_message.remote_device.get_64bit_addr())
                print(xbee_message.data)
                msg = json.loads(str(xbee_message.data))
                image = receive_image(address, msg["size"])
                cv2.imshow('img', image)
                cv2.waitKey(0)
                break
    finally:
        if xbee is not None and xbee.is_open():
            xbee.close()
    
if __name__ == '__main__':
    main()
