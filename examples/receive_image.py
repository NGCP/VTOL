import sys
import cv2
import json

sys.path.append('..')
from autonomy import setup_xbee
from autonomy import receive_image

def main():
    # Instantiate XBee device
    xbee = setup_xbee()
    while True:
        xbee_message = xbee.read_data()
        if xbee_message is not None:
            address = xbee_message.remote_device.get_64bit_addr()
            msg = json.loads(xbee_message.data)
            image = receive_image(address, msg["size"])

    xbee.close()
    
if __name__ == '__main__':
    main()
