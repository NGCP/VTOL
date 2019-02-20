import sys
from digi.xbee.devices import RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
import cv2
import numpy

sys.path.append('..')
from autonomy import setup_xbee
from autonomy import send_image

# Insert the MAC Address of desired radio here
MAC_ADDR = "0013A200409BD79C"
DATA_TO_SEND = "Hello XBee!"


def main():
    # Instantiate XBee device
    image = cv2.imread('images/download.jpg')
    image = send_image(image, MAC_ADDR)
    
if __name__ == '__main__':
    main()
