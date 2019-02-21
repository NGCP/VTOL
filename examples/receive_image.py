import sys
from digi.xbee.devices import RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress
import cv2
import numpy

sys.path.append('..')
from autonomy import setup_xbee
from autonomy import receive_image

# Insert the MAC Address of desired radio here
MAC_ADDR = "0013A200409BD79C"
DATA_TO_SEND = "Hello XBee!"


def main():
    # Instantiate XBee device
    xbee = setup_xbee()
    image = receive_image()
    cv2.imshow('received image', image)
    cv2.waitKey(0) 
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
