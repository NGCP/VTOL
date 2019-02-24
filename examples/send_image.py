import sys
import cv2

sys.path.append('..')
from autonomy import setup_xbee
from autonomy import send_image

# Insert the MAC Address of desired radio here
MAC_ADDR = "0013A20040F8064D"
PATH_TO_IMAGE = "images/medium.jpg"

def main():
    # Instantiate XBee device
    xbee = setup_xbee()
    image = cv2.imread(PATH_TO_IMAGE)
    image = send_image(image, MAC_ADDR)
    xbee.close()
    
if __name__ == '__main__':
    main()
