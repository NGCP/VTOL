import sys
import cv2
import time
sys.path.append('..')
from autonomy import setup_xbee
from autonomy import send_image

# Insert the MAC Address of desired radio here
MAC_ADDR = "0013A20040F8064D"
PATH_TO_IMAGE = "images/raw.TIF"

def main():
    # Instantiate XBee device
    xbee = setup_xbee()
    image = cv2.imread(PATH_TO_IMAGE, cv2.IMREAD_COLOR)
    print(image)
    start = time.time()
    image = send_image(image, MAC_ADDR, ".png")
    end = time.time()
    print(end - start)
    xbee.close()
    
if __name__ == '__main__':
    main()
