import sys
import cv2

sys.path.append('..')
from autonomy import setup_xbee
from autonomy import receive_image

def main():
    # Instantiate XBee device
    xbee = setup_xbee()
    image = receive_image()
    xbee.close()
    
if __name__ == '__main__':
    main()
