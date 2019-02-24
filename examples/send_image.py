import sys
import cv2
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
    image = send_image(image, XBee64BitAddress.from_hex_string(MAC_ADDR), ".png")
    xbee.close()
    
if __name__ == '__main__':
    main()
