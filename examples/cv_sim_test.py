import sys
import cv2
from cv import cv_simulation

sys.path.append('..')
from util import parse_configs


# Runs cv_simulation in a loop and displays the image that is returned from it
# Uses the directory that is in the config file
def main():
    argv = ["f", "../configs.json"]
    for i in range(10):
        cv2.imshow('image', cv_simulation(parse_configs(argv)))
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
