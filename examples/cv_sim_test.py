'''tests cv module'''
import sys
import cv2
from util import parse_configs
from cv import Vision

sys.path.append('..')


def main():
    '''Runs cv_simulation in a loop and displays the image that is returned from it
    Uses the directory that is in the config file'''
    argv = ["f", "../configs.json"]
    for _ in range(10):
        cv2.imshow('image', Vision().cv_simulation(parse_configs(argv)))
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
