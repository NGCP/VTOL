'''Outline for processing feature points'''

from pickle import dump, HIGHEST_PROTOCOL
import cv2

def main():
    '''demo feature gathering'''
    scale = 0.5

    orb = cv2.ORB_create(nfeatures=100000, scoreType=cv2.ORB_FAST_SCORE)

    area_map = cv2.imread("images/stitch_flip.jpg")
    area_map = cv2.resize(area_map, (0, 0), fx=scale, fy=scale)
    area_map = cv2.cvtColor(area_map, cv2.COLOR_BGR2GRAY)

    # find the keypoints and descriptors with ORB
    keys, descs = orb.detectAndCompute(area_map, None)
    length, width = area_map.shape

    with open('map.bin', 'wb') as output:  # Overwrites any existing file.
        obj = (map(lambda x: {'pt': x.pt, 'size': x.size, 'angle': x.angle, \
            'response': x.response, 'octave': x.octave, 'class_id': x.class_id}, keys))
        dump({'length': length, 'width': width, 'descs': descs, 'keys': obj}, \
            output, HIGHEST_PROTOCOL)

        print('dumped keys and descs to \'map.bin\'')
        output.close()

if __name__ == "__main__":
    main()
