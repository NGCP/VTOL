import cv2
from pickle import dump, HIGHEST_PROTOCOL, load
import sys
from deepdiff import DeepDiff
import json

# Quick configs
SCALE = 0.5

orb = cv2.ORB_create(nfeatures=100000, scoreType=cv2.ORB_FAST_SCORE)

areaMap = cv2.imread("images/stitch_flip.jpg")
areaMap = cv2.resize(areaMap, (0, 0), fx = SCALE, fy = SCALE)
areaMap = cv2.cvtColor(areaMap, cv2.COLOR_BGR2GRAY)

# find the keypoints and descriptors with ORB
keys, descs = orb.detectAndCompute(areaMap, None)
length, width = areaMap.shape

with open('map.bin', 'wb') as output:  # Overwrites any existing file.
    ob = (map(lambda x: {'pt': x.pt, 'size': x.size, 'angle': x.angle, 'response': x.response, 'octave': x.octave, 'class_id': x.class_id}, keys))
    dump({'length': length, 'width': width, 'descs': descs, 'keys': ob}, output, HIGHEST_PROTOCOL)

    print('dumped keys and descs to \'map.bin\'')
    output.close()
