import cv2
import re
from os import listdir

#Globals
imgCounter = 0

def sorted_aphanumeric(data):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
    return sorted(data, key=alphanum_key)

def cv_simulation(configs):
    global imgCounter
    files = sorted_aphanumeric(listdir(configs["cv_simulated"]["directory"]))
    path = configs["cv_simulated"]["directory"] + "/" + files[imgCounter % len(files)]
    img = cv2.imread(path, 1)
    imgCounter += 1
    return img
