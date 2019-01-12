import cv2
from os import listdir

#Globals
imgCounter = 0

def cv_simulation(configs):
    global imgCounter
    files = listdir(configs["cv_simulated"]["directory"])
    img = cv2.imread(files[imgCounter % len(files)], 0)
    imgCounter += 1
    return img