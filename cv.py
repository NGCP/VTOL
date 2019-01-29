import cv2
from os import listdir

#Globals
imgCounter = 0

def cv_simulation(configs):
    global imgCounter
    files = listdir(configs["cv_simulated"]["directory"])
    path = configs["cv_simulated"]["directory"] + "/" + files[imgCounter % len(files)]
    img = cv2.imread(path, 1)
    imgCounter += 1
    return img
