import cv2
import re
from os import listdir, environ
from threading import Thread
import time
import subprocess

img_counter = 0

def sorted_alphanumeric(data):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ] 
    return sorted(data, key=alphanum_key)

def cv_simulation(configs):
    global img_counter
    files = sorted_alphanumeric(listdir(configs["cv_simulated"]["directory"]))
    path = configs["cv_simulated"]["directory"] + "/" + files[img_counter % len(files)]
    img = cv2.imread(path, 1)
    img_counter += 1
    return img

def connect_solo_wifi():
   # execute the shell script (does not terminate)
   subprocess.check_output(["nc", "10.1.1.1", "5502"])


def init_camera(configs):
   # initialize camera for 3DR Solo
   if configs["3dr_solo"]:
       solo_connect_thread = Thread(target=connect_solo_wifi)
       solo_connect_thread.daemon = True
       solo_connect_thread.start()
       time.sleep(1)
       environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'protocol_whitelist;file,rtp,udp'
       cam = cv2.VideoCapture("./sololink.sdp")
       return cam
   # initialize camera for Raspberry PI
   else:
       from picamera import PiCamera
       cam = PiCamera()
       return cam


def take_picture(camera, configs):
   if configs["3dr_solo"]:
       return camera.read()
   else:
       from picamera.array import PiRGBArray
       raw_capture = PiRGBArray(camera)
       camera.capture(raw_capture, format="bgr")
       return raw_capture.array
      
