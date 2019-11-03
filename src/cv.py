'''
cv simulation and module for VTOL
'''

import re
from os import listdir, environ
from threading import Thread
import time
import subprocess
import cv2

class Vision():
    ''' vision class for managing cv '''
    image_counter = 0

    @classmethod
    def sorted_alphanumeric(cls, data):
        ''' sorts data alphanumerically'''
        convert = lambda text: int(text) if text.isdigit() else text.lower()
        alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
        return sorted(data, key=alphanum_key)

    def cv_simulation(self, configs):
        '''simulation for VTOL cv'''
        files = self.sorted_alphanumeric(listdir(configs["cv_simulated"]["directory"]))
        path = configs["cv_simulated"]["directory"] + "/" + files[self.image_counter % len(files)]
        img = cv2.imread(path, 1)
        self.image_counter += 1
        return img

    @classmethod
    def init_camera(cls, configs):
        '''initialize camera for 3DR Solo'''
        if configs["3dr_solo"]:
            solo_connect_thread = Thread(target=lambda _: \
                subprocess.check_output(["nc", "10.1.1.1", "5502"]))
            solo_connect_thread.daemon = True
            solo_connect_thread.start()
            time.sleep(1)
            environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'protocol_whitelist;file,rtp,udp'
            cam = cv2.VideoCapture("./sololink.sdp")
            return cam
        # initialize camera for Raspberry PI
        from picamera import PiCamera #pylint: disable=import-outside-toplevel,import-error
        cam = PiCamera() #pylint: disable=import-error
        return cam


    @classmethod
    def take_picture(cls, camera, configs):
        '''takes picture using config'''
        if configs["3dr_solo"]:
            return camera.read()
        from picamera.array import PiRGBArray #pylint: disable=import-outside-toplevel,import-error
        raw_capture = PiRGBArray(camera) #pylint: disable=import-error
        camera.capture(raw_capture, format="bgr")
        return raw_capture.array
        