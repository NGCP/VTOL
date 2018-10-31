from picamera import PiCamera
from time import sleep
#Code referenced from picamera.readthedocs.io
def main():
    camera = PiCamera()
    camera.iso = 200 #How should iso. Are missions always during the day?
    camera.shutter_speed = camera.exposure_speed
    g = camera.awb_gains
    camera.exposure_mode ='off'
    camera.awb_gains = g
    camera.resolution = (1024, 768)
    camera.start_preview()
    i = 0 #counter for the images
    while True:
        sleep(.1)
        camera.capture('/home/pi/camtestdir/image_%s.jpg' % i)
        i+=1
    camera.stop_preview()
if __name__ == "__main__":
    main()
