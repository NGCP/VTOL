'''The purpose of this script is to have the PiCamera automatically take photos
when the Raspberry Pi turns on at the start of a mission.'''

from picamera import PiCamera #pylint: disable=import-error


def main():
    '''demo picture taking on rpi'''
    camera = PiCamera()
    camera.iso = 150  # Standard ISO values are 100/200 for similar aperature
    camera.shutter_speed = camera.exposure_speed
    gain = camera.awb_gains
    camera.exposure_mode = 'off'
    camera.awb_gains = gain
    camera.resolution = (1024, 768)
    i = 0  # counter for the images
    while True:
        camera.capture('/home/pi/camtestdir/image_%s.jpg' % i)
        i += 1


if __name__ == "__main__":
    main()
