# this imports the camera

from picamera import PiCamera
import numpy as np
import cv2
from time import sleep

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
#initialize

camera = PiCamera()

def testCamera():
    print("Camera test")
    camera.start_preview()
    sleep(5)
    #we capture to openCV compatible format
    #you might want to increase resolution
    camera.resolution = (750, 750)
    camera.framerate = 24
    sleep(2)
    image = np.empty((750, 750, 3), dtype=np.uint8)
    camera.capture(image, 'bgr')
    cv2.imwrite('out.png', image)
    camera.stop_preview()
    print("saved image to out.png")

if __name__ == '__main__':
    testCamera()

    # img = mpimg.imread('out.png')
    # imgplot = plt.imshow(img)
    # plt.show()
