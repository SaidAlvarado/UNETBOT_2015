import numpy as np
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time


# inicializar camara
camera = PiCamera()
camera.framerate = 20
time.sleep(2)
camera.exposure_mode = 'sports'

rawCapture = PiRGBArray(camera)
# Dejar que la camara arranque
time.sleep(0.1)

awb = camera.awb_gains
print("awb = {}\n".format(awb))

camera.capture(rawCapture, format="bgr")
frame = rawCapture.array

cv2.imwrite('awb_test.png',frame)
