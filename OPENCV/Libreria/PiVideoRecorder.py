# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from fractions import Fraction
import argparse

#Build the argument parser
ap = argparse.ArgumentParser()
ap.add_argument("-o", "--out", help="Name of the recorded file")
ap.add_argument("-t", "--time", type=int, default=5, help="Recording time")
ap.add_argument("-x", "--width", type=int, default=640, help="Horizontal resolution of the video")
ap.add_argument("-y", "--height", type=int, default=480, help="Vertical resolution of the video")
args = vars(ap.parse_args())

resolution = (args['width'],args['height'])

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = resolution
camera.framerate = 20
# Shut down Auto white Balancig
camera.awb_mode = 'off'
camera.awb_gains = (Fraction(7, 4), Fraction(249, 256))
# High speed mode
camera.exposure_mode = 'sports'
rawCapture = PiRGBArray(camera, size=resolution)


# allow the camera to warmup
time.sleep(0.1)

#We set up the opencv recording object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(args['out'],fourcc, 10.0, resolution)

# capture frames from the camera
start_time = time.time()
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    frame_time = time.time()
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array

    out.write(image)



    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    if time.time() - start_time > args['time']:
        out.release()
        break

    while ((time.time() - frame_time) < 1.0/10.0):
        pass


