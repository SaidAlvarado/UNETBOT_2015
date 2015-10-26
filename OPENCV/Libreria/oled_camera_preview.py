# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import ssd1351

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 40
rawCapture = PiRGBArray(camera, size=(640, 480))

#Start OLED
oled = ssd1351.SSD1351()
oled.begin()

# allow the camera to warmup
time.sleep(0.1)

last_time = time.time()
cuenta = 0
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array

    image_small = cv2.resize(image,(128,128), interpolation = cv2.INTER_AREA)
    image565 = oled.convertBitmap565(image_small)
    oled.drawBitmap(image565,0,0)
    # show the frame
    # color = image[image.shape[0]/2][image.shape[1]/2]
    # oledColor = oled.color565((color[2], color[1],color[0]))

    # oled.fillScreen(oledColor)
    cuenta += 1
    if (time.time() - last_time > 1):
        print "fps = {}".format(cuenta)
        cuenta = 0
        last_time = time.time()

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)



