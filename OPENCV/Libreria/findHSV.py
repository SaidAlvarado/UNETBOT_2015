__author__ = 'Said'

import cv2
import numpy as np
import warnings
import sys

def nothing(x):
    pass


#Evitamos que falten argumentos
if len(sys.argv) < 2:
    print ("Necesitas especificar el nombre del archivo cargar")
    exit()

warnings.filterwarnings("ignore")

#Predetermined values
Hmi = 20
Smi = 34
Vmi = 0

Hma = 118
Sma = 152
Vma = 141



cap = cv2.VideoCapture(sys.argv[1])
ret,frame = cap.read()    #Definimos unas variables con el tamano de la imagen
frame = cv2.resize(frame,None,fx=.4, fy=.4, interpolation = cv2.INTER_AREA)
cv2.namedWindow('image')

# print "{}x{}x{}".format(frame.shape[0], frame.shape[1], frame.shape[2])

cv2.createTrackbar('Hmin', 'image', 0, 179, nothing)
cv2.createTrackbar('Hmax', 'image', 0, 179, nothing)

cv2.createTrackbar('Smin', 'image', 0, 255, nothing)
cv2.createTrackbar('Smax', 'image', 0, 255, nothing)

cv2.createTrackbar('Vmin', 'image', 0, 255, nothing)
cv2.createTrackbar('Vmax', 'image', 0, 255, nothing)

#Put on the values to search for
cv2.setTrackbarPos('Hmin', 'image',Hmi)
cv2.setTrackbarPos('Hmax', 'image',Hma)
cv2.setTrackbarPos('Smin', 'image',Smi)
cv2.setTrackbarPos('Smax', 'image',Sma)
cv2.setTrackbarPos('Vmin', 'image',Vmi)
cv2.setTrackbarPos('Vmax', 'image',Vma)

while(1):

    ret,frame = cap.read()    #Definimos unas variables con el tamano de la imagen

    if frame == None or ret == None:
        cap = cv2.VideoCapture(sys.argv[1])
        ret,frame = cap.read()

    frame = cv2.resize(frame,None,fx=.4, fy=.4, interpolation = cv2.INTER_AREA)

    Hmi = cv2.getTrackbarPos('Hmin', 'image')
    Smi = cv2.getTrackbarPos('Smin', 'image')
    Vmi = cv2.getTrackbarPos('Vmin', 'image')

    Hma = cv2.getTrackbarPos('Hmax', 'image')
    Sma = cv2.getTrackbarPos('Smax', 'image')
    Vma = cv2.getTrackbarPos('Vmax', 'image')

    lower_bound = np.array([Hmi, Smi, Vmi])
    upper_bound = np.array([Hma, Sma, Vma])


    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv,lower_bound,upper_bound)
    res = cv2.bitwise_and(frame,frame, mask = mask)

    mask3 = np.dstack((mask,mask,mask))

    blank2 = np.vstack((res,mask3))
    blank = np.vstack((frame,hsv))       #pegamos las 2 imagenes una sobre otra



    blank3 = np.hstack((blank,blank2))

    # print blank3.shape

    cv2.imshow('image',blank3)

    #time.sleep(0.2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print "Lower Bound =  Hmin: {}   Smin: {}   Vmin: {}".format(Hmi,Smi,Vmi)
print "Higher Bound=  Hmax: {}   Smax: {}   Vmax: {}".format(Hma,Sma,Vma)

cv2.destroyAllWindows()


