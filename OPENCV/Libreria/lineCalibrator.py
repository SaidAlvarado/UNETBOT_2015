__author__ = 'Said'

import cv2
import numpy as np
import sys
import math
import random
import warnings

def nothing(x):
    pass


#Evitamos que falten argumentos
if len(sys.argv) < 2:
    print ("Necesitas especificar el nombre del archivo cargar")
    exit()

warnings.filterwarnings("ignore")


#Initial variables
Separation = 207
Angle = 50
Roi_Line_Size = 98
Lateral_Offset = -7



cap = cv2.VideoCapture(sys.argv[1])
ret,frame = cap.read()    #Definimos unas variables con el tamano de la imagen
frame = cv2.resize(frame,None,fx=.5, fy=.5, interpolation = cv2.INTER_AREA)
cv2.namedWindow('image')


print "{}x{}x{}".format(frame.shape[0], frame.shape[1], frame.shape[2])

cv2.createTrackbar('Separation', 'image', 0, frame.shape[1], nothing)
cv2.createTrackbar('Angle', 'image', 0, 90, nothing)
cv2.createTrackbar('Roi Size', 'image', 1, 200, nothing)
cv2.createTrackbar('Offset', 'image', 0, frame.shape[1], nothing)

#Put on the values to search for
cv2.setTrackbarPos('Separation', 'image',Separation)
cv2.setTrackbarPos('Angle', 'image',Angle)
cv2.setTrackbarPos('Roi Size', 'image',Roi_Line_Size)
cv2.setTrackbarPos('Offset', 'image',320 + Lateral_Offset)

while(1):

    ret,frame = cap.read()    #Definimos unas variables con el tamano de la imagen

    if frame == None or ret == None:
        cap = cv2.VideoCapture(sys.argv[1])
        ret,frame = cap.read()


    frame = cv2.resize(frame,None,fx=.5, fy=.5, interpolation = cv2.INTER_AREA)

    separation = cv2.getTrackbarPos('Separation', 'image')
    angle = cv2.getTrackbarPos('Angle', 'image')
    roiSize = cv2.getTrackbarPos('Roi Size', 'image')
    offset = cv2.getTrackbarPos('Offset', 'image')


    # Caculamos las posiciones de las lineas
    linDer_center = [frame.shape[1]/2 + separation + offset - frame.shape[1]/2, frame.shape[0]/2]
    linIzq_center = [frame.shape[1]/2 - separation + offset - frame.shape[1]/2, frame.shape[0]/2]


    # Calculamos la desviacion angular
    offsetX = frame.shape[0]/2*math.tan(math.radians(angle))

    #Calculamos el segundo punto de las rectas.
    linDer_end = [frame.shape[1]/2 + int(offsetX) + separation + offset - frame.shape[1]/2, frame.shape[0]]
    linIzq_end = [frame.shape[1]/2 - int(offsetX) - separation + offset - frame.shape[1]/2, frame.shape[0]]

    #Calculamos el tercer punto de la recta
    linDer_top = [frame.shape[1]/2 - int(offsetX) + separation + offset - frame.shape[1]/2, 0]
    linIzq_top = [frame.shape[1]/2 + int(offsetX) - separation + offset - frame.shape[1]/2, 0]


    #Creamos la mascara
    mask = np.zeros_like(frame)
    cv2.line(mask,tuple(linDer_top),tuple(linDer_end), (255,255,255), roiSize)
    cv2.line(mask,tuple(linIzq_top),tuple(linIzq_end), (255,255,255), roiSize)
    roi = np.bitwise_and(frame,mask)


    #Solo chequeando que los circulos estuvieran donde deben
    # cv2.circle(roi,tuple(linDer_top),10,(255,0,255),thickness = 5)
    # cv2.circle(roi,tuple(linIzq_top),10,(255,0,255),thickness = 5)
    # cv2.circle(roi,tuple(linDer_center),10,(0,0,255),thickness = 5)
    # cv2.circle(roi,tuple(linIzq_center),10,(0,0,255),thickness = 5)
    # cv2.circle(roi,tuple(linDer_end),10,(0,255,255),thickness = 5)
    # cv2.circle(roi,tuple(linIzq_end),10,(0,255,255),thickness = 5)


    resultado = np.hstack((frame,roi))


    cv2.imshow('image',resultado)

    #time.sleep(0.2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print "Separation = {} ".format(separation)
print "Angle = {} ".format(angle)
print "Roi Line Size = {}".format(roiSize)
print "Lateral Offset = {}".format(offset - frame.shape[1]/2)

cv2.destroyAllWindows()


# Separation = 207
# Angle = 50
# Roi Line Size = 98
# Lateral Offset = -7
