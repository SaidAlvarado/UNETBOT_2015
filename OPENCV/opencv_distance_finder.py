import numpy as np
import cv2
import random
#Importamos el script local
from opencv_support import *



cap = cv2.VideoCapture("udpsrc port=5000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false")

ret,frame = cap.read()    #Definimos unas variables con el tamano de la imagen
height = frame.shape[0]
width = frame.shape[1]
# print ('w{} x h{}'.format(width,height))


while(True):

    # Capture frame-by-frame
    ret, frame = cap.read()

    # Detectamos las lineas de transmision y las mostramos en la pantalla.
    lineaIzq,lineaDer =  line_detector(frame)
    # print ("LineaDer = {},  lineaIzq = {}".format(lineaDer,lineaIzq))
    frame_copy = frame.copy()
    draw_lines(frame_copy,[lineaDer,lineaIzq],(255,255,255), 1)

    # buscamos las fallas sobre la linea
    fallas = object_finder(frame, ['rojo','amarillo','blanco'], (lineaIzq,lineaDer))

    for color in fallas:
        i = 1
        for cenX,cenY in fallas[color]:
            cv2.circle(frame_copy,(cenX,cenY),40,coloresDibujo[color],3)
            cv2.putText(frame_copy, "{} #{}".format(color,i), (cenX + 45, cenY), cv2.FONT_HERSHEY_SIMPLEX,1.0, coloresDibujo[color], 2)
            i+=1


### Ahora viene el calculo de lineas paralelas ###










    # Unimos las imagenes para hacer un display simultaneo y rescalamos
    # im0 = np.dstack((mask_blanco,mask_blanco,mask_blanco))
    # im1 = np.dstack((gray,gray,gray))
    # im2 = np.dstack((mask,mask,mask))
    # resultado = np.hstack((frame_copy,roi))
    # resultado = np.vstack((resultado,np.hstack((im0,img_blanco))))
    # resultado = cv2.resize(resultado,None,fx=0.4, fy=0.4, interpolation = cv2.INTER_AREA)
    resultado = frame_copy

    # Display the resulting frame
    cv2.imshow('frame',resultado)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
