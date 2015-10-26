"""
Medidor de Distancia - toma todos los puntos de fallas detectados en la funcion anterior.
Tiene dos partes:

Deteccion de lineas perpendiculares. Se usan transformaciones de perspectiva para encontrar la recta perpendicular entre las lineas de transmision.
Para luego destransformar y usar la ecuacion de la distancia focal para encontrar la distancia al objetivo.

NOOOOOOOOTAAAAAAAAAAA!!!!! VER SI SE PUEDE CALCULAR EL ANGULO DE INCLINACION A PARTIR DE LA MATRIZ DE PERSPECTIVA.


IDEAAAAAA!!!!!       Usar los aisladores de las torres para sacar las rectas  paralelas reales.


"""


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

    # buscamos las fallas sobre la linea.
    fallas = object_finder(frame, ['rojo','amarillo','blanco'], (lineaIzq,lineaDer))



### Ahora viene el calculo de lineas paralelas ###

    # Variables importantes
    distaciaFocal = 1286    #1286.07
    largoEntreLineas = 25   #cm
    altura_camara = 11.2 #cm

    # Primero calculamos los cuatro puntos limites de las lineas electricas.
    rectOriginal = np.float32(perspective_bound(frame, [lineaIzq,lineaDer]))
    # Ahora definimos los limites de la nueva imagen
    rectPerspective = np.float32([[0,0], [height,0], [0,height], [height,height]])
    # Calculamos la matriz de perspectiva
    if lineaIzq == None:
        frame_perspective = np.zeros((height,height,3))
    else:
        M = cv2.getPerspectiveTransform(rectOriginal,rectPerspective)
        # Transformamos la imagen para ver como queda
        frame_perspective = cv2.warpPerspective(frame,M, (height,height))

        # #Iteramos sobre todos los puntos que encontramos para encontrar su paralela
        for color in fallas:
            for centro in fallas[color]:
                #Transformamos el centroide del
                centro_perspectiva = cv2.perspectiveTransform(np.array([[centro[0]]],dtype = 'float32'),M)    #Los puntos que le entran a la funcion perspectiveTransform, tienen que ser de la forma [[[x1,y1],[x2,y2, ...]]], con ESA cantidad de parentesis.
                # Calculamos el mismo punto en la otra linea
                centro_perspectiva = centro_perspectiva[0][0].copy()
                if centro_perspectiva[0] < height/2:
                    punto_contrario_perspectiva = np.array([height, centro_perspectiva[1]])
                else:
                    punto_contrario_perspectiva = np.array([0, centro_perspectiva[1]], dtype = 'float')
                # Destransformamos el punto
                # print "M = {}".format(M)
                # print "invM = {}".format(cv2.invert(M))
                punto_contrario = cv2.perspectiveTransform(np.array([[punto_contrario_perspectiva]],dtype = 'float32'),cv2.invert(M)[1])
                # Graficamos, las lineas que encontramos
                punto_contrario = punto_contrario[0][0].copy()
                cv2.line(frame_copy,tuple(centro[0]),tuple(punto_contrario),coloresDibujo['azul'],2)
                # Calculamos la distancia en pixeles
                distPixeles = np.sqrt(np.square(centro[0][0] - punto_contrario[0]) + np.square(centro[0][1] - punto_contrario[1]))
                # Usamos la ecuacion de similitud triangular para sacar la distancia
                distCamara = largoEntreLineas*distaciaFocal/distPixeles
                # Ahora usamos la relacion de pitagoras para calcular la distancia de la falla al chasis
                distChasis = np.sqrt(np.square(distCamara) - np.square(altura_camara))
                # guardamos la distancia en centimetros en el diccionario de fallas

                centro[1] = distChasis

        # Ahora imprimimos y mostramos todo.
        # Pequeno for para graficar las fallas y sus distancias.
        for color in fallas:
            i = 1
            for objeto in fallas[color]:
                cv2.circle(frame_copy,tuple(objeto[0]),40,coloresDibujo[color],3)
                cv2.putText(frame_copy, "{} #{}".format(color,i), (objeto[0][0] + 45, objeto[0][1]), cv2.FONT_HERSHEY_SIMPLEX,1.0, coloresDibujo[color], 2)
                cv2.putText(frame_copy, "{:0.2f}cm".format(objeto[1]), (objeto[0][0] + 45, objeto[0][1] + 20), cv2.FONT_HERSHEY_SIMPLEX,0.7, coloresDibujo[color], 2)
                i+=1








    # Unimos las imagenes para hacer un display simultaneo y rescalamos
    # im0 = np.dstack((mask_blanco,mask_blanco,mask_blanco))
    # im1 = np.dstack((gray,gray,gray))
    # im2 = np.dstack((mask,mask,mask))
    # resultado = np.hstack((frame_copy,frame_perspective))
    # resultado = np.vstack((resultado,np.hstack((im0,img_blanco))))
    # resultado = cv2.resize(resultado,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_AREA)
    resultado = frame_copy

    # Display the resulting frame
    cv2.imshow('frame',resultado)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
