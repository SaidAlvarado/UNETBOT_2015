"""
17/07/2015 10:25pm  Puntofijo
Buscar una mejor forma de calcular los puntos en la linea para que no se recalculen tanto.

Buscar reorganizar el codigo para que agrupe  "CLUMPS" lineas de caracteristicas similares y que cual se muestre se decia por el mayor CLUMP
La forma actual muestra problemas cuando lineas empiezan junto con las otras per tienen pendientes locas (lineas verticales de las cortinas)

Robustecer el codigo para que no tenga excepciones cuando desaparezcan las lineas, si no que mande un mensaje.

Hacer la ventana para empezar el analisis de reconocimiento de color



18/07/2015 10:51pm Puntofijo

El agrupador por cumulos funciona relativamente bien, tengo el problema que tengo cumulos repetidos, que no logro unir,
Necesito averiguar como hago para unir listas de lineas que sean suficientemente iguales.


Me acabo de darcuenta que las excepciones ocurren con lineas muy horizontales, el valor de corte con el eje superior y el inferior
son tan elevados que opencv no las puede graficar. Necesito actualizar la funcion de graficas para que sea mas inteligente respecto a
como grafica las lineas. y para que le puedas especificar el grosor de la misma.



21/07/2015 2:05pm Mecatronica

Eureka! quizas a las listas no se les puede aplicar un HASH, pero a las tuplas si. asi que una lista de tuplas es una buena opcion
para acomodar los vecindarios.

"""






def polar2poly(rt):
    """ Funcion que toma un tupla de valores (rho,theta) que salen de la transformada de linea de Hough,
        y usando la funcion de regresion linear 'polyfit', calcula la ecuacion  Y = mX + b de la linea, en pixeles
        y devuelve la lista [m, b] """

    rho,theta = rt              # Desempaquetamos la tupla
    # if theta == 0: return None  # Ignoramos las lineas de pendiente infinita

    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho

    x1 = x0 + 1000*(-b)         # Generamos los puntos para la ecuacion de Polyfit
    y1 = y0 + 1000*(a)
    x2 = x0 - 1000*(-b)
    y2 = y0 - 1000*(a)

    eq = np.polyfit([x1,x2],[y1,y2],1)                  # Usamos Polyfit para encontrar la ecuacion de la recta de las lineas
                                                        # Eq[0] es la pendiente, y Eq[1] es el corte en Y
    # return tuple([round(eq[0],5),round(eq[1],5)])
    return tuple(eq)

def draw_lines(frame, lines, colors = None, size = 2):
    """Esta funcion recibe una imagen y unas lineas definidas como listas [pendiente corte_con_y] y dibujandolas con un color
       aleatorio o dado por el usuario"""

    if type(lines) == list or tuple:
        for eq in lines:
            y1 = int(0)
            x1 = int((y1 - eq[1])/eq[0])
            y2 = int(frame.shape[0])
            x2 = int((y2 - eq[1])/eq[0])

            if (abs(x1) > 5E4 or abs(x2) > 5E4):
                x1 = int(0)
                y1 = int(eq[0]*x1 + eq[1])
                x2 = int(1280)
                y2 = int(eq[0]*x2 + eq[1])

            if colors == None:
                color =(random.randint(0,255),random.randint(0,255),random.randint(0,255))
            else:
                color = colors
                # print "x1 = {} x2 = {} y1 = {} y2 = {}".format(x1,x2,y1,y2)
            cv2.line(frame,(x1,y1),(x2,y2),color,size)
    else:
        y1 = int(0)
        x1 = int((y1 - lines[1])/lines[0])
        y2 = int(frame.shape[0])
        x2 = int((y2 - lines[1])/lines[0])

        if (abs(x1) > 5E4 or abs(x2) > 5E4):
            x1 = int(0)
            y1 = int(eq[0]*x1 + eq[1])
            x2 = int(1280)
            y2 = int(eq[0]*x2 + eq[1])

        if colors == None:
            color =(random.randint(0,255),random.randint(0,255),random.randint(0,255))
        else:
            color = colors
        cv2.line(frame,(x1,y1),(x2,y2),color,size)




import numpy as np
import cv2
import random

cap = cv2.VideoCapture("udpsrc port=5000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false")

ret,frame = cap.read()    #Definimos unas variables con el tamano de la imagen
height = frame.shape[0]
width = frame.shape[1]

print ('w{} x h{}'.format(width,height))

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Our operations on the frame come here
    # Pasamos la imagen a blanco y negro
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Calculamos la derivada en x de la imagen.
    # Operador de Sobel
    sobelx = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=3)
    sobelx = np.uint8(np.absolute(sobelx))

    # Filtramos un poco la imagen para quitar el ruido
    # blurred = cv2.GaussianBlur(sobelx, (7, 7), 0)
    # blurred = cv2.bilateralFilter(gray,9,75,75)

    # Tomamos un threshold para eliminar el ruido de la imagen
    # thres = cv2.adaptiveThreshold(sobelx,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,3,2)
    ret,thres = cv2.threshold(sobelx,27,255,cv2.THRESH_BINARY)

    # edges = cv2.Canny(blurred,50,150)

    # Hacemos una operacion morfologica de Apertura para quitar el ruido restante, kernel 5x5
    # kernel = np.ones((2,2),np.uint8)
    # opening = cv2.morphologyEx(thres, cv2.MORPH_OPEN, kernel, iterations=1)


    #Aplicamos la transformada de lineas probabilisticas de Hough para intentar ubicar las lineas
    # minLineLength = 100
    # maxLineGap = 10
    # lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
    # for x1,y1,x2,y2 in lines[0]:
    #     cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)

    # print len(lines[0])

    lines = cv2.HoughLines(thres,1,np.pi/180,200)           # antes estaba en 300
    # print(len(lines[0]))
    # theta1 = np.mean(theta1)
    # rho1 = np.mean(rho1)

    #Conseguimos las ecuaciones de todas las rectas
    linesp = [ polar2poly(rt) for rt in lines[0]]         #Conseguimos las ecuaciones de todas las rectas

    cumulos = []
    deltaM = 4
    deltaB = 200
    minSizeClump = 4
    linesp.sort(cmp = lambda x,y: int(y[1] - x[1]))
    for linea in linesp:
        vecindario = [ x for x in linesp if ((abs(x[0] - linea[0]) <  deltaM) and (abs(((height - x[1])/x[0]) - ((height - linea[1])/linea[0])) < deltaB))]
        dummy = [linesp.remove(x) for x in vecindario]
        cumulos.append(vecindario)


    # for linea in linesp:
    #     vecindario = [ x for x in linesp if ((abs(x[0] - linea[0]) <  deltaM) and (abs(x[1] - linea[1]) < deltaB))]     #Agregamos todas las  lineas suficientemente parecidas
    #     if len(vecindario) < minSizeClump: continue                                                                     #si tiene muy pocos vecinos, la ignoramos
    #     vecindario.sort()
    #     # print "vencindario = {}".format(vecindario)                                                                   #Ordenamos el vecindario
    #     if vecindario not in cumulos: cumulos.append(vecindario)                                                        #Si no esta repetido, lo agregamos


    # Hora de filtrar y unir vecindarios!!!!

    # cumulos = map(lambda x: set(x), cumulos)        # Convertimos a las listas de cumulos, en sets de tuplas
    #                                                 # Iteramos dos veces por los sets, y unimos los sets que sean mas  de 30% iguales
    # dummy = [x.update(y) for x in cumulos \
    #                      for y in cumulos \
    #                      # if float(len(x.intersection(y)))/float(min(len(x),len(y))) > 0.2]
    #                      if len(x.intersection(y)) >= minIntersect]

    # cumulos = list(set([tuple(x) for x in cumulos]))  # Convertimos todo "cumulos" en un set para eliminar los repetidos.

    cumulos.sort(cmp = lambda x,y: len(y) - len(x))   # Los ordenamos por tamano.


    # Ahora que tenemos todos los cumulos de lineas intentaremos sacar los mayores
    # for x in cumulos:
    #     color =(random.randint(0,255),random.randint(0,255),random.randint(0,255))
    #     draw_lines(frame,x,color)

    # Calculamos el promedio de los dos cumulos mayores, y dibujamos la mejor aproximacion
    lineaIzq = [[x[0] for x in cumulos[0]], [x[1] for x in cumulos[0]]]
    lineaIzq = [np.mean(lineaIzq[0]),np.mean(lineaIzq[1])]
    lineaDer = [[x[0] for x in cumulos[1]], [x[1] for x in cumulos[1]]]
    lineaDer = [np.mean(lineaDer[0]),np.mean(lineaDer[1])]
    # print ("LineaDer = {},  lineaIzq = {}".format(lineaDer,lineaIzq))
    draw_lines(frame,[lineaDer,lineaIzq],(0,255,0), 6)
    # draw_lines(frame,lineaIzq,(0,255,0),size = 3)

    ######
    ###### Codigo exesivamente largo para dibujar en verde las 2 lineas centrales.
    ######
    # p1 = [int((0-meanDer[1])/meanDer[0]),0]
    # p2 = [int((720-meanDer[1])/meanDer[0]),720]
    # cv2.line(frame,(p1[0],p1[1]),(p2[0],p2[1]),(0,255,0),5)

    # p1 = [int((0-meanIzq[1])/meanIzq[0]),0]
    # p2 = [int((720-meanIzq[1])/meanIzq[0]),720]
    # cv2.line(frame,(p1[0],p1[1]),(p2[0],p2[1]),(0,255,0),5)


    # Unimos las imagenes para hacer un display simultaneo y rescalamos
    im0 = np.dstack((thres,thres,thres))
    im1 = np.dstack((sobelx,sobelx,sobelx))
    im2 = np.dstack((gray,gray,gray))
    resultado = np.hstack((frame,im2))
    resultado = np.vstack((resultado,np.hstack((im1,im0))))
    resultado = cv2.resize(resultado,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_AREA)
    # resultado = frame

    # Display the resulting frame
    cv2.imshow('frame',resultado)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


"""
    #Eliminamos las lineas de inclinacion menor a 45 grados
    linesp = [x for x in linesp if (x[0] > 1) or (x[0] < -1) ]

    #Aislamos las lineas de la derecha
    linDer = [x for x in linesp if (((height - x[1])/x[0]) - width/2) > 0 ]
    #Aislamos las lineas de la izquiera
    linIzq = [x for x in linesp if (((height - x[1])/x[0]) - width/2) < 0 ]

    #Buscamos el valor del borde inferior mas cercano al centro de la pantalla
    derMin = min([((height - x[1])/x[0]) for x in linDer])
    izqMax = max([((height - x[1])/x[0]) for x in linIzq])

    #print("izqMax = {}  ,   derMin = {}".format(izqMax,derMin))

    # Borramos todos las lineas que esten demasiado lejos del centro
    linDer = [x for x in linDer if ((height - x[1])/x[0]) < derMin + 50 ]
    linIzq = [x for x in linIzq if ((height - x[1])/x[0]) > izqMax - 50 ]

    #Promediamos todas las lineas de la derecha y todas las de la izquierda para obtener el estimado central.
    meanDer = [ np.mean([x[0] for x in linDer]), np.mean([x[1] for x in linDer])]
    meanIzq = [ np.mean([x[0] for x in linIzq]), np.mean([x[1] for x in linIzq])]

    lineas = linDer + linIzq
"""
