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


23/07/2015 3:30pm Mecatronica

Jorge se le ocurrio una solucion genial! usando list.remove() a medida que se itera la lista puedes eliminar aquellas lineas que ya se les
asigno un vecindario, y evitar las repeticiones. Luego puedes felizmente tomar los dos cumulos mas grandes (que son los primeros porque le hicimos
sort al arreglo) y promediarlos.

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


def line_detector(frame):

    # Pasamos la imagen a blanco y negro
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Calculamos la derivada en x de la imagen. con Sobel (Asi ignoramos las lineas horizontales en la imagen)
    sobelx = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=3)
    sobelx = np.uint8(np.absolute(sobelx))
    # Hacemos un Threshold binario normal.
    ret,thres = cv2.threshold(sobelx,27,255,cv2.THRESH_BINARY)
    # Usamos la imagen binaria del Threshold, y la pasamos por la transformada lenta de Hough
    lines = cv2.HoughLines(thres,1,np.pi/180,200)

    ### Aqui empezamos a deducir las lineas, apartir del procesamiento de imagenes de arriba ###

    #Conseguimos las ecuaciones de todas las rectas en forma polinomial
    linesp = [ polar2poly(rt) for rt in lines[0]]
    # Definimos las variables que necesitamos para ordenar y filtrar las lineas.
    cumulos = []        # Arreglo donde se van a guardar los vecindarios.
    deltaM = 4          # Desviacion maxima de la pendiente de un solo vecindario.
    deltaB = 200        # Desviacion maxima del corte con el fondo de la imagen para un vecindario.
    # Ordenamos las lineas de izquierda a derecha acorde a su punto de corte con el fondo de la imagen.
    linesp.sort(cmp = lambda x,y: int(y[1] - x[1]))
    # Recorremos cada linea del arreglo y las agrupamos de acuerdo a su similitud con otras lineas.
    for linea in linesp:
        # Linea a linea guardamos un vecindario, si y solo si, la desviacion de los parametros sea menor a deltaM y delta B.
        vecindario = [  x for x in linesp if ((abs(x[0] - linea[0]) <  deltaM) and (abs(((height - x[1])/x[0]) - ((height - linea[1])/linea[0])) < deltaB))]
        # Borramos del arreglo original todas las lineas que se asignaron a un vecindario. (Nota, la variable "dummy" es porque list.remove() no devuelve nada)
        dummy = [linesp.remove(x) for x in vecindario]
        # Agregamos el vecindario al cumulo.
        cumulos.append(vecindario)
    # Ordenamos los vecindarios de mayor a menor
    cumulos.sort(cmp = lambda x,y: len(y) - len(x))


    # Decomentar para graficar todas las lineas de los cumulos mas grandes.
    # for x in cumulos[0:2]:
    #     color =(random.randint(0,255),random.randint(0,255),random.randint(0,255))
    #     draw_lines(frame,x,color)


    # Calculamos el promedio de los dos cumulos mayores, y los asignamos a las dos lineas de transmision.
    # cada linea es una lista con pendiente y offset. Asi que acumulamos todas las pendientes en lineaXXX[0], y todas
    # los offset en lineaXXX[1]. y les sacamos el promedio.
    if len(cumulos) >= 2:
        lineaIzq = [np.mean([x[0] for x in cumulos[0]]), np.mean([x[1] for x in cumulos[0]])]
        lineaDer = [np.mean([x[0] for x in cumulos[1]]), np.mean([x[1] for x in cumulos[1]])]
    else:
        lineaIzq = None
        lineaDer = None

    return [lineaIzq, lineaDer]





import numpy as np
import cv2
import random

cap = cv2.VideoCapture("udpsrc port=5000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false")

ret,frame = cap.read()    #Definimos unas variables con el tamano de la imagen
height = frame.shape[0]
width = frame.shape[1]
lineaIzq_old,lineaDer_old =  line_detector(frame)
alfa = 0.2 #Factor de velocidad
print ('w{} x h{}'.format(width,height))

while(True):

    # Capture frame-by-frame
    ret, frame = cap.read()

    # Detectamos las lineas de transmision y las mostramos en la pantalla.
    lineaIzq,lineaDer =  line_detector(frame)
    # print ("LineaDer = {},  lineaIzq = {}".format(lineaDer,lineaIzq))
    draw_lines(frame,[lineaDer,lineaIzq],(0,255,0), 5)



    ################  Aqui empieza el analisis de contornos y de colores #############3

    # Creamos la imagen mascara.
    # mask = np.zeros_like(frame)
    # draw_lines(mask,[lineaDer,lineaIzq],(255,255,255), 150)

    # roi = np.bitwise_and(mask,frame)


    # Unimos las imagenes para hacer un display simultaneo y rescalamos
    # im0 = np.dstack((thres,thres,thres))
    # im1 = np.dstack((gray,gray,gray))
    # im2 = np.dstack((mask,mask,mask))
    # resultado = np.hstack((frame,roi))
    # resultado = np.vstack((resultado,np.hstack((im1,im0))))
    # resultado = cv2.resize(resultado,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_AREA)
    resultado = frame

    # Display the resulting frame
    cv2.imshow('frame',resultado)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

