import numpy as np
import cv2
import random


# Filtros de color de HSV
rojoLow     = np.array([0,198,68])
rojoHigh    = np.array([17,255,255])
amarilloLow = np.array([24,210,67])
amarilloHigh= np.array([34,255,237])
azulLow     = np.array([90,54,0])
azulHigh    = np.array([116,255,93])
verdeLow    = np.array([37,144,0])
verdeHigh   = np.array([67,255,212])
blancoLow   = np.array([0,0,104])
blancoHigh  = np.array([179,255,255])


# Diccionario de colores
coloresDibujo = {'rojo':(0,0,255),'verde':(0,255,0),'azul':(255,0,0),'amarillo':(0,255,255),'blanco':(255,255,255)}


def polar2poly(rt):
    """ Funcion que toma un tupla de valores (rho,theta) que salen de la transformada de linea de Hough,
        y usando la funcion de regresion linear 'polyfit', calcula la ecuacion  Y = mX + b de la linea, en pixeles
        y devuelve la tupla (m, b)


        Argumentos:
            rt (tupla): tupla con los valores (rho,theta) resultantes de la transformada de Hough (HoughLines())


        Devuelve:
            tupla: devuelve la version de tra##################################################################################################################################################3


    """

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
    """Esta funcion recibe una imagen y unas lineas definidas como listas
       [pendiente, corte_con_y] y dibujandolas con un color
       aleatorio o dado por el usuario"""

    if type(lines) == list or tuple:
        for eq in lines:
            if eq == None: return
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
        if eq == None: return
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
    ret,thres = cv2.threshold(sobelx,60,255,cv2.THRESH_BINARY)    # valor normal 27
    # Usamos la imagen binaria del Threshold, y la pasamos por la transformada lenta de Hough
    lines = cv2.HoughLines(thres,1,np.pi/180,200)

    ### Aqui empezamos a deducir las lineas, apartir del procesamiento de imagenes de arriba ###

    #Si es que conseguimos rectas en la imagen
    if lines != None:
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
            vecindario = [  x for x in linesp if ((abs(x[0] - linea[0]) <  deltaM) and (abs(((frame.shape[0] - x[1])/x[0]) - ((frame.shape[0] - linea[1])/linea[0])) < deltaB))]
            # Borramos del arreglo original todas las lineas que se asignaron a un vecindario. (Nota, la variable "dummy" es porque list.remove() no devuelve nada)
            dummy = [linesp.remove(x) for x in vecindario]
            # Agregamos el vecindario al cumulo.
            cumulos.append(vecindario)
        # Ordenamos los vecindarios de mayor a menor
        cumulos.sort(cmp = lambda x,y: len(y) - len(x))
    else:
        # Valor de error en caso de emergencia
        cumulos = None


    # Decomentar para graficar todas las lineas de los cumulos mas grandes.
    # for x in cumulos[0:2]:
    #     color =(random.randint(0,255),random.randint(0,255),random.randint(0,255))
    #     draw_lines(frame,x,color)


    # Calculamos el promedio de los dos cumulos mayores, y los asignamos a las dos lineas de transmision.
    # cada linea es una lista con pendiente y offset. Asi que acumulamos todas las pendientes en lineaXXX[0], y todas
    # los offset en lineaXXX[1]. y les sacamos el promedio.
    if  cumulos != None and len(cumulos) >= 2:
        lineaIzq = [np.mean([x[0] for x in cumulos[0]]), np.mean([x[1] for x in cumulos[0]])]
        lineaDer = [np.mean([x[0] for x in cumulos[1]]), np.mean([x[1] for x in cumulos[1]])]
        #Nos aseguramos de que las lineas esten en el orden correcto.
        if (((frame.shape[0] - lineaDer[1])/lineaDer[0]) - ((frame.shape[0] - lineaIzq[1])/lineaIzq[0]) < 0):
            #si la que asignamos como lineaDer es la linea mas izquierda, invertimos lo que devolvemos
            return [lineaDer, lineaIzq]

    else:
        lineaIzq = None
        lineaDer = None

    return [lineaIzq, lineaDer]


def object_finder(frame, colores = ['verde','blanco'], lines = None):

    if lines != None:
        # Filtros de color de HSV
        rojoLow     = np.array([0,121,34])
        rojoHigh    = np.array([26,255,122])
        amarilloLow = np.array([22,246,99])
        amarilloHigh= np.array([38,255,231])
        azulLow     = np.array([90,54,0])
        azulHigh    = np.array([116,255,93])
        verdeLow    = np.array([37,144,0])
        verdeHigh   = np.array([67,255,212])
        blancoLow   = np.array([0,52,134])
        blancoHigh  = np.array([179,95,221])

        #Diccionario con los colores
        dicColores = {'rojo': [rojoLow,rojoHigh],'amarillo': [amarilloLow,amarilloHigh],'azul': [azulLow,azulHigh],'verde': [verdeLow,verdeHigh],'blanco': [blancoLow,blancoHigh] }

        #Definimos las variables
        roiLineSize = 50        #15
        minContourArea = 30     #150
        kernel = np.ones((5,4),np.uint8)

        #Desempaquetamos las lineas las lineas
        lineaDer,lineaIzq = lines

        # Creamos la imagen mascara.
        mask = np.zeros_like(frame)
        draw_lines(mask,[lineaDer,lineaIzq],(255,255,255), roiLineSize)
        roi = np.bitwise_and(frame,mask)

        # Este es el diccionario de salida
        dictFallas = {}

        for color in colores:
            # Convertimos el Roi a HSV
            hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
            #Y filtramos por color Rojo
            mask = cv2.inRange(hsv,dicColores[color][0],dicColores[color][1])
            # kernel = np.ones((2,2),np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            # Buscamos los contornos en la mascara de color
            contours, hierarchy = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            # Eliminamos los contornos muy pequenos
            contours = [x for x in contours if cv2.contourArea(x) > minContourArea]
            # Revisamos si conseguimos contornos validos.
            if type(contours) != bool and len(contours) > 0:
                # Creamos la entrada del diccionario de salida.
                dictFallas.update({color:[]})
                # Iteramos sobre todos los contornos encontrados, y los agregamos a la lista.
                for cnt in contours:
                    # Conseguimos el centroide
                    M = cv2.moments(cnt)
                    cenX = int(M['m10']/M['m00'])
                    cenY = int(M['m01']/M['m00'])
                    # Agregamos el centroide al diccionario
                    dictFallas[color].append([[cenX,cenY],None])            # <==== Nota que es una lista dentro de una lista

        # Devolvemos el diccionario con todos los centroides.
        return dictFallas




def perspective_bound(frame, lineas):

    """
    Le pasas las dos lineas y te devuelva los puntos limitrofes de cada una

    """

    # Si algo falla, devuelve None
    if lineas[0] == None: return None

    #Desempacamos
    [lineaIzq,lineaDer] = lineas

    #Analizamos la linea Izquierda.
    y1 = int(0)
    x1 = int((y1 - lineaIzq[1])/lineaIzq[0])
    y2 = int(frame.shape[0])
    x2 = int((y2 - lineaIzq[1])/lineaIzq[0])

    if (abs(x1) > 5E4 or abs(x2) > 5E4):
        x1 = int(0)
        y1 = int(lineaIzq[0]*x1 + lineaIzq[1])
        x2 = int(frame.shape[1])
        y2 = int(lineaIzq[0]*x2 + lineaIzq[1])

    # Asignamos los puntos izquierdos
    topLeft = [x1, y1]
    botomLeft = [x2, y2]

    #Analizamos la linea Derecha.
    y1 = int(0)
    x1 = int((y1 - lineaDer[1])/lineaDer[0])
    y2 = int(frame.shape[0])
    x2 = int((y2 - lineaDer[1])/lineaDer[0])

    if (abs(x1) > 5E4 or abs(x2) > 5E4):
        x1 = int(0)
        y1 = int(lineaDer[0]*x1 + lineaDer[1])
        x2 = int(frame.shape[1])
        y2 = int(lineaDer[0]*x2 + lineaDer[1])

    # Asignamos los puntos Derechos
    topRight = [x1, y1]
    botomRight = [x2, y2]

    # Ahora devolvemos todos los puntos.
    return [topLeft, topRight, botomLeft, botomRight]




# Algoritmo que mide las distancias a la falla.
def distance_finder(frame, lines = None, fallas = None):

    if lines != None:
        # Variables importantes
        distaciaFocal = 1286    #1286.07
        largoEntreLineas = 25   #cm
        altura_camara = 11.2 #cm

        # Variables menos importantes
        height = frame.shape[0]
        width = frame.shape[1]

        #Desempacamos
        [lineaIzq,lineaDer] = lines

        ### Ahora viene el calculo de lineas paralelas ###

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
                    cv2.line(frame,tuple(centro[0]),tuple(punto_contrario),coloresDibujo['azul'],2)
                    # Calculamos la distancia en pixeles
                    distPixeles = np.sqrt(np.square(centro[0][0] - punto_contrario[0]) + np.square(centro[0][1] - punto_contrario[1]))
                    # Usamos la ecuacion de similitud triangular para sacar la distancia
                    distCamara = largoEntreLineas*distaciaFocal/distPixeles
                    # Ahora usamos la relacion de pitagoras para calcular la distancia de la falla al chasis
                    distChasis = np.sqrt(np.square(distCamara) - np.square(altura_camara))
                    # guardamos la distancia en centimetros en el diccionario de fallas

                    centro[1] = distChasis

            return fallas
