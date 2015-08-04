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
            vecindario = [  x for x in linesp if ((abs(x[0] - linea[0]) <  deltaM) and (abs(((height - x[1])/x[0]) - ((height - linea[1])/linea[0])) < deltaB))]
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
    else:
        lineaIzq = None
        lineaDer = None

    return [lineaIzq, lineaDer]


def object_finder(frame, colores = ['verde','blanco'], lines = None):

    if lines != None:
        # Filtros de color de HSV
        rojoLow     = np.array([0,198,95])
        rojoHigh    = np.array([17,255,255])
        amarilloLow = np.array([22,246,99])
        amarilloHigh= np.array([26,255,231])
        azulLow     = np.array([90,54,0])
        azulHigh    = np.array([116,255,93])
        verdeLow    = np.array([37,144,0])
        verdeHigh   = np.array([67,255,212])
        blancoLow   = np.array([0,0,130])
        blancoHigh  = np.array([179,196,255])

        #Diccionario con los colores
        dicColores = {'rojo': [rojoLow,rojoHigh],'amarillo': [amarilloLow,amarilloHigh],'azul': [azulLow,azulHigh],'verde': [verdeLow,verdeHigh],'blanco': [blancoLow,blancoHigh] }

        #Definimos las variables
        roiLineSize = 15
        minContourArea = 120
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
                    dictFallas[color].append((cenX,cenY))

        # Devolvemos el diccionario con todos los centroides.
        return dictFallas








import numpy as np
import cv2
import random

cap = cv2.VideoCapture("udpsrc port=5000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false")

ret,frame = cap.read()    #Definimos unas variables con el tamano de la imagen
height = frame.shape[0]
width = frame.shape[1]
# print ('w{} x h{}'.format(width,height))
coloresDibujo = {'rojo':(0,0,255),'verde':(0,255,0),'azul':(255,0,0),'amarillo':(0,255,255),'blanco':(255,255,255)}

# Filtros de color de HSV
rojoLow     = np.array([0,198,95])
rojoHigh    = np.array([17,255,255])
amarilloLow = np.array([22,246,99])
amarilloHigh= np.array([26,255,231])
azulLow     = np.array([90,54,0])
azulHigh    = np.array([116,255,93])
verdeLow    = np.array([37,144,0])
verdeHigh   = np.array([67,255,212])
blancoLow   = np.array([0,0,104])
blancoHigh  = np.array([179,255,255])


while(True):

    # Capture frame-by-frame
    ret, frame = cap.read()

    # Detectamos las lineas de transmision y las mostramos en la pantalla.
    lineaIzq,lineaDer =  line_detector(frame)
    # print ("LineaDer = {},  lineaIzq = {}".format(lineaDer,lineaIzq))
    frame_copy = frame.copy()
    draw_lines(frame_copy,[lineaDer,lineaIzq],(255,255,255), 1)

    # buscamos las fallas sobre la linea
    # fallas = object_finder(frame, ['rojo','amarillo','blanco'], (lineaIzq,lineaDer))

    # for color in fallas:
    #     i = 1
    #     for cenX,cenY in fallas[color]:
    #         cv2.circle(frame_copy,(cenX,cenY),40,coloresDibujo[color],3)
    #         cv2.putText(frame_copy, "{} #{}".format(color,i), (cenX + 45, cenY), cv2.FONT_HERSHEY_SIMPLEX,1.0, coloresDibujo[color], 2)
    #         i+=1


    ################  Aqui empieza el analisis de contornos y de colores #############3

    #Definimos las variables
    roiLineSize = 15
    minContourArea = 120
    kernel = np.ones((5,4),np.uint8)

    # Creamos la imagen mascara.
    mask = np.zeros_like(frame)
    draw_lines(mask,[lineaDer,lineaIzq],(255,255,255), roiLineSize)
    roi = np.bitwise_and(frame,mask)

    # Convertimos el Roi a HSV
    hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    #Y filtramos por color Rojo
    mask_rojo = cv2.inRange(hsv,rojoLow,rojoHigh)
    # kernel = np.ones((2,2),np.uint8)
    mask_rojo = cv2.morphologyEx(mask_rojo, cv2.MORPH_OPEN, kernel, iterations=1)
    img_rojo = cv2.bitwise_and(roi,roi, mask = mask_rojo)

    # Buscamos los contornos en la mascara de color
    contours, hierarchy = cv2.findContours(mask_rojo.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    # Eliminamos los contornos muy pequenos
    contours = [x for x in contours if cv2.contourArea(x) > minContourArea]
    # Revisamos si conseguimos contornos validos.
    if type(contours) != bool and len(contours) > 0:
        # Iteramos sobre todos los contornos encontrados
        i=1
        for cnt in contours:
            # Conseguimos el centroide y dibujamos los contornos
            M = cv2.moments(cnt)
            cenX = int(M['m10']/M['m00'])
            cenY = int(M['m01']/M['m00'])
            cv2.circle(frame_copy,(cenX,cenY),40,(0,0,255),3)
            cv2.putText(frame_copy, "Rojo #{}".format(i), (cenX + 45, cenY), cv2.FONT_HERSHEY_SIMPLEX,1.0, (0, 0, 255), 2)
            i+=1

#Y filtramos por color Amarillo
    mask_amarillo = cv2.inRange(hsv,amarilloLow,amarilloHigh)
    # kernel = np.ones((2,2),np.uint8)
    mask_amarillo = cv2.morphologyEx(mask_amarillo, cv2.MORPH_OPEN, kernel, iterations=1)
    img_amarillo = cv2.bitwise_and(roi,roi, mask = mask_amarillo)

    # Buscamos los contornos en la mascara de color
    contours, hierarchy = cv2.findContours(mask_amarillo.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    # Eliminamos los contornos muy pequenos
    contours = [x for x in contours if cv2.contourArea(x) > minContourArea]
    # Revisamos si conseguimos contornos validos.
    if type(contours) != bool and len(contours) > 0:
        # Iteramos sobre todos los contornos encontrados
        i=1
        for cnt in contours:
            # Conseguimos el centroide y dibujamos los contornos
            M = cv2.moments(cnt)
            cenX = int(M['m10']/M['m00'])
            cenY = int(M['m01']/M['m00'])
            cv2.circle(frame_copy,(cenX,cenY),40,(0,255,255),3)
            cv2.putText(frame_copy, "Amarillo #{}".format(i), (cenX + 45, cenY), cv2.FONT_HERSHEY_SIMPLEX,1.0, (0, 255, 255), 2)
            i+=1


#Y filtramos por color blanco
    mask_blanco = cv2.inRange(hsv,blancoLow,blancoHigh)
    # kernel = np.ones((7,7),np.uint8)
    mask_blanco = cv2.morphologyEx(mask_blanco, cv2.MORPH_OPEN, kernel, iterations=2)
    img_blanco = cv2.bitwise_and(roi,roi, mask = mask_blanco)

    # Buscamos los contornos en la mascara de color
    contours, hierarchy = cv2.findContours(mask_blanco.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    # Eliminamos los contornos muy pequenos
    # O que no tengan 4 esquinas.
    esquinas = len(cv2.approxPolyDP(x, 0.02 * cv2.arcLength(x, True), True))
    contours = [x for x in contours if cv2.contourArea(x) > minContourArea and esquinas >= 4  and esquinas <= 5]
    # Revisamos si conseguimos contornos validos.
    if type(contours) != bool and len(contours) > 0:
        # Iteramos sobre todos los contornos encontrados
        i=1
        for cnt in contours:
            # Conseguimos el centroide y dibujamos los contornos
            M = cv2.moments(cnt)
            cenX = int(M['m10']/M['m00'])
            cenY = int(M['m01']/M['m00'])
            cv2.circle(frame_copy,(cenX,cenY),40,(255,255,255),3)
            cv2.putText(frame_copy, "Blanco #{}".format(i), (cenX + 45, cenY), cv2.FONT_HERSHEY_SIMPLEX,1.0, (255, 255, 255), 2)
            i+=1








    # Unimos las imagenes para hacer un display simultaneo y rescalamos
    im0 = np.dstack((mask_blanco,mask_blanco,mask_blanco))
    # im1 = np.dstack((gray,gray,gray))
    # im2 = np.dstack((mask,mask,mask))
    resultado = np.hstack((frame_copy,roi))
    resultado = np.vstack((resultado,np.hstack((im0,img_blanco))))
    resultado = cv2.resize(resultado,None,fx=0.4, fy=0.4, interpolation = cv2.INTER_AREA)
    # resultado = frame_copy

    # Display the resulting frame
    cv2.imshow('frame',resultado)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
