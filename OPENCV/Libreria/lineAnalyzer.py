 # -*- coding: utf-8 -*-
import numpy as np
import cv2
import random
import time
#silenciar las advertencias
import warnings


class lineAnalyzer(object):

#########################################################################################################################
######                                         Variables Importantes                                               ######
#########################################################################################################################


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

    #Diccionario de colores de analisis
    dicColores = {'rojo': [rojoLow,rojoHigh],'amarillo': [amarilloLow,amarilloHigh],'azul': [azulLow,azulHigh],'verde': [verdeLow,verdeHigh],'blanco': [blancoLow,blancoHigh] }

    #Filtro del cable nada de colores Rojos, ni mucha saturacion, no poco valor
    # cableLow     = np.array([53,0,0])      #No funcionan
    # cableHigh    = np.array([135,44,142])
    cableLow     = np.array([71,0,97])
    cableHigh    = np.array([135,98,255])


    # Diccionario de colores de dibujo
    coloresDibujo = {'rojo':(0,0,255),'verde':(0,255,0),'azul':(255,0,0),'amarillo':(0,255,255),'blanco':(255,255,255)}


    # Etapa 1: Detector de lineas
    deltaM = 2         # Desviacion maxima de la pendiente de un solo vecindario.
    deltaB = 200        # Desviacion maxima del corte con el fondo de la imagen para un vecindario.
    deltaPendientePrimera = 0.5 #Desviacion maxima de las lineas de la pendiente buscada
    # deltaM = 4         # Desviacion maxima de la pendiente de un solo vecindario.
    # deltaB = 200        # Desviacion maxima del corte con el fondo de la imagen para un vecindario.

    # Etapa 2: Detector de objetos
    roiLineSize = 50                    # Tamano  de la ventana de analisis alrededor de las lineas
    minContourArea = 30                 # Area minima de una mancha en la linea para ser detectada
    kernel = np.ones((5,4),np.uint8)    # Kernel de las operaciones morfologicas

    # Etapa 3: Medidor de Distancias
    distaciaFocal =    1286             #1286.07
    largoEntreLineas = 25               #cm
    altura_camara =    11.2             #cm


#########################################################################################################################
######                                         Funciones de Uso                                                    ######
#########################################################################################################################


    def __init__(self, mode, uri = None, debug = []):

        # Forma de adquisicion de imagen
        self.mode = mode
        # Tipos de Debug disponibles            full debug output = ['line', 'object', 'distance']
        self.debug_mode = debug
        # Imagen Pura
        self.frame = None
        # Imagenes con la informacion debug
        self.frame_debug_line = None
        self.frame_debug_object = None
        self.frame_debug_distance = None
        # Objeto de captura de video
        self.camera = None
        self.rawCapture = None
        # Informacion de la imagen
        self.height = None
        self.width = None

        # En caso de que el analisis sea pregrabado.
        self.uri = None

        # Silenciamos las advertencias
        warnings.filterwarnings("ignore")

        # Variables de medicion de tiempo
        self.start_time = None
        self.last_time = None


    def begin(self):
        """Inicialza el tipo de debbugeo y el por donde se van a obtener los fotogramas"""


        # Modo 0: Modulo del Raspberry PI
        if self.mode == 0:
            from picamera.array import PiRGBArray
            from picamera import PiCamera

            # inicializar camara
            self.camera = PiCamera()
            self.rawCapture = PiRGBArray(self.camera)

            # Dejar que la camara arranque
            time.sleep(0.1)

            # Capturar pantalla
            self.camera.capture(self.rawCapture, format="bgr")
            self.frame = self.rawCapture.array
            # Informacion de la imagen
            self.height = self.frame.shape[0]
            self.width = self.frame.shape[1]

        # Modo 1: Webcam
        if self.mode == 1:

            self.camera = cv2.VideoCapture(0)

            ret,self.frame = self.camera.read()    #Definimos unas variables con el tamano de la imagen
            self.height = self.frame.shape[0]
            self.width = self.frame.shape[1]

        # Modo 2: Gstreamer
        if self.mode == 2:

            self.camera = cv2.VideoCapture("udpsrc port=5000 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false")

            ret,self.frame = self.camera.read()    #Definimos unas variables con el tamano de la imagen
            self.height = self.frame.shape[0]
            self.width = self.frame.shape[1]

        # Modo 3: Archivo de video
        if self.mode == 3:

            self.camera = cv2.VideoCapture(self.uri)

            ret,self.frame = self.camera.read()    #Definimos unas variables con el tamano de la imagen
            self.height = self.frame.shape[0]
            self.width = self.frame.shape[1]


    def readFrame(self):
        """ Esto abstrae obtener el cuadro, ya sea por el API raro del Raspberry PI, Video Capture, o Gstreamer,
            depende de la variable self.type
        """

        # Modo 0: Modulo del Raspberry PI
        if self.mode == 0:

            # Capturar pantalla
            self.camera.capture(self.rawCapture, format="bgr")
            self.frame = self.rawCapture.array
            return self.frame

        # Modo 1: Webcam
        if self.mode == 1:

            #Capturar pantalla
            ret,self.frame = self.camera.read()
            return self.frame

        # Modo 2: Gstreamer
        if self.mode == 2:

            ret,self.frame = self.camera.read()    #Definimos unas variables con el tamano de la imagen
            return self.frame

        # Modo 3: Archivo de video
        if self.mode == 3:

            ret,self.frame = self.camera.read()    #Definimos unas variables con el tamano de la imagen
            return self.frame



    def analyzeFrame(self):
        """ Esta es la funcion que usa el usuario para recibir el analisis completo de la imagen"""

        # Leemos la imagen mas actual de la camara
        image = self.readFrame()

        # Corremos el detector de lineas.
        lineaIzq,lineaDer =  self.lineDetector(image)

        # Corremos el ubicador de objetos.
        fallas = self.objectFinder(image, ['verde','blanco'], (lineaIzq,lineaDer))

        # Corremos el medidor de distancias.
        fallas = self.distanceFinder(image,(lineaIzq,lineaDer),fallas)

        # Devolvemos el resultados
        return fallas


    def processPicture(self, uri):
        """ Funcion de debug pensada para analizar fotos estaticas, para pruebas """

        self.start_time = time.time()
        image = cv2.imread(uri,1)
        self.frame = image.copy()

        print("[+] Detector de lineas:")
        line_time = time.time()

        print("    [-] Leer imagen: {}".format(time.time() - self.start_time))
        self.last_time = time.time()


        # Primero intentamos limpiar la imagen de distracciones
        # Convertimos el Roi a HSV
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        # Y filtramos por color Rojo
        mask = cv2.inRange(hsv,self.cableLow,self.cableHigh)

        print("    [-] Detectar color de lineas: {}".format(time.time() - self.last_time))
        self.last_time = time.time()

        # # Operacion morfologica de apertura
        # kernel1 = np.ones((30,30),np.uint8)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1, iterations=1)
        # imagen cortada
        imagen_cortada = cv2.bitwise_and(image, image, mask = mask)

        # print("    [-] Apertura 30x30: {}".format(time.time() - self.last_time))
        # self.last_time = time.time()

        # imagen_cortada = image

        # Corremos el detector de lineas.
        lineaIzq,lineaDer =  self.lineDetector(imagen_cortada)

        print("[*] Tiempo conjunto: {}\n".format(time.time() - line_time))
        self.last_time = time.time()

        print("[+] Detector de objetos:")
        object_time = time.time()

        # Corremos el ubicador de objetos.
        fallas = self.objectFinder(image, ['verde','blanco'], (lineaIzq,lineaDer))

        print("[*] Tiempo conjunto: {}\n".format(time.time() - object_time))
        self.last_time = time.time()

        print("[+] Detector de distancias:")
        distance_time = time.time()

        # Corremos el medidor de distancias.
        fallas = self.distanceFinder(image,(lineaIzq,lineaDer),fallas)

        print("[*] Tiempo conjunto: {}\n".format(time.time() - distance_time))
        self.last_time = time.time()


        print("Tiempo total de ejecucion: {}".format(time.time() -  self.start_time))

        # Devolvemos el resultados
        return fallas




#########################################################################################################################
######                                         Funciones de Apoyo                                                  ######
#########################################################################################################################

    def polar2poly(self, rt):
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




    def draw_lines(self, frame, lines, colors = None, size = 2):
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


    def perspective_bound(self, frame, lineas):

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


#########################################################################################################################
######                                   Etapa 1: Encontrar lineas                                                 ######
#########################################################################################################################

    def lineDetector(self, frame):

        # Pasamos la imagen a blanco y negro
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Calculamos la derivada en x de la imagen. con Sobel (Asi ignoramos las lineas horizontales en la imagen)
        sobelx = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=3)
        sobelx = np.uint8(np.absolute(sobelx))

        # Hacemos un Threshold binario normal.
        ret,thres = cv2.threshold(sobelx,60,255,cv2.THRESH_BINARY)    # valor normal 60

        print("    [-] Pre-Hough: {}".format(time.time() - self.last_time))
        self.last_time = time.time()

        # Usamos la imagen binaria del Threshold, y la pasamos por la transformada lenta de Hough
        lines = cv2.HoughLines(thres,1,np.pi/180,200)

        print("    [-] Detector de lineas de Hough: {}".format(time.time() - self.last_time))
        self.last_time = time.time()

        ### Aqui empezamos a deducir las lineas, apartir del procesamiento de imagenes de arriba ###

        #Si es que conseguimos rectas en la imagen
        if lines != None:
            #Conseguimos las ecuaciones de todas las rectas en forma polinomial
            linesp = [ self.polar2poly(rt) for rt in lines[0]]
            # Definimos las variables que necesitamos para ordenar y filtrar las lineas.
            cumulos = []        # Arreglo donde se van a guardar los vecindarios.
            # Ordenamos las lineas de izquierda a derecha acorde a su punto de corte con el fondo de la imagen.
            linesp.sort(cmp = lambda x,y: int(y[1] - x[1]))
            # Recorremos cada linea del arreglo y las agrupamos de acuerdo a su similitud con otras lineas.
            for linea in linesp:
                # Linea a linea guardamos un vecindario, si y solo si, la desviacion de los parametros sea menor a deltaM y delta B.
                vecindario = [  x for x in linesp if ((abs(x[0] - linea[0]) <  self.deltaM) and (abs(((frame.shape[0] - x[1])/x[0]) - ((frame.shape[0] - linea[1])/linea[0])) < self.deltaB))]
                # Borramos del arreglo original todas las lineas que se asignaron a un vecindario. (Nota, la variable "dummy" es porque list.remove() no devuelve nada)
                dummy = [linesp.remove(x) for x in vecindario]
                # Agregamos el vecindario al cumulo.
                cumulos.append(vecindario)
            # Ordenamos los vecindarios de mayor a menor
            cumulos.sort(cmp = lambda x,y: len(y) - len(x))
        else:
            # Valor de error en caso de emergencia
            cumulos = None

        print("    [-] Sorting de Cumulos: {}".format(time.time() - self.last_time))
        self.last_time = time.time()

        print ("Numero de cumulos = {}".format(cumulos))

        # Intento para que el programa funcione como debe ser en ambientes con mucho ruido, restrigiendo
        # la busqueda a lineas con pendientes opuestas.
        # cumulos_prom = []
        # for i in cumulos:
        #     cumulos_prom.append([np.mean([x[0] for x in i]), np.mean([x[1] for x in i])])

        cumulos_prom = [[np.mean([x[0] for x in i]), np.mean([x[1] for x in i])] for i in cumulos]
        pendiente_primera = -1*cumulos_prom[0][0]

        cumulos_prom_filt = filter(lambda x: np.abs(x[0] - pendiente_primera) < self.deltaPendientePrimera, cumulos_prom)

        print("    [-] Filtro de simetria de lineas: {}".format(time.time() - self.last_time))
        self.last_time = time.time()


        # Calculamos el promedio de los dos cumulos mayores, y los asignamos a las dos lineas de transmision.
        # cada linea es una lista con pendiente y offset. Asi que acumulamos todas las pendientes en lineaXXX[0], y todas
        # los offset en lineaXXX[1]. y les sacamos el promedio.
        inv = False
        if  cumulos != None and len(cumulos) >= 2:
            lineaIzq = [np.mean([x[0] for x in cumulos[0]]), np.mean([x[1] for x in cumulos[0]])]
            # lineaDer = [np.mean([x[0] for x in cumulos[1]]), np.mean([x[1] for x in cumulos[1]])]
            lineaDer = cumulos_prom_filt[0]
            #Nos aseguramos de que las lineas esten en el orden correcto.
            if (((frame.shape[0] - lineaDer[1])/lineaDer[0]) - ((frame.shape[0] - lineaIzq[1])/lineaIzq[0]) < 0):
                #si la que asignamos como lineaDer es la linea mas izquierda, invertimos lo que devolvemos
                inv = True
        else:
            lineaIzq = None
            lineaDer = None

        print("    [-] Arreglos finales: {}".format(time.time() - self.last_time))
        self.last_time = time.time()


        #Debugging output
        if 'line' in self.debug_mode:

            # Sacamos una copia de la imagen
            frame_copy = self.frame.copy()

            lineas_a_dibujar = filter(lambda x: np.abs(np.mean([i[0] for i in x]) - pendiente_primera) < 0.05, cumulos)
            # print lineas_a_dibujar[0]
            # print cumulos[0:1]

            if cumulos != None:
                for x in cumulos[:] + lineas_a_dibujar:
                    color =(random.randint(0,255),random.randint(0,255),random.randint(0,255))
                    self.draw_lines(frame_copy,x,color)

                self.draw_lines(frame_copy,[lineaDer,lineaIzq],(0,255,0), 15)



                im0 = np.dstack((thres,thres,thres))
                im1 = np.dstack((sobelx,sobelx,sobelx))
                im2 = np.dstack((gray,gray,gray))
                resultado = np.hstack((frame_copy,im2))
                resultado = np.vstack((resultado,np.hstack((im1,im0))))
                resultado = cv2.resize(resultado,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_AREA)

                self.frame_debug_line = resultado.copy()


                print("    [-] Debug de lineas: {}".format(time.time() - self.last_time))
                self.last_time = time.time()



        if inv == False:
            return [lineaIzq, lineaDer]
        else:
            return [lineaDer, lineaIzq]



#########################################################################################################################
######                                     Etapa 2: Encontrar objetos                                              ######
#########################################################################################################################


    def objectFinder(self, frame, colores = ['verde','blanco'], lines = None):

        if lines != None:

            #Desempaquetamos las lineas las lineas
            lineaDer,lineaIzq = lines

            # Creamos la imagen mascara.
            mask = np.zeros_like(frame)
            self.draw_lines(mask,[lineaDer,lineaIzq],(255,255,255), self.roiLineSize)
            roi = np.bitwise_and(frame,mask)

            # Este es el diccionario de salida
            dictFallas = {}

            print("    [-] Preparar Roi: {}".format(time.time() - self.last_time))
            self.last_time = time.time()

            # Multi-mascara
            if 'object' in self.debug_mode:
                multi_mask = np.zeros((self.height, self.width),dtype = np.uint8)


            for color in colores:
                # Convertimos el Roi a HSV
                hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
                #Y filtramos por color Rojo
                mask = cv2.inRange(hsv,self.dicColores[color][0],self.dicColores[color][1])
                # kernel = np.ones((2,2),np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)

                #agregamos la mascaras
                if 'object' in self.debug_mode:
                    multi_mask = np.bitwise_or(multi_mask,mask)

                # Buscamos los contornos en la mascara de color
                if cv2.__version__ == '3.0.0':
                    im2, contours, hierarchy = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                else:
                    contours, hierarchy = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                # Eliminamos los contornos muy pequenos
                contours = [x for x in contours if cv2.contourArea(x) > self.minContourArea]
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

            print("    [-] Filtro de color y Contornos: {}".format(time.time() - self.last_time))
            self.last_time = time.time()


            #Debugging output
            if 'object' in self.debug_mode:

                # Sacamos una copia de la imagen
                frame_copy = self.frame.copy()

                #imagen cortada
                imagen_cortada = cv2.bitwise_and(frame_copy, frame_copy, mask = multi_mask)

                # Unimos las imagenes para hacer un display simultaneo y rescalamos
                im0 = np.dstack((multi_mask,multi_mask,multi_mask))
                # im1 = np.dstack((gray,gray,gray))
                # im2 = np.dstack((mask,mask,mask))
                resultado = np.hstack((frame_copy,roi))
                resultado = np.vstack((resultado,np.hstack((im0,imagen_cortada))))
                resultado = cv2.resize(resultado,None,fx=0.4, fy=0.4, interpolation = cv2.INTER_AREA)

                self.frame_debug_object = resultado.copy()

                print("    [-] Debug de objetos: {}".format(time.time() - self.last_time))
                self.last_time = time.time()

            # Devolvemos el diccionario con todos los centroides.
            return dictFallas



#########################################################################################################################
######                                         Etapa 3: Medir distancias                                           ######
#########################################################################################################################

    # Algoritmo que mide las distancias a la falla.
    def distanceFinder(self, frame, lines = None, fallas = None):

        if lines != None:

            #Desempacamos
            [lineaIzq,lineaDer] = lines

            ### Ahora viene el calculo de lineas paralelas ###

            # Primero calculamos los cuatro puntos limites de las lineas electricas.
            rectOriginal = np.float32(self.perspective_bound(frame, [lineaIzq,lineaDer]))
            # Ahora definimos los limites de la nueva imagen
            rectPerspective = np.float32([[0,0], [self.height,0], [0,self.height], [self.height,self.height]])
            # Calculamos la matriz de perspectiva
            if lineaIzq == None:
                frame_perspective = np.zeros((self.height,self.height,3))
            else:
                M = cv2.getPerspectiveTransform(rectOriginal,rectPerspective)

                if 'distance' in self.debug_mode:
                    # Transformamos la imagen para ver como queda
                    frame_perspective = cv2.warpPerspective(frame,M, (self.height,self.height))

                print("    [-] Calcular matrix de perspectiva: {}".format(time.time() - self.last_time))
                self.last_time = time.time()

                # #Iteramos sobre todos los puntos que encontramos para encontrar su paralela
                for color in fallas:
                    for centro in fallas[color]:
                        #Transformamos el centroide del
                        centro_perspectiva = cv2.perspectiveTransform(np.array([[centro[0]]],dtype = 'float32'),M)    #Los puntos que le entran a la funcion perspectiveTransform, tienen que ser de la forma [[[x1,y1],[x2,y2, ...]]], con ESA cantidad de parentesis.
                        # Calculamos el mismo punto en la otra linea
                        centro_perspectiva = centro_perspectiva[0][0].copy()
                        if centro_perspectiva[0] < self.height/2:
                            punto_contrario_perspectiva = np.array([self.height, centro_perspectiva[1]])
                        else:
                            punto_contrario_perspectiva = np.array([0, centro_perspectiva[1]], dtype = 'float')
                        # Destransformamos el punto
                        # print "M = {}".format(M)
                        # print "invM = {}".format(cv2.invert(M))
                        punto_contrario = cv2.perspectiveTransform(np.array([[punto_contrario_perspectiva]],dtype = 'float32'),cv2.invert(M)[1])
                        # Graficamos, las lineas que encontramos
                        punto_contrario = punto_contrario[0][0].copy()
                        cv2.line(frame,tuple(centro[0]),tuple(punto_contrario),self.coloresDibujo['azul'],2)
                        # Calculamos la distancia en pixeles
                        distPixeles = np.sqrt(np.square(centro[0][0] - punto_contrario[0]) + np.square(centro[0][1] - punto_contrario[1]))
                        # Usamos la ecuacion de similitud triangular para sacar la distancia
                        distCamara = self.largoEntreLineas*self.distaciaFocal/distPixeles
                        # Ahora usamos la relacion de pitagoras para calcular la distancia de la falla al chasis
                        distChasis = np.sqrt(np.square(distCamara) - np.square(self.altura_camara))
                        # guardamos la distancia en centimetros en el diccionario de fallas

                        centro[1] = distChasis

                print("    [-] Trans y destransformaciones de perspectiva {}".format(time.time() - self.last_time))
                self.last_time = time.time()

                #Debugging output
                if 'distance' in self.debug_mode:

                    # Sacamos una copia de la imagen
                    frame_copy = self.frame.copy()

                    # Dibujamos un poco de informacion de utilidad
                    for color in fallas:
                        i = 1
                        for objeto in fallas[color]:
                            cv2.circle(frame_copy,tuple(objeto[0]),40,self.coloresDibujo[color],3)
                            cv2.putText(frame_copy, "{} #{}".format(color,i), (objeto[0][0] + 45, objeto[0][1]), cv2.FONT_HERSHEY_SIMPLEX,1.0, self.coloresDibujo[color], 2)
                            cv2.putText(frame_copy, "{:0.2f}cm".format(objeto[1]), (objeto[0][0] + 45, objeto[0][1] + 20), cv2.FONT_HERSHEY_SIMPLEX,0.7, self.coloresDibujo[color], 2)
                            i+=1

                    resultado = np.hstack((frame_copy,frame_perspective))
                    resultado = cv2.resize(resultado,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_AREA)

                    self.frame_debug_distance = resultado.copy()

                    print("    [-] Distance debug: {}".format(time.time() - self.last_time))
                    self.last_time = time.time()


                return fallas




def main():

    #Inicializamos un objeto
    analizador = lineAnalyzer(1, debug = ['line', 'object', 'distance'])

    analizador.begin()
    cosa = analizador.analyzeFrame()

    while(1):
        # Display the resulting frame
        print type(analizador.frame_debug_distance)
        cv2.imshow('frame',analizador.frame_debug_distance)
        cosa = analizador.analyzeFrame()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    analizador.camera.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
