import cv2
import numpy as np
import sys
import math
import time
import random

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


def lineDetector(frame, ):


    #Initial Roi Variables
    separation = 207 * 2
    angle = 50
    roiSize = 98 * 2
    offset = 313 * 2

    #Initial Color filter variables
    cableLow     = np.array([54,34,0])
    cableHigh    = np.array([118,152,141])

   # Calculamos la desviacion angular
    offsetX = frame.shape[0]/2*math.tan(math.radians(angle))

    #Calculamos el segundo punto de las rectas.
    linDer_end = [frame.shape[1]/2 + int(offsetX) + separation + offset - frame.shape[1]/2, frame.shape[0]]
    linIzq_end = [frame.shape[1]/2 - int(offsetX) - separation + offset - frame.shape[1]/2, frame.shape[0]]

    #Calculamos el tercer punto de la recta
    linDer_top = [frame.shape[1]/2 - int(offsetX) + separation + offset - frame.shape[1]/2, 0]
    linIzq_top = [frame.shape[1]/2 + int(offsetX) - separation + offset - frame.shape[1]/2, 0]

    # print("    [-] Calcular punto del ROI: {}ms".format(int(1000*(time.time() - last_time))))
    # last_time = time.time()



    #Creamos la mascara
    mask = np.zeros_like(frame)
    cv2.line(mask,tuple(linDer_top),tuple(linDer_end), (255,255,255), roiSize)
    cv2.line(mask,tuple(linIzq_top),tuple(linIzq_end), (255,255,255), roiSize)
    roi = np.bitwise_and(frame,mask)


    # print("    [-] Mascara con el Roi: {}ms".format(int(1000*(time.time() - last_time))))
    # last_time = time.time()

    #Filtro de Color
    hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,cableLow,cableHigh)
    # res = cv2.bitwise_and(roi,roi, mask = mask)


    # print("    [-] Filtro de Color: {}ms".format(int(1000*(time.time() - last_time))))
    # last_time = time.time()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.blur(gray,(3,3))
    # Derivada derecha
    kernel_der = np.asarray([[0,1,2],[-1,0,1],[-2,-1,0]])
    diagonal_der = cv2.filter2D(gray[:,offset:],-1, kernel_der)
    sobel_der = np.uint8(np.absolute(diagonal_der))
    _,thres_der = cv2.threshold(sobel_der,30,255,cv2.THRESH_BINARY)
    kernel_der = np.asarray([[1,1,0,0,0],[1,1,1,0,0],[0,1,1,1,0],[0,0,1,1,1], [0,0,0,1,1] ], dtype = np.uint8)
    thres_der = cv2.morphologyEx(thres_der, cv2.MORPH_DILATE, kernel_der, iterations=1)

    # Derivada izquierda
    kernel_izq = np.asarray([[-2,-1,0],[-1,0,1],[0,1,2]])
    diagonal_izq = cv2.filter2D(gray[:,:offset],-1, kernel_izq)
    sobel_izq = np.uint8(np.absolute(diagonal_izq))
    _,thres_izq = cv2.threshold(sobel_izq,30,255,cv2.THRESH_BINARY)
    kernel_izq = np.asarray([[0,0,0,1,1], [0,0,1,1,1], [0,1,1,1,0], [1,1,1,0,0], [1,1,0,0,0]], dtype = np.uint8)
    thres_izq = cv2.morphologyEx(thres_izq, cv2.MORPH_DILATE, kernel_izq, iterations=1)

    #Unir derivadas
    sobel_combined = np.hstack((thres_izq,thres_der))
    sobel_mask = np.bitwise_and(sobel_combined,roi[:,:,0])
    # mask = sobel_mask

    #Combinamos ambas mascaras
    # mask = np.bitwise_and(sobel_combined, mask)


    # print("    [-] Derivadas: {}ms".format(int(1000*(time.time() - last_time))))
    # last_time = time.time()

    split_blocks = 16
    vertical_division = mask.shape[0]/split_blocks
    left_centroids = []
    right_centroids = []
    for i in xrange(2, split_blocks -2):
        #left side
        try:
            M = cv2.moments(mask[i*vertical_division:(i+1)*vertical_division - 1,0:offset],False)
            moments_center = (int(M['m10']/M['m00']) , int(M['m01']/M['m00']) + i*vertical_division)
            left_centroids.append(moments_center)
        except:
            pass
        #right side
        try:
            M = cv2.moments(mask[i*vertical_division:(i+1)*vertical_division - 1,offset:mask.shape[1] -1],False)
            moments_center = (int(M['m10']/M['m00']) + offset, int(M['m01']/M['m00']) + i*vertical_division)
            right_centroids.append(moments_center)
        except:
            pass


    for i in xrange(2, split_blocks -2):
        #left side
        try:
            M = cv2.moments(sobel_mask[i*vertical_division:(i+1)*vertical_division - 1,0:offset],False)
            moments_center = (int(M['m10']/M['m00']) , int(M['m01']/M['m00']) + i*vertical_division)
            left_centroids.append(moments_center)
        except:
            pass
        #right side
        try:
            M = cv2.moments(sobel_mask[i*vertical_division:(i+1)*vertical_division - 1,offset:sobel_mask.shape[1] -1],False)
            moments_center = (int(M['m10']/M['m00']) + offset, int(M['m01']/M['m00']) + i*vertical_division)
            right_centroids.append(moments_center)
        except:
            pass


    if len(left_centroids) > 0:
        left_line = np.polyfit([i[0]  for i in left_centroids], [i[1]  for i in left_centroids], 1)


    if len(right_centroids) > 0:
        right_line = np.polyfit([i[0]  for i in right_centroids], [i[1]  for i in right_centroids], 1)


    # print("    [-] Momentos: {}ms".format(int(1000*(time.time() - last_time))))
    # last_time = time.time()

    return [left_line, right_line]


#Evitamos que falten argumentos
if len(sys.argv) < 2:
    print ("Necesitas especificar el nombre del archivo cargar")
    exit()



#########################################################################################################################
######                                           Video Saving                                                      ######
#########################################################################################################################

# Define the codec and create VideoWriter object
if cv2.__version__ == '3.0.0':
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
else:
    fourcc = cv2.cv.CV_FOURCC('X', 'V', 'I', 'D')

out_name = sys.argv[1].split('/')[-1].split('.')

out = cv2.VideoWriter(out_name[0] + '_out.' + out_name[1],fourcc, 20.0, (1280,360))


#Initial Color filter variables
blancoLow     = np.array([45,0,168])
blancoHigh    = np.array([80,58,255])



#Obtain image
cap = cv2.VideoCapture(sys.argv[1])
ret,frame = cap.read()    #Definimos unas variables con el tamano de la imagen

print "{}x{}x{}".format(frame.shape[0], frame.shape[1], frame.shape[2])

first_time = time.time()
last_time = first_time


while(1):

    # print("[+] Detector de lineas:")
    # frame_time = time.time()

    ret,frame = cap.read()    #Definimos unas variables con el tamano de la imagen

    if frame == None or ret == None:
        cap = cv2.VideoCapture(sys.argv[1])
        ret,frame = cap.read()


    left_line, right_line = lineDetector(frame)

    # No calcules mas nada, si no encuentras las lineas.
    if left_line == None or right_line == None:
        continue

    #Creamos la mascara
    mask = np.zeros_like(frame)
    draw_lines(mask, [left_line, right_line], (255,255,255), 60)
    roi = np.bitwise_and(frame,mask)
    draw_lines(frame, [left_line, right_line], (0,0,255), 6)


    #Intentamos encontrar los teipes de colores
    hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,blancoLow,blancoHigh)



    # print("    [-] Leer imagen: {}ms".format(int(1000*(time.time() - last_time))))
    # last_time = time.time()










    mask3 = np.dstack((mask,mask,mask))
    # mask3 = np.dstack((sobel_mask,sobel_mask,sobel_mask))

    resultado = np.hstack((frame,mask3))

    # print("    [-] Stacking: {}ms".format(int(1000*(time.time() - last_time))))
    # last_time = time.time()


    resultado = cv2.resize(resultado,None,fx=.5, fy=.5, interpolation = cv2.INTER_AREA)

    # print("    [-] Resize: {}ms".format(int(1000*(time.time() - last_time))))
    # last_time = time.time()

    cv2.imshow('image',resultado)
    # out.write(resultado)

    # print("    [-] Show image: {}ms".format(int(1000*(time.time() - last_time))))
    # last_time = time.time()

    # print("[+] Frame time: {}ms\n".format(int(1000*(time.time() - frame_time))))
    # last_time = time.time()

    #time.sleep(0.2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        out.release()
        cv2.destroyAllWindows()
        break
