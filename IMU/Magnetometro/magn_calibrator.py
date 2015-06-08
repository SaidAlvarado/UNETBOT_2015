""" Calibrador del Magnetometro

Este codigo lee el archivo CSV indicado en la linea de comandos que es un archivo con una cantidad indefinida de valores enteros tomados del
magnetometro LSM303 en escala de +-1.3gauss.
El codigo grafica el elipsoide que generan estos valores y saca las cuentas de offset y ganancia para normalizarla a una
esfera, y regrafica el scatter plot. Para esto usamos numpy y matplotlib.


Para calibrar las medidas se usa la siguiente formula
X = magn.readX()
X_cal = (X - x_offset)*x_scale


Formato del archivo CSV:
'X','Y','Z'
'dataX1','dataY1','dataZ1'
'dataX2','dataY2','dataZ2'
...
'dataX500','dataY500','dataZ500'

Ejemplo de uso:
python magn_calibrator.py prueba.csv

"""
import matplotlib
matplotlib.use('Tkagg')
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sys
import csv

#Parseamos los argumentos de la linea de comandos

#Evitamos que falten argumentos
if len(sys.argv) < 2:
    print "Necesitas especificar (1) el nombre del archivo a guardar y (2) la cantidad de muestras"
    exit()
#Evitamos que el archivo de escritura no sea un archivo .csv
if ".csv" not in sys.argv[1]:
    print "'{}' no es un nombre valido de archivo csv".format(sys.argv[1])
    exit()


#abrimos y leemos el archivo
print("Abrimos el archivo '{}'".format(sys.argv[1]))

#Leemos los datos
data = np.genfromtxt(sys.argv[1], delimiter=',', names=True)   #Funcion de Numpy que te deja sacar datos de un archivo de texto (devuelve un diccionario de listas)
                                                                       #delimiter = que simbolo separa los valores de cada columna
                                                                       #names = True; saca los nombres de la columna del mismo archivo (alternativa; names=['x', 'y', 'z'])

#Creamos las figuras de Matplotlib
fig1 = plt.figure()
ax1 = fig1.add_subplot(111, projection='3d')      #Generamos el Axes handler

ax1.scatter(data['X'], data['Y'], data['Z'], c='r', marker='o')

ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')


######################################################################################################################
#                                           Compensacion de errores                                                  #
######################################################################################################################

#Calculamos el offset de cada Eje para eliminar el HARD IRON OFFSET
x_offset = (max(data['X']) + min(data['X']))/2
y_offset = (max(data['Y']) + min(data['Y']))/2
z_offset = (max(data['Z']) + min(data['Z']))/2
#Imprimimos el resultado
print("Hard Iron Offset calculated X = {}, Y = {}, Z = {}".format(x_offset,y_offset,z_offset))


#Calculamos las correciones para el Soft Iron Offset.

#Restamos el offset del valor maximo y minimo de los datos
x_max = max(data['X']) - x_offset
y_max = max(data['Y']) - y_offset
z_max = max(data['Z']) - z_offset

x_min = min(data['X']) - x_offset
y_min = min(data['Y']) - y_offset
z_min = min(data['Z']) - z_offset

#Sacamos la distancia promedio al centro de la esfera
x_avgs = (x_max - x_min)/2
y_avgs = (y_max - y_min)/2
z_avgs = (z_max - z_min)/2

#Ahora calculamos el promedio de las distancias al centro.
avgs_rad = (x_avgs + y_avgs +  z_avgs)/3

#Ahora calculamos la correcion de escala como:
x_scale = avgs_rad/x_avgs
y_scale = avgs_rad/y_avgs
z_scale = avgs_rad/z_avgs

#Imprimimos el resultado de las correciones del Soft Iron offset.
print("Soft Iron Offset calculated X = {}, Y = {}, Z = {}".format(x_scale,y_scale,z_scale))


#Procedemos a arreglar las medidas y regraficar.
DataX_cal = map(lambda x: (x-x_offset)*x_scale, data['X'])
DataY_cal = map(lambda x: (x-y_offset)*y_scale, data['Y'])
DataZ_cal = map(lambda x: (x-z_offset)*z_scale, data['Z'])

#Abrimos una segunda figura para graficar los puntos arreglados
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection='3d')      #Generamos el Axes handler

#Imprimimos unos resultados de comparacion, para saber que todo salio bien
print("\nX -> pre = [{}, {}]    post = [{}, {}]".format(min(data['X']), max(data['X']), min(DataX_cal), max(DataX_cal)))
print("Y -> pre = [{}, {}]    post = [{}, {}]".format(min(data['Y']), max(data['Y']), min(DataY_cal), max(DataY_cal)))
print("Z -> pre = [{}, {}]    post = [{}, {}]".format(min(data['Z']), max(data['Z']), min(DataZ_cal), max(DataZ_cal)))

#Graficamos los nuevos datos!
ax2.scatter(DataX_cal, DataY_cal, DataZ_cal, c='r', marker='o')

ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

plt.show()


#########################################################################################
#                           Escribimos la salida en un archivo CSV                      #
#########################################################################################


#Ahora tenemos un pequeno codigo que escribe los resultados de la calibracion en otro archivo csv
f = open(sys.argv[1][:-4] + '_calibrated.csv','wt')                  #Abrimos el archivo csv de salida
try:
    out_file =  csv.writer(f)
    out_file.writerow(('X','Y','Z'))                    #Escribimos el encabezado del archivo
    for i in range(len(DataX_cal)):
        out_file.writerow(( str(DataX_cal[i]), str(DataY_cal[i]), str(DataZ_cal[i]) ))         #escribimos los datos calibrados
    print('datos escritos!')
finally:
    #Nos aseguramos de que si pasa algo, el archivo se cierre.
    f.close()

















