""" Graficacion de archivos csv para el giroscopio.


Este codigo lee el archivo CSV indicado en la linea de comandos que es un archivo con una cantidad indefinida de valores en punto flotante tomados del
giroscopio LG3D20H en diferentes configuraciones.
El codigo grafica las velocidades angulares registradas (a la izquierda) y la posicion angular integrada  (derecha). Tambien calculamos la regresion lineal
De las graficas de velocidad. Y el promedio y la desviacion estandar para poder analizar el efecto de las diferentes configuraciones
sobre el drift estatico y el ruido de alta frecuencia que introduce el filtro pasa alto. Para esto usamos numpy y matplotlib.

el reultado de estas cuentas salen por stdout.

Ejemplo de uso:
python gyro_csv_grapher.py prueba.csv

"""
import matplotlib
matplotlib.use('Tkagg')
#matplotlib.use('GTKAgg')
import numpy as np
import matplotlib.pyplot as plt
import sys

#Parseamos los argumentos de la linea de comandos

#Evitamos que falten argumentos
if len(sys.argv) < 2:
    print ("Necesitas especificar (1) el nombre del archivo a guardar y (2) la cantidad de muestras")
    exit()
#Evitamos que el archivo de escritura no sea un archivo .csv
if ".csv" not in sys.argv[1]:
    print ("'{}' no es un nombre valido de archivo csv".format(sys.argv[1]))
    exit()

##################################################################################################################
##                                         Lectura de archivo csv                                               ##
##################################################################################################################

#abrimos y leemos el archivo
print("Abrimos el archivo '{}'".format(sys.argv[1]))

#Leemos los datos
data = np.genfromtxt(sys.argv[1], delimiter=',', names=True)   #Funcion de Numpy que te deja sacar datos de un archivo de texto (devuelve un diccionario de listas)
                                                                       #delimiter = que simbolo separa los valores de cada columna
                                                                       #names = True; saca los nombres de la columna del mismo archivo (alternativa; names=['x', 'y', 'z'])


##################################################################################################################
##                                       Regresion Lineal                                                       ##
##################################################################################################################


#Aplicamos una regresion lineal de primer orden a todos los sets de datos
#A las velocidades angulares les encontramos su Corte con Y, y su valor de R^2
#Y a las estimaciones les encontramos su pendiente.

#Velocidades Angulares
regrXdps = np.polyfit(data['tiempo'], data['Xdps'], 1)
regrYdps = np.polyfit(data['tiempo'], data['Ydps'], 1)
regrZdps = np.polyfit(data['tiempo'], data['Zdps'], 1)

#Posiciones Angulares estimadas
regrX = np.polyfit(data['tiempo'], data['X'], 1)
regrY = np.polyfit(data['tiempo'], data['Y'], 1)
regrZ = np.polyfit(data['tiempo'], data['Z'], 1)

#Mismo procedimiento, pero por estadisticas
Xdps_mean = np.mean(data['Xdps'])
Xdps_std  = np.std(data['Xdps'])
Ydps_mean = np.mean(data['Ydps'])
Ydps_std  = np.std(data['Ydps'])
Zdps_mean = np.mean(data['Zdps'])
Zdps_std  = np.std(data['Zdps'])


#Imprimimos los resultados.
# print("EjeX: drift = ({:0.4f} +-{:0.4f})dps\n      Angulo = {:0.4f}*t +{:0.4f}".format(Xdps_mean,Xdps_std,regrX[0],regrX[1]))
# print("EjeY: drift = ({:0.4f} +-{:0.4f})dps\n      Angulo = {:0.4f}*t +{:0.4f}".format(Ydps_mean,Ydps_std,regrY[0],regrY[1]))
# print("EjeZ: drift = ({:0.4f} +-{:0.4f})dps\n      Angulo = {:0.4f}*t +{:0.4f}".format(Zdps_mean,Zdps_std,regrZ[0],regrZ[1]))

print("EjeX: Drift = ({:0.4f} +-{:0.4f})dps\tEjeY: Drift = ({:0.4f} +-{:0.4f})dps\tEjeZ: Drift = ({:0.4f} +-{:0.4f})dps".format(Xdps_mean,Xdps_std,Ydps_mean,Ydps_std,Zdps_mean,Zdps_std))
print("      Angulo = {:0.4f}*t +{:0.4f}\t      Angulo = {:0.4f}*t +{:0.4f}\t      Angulo = {:0.4f}*t +{:0.4f}".format(regrX[0],regrX[1],regrY[0],regrY[1],regrZ[0],regrZ[1]))




##################################################################################################################
##                                          Graficacion                                                         ##
##################################################################################################################


#Creamos las figuras de Matplotlib
fig1 = plt.figure()

#Generamos el Axes handler
ax1 = fig1.add_subplot(3,2,1)
ax2 = fig1.add_subplot(3,2,2)
ax3 = fig1.add_subplot(3,2,3)
ax4 = fig1.add_subplot(3,2,4)
ax5 = fig1.add_subplot(3,2,5)
ax6 = fig1.add_subplot(3,2,6)

#Generamos las graficas


#Top Left  = Xdps
ax1.plot(data['tiempo'], data['Xdps'], 'b')
ax1.plot(data['tiempo'], list(map(lambda x: x*regrXdps[0]+regrXdps[1],data['tiempo'])),'k')
ax1.set_title('Eje X, Velocidad Angular')
# ax1.set_xlabel('tiempo (segundos)')
ax1.set_ylabel('Xdps (grados por segundo)')
ax1.grid()

#Top Rigth  = X
ax2.plot(data['tiempo'], data['X'], 'r')
ax2.set_title('Eje X, Posicion Angular (Integrada)')
# ax2.set_xlabel('tiempo (segundos)')
ax2.set_ylabel('Xdps (grados por segundo)')
ax2.grid()

#Mid Left  = Ydps
ax3.plot(data['tiempo'], data['Ydps'], 'b')
ax3.plot(data['tiempo'], list(map(lambda x: x*regrYdps[0]+regrYdps[1],data['tiempo'])),'k')
ax3.set_title('Eje Y, Velocidad Angular')
# ax3.set_xlabel('tiempo (segundos)')
ax3.set_ylabel('Xdps (grados por segundo)')
ax3.grid()

#Mid Rigth  = Y
ax4.plot(data['tiempo'], data['Y'], 'r')
ax4.set_title('Eje Y, Posicion Angular (Integrada)')
# ax4.set_xlabel('tiempo (segundos)')
ax4.set_ylabel('Xdps (grados por segundo)')
ax4.grid()

#Bottom Left  = Zdps
ax5.plot(data['tiempo'], data['Zdps'], 'b')
ax5.plot(data['tiempo'], list(map(lambda x: x*regrZdps[0]+regrZdps[1],data['tiempo'])),'k')
ax5.set_title('Eje Z, Velocidad Angular')
ax5.set_xlabel('tiempo (segundos)')
ax5.set_ylabel('Xdps (grados por segundo)')
ax5.grid()

#Bottom Rigth  = Z
ax6.plot(data['tiempo'], data['Z'], 'r')
ax6.set_title('Eje Z, Posicion Angular (Integrada)')
ax6.set_xlabel('tiempo (segundos)')
ax6.set_ylabel('Xdps (grados por segundo)')
ax6.grid()

#Mostramos las graficas
plt.show()






