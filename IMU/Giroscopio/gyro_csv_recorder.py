""" Generador del Archivo CSV de analisis para el giroscopio

Este codigo se conecta por el bus de I2C del Raspberry PI modelo 2 al gyroscopio L3GD20H.
Toma mediciones por una cantidad de tiempo definido definido en la linea de comandos (una cada 2ms) que se escriben en el archivo CSV nombrado en la linea de comando para
despues ser analizado para medir varios valores del giroscopio.


Configuracion:
20h = 0x8F (DataRate 400Hz, BW 20Hz, All Axis enabled, Gyro ON)
23h = 0xA0 (Escala 2000dps, BlockUpdates )
24h = 0x00 (OutSel = 10h, skip HPF, use LPF2)


Ejemplo de uso:
python gyro_csv_recorder.py prueba.csv 20

"""

import smbus
import time
import csv
import sys

#Parseamos los argumentos de la linea de comandos

#Evitamos que falten argumentos
if len(sys.argv) < 3:
    print "Necesitas especificar (1) el nombre del archivo a guardar y (2) la cantidad de muestras"
    exit()
#Evitamos que el primer argumento no sean numeros
if not sys.argv[2].isdigit():
    print "'{}' no es un tiempo valido".format(sys.argv[2])
    exit()
#Evitamos que el archivo de escritura no sea un archivo .csv
if ".csv" not in sys.argv[1]:
    print "'{}' no es un nombre valido de archivo csv".format(sys.argv[1])
    exit()


gyro = smbus.SMBus(1)      #inicializamos el puerto
gyro_addr = 0x6B           #Direccion del giroscopio

print("Puerto creado")
time.sleep(3)

print("Prendiendo giroscopio")

gyro.write_byte_data(gyro_addr,0x20,0x8F)         #DataRate 400Hz, BW 20Hz, All Axis enabled, Gyro ON
gyro.write_byte_data(gyro_addr,0x23,0xA0)         #Escala 2000dps, BlockUpdates
gyro.write_byte_data(gyro_addr,0x24,0x02)         #OutSel = 10h, use HPF and LPF2, HPen = 0.
time.sleep(1)

#Definimos las variables referentes al timepo de medicion para poder medir los datos correctamente
tiempo_inicial = time.time()
tiempo_actual = tiempo_inicial
segundos_transcurridos = 0
tiempo_pasado = tiempo_actual

#Inicializamos los valores iniciales de las estimaciones integrales de los angulos
degx = 0
degy = 0
degz = 0

print('Iniciando la recoleccion de datos en "{}"'.format(sys.argv[2]))
f = open(sys.argv[1],'wt')                      #Abrimos el archivo csv definido en la linea de comandos

try:
    out_file =  csv.writer(f)
    out_file.writerow(('tiempo','Xdps','Ydps','Zdps','X','Y','Z'))                    #Escribimos el encabezado del archivo
    while(segundos_transcurridos < float(sys.argv[2])):                   #definimos que se van a tomar las muestras definidas en la linea de commandos

        ##Sacamos los datos de campo giroscopio de los 3 ejes

        #Eje X
        xh = gyro.read_byte_data(gyro_addr,0x29)
        xl = gyro.read_byte_data(gyro_addr,0x28)

        #Eje Y
        yh = gyro.read_byte_data(gyro_addr,0x2B)
        yl = gyro.read_byte_data(gyro_addr,0x2A)

        #Eje Z
        zh = gyro.read_byte_data(gyro_addr,0x2D)
        zl = gyro.read_byte_data(gyro_addr,0x2C)


        #Convertimos los resultados a binario para poder verlos
        xl = format(xl, '#010b')[2:]
        xh = format(xh, '#010b')[2:]

        yl = format(yl, '#010b')[2:]
        yh = format(yh, '#010b')[2:]

        zl = format(zl, '#010b')[2:]
        zh = format(zh, '#010b')[2:]


        #Y aplicamos el complemento a 2 para conseguir el numero
        x = int( xh[1:] + xl,2) - int(xh[0])*(2**(len(xh+xl)-1))
        y = int( yh[1:] + yl,2) - int(yh[0])*(2**(len(yh+yl)-1))
        z = int( zh[1:] + zl,2) - int(zh[0])*(2**(len(zh+zl)-1))


        #Calculamos los grados por segundo (para 2000dps)
        degx_s = float(x)*70/1000
        degy_s = float(y)*70/1000
        degz_s = float(z)*70/1000

        #Actualizamos el tiempo de la medicion
        tiempo_actual = time.time()
        segundos_transcurridos = tiempo_actual - tiempo_inicial

        #Calculamos el estimado (integrado) de los angulos para el momento actual
        degx += degx_s * (tiempo_actual - tiempo_pasado)
        degy += degy_s * (tiempo_actual - tiempo_pasado)
        degz += degz_s * (tiempo_actual - tiempo_pasado)

        #Copiamos el tiempo pasado para usarlos en la proxima vuelta.
        tiempo_pasado = tiempo_actual

        #Escribimos esta tanda de datos en el archivo csv
        out_file.writerow((segundos_transcurridos, str(degx_s), str(degy_s), str(degz_s), str(degx), str(degy), str(degz) ))
        print("{:0.2f}s   X_s: {:0.3f}dps    Y_s: {:0.3f}dps   Z_s: {:0.3f}dps ||  X:{:0.3f}deg   Y:{:0.3f}deg   Z:{:0.3f}deg   ".format(segundos_transcurridos, degx_s, degy_s, degz_s, degx, degy, degz))
        #Medimos cada 2ms
        time.sleep(0.002)

    print('finalizado la recoleccion de datos')
finally:
    #Nos aseguramos de que si pasa algo, el archivo se cierre.
    f.close()
    print('proceso finalizado')


