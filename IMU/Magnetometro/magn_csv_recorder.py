""" Generador del Archivo CSV de calibracion para el magnetometro

Este codigo se conecta por el bus de I2C del Raspberry PI modelo 2 al magnetometro LSM303.
Toma 500 mediciones (una cada 100ms) que se escriben en el archivo "magn_csv_data.csv" para despues ser analizado para
calibrar correctamente el magnetometro.


Configuracion:
00h = 0x10 (15Hz)
01h = 0b00100000 (la escala +-1.3Gauss)
02h = 0x00 (Continuous Conversion)


Nota: para todas las configuraciones usan 12bits y el resultado siempre esta entre  -2048 y 2047

High = [x x x x D D D D] : [D D D D D D D D] = Low
       MSB                       LSB

El valor especial 11110000 00000000 (-4096) es el valor de overflow, si este valor aparece es que te fuiste del rango.

"""

import smbus
import time
import csv


magn = smbus.SMBus(1)      #inicializamos el puerto
magn_addr = 0x1E           #Direccion del magnetometro

print("Puerto creado")
time.sleep(3)

print("Prendiendo magnetometro")
magn.write_byte_data(magn_addr,0x00,0x10)         #Seteamos la velocidad de las mediciones a 15Hz
magn.write_byte_data(magn_addr,0x01,0x20)         #Ponemos la escala +-1.3g
magn.write_byte_data(magn_addr,0x02,0x00)         #Prendemos el magnetometro
time.sleep(1)


print("Iniciando la recoleccion de datos")
f = open("magn_csv_data.csv",'wt')                      #Abrimos el archivo csv

try:
    out_file =  csv.writer(f)
    out_file.writerow(('X','Y','Z'))                    #Escribimos el encabezado del archivo
    for i in range(500):                                #definimos que se van a tomar 500 muestras

        ##Sacamos los datos de campo magnetico de los 3 ejes

        #Eje X
        xh = magn.read_byte_data(magn_addr,0x03)
        xl = magn.read_byte_data(magn_addr,0x04)

        #Eje Y
        yh = magn.read_byte_data(magn_addr,0x07)
        yl = magn.read_byte_data(magn_addr,0x08)

        #Eje Z
        zh = magn.read_byte_data(magn_addr,0x05)
        zl = magn.read_byte_data(magn_addr,0x06)



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

        #Escribimos esta tanda de datos en el archivo csv
        out_file.writerow(( str(x), str(y), str(z) ))
        print("{} medicion    X: {}    Y: {}   Z: {}".format(i, x, y, z))
        time.sleep(0.1)

    print('finalizado la recoleccion de datos')
finally:
    f.close()
    print('proceso finalizado')


