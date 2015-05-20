""" Lector de acceleracion

Este codigo se conecta por el bus de I2C del Raspberry PI modelo 2 al accelerometro LSM303.
Lee la acceleracion en los 3 ejes escala la unidad de medida a gravedades y los imprime.  Luego calcula los
angulos de Euler "Pitch" y "Roll"  los imprime en grados.

El accelerometro se configuro como:
- Velocidad de medicion = 10Hz
- Modo Alta resolucion = ON
- Data Uptade Hold = ON

Formato del Resultado:  (D = dato, X = no usado (se queda en 0 siempre))

High = [D D D D D D D D] : [D D D D X X X X] = Low
       MSB                       LSB

"""

import smbus
import time
import math


accel = smbus.SMBus(1)      #inicializamos el puerto
accel_addr = 0x19           #Direccion del accelerometro

print("Puerto creado")
time.sleep(3)

print("Prendiendo accelerometro")
accel.write_byte_data(accel_addr,0x23,0b10001000)     #Prendemos alta resolucion y hold de update de los registros de salida en el reg 23h
accel.write_byte_data(accel_addr,0x20,0x27)     #sacamos el accelerometro del shutdown mode
time.sleep(1)

while(1):
    ##Sacamos los datos de acceleracion de los 3 ejes

    #Eje X
    xl = format(accel.read_byte_data(accel_addr,0x28), '#010b')[2:6]
    xh = format(accel.read_byte_data(accel_addr,0x29), '#010b')[2:]

    #Eje Y
    yl = format(accel.read_byte_data(accel_addr,0x2A), '#010b')[2:6]
    yh = format(accel.read_byte_data(accel_addr,0x2B), '#010b')[2:]

    #Eje Z
    zl = format(accel.read_byte_data(accel_addr,0x2C), '#010b')[2:6]
    zh = format(accel.read_byte_data(accel_addr,0x2D), '#010b')[2:]

    ## Combinamos juntos los 2 bytes.

    #Eje X
    x = int('0b' + xh[1:] + xl,2) - int(xh[0])*(2**(len(xh+xl)-1))    #Unimos los bytes en complemento a 2
    accelX = float(x)/1000                                         #Dividimos entre 1000 para conseguir las gravedades

    #Eje Y
    y = int('0b' + yh[1:] + yl,2) - int(yh[0])*(2**(len(yh+yl)-1))    #Unimos los bytes en complemento a 2
    accelY = float(y)/1000              #Dividimos entre 1000 para conseguir las gravedades

    #Eje Z
    z = int('0b' + zh[1:] + zl,2) - int(zh[0])*(2**(len(zh+zl)-1))    #Unimos los bytes en complemento a 2
    accelZ = float(z)/1000               #Dividimos entre 1000 para conseguir las gravedades


    #Calculando los angulos de euler
    pitch = math.degrees(math.atan2(y,math.sqrt(x**2 + z**2)))
    roll  = math.degrees(math.atan2(-x,z))

    #imprimimos el resultado
    #print("Acceleracion    X: {}    Y: {}   Z: {}".format(xh+' '+xl, yh+' '+yl, zh+' '+zl))
    print("Acceleracion    X: {}g    Y: {}g   Z: {}g".format(accelX, accelY, accelZ))
    print("Angulos  Pitch: {0:0.1f}   Roll: {1:0.1f}".format(pitch,roll))
    time.sleep(0.3) #Esperamos un poco antes de volver a medir


