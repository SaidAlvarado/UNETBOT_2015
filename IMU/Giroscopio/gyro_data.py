""" Lector del Giroscopio

Este codigo se conecta por el bus de I2C del Raspberry PI modelo 2 al giroscopio L3GD20H.
he imprime los valores en binario y en decimal cada 150ms.


Configuracion:
20h = 0xBF (DataRate 400Hz, BW 110Hz, All Axis enabled, Gyro ON)
23h = 0xA0 (Escala 2000dps, BlockUpdates )
24h = 0x00 (OutSel = 00h, skip HPF and LPF2)


"""

import smbus
import time
import math


gyro = smbus.SMBus(1)      #inicializamos el puerto
gyro_addr = 0x6B           #Direccion del giroscopio

print("Puerto creado")
time.sleep(3)

print("Prendiendo giroscopio")
gyro.write_byte_data(gyro_addr,0x20,0xBF)         #DataRate 400Hz, BW 110Hz, All Axis enabled, Gyro ON
gyro.write_byte_data(gyro_addr,0x23,0xA0)         #Escala 245dps, BlockUpdates
gyro.write_byte_data(gyro_addr,0x24,0x00)         #OutSel = 00h, skip HPF and LPF2
time.sleep(1)



while(1):

    ##Sacamos los datos de campo giroscopio de los 3 ejes

    #Eje X
    xh = gyro.read_byte_data(gyro_addr,0x029)
    xl = gyro.read_byte_data(gyro_addr,0x28)

    #Eje Y
    yh = gyro.read_byte_data(gyro_addr,0x2B)
    yl = gyro.read_byte_data(gyro_addr,0x2A)

    #Eje Z
    zh = gyro.read_byte_data(gyro_addr,0x2D)
    zl = gyro.read_byte_data(gyro_addr,0x2C)


#    respuesta = format(respuesta, '#018b')[2:]

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
    degx = float(x)*70/1000
    degy = float(y)*70/1000
    degz = float(z)*70/1000

    #imprimimos el resultado
    print("Medida:")
    print("   X = {} {}  ;  Xbin = {}  ;  Xgyro = {:0.2f}dps".format(xh, xl, x, degx))
    print("   Y = {} {}  ;  Ybin = {}  ;  Ygyro = {:0.2f}dps".format(yh, yl, y, degy))
    print("   Z = {} {}  ;  Zbin = {}  ;  Zgyro = {:0.2f}dps".format(zh, zl, z, degz))

    time.sleep(0.15)


