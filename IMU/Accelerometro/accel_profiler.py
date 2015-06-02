

""" Perfilador del Accelerometro

Este codigo se conecta por el bus de I2C del Raspberry PI modelo 2 al accelerometro LSM303.
Pasa por todos los valores de de escala y resolucion, y calcula que formato genera la salida.

CTRL_REG1_A (20h) =  ODR[3:0] | Lpen | Zen | Yen | Xen   (Default = 00000111)

ODR[3:0] (Data rate) =  0000    Power Down
                        0001    1   Hz
                        0010    10  Hz
                        0011    25  Hz
                        0100    50  Hz
                        0101    100 Hz
                        0110    200 Hz
                        0111    400 Hz
Lpen = Low power enable


CTRL_REG4_A (23h) =  BDU | BLE | FS[1:0] | HR | 0 | 0 | SIM   (Default = 00000000)

BDU = Block Data Update, output register don't actualize until you read them both (1 ON, 0 OFF)
BLE =  Big-little endian
FS  (Full scale) =  00  +-2G
                    01  +-4G
                    10  +-8G
                    11  +-16G
HR = 1 high resolution, 0 low resolution


Configuracion:
20h = 0x47 (100Hz, all axis ON)
23h = 0b10XXX000 (BDU ON, FS y Hr varian)


"""

import smbus
import time
import math

ctrl_reg4 = [ 0b10000000, 0b10001000, 0b10010000, 0b10011000, 0b10100000, 0b10101000, 0b10110000, 0b10111000 ]


accel = smbus.SMBus(1)      #inicializamos el puerto
accel_addr = 0x19           #Direccion del accelerometro

print("Puerto creado")
time.sleep(3)

print("Prendiendo accelerometro")
time.sleep(1)


for config in ctrl_reg4:

    #Sacamos la informacion de la configuracion para poder imprimir el mensaje
    configuracion = format(config, '#010b')[2:]
    FS = int(configuracion[2:4],2)
    HR = bool(int(configuracion[4],2))
    escala = 2**(FS+1)

    print("[+] Escala = +-{},  High Resolution = {}".format(escala, HR))

    accel.write_byte_data(accel_addr,0x20,0x47)         #Apagamos el accelerometro
    accel.write_byte_data(accel_addr,0x23,config)       #Cargamos la configuracion a probar

    x = 0
    y = 0
    z = 0

    for i in range(100):

        ##Sacamos los datos de acceleracion de los 3 ejes

        #Eje X
        xl = accel.read_byte_data(accel_addr,0x28)
        xh = accel.read_byte_data(accel_addr,0x29)

        #Eje Y
        yl = accel.read_byte_data(accel_addr,0x2A)
        yh = accel.read_byte_data(accel_addr,0x2B)

        #Eje Z
        zl = accel.read_byte_data(accel_addr,0x2C)
        zh = accel.read_byte_data(accel_addr,0x2D)

        ## Combinamos juntos los 2 bytes.
        #Eje X
        x |= int((xh << 8) | xl)
        #Eje Y
        y |= int((yh << 8) | yl)
        #Eje Z
        z |= int((zh << 8) | zl)

        #Unimos las respuestas juntas
        respuesta = x | y | z
        respuesta = format(respuesta, '#018b')[2:]


    #imprimimos el resultado
    print("   [*] Salida = {} {}".format(respuesta[:8], respuesta[8:]))
    print("   [*] Cantidad de bits = {}".format(respuesta.count('1')))
    print("   [*] factor de escalamiento = {}\n".format((2**(respuesta.count('1')-1))/escala))


