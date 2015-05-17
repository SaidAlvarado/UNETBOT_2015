import smbus
import time


accel = smbus.SMBus(1)      #inicializamos el puerto
accel_addr = 0x19           #Direccion del accelerometro

print("Puerto creado")
time.sleep(5)

print("Prendiendo accelerometro")
accel.write_byte_data(accel_addr,0x20,0x27)     #sacamos el accelerometro del shutdown mode
time.sleep(1)

while(1):
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
    x = (xh << 8) | xl                  #Unimos los 2 bytes
    if (x > (2^15)-1): x = x - 2^16     #implementamos el complemento a 2
    accelX = float(x)/1000              #Dividimos entre 1000 para conseguir las gravedades

    #Eje Y
    y = (yh << 8) | yl                  #Unimos los 2 bytes
    if (y > (2^15)-1): y = y - 2^16     #implementamos el complemento a 2
    accelY = float(y)/1000              #Dividimos entre 1000 para conseguir las gravedades

    #Eje Z
    z = (zh << 8) | zl                  #Unimos los 2 bytes
    if (z > (2^15)-1): z = z - 2^16     #implementamos el complemento a 2
    accelZ = float(z)/1000              #Dividimos entre 1000 para conseguir las gravedades


    #imprimimos el resultado
    print("Acceleracion    X: {}   Y: {}   Z: {}".format(accelX, accelY, accelZ))
    time.sleep(0.3) #Esperamos un poco antes de volver a medir


