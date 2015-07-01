""" AHRS - Madgwicks, basico

Este codigo se conecta por el bus de I2C del Raspberry PI modelo 2 al IMU10 de Adafruit, y usa los datos de los sensores para
alimentar una implementacion del filtro de Madgwicks que retorna la orientacion en quaterniones del sensor (que son transformadas a Angulos
de Euler). Luego lo enivia por tcp/ip a una computadora que grafica el resultado.

"""

# Funciones de comunicacion

def get_interfaces():
    """ (Python 3) Funcion que devuelve una lista con strings de todos las interfaces de red que tenga tu computadora
        *NOTA: Solo funciona en Linux

        get_ifaces()
        ['enp3s0', 'vmnet1', 'vmnet8', 'wlp2s0', '    lo']"""

    with open('/proc/net/dev','r') as f:        #Abrimos el archivo con la informacion de red
        interfaces = []
        for linea in f:
            if ':' in linea:
                interfaces.append(linea[:linea.find(':')])  #Extraemos los primeros caracteres de las lineas con informacion de las interfaces
    return [iface.lstrip().rstrip() for iface in interfaces]


def get_ip_address2(ifname):
    """ (Python 2)Funcion que recibe un string con el nombre de una interfaz de red y devuelve
        un string con la direccion IP de la interfaz, o None si dicha interfaz no
        tiene direccion IP asignada.

            get_ip_address('wlp2s0')
            '192.168.1.4'               """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15])
        )[20:24])
    except:
        return None



def get_network_config2():
    """ (Python 2) Funcion que devuelve un diccionario  con las interfaces de red de la computadora y sus respectivas direcciones
        ip. """
    interfaces = get_interfaces()
    ips = [get_ip_address2(ip) for ip in interfaces]
    return dict(zip(interfaces,ips))




# Funciones que configuran  los sensores
def accel_setup():
    global ahrs
    global accel_addr
    ahrs.write_byte_data(accel_addr,0x23,0x88)     #Prendemos alta resolucion y hold de update de los registros de salida en el reg 23h
    ahrs.write_byte_data(accel_addr,0x20,0x27)     #sacamos el accelerometro del shutdown mode

def magn_setup():
    global ahrs
    global magn_addr
    ahrs.write_byte_data(magn_addr,0x00,0x10)         #Seteamos la velocidad de las mediciones a 15Hz
    ahrs.write_byte_data(magn_addr,0x01,0x20)         #Ponemos la escala +-1.3g
    ahrs.write_byte_data(magn_addr,0x02,0x00)         #Prendemos el magnetometro

def gyro_setup():
    global ahrs
    global gyro_addr
    ahrs.write_byte_data(gyro_addr,0x20,0x8F)         #DataRate 400Hz, BW 20Hz, All Axis enabled, Gyro ON
    ahrs.write_byte_data(gyro_addr,0x23,0xA0)         #Escala 2000dps, BlockUpdates
    ahrs.write_byte_data(gyro_addr,0x24,0x02)         #OutSel = 10h, use HPF and LPF2, HPen = 0.


# Funciones que sacan los valores de los sensores.

def accel_read():
    global ahrs
    global accel_addr
    accel_data = [0,0,0]
    ##Sacamos los datos de acceleracion de los 3 ejes
    #Eje X
    xl = format(ahrs.read_byte_data(accel_addr,0x28), '#010b')[2:6]
    xh = format(ahrs.read_byte_data(accel_addr,0x29), '#010b')[2:]
    #Eje Y
    yl = format(ahrs.read_byte_data(accel_addr,0x2A), '#010b')[2:6]
    yh = format(ahrs.read_byte_data(accel_addr,0x2B), '#010b')[2:]
    #Eje Z
    zl = format(ahrs.read_byte_data(accel_addr,0x2C), '#010b')[2:6]
    zh = format(ahrs.read_byte_data(accel_addr,0x2D), '#010b')[2:]
    ## Combinamos juntos los 2 bytes.
    accel_data[0] = int('0b' + xh[1:] + xl,2) - int(xh[0])*(2**(len(xh+xl)-1))  #Eje X  #Unimos los bytes en complemento a 2
    accel_data[1] = int('0b' + yh[1:] + yl,2) - int(yh[0])*(2**(len(yh+yl)-1))  #Eje Y  #Unimos los bytes en complemento a 2
    accel_data[2] = int('0b' + zh[1:] + zl,2) - int(zh[0])*(2**(len(zh+zl)-1))  #Eje Z  #Unimos los bytes en complemento a 2
    #Normalizamos el vector antes de retornarlo
    norma = np.linalg.norm(accel_data)
    accel_data = list(map(lambda x: x/norma,accel_data))
    return accel_data


def magn_read():
    global ahrs
    global magn_addr
    magn_data = [0,0,0]
    ##Sacamos los datos de campo magnetico de los 3 ejes
    #Eje X
    xh = ahrs.read_byte_data(magn_addr,0x03)
    xl = ahrs.read_byte_data(magn_addr,0x04)
    #Eje Y
    yh = ahrs.read_byte_data(magn_addr,0x07)
    yl = ahrs.read_byte_data(magn_addr,0x08)
    #Eje Z
    zh = ahrs.read_byte_data(magn_addr,0x05)
    zl = ahrs.read_byte_data(magn_addr,0x06)
    #Convertimos los resultados a binario para poder verlos
    xl = format(xl, '#010b')[2:]
    xh = format(xh, '#010b')[2:]
    yl = format(yl, '#010b')[2:]
    yh = format(yh, '#010b')[2:]
    zl = format(zl, '#010b')[2:]
    zh = format(zh, '#010b')[2:]
    #Y aplicamos el complemento a 2 para conseguir el numero
    magn_data[0] = int( xh[1:] + xl,2) - int(xh[0])*(2**(len(xh+xl)-1))
    magn_data[1] = int( yh[1:] + yl,2) - int(yh[0])*(2**(len(yh+yl)-1))
    magn_data[2] = int( zh[1:] + zl,2) - int(zh[0])*(2**(len(zh+zl)-1))
    #Escalamos los datos
    magn_data[0] = (magn_data[0] - 35.0) * 1.0
    magn_data[1] = (magn_data[1] + 35.0) * 1.02702702703
    magn_data[2] = (magn_data[2] - 3.0) * 0.974358974359
    #Normalizamos el vector
    norma = np.linalg.norm(magn_data)
    magn_data = list(map(lambda x: x/norma,magn_data))
    return magn_data


def gyro_read():
    global ahrs
    global gyro_addr
    gyro_data = [0,0,0]
    #Eje X
    xh = ahrs.read_byte_data(gyro_addr,0x29)
    xl = ahrs.read_byte_data(gyro_addr,0x28)
    #Eje Y
    yh = ahrs.read_byte_data(gyro_addr,0x2B)
    yl = ahrs.read_byte_data(gyro_addr,0x2A)
    #Eje Z
    zh = ahrs.read_byte_data(gyro_addr,0x2D)
    zl = ahrs.read_byte_data(gyro_addr,0x2C)
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
    gyro_data[0] = float(x)*70/1000
    gyro_data[1] = float(y)*70/1000
    gyro_data[2] = float(z)*70/1000
    #Transformamos los datos a radianes/seg
    gyro_data = list(map(math.radians, gyro_data))
    return gyro_data


def madgwicks_filter(accel_datas, magn_datas, gyro_datas, deltat):
    global SEq
    global b_x
    global b_z
    global w_b
    global beta
    global zeta

    # print "accel = {}".format(accel_datas)
    # print "magn = {}".format(magn_datas)
    # print "gyro = {}".format(gyro_datas)
    # print "deltat = {}".format(deltat)

    # print SEq
    # print b_x
    # print w_b
    # print beta


    #axulirary variables to avoid reapeated calcualtions
    halfSEq_1 = 0.5 * SEq[0]
    halfSEq_2 = 0.5 * SEq[1]
    halfSEq_3 = 0.5 * SEq[2]
    halfSEq_4 = 0.5 * SEq[3]
    twoSEq_1 = 2.0 * SEq[0]
    twoSEq_2 = 2.0 * SEq[1]
    twoSEq_3 = 2.0 * SEq[2]
    twoSEq_4 = 2.0 * SEq[3]
    twob_x = 2.0 * b_x
    twob_z = 2.0 * b_z
    twob_xSEq_1 = 2.0 * b_x * SEq[0]
    twob_xSEq_2 = 2.0 * b_x * SEq[1]
    twob_xSEq_3 = 2.0 * b_x * SEq[2]
    twob_xSEq_4 = 2.0 * b_x * SEq[3]
    twob_zSEq_1 = 2.0 * b_z * SEq[0]
    twob_zSEq_2 = 2.0 * b_z * SEq[1]
    twob_zSEq_3 = 2.0 * b_z * SEq[2]
    twob_zSEq_4 = 2.0 * b_z * SEq[3]
    SEq_1SEq_2 = SEq[0] * SEq[1]
    SEq_1SEq_3 = SEq[0] * SEq[2]
    SEq_1SEq_4 = SEq[0] * SEq[3]
    SEq_2SEq_3 = SEq[1] * SEq[2]
    SEq_2SEq_4 = SEq[1] * SEq[3]
    SEq_3SEq_4 = SEq[2] * SEq[3]
    twom_x = 2.0 * magn_datas[0]
    twom_y = 2.0 * magn_datas[1]
    twom_z = 2.0 * magn_datas[2]


    # compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq[3] - twoSEq_1 * SEq[2] - accel_datas[0]
    f_2 = twoSEq_1 * SEq[1] + twoSEq_3 * SEq[3] - accel_datas[1]
    f_3 = 1.0 - twoSEq_2 * SEq[1] - twoSEq_3 * SEq[2] - accel_datas[2]
    f_4 = twob_x * (0.5 - SEq[2] * SEq[2] - SEq[3] * SEq[3]) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - magn_datas[0]
    f_5 = twob_x * (SEq[1] * SEq[2] - SEq[0] * SEq[3]) + twob_z * (SEq[0] * SEq[1] + SEq[2] * SEq[3]) - magn_datas[1]
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5 - SEq[1] * SEq[1] - SEq[2] * SEq[2]) - magn_datas[2]
    J_11or24 = twoSEq_3                                                    # J_11 negated in matrix multiplication
    J_12or23 = 2.0 * SEq[3]
    J_13or22 = twoSEq_1                                                    # J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2
    J_32 = 2.0 * J_14or21                                                 # negated in matrix multiplication
    J_33 = 2.0 * J_11or24                                                 # negated in matrix multiplication
    J_41 = twob_zSEq_3                                                     # negated in matrix multiplication
    J_42 = twob_zSEq_4
    J_43 = 2.0 * twob_xSEq_3 + twob_zSEq_1                                # negated in matrix multiplication
    J_44 = 2.0 * twob_xSEq_4 - twob_zSEq_2                                # negated in matrix multiplication
    J_51 = twob_xSEq_4 - twob_zSEq_2                                       # negated in matrix multiplication
    J_52 = twob_xSEq_3 + twob_zSEq_1
    J_53 = twob_xSEq_2 + twob_zSEq_4
    J_54 = twob_xSEq_1 - twob_zSEq_3                                       # negated in matrix multiplication
    J_61 = twob_xSEq_3
    J_62 = twob_xSEq_4 - 2.0 * twob_zSEq_2
    J_63 = twob_xSEq_1 - 2.0 * twob_zSEq_3
    J_64 = twob_xSEq_2


    #print "f_1 = {}  f_2 = {} f_3 = {} f_4 = {} f_5 = {} f_6 = {}".format(f_1,f_2,f_3,f_4,f_5,f_6)

#    print "J_64 = {}  J_63 = {} J_62 = {} J_61 = {} J_54 = {} J_53 = {} J_52 = {} J_51 = {} J_44 = {} J_43 = {} J_42 = {} J_41 = {}".format(J_64,J_63,J_62,J_61,J_54,J_53,J_52,J_51,J_44,J_43,J_42,J_41)

    # compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6
###
    # print SEqHatDot_1
    # print SEqHatDot_2
    # print SEqHatDot_3
    # print SEqHatDot_4
    # print

    # normalise the gradient to estimate direction of the gyroscope error
    norm = math.sqrt(SEqHatDot_1**2 + SEqHatDot_2**2 + SEqHatDot_3**2 + SEqHatDot_4**2)
    SEqHatDot_1 = SEqHatDot_1 / norm
    SEqHatDot_2 = SEqHatDot_2 / norm
    SEqHatDot_3 = SEqHatDot_3 / norm
    SEqHatDot_4 = SEqHatDot_4 / norm


###
    # print "SEqHatDot_1: {} SEqHatDot_2: {} SEqHatDot_3: {} SEqHatDot_4: {}".format(SEqHatDot_1,SEqHatDot_2,SEqHatDot_3,SEqHatDot_4)
    # compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3
    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2
    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1

    # print "w_err_x: {}, w_err_y:{}, w_err_z:{}".format(w_err_x, w_err_y, w_err_z)
    # print "zeta: {}".format(zeta)
    # print "deltat: {}".format(deltat)

    # compute and remove the gyroscope baises
    # print "w_b1: {}".format(w_b)
    w_b[0] += w_err_x * deltat * zeta
    w_b[1] += w_err_y * deltat * zeta
    w_b[2] += w_err_z * deltat * zeta
    # print "w_b2: {}".format(w_b)
    gyro_datas[0] -= w_b[0]
    gyro_datas[1] -= w_b[1]
    # gyro_datas[2] -= w_b[2]
###
    # compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * gyro_datas[0] - halfSEq_3 * gyro_datas[1] - halfSEq_4 * gyro_datas[2]
    SEqDot_omega_2 = halfSEq_1 * gyro_datas[0] + halfSEq_3 * gyro_datas[2] - halfSEq_4 * gyro_datas[1]
    SEqDot_omega_3 = halfSEq_1 * gyro_datas[1] - halfSEq_2 * gyro_datas[2] + halfSEq_4 * gyro_datas[0]
    SEqDot_omega_4 = halfSEq_1 * gyro_datas[2] + halfSEq_2 * gyro_datas[1] - halfSEq_3 * gyro_datas[0]

    # compute then integrate the estimated quaternion rate
    SEq[0] += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat
    SEq[1] += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat
    SEq[2] += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat
    SEq[3] += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat

    # Normalizamos los quaterniones
    norm = np.linalg.norm(SEq)
    SEq = map(lambda x: x/norm,SEq)

    # compute flux in the earth frame
    SEq_1SEq_2 = SEq[0] * SEq[1]                                             # recompute axulirary variables
    SEq_1SEq_3 = SEq[0] * SEq[2]
    SEq_1SEq_4 = SEq[0] * SEq[3]
    SEq_3SEq_4 = SEq[2] * SEq[3]
    SEq_2SEq_3 = SEq[1] * SEq[2]
    SEq_2SEq_4 = SEq[1] * SEq[3]
    h_x = twom_x * (0.5 - SEq[2] * SEq[2] - SEq[3] * SEq[3]) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3)
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5 - SEq[1] * SEq[1] - SEq[3] * SEq[3]) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2)
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5 - SEq[1] * SEq[1] - SEq[2] * SEq[2])

    # normalise the flux vector to have only components in the x and z
    b_x = math.sqrt((h_x * h_x) + (h_y * h_y))
    b_z = h_z



def Quat_to_Euler(quater):

    euler = [0,0,0]

    euler[0] = math.atan2(2*(quater[0]*quater[1] + quater[2]*quater[3]),quater[0]*quater[0] - quater[1]*quater[1] - quater[2]*quater[2] + quater[3]*quater[3])
    euler[1] = math.asin(-2*((quater[0]*quater[2] - quater[1]*quater[3]))/(quater[0]*quater[0] + quater[1]*quater[1] + quater[2]*quater[2] + quater[3]*quater[3]))
    euler[2] = math.atan2(2*(quater[1]*quater[2] + quater[0]*quater[3]),-quater[0]*quater[0] - quater[1]*quater[1] + quater[2]*quater[2] + quater[3]*quater[3])

    euler = map(math.degrees,euler)
    return euler




import smbus
import time
import numpy as np
import math
import socket
import fcntl
import struct


#Analizamos la red para encontrar el ip correcto
inter_faces = get_network_config2()
if inter_faces['eth0'] == None:        #Le damos prioridad a la conexion ethernet
    host = inter_faces['wlan0']
    tarjeta = 'wlan0'
else:
    host = inter_faces['eth0']
    tarjeta = 'eth0'


print("Intentando establecer conexion en interfaz {} con la direccion ip {}".format(tarjeta, host))
#Establecemos la conexion
try:
    port = 23322
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host,port))
    s.listen(1)
    conn,addr = s.accept()
except:
    s.close()                       #Si algo falla, cierra todo.
    print("[-] ERROR = No se pudo establecer la conexion")
    exit()


#Abrimos el puerto I2C
ahrs = smbus.SMBus(1)

#Definimos las direcciones de los sensores
gyro_addr = 0x6B
accel_addr = 0x19
magn_addr = 0x1E

#Variables globales
SEq = [0.1,0.1,0.1,0.1] #Quaterniones
b_x = 1         #Earth Flux
b_z = 0
w_b = [0,0,0]   #Gyroscopic Bias Error
beta = math.sqrt(3.0/4.0)*math.radians(5)   #gyro measurment error rad/s  (5 deg/s)
zeta = math.sqrt(3.0/4.0)*math.radians(0.2)   #gyro drift error rad/s/s   (0.2 deg/s/s)


#Colocamos los valores de configuracion
accel_setup()
magn_setup()
gyro_setup()

#Leemos los datos de los sensores.
accel_data = accel_read()
magn_data = magn_read()
gyro_data = gyro_read()

#Variables de tiempo
time_new = 0
time_old = time.time()

#loop de control
while(1):


    #sacamos medidas de sensores
    accel_data = accel_read()
    magn_data = magn_read()
    gyro_data = gyro_read()

    #medimos tiempo
    time_new = time.time()

    #corremos el filtro
    madgwicks_filter(accel_data, magn_data, gyro_data, time_new - time_old)

    #Actualizamos el tiempo
    time_old = time_new

    #Calculamos los Angulos de Euler
    Angulos = Quat_to_Euler(SEq)


    #Imprimimos
    print("Pitch: {:+.2f}deg   Roll: {:+.2f}deg   Yaw: {:+.2f}deg    Quaternion:({:+.3f}, {:+.3f}, {:+.3f}, {:+.3f})".format(Angulos[0],Angulos[1],Angulos[2], SEq[0], SEq[1], SEq[2], SEq[3] ))

    mensaje = "{:+.2f},{:+.2f},{:+.2f}\n".format(Angulos[0],Angulos[1],Angulos[2])
    try:
        conn.sendall(mensaje)           #Enviamos por TCP la informacion
    except:
        s.close()                       #Si algo falla, cierra todo.
        print("[-] ERROR = No se pudo mandar el paquete")
        exit()

    time.sleep(0.01)
    # print("Accel:({:+.3f},{:+.3f},{:+.3f})     Magn:({:+.3f},{:+.3f},{:+.3f})     Gyro:({:+.3f},{:+.3f},{:+.3f})".format(accel_data[0],accel_data[1],accel_data[2],magn_data[0],magn_data[1],magn_data[2],gyro_data[0],gyro_data[1],gyro_data[2]))







