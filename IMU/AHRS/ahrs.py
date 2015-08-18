""" AHRS - Madgwicks, basico

Este codigo se conecta por el bus de I2C del Raspberry PI modelo 2 al gyroscopio L3GD20H.
Toma mediciones por una cantidad de tiempo definido definido en la linea de comandos (una cada 100ms) que se escriben en el archivo CSV nombrado en la linea de comando para
despues ser analizado para medir varios valores del giroscopio.


Configuracion:
20h = 0x8F (DataRate 400Hz, BW 20Hz, All Axis enabled, Gyro ON)
23h = 0xA0 (Escala 2000dps, BlockUpdates )
24h = 0x00 (OutSel = 10h, skip HPF, use LPF2)


Ejemplo de uso:
python gyro_csv_recorder.py prueba.csv 20

"""

#If we are run instead of imported, run the example



#Importaciones
import smbus
import time
import numpy as np
import math



class MadgwicksFilter():

    #Direcciones de los sensores
    gyro_addr = 0x6B
    accel_addr = 0x19
    magn_addr = 0x1E

    def __init__(self,bus = 1,beta = 5, zeta = 0.2, init_quaternion = [0.0, 0.0, 0.0, 1]):

        #Definicion del bus
        self.i2c = smbus.SMBus(bus)
        #Variables globales
        self.SEq = init_quaternion  #Quaterniones
        self.b_x = 1                #Earth Flux
        self.b_z = 0
        self.w_b = [0,0,0]          #Gyroscopic Bias Error
        self.beta = math.sqrt(3.0/4.0)*math.radians(beta)   #gyro measurment error rad/s  (5 deg/s)
        self.zeta = math.sqrt(3.0/4.0)*math.radians(zeta)   #gyro drift error rad/s/s   (0.2 deg/s/s)
        #Variables de tiempo
        self.time_new = 0
        self.time_old = time.time()
        #Inicializamos las variables que guardan los datos de los sensores
        self.accel_data = [0,0,0]            # Vector normalizado de acceleraciones lineales [x,y,z]
        self.magn_data = [0,0,0]             # Vector normalizado del campo magnetico [x,y,z]
        self.gyro_data = [0,0,0]             # Vector de velocidades angulares (en rad/seg) [x,y,z]
        #Guardamos un arreglo especial para guardar el valor arreglado del gyroscopio
        self.gyro_data_fixed = [0,0,0]


    def begin(self):

        #Inicializamos los sensores
        self.accelSetup()
        self.magnSetup()
        self.gyroSetup()

        #Inicializamos las variables de tiempo
        self.time_new = 0
        self.time_old = time.time()

        #Hacemos las primeras lecturas
        self.accelRead()
        self.magnRead()
        self.gyroRead()
        return


# Funciones que configuran  los sensores
    def accelSetup(self):

        self.i2c.write_byte_data(self.accel_addr,0x23,0x88)     #Prendemos alta resolucion y hold de update de los registros de salida en el reg 23h
        self.i2c.write_byte_data(self.accel_addr,0x20,0x27)     #sacamos el accelerometro del shutdown mode
        return

    def magnSetup(self):

        self.i2c.write_byte_data(self.magn_addr,0x00,0x10)         #Seteamos la velocidad de las mediciones a 15Hz
        self.i2c.write_byte_data(self.magn_addr,0x01,0x20)         #Ponemos la escala +-1.3g
        self.i2c.write_byte_data(self.magn_addr,0x02,0x00)         #Prendemos el magnetometro
        return

    def gyroSetup(self):

        self.i2c.write_byte_data(self.gyro_addr,0x20,0x8F)         #DataRate 400Hz, BW 20Hz, All Axis enabled, Gyro ON
        self.i2c.write_byte_data(self.gyro_addr,0x23,0xA0)         #Escala 2000dps, BlockUpdates
        self.i2c.write_byte_data(self.gyro_addr,0x24,0x02)         #OutSel = 10h, use HPF and LPF2, HPen = 0.
        return


    # Funciones que sacan los valores de los sensores.

    def accelRead(self):
        ##Sacamos los datos de acceleracion de los 3 ejes
        #Eje X
        xl = format(self.i2c.read_byte_data(self.accel_addr,0x28), '#010b')[2:6]
        xh = format(self.i2c.read_byte_data(self.accel_addr,0x29), '#010b')[2:]
        #Eje Y
        yl = format(self.i2c.read_byte_data(self.accel_addr,0x2A), '#010b')[2:6]
        yh = format(self.i2c.read_byte_data(self.accel_addr,0x2B), '#010b')[2:]
        #Eje Z
        zl = format(self.i2c.read_byte_data(self.accel_addr,0x2C), '#010b')[2:6]
        zh = format(self.i2c.read_byte_data(self.accel_addr,0x2D), '#010b')[2:]
        ## Combinamos juntos los 2 bytes.
        self.accel_data[0] = int('0b' + xh[1:] + xl,2) - int(xh[0])*(2**(len(xh+xl)-1))  #Eje X  #Unimos los bytes en complemento a 2
        self.accel_data[1] = int('0b' + yh[1:] + yl,2) - int(yh[0])*(2**(len(yh+yl)-1))  #Eje Y  #Unimos los bytes en complemento a 2
        self.accel_data[2] = int('0b' + zh[1:] + zl,2) - int(zh[0])*(2**(len(zh+zl)-1))  #Eje Z  #Unimos los bytes en complemento a 2
        #Normalizamos el vector antes de retornarlo
        norma = np.linalg.norm(self.accel_data)
        self.accel_data = list(map(lambda x: x/norma,self.accel_data))
        return
        # return accel_data


    def magnRead(self):
        ##Sacamos los datos de campo magnetico de los 3 ejes
        #Eje X
        xh = self.i2c.read_byte_data(self.magn_addr,0x03)
        xl = self.i2c.read_byte_data(self.magn_addr,0x04)
        #Eje Y
        yh = self.i2c.read_byte_data(self.magn_addr,0x07)
        yl = self.i2c.read_byte_data(self.magn_addr,0x08)
        #Eje Z
        zh = self.i2c.read_byte_data(self.magn_addr,0x05)
        zl = self.i2c.read_byte_data(self.magn_addr,0x06)
        #Convertimos los resultados a binario para poder verlos
        xl = format(xl, '#010b')[2:]
        xh = format(xh, '#010b')[2:]
        yl = format(yl, '#010b')[2:]
        yh = format(yh, '#010b')[2:]
        zl = format(zl, '#010b')[2:]
        zh = format(zh, '#010b')[2:]
        #Y aplicamos el complemento a 2 para conseguir el numero
        self.magn_data[0] = int( xh[1:] + xl,2) - int(xh[0])*(2**(len(xh+xl)-1))
        self.magn_data[1] = int( yh[1:] + yl,2) - int(yh[0])*(2**(len(yh+yl)-1))
        self.magn_data[2] = int( zh[1:] + zl,2) - int(zh[0])*(2**(len(zh+zl)-1))
        #Le aplicamos los factores de calibracion a las lecturas
        self.magn_data[0] = (self.magn_data[0] - 35.0) * 1.0
        self.magn_data[1] = (self.magn_data[1] + 35.0) * 1.02702702703
        self.magn_data[2] = (self.magn_data[2] - 3.0) * 0.974358974359
        #Normalizamos el vector
        norma = np.linalg.norm(self.magn_data)
        self.magn_data = list(map(lambda x: x/norma,self.magn_data))
        return


    def gyroRead(self):
        global ahrs
        global gyro_addr
        #Eje X
        xh = self.i2c.read_byte_data(self.gyro_addr,0x29)
        xl = self.i2c.read_byte_data(self.gyro_addr,0x28)
        #Eje Y
        yh = self.i2c.read_byte_data(self.gyro_addr,0x2B)
        yl = self.i2c.read_byte_data(self.gyro_addr,0x2A)
        #Eje Z
        zh = self.i2c.read_byte_data(self.gyro_addr,0x2D)
        zl = self.i2c.read_byte_data(self.gyro_addr,0x2C)
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
        self.gyro_data[0] = float(x)*70/1000
        self.gyro_data[1] = float(y)*70/1000
        self.gyro_data[2] = float(z)*70/1000
        #Transformamos los datos a radianes/seg
        self.gyro_data = list(map(math.radians, self.gyro_data))
        return


    def filter(self, deltat):

        #Kind of OK, slightly primitive python implementation of Sebastian Madgwicks Filters


        # print "accel = {}".format(accel_datas)
        # print "magn = {}".format(magn_datas)
        # print "gyro = {}".format(gyro_datas)
        # print "deltat = {}".format(deltat)

        # print SEq
        # print b_x
        # print w_b
        # print beta


        #axulirary variables to avoid reapeated calcualtions
        halfSEq_1 = 0.5 * self.SEq[0]
        halfSEq_2 = 0.5 * self.SEq[1]
        halfSEq_3 = 0.5 * self.SEq[2]
        halfSEq_4 = 0.5 * self.SEq[3]
        twoSEq_1 = 2.0 * self.SEq[0]
        twoSEq_2 = 2.0 * self.SEq[1]
        twoSEq_3 = 2.0 * self.SEq[2]
        twoSEq_4 = 2.0 * self.SEq[3]
        twob_x = 2.0 * self.b_x
        twob_z = 2.0 * self.b_z
        twob_xSEq_1 = 2.0 * self.b_x * self.SEq[0]
        twob_xSEq_2 = 2.0 * self.b_x * self.SEq[1]
        twob_xSEq_3 = 2.0 * self.b_x * self.SEq[2]
        twob_xSEq_4 = 2.0 * self.b_x * self.SEq[3]
        twob_zSEq_1 = 2.0 * self.b_z * self.SEq[0]
        twob_zSEq_2 = 2.0 * self.b_z * self.SEq[1]
        twob_zSEq_3 = 2.0 * self.b_z * self.SEq[2]
        twob_zSEq_4 = 2.0 * self.b_z * self.SEq[3]
        SEq_1SEq_2 = self.SEq[0] * self.SEq[1]
        SEq_1SEq_3 = self.SEq[0] * self.SEq[2]
        SEq_1SEq_4 = self.SEq[0] * self.SEq[3]
        SEq_2SEq_3 = self.SEq[1] * self.SEq[2]
        SEq_2SEq_4 = self.SEq[1] * self.SEq[3]
        SEq_3SEq_4 = self.SEq[2] * self.SEq[3]
        twom_x = 2.0 * self.magn_data[0]
        twom_y = 2.0 * self.magn_data[1]
        twom_z = 2.0 * self.magn_data[2]


        # compute the objective function and Jacobian
        f_1 = twoSEq_2 * self.SEq[3] - twoSEq_1 * self.SEq[2] - self.accel_data[0]
        f_2 = twoSEq_1 * self.SEq[1] + twoSEq_3 * self.SEq[3] - self.accel_data[1]
        f_3 = 1.0 - twoSEq_2 * self.SEq[1] - twoSEq_3 * self.SEq[2] - self.accel_data[2]
        f_4 = twob_x * (0.5 - self.SEq[2] * self.SEq[2] - self.SEq[3] * self.SEq[3]) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - self.magn_data[0]
        f_5 = twob_x * (self.SEq[1] * self.SEq[2] - self.SEq[0] * self.SEq[3]) + twob_z * (self.SEq[0] * self.SEq[1] + self.SEq[2] * self.SEq[3]) - self.magn_data[1]
        f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5 - self.SEq[1] * self.SEq[1] - self.SEq[2] * self.SEq[2]) - self.magn_data[2]
        J_11or24 = twoSEq_3                                                    # J_11 negated in matrix multiplication
        J_12or23 = 2.0 * self.SEq[3]
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
        self.w_b[0] += w_err_x * deltat * self.zeta
        self.w_b[1] += w_err_y * deltat * self.zeta
        self.w_b[2] += w_err_z * deltat * self.zeta
        # print "w_b2: {}".format(w_b)
        self.gyro_data[0] -= self.w_b[0]
        self.gyro_data[1] -= self.w_b[1]
        self.gyro_data[2] -= self.w_b[2]

        #Copiamos el valor arreglado del gyroscopio para tenerlo para futuras referencias
        self.gyro_data_fixed = self.gyro_data[:]
    ###
        # compute the quaternion rate measured by gyroscopes
        SEqDot_omega_1 = -halfSEq_2 * self.gyro_data[0] - halfSEq_3 * self.gyro_data[1] - halfSEq_4 * self.gyro_data[2]
        SEqDot_omega_2 = halfSEq_1 * self.gyro_data[0] + halfSEq_3 * self.gyro_data[2] - halfSEq_4 * self.gyro_data[1]
        SEqDot_omega_3 = halfSEq_1 * self.gyro_data[1] - halfSEq_2 * self.gyro_data[2] + halfSEq_4 * self.gyro_data[0]
        SEqDot_omega_4 = halfSEq_1 * self.gyro_data[2] + halfSEq_2 * self.gyro_data[1] - halfSEq_3 * self.gyro_data[0]

        # compute then integrate the estimated quaternion rate
        self.SEq[0] += (SEqDot_omega_1 - (self.beta * SEqHatDot_1)) * deltat
        self.SEq[1] += (SEqDot_omega_2 - (self.beta * SEqHatDot_2)) * deltat
        self.SEq[2] += (SEqDot_omega_3 - (self.beta * SEqHatDot_3)) * deltat
        self.SEq[3] += (SEqDot_omega_4 - (self.beta * SEqHatDot_4)) * deltat

        # Normalizamos los quaterniones
        norm = np.linalg.norm(self.SEq)
        self.SEq = map(lambda x: x/norm,self.SEq)

        # compute flux in the earth frame
        SEq_1SEq_2 = self.SEq[0] * self.SEq[1]                                             # recompute axulirary variables
        SEq_1SEq_3 = self.SEq[0] * self.SEq[2]
        SEq_1SEq_4 = self.SEq[0] * self.SEq[3]
        SEq_3SEq_4 = self.SEq[2] * self.SEq[3]
        SEq_2SEq_3 = self.SEq[1] * self.SEq[2]
        SEq_2SEq_4 = self.SEq[1] * self.SEq[3]
        h_x = twom_x * (0.5 - self.SEq[2] * self.SEq[2] - self.SEq[3] * self.SEq[3]) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3)
        h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5 - self.SEq[1] * self.SEq[1] - self.SEq[3] * self.SEq[3]) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2)
        h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5 - self.SEq[1] * self.SEq[1] - self.SEq[2] * self.SEq[2])

        # normalise the flux vector to have only components in the x and z
        self.b_x = math.sqrt((h_x * h_x) + (h_y * h_y))
        self.b_z = h_z



    def quat2Euler(self,quater):

        #Transforma un quaternion a representacion de angulos de Euler, de la forma = [Pitch,Roll,Yaw]
        euler = [0,0,0]

        euler[0] = math.atan2(2*(quater[0]*quater[1] + quater[2]*quater[3]),quater[0]*quater[0] - quater[1]*quater[1] - quater[2]*quater[2] + quater[3]*quater[3])
        euler[1] = math.asin(-2*((quater[0]*quater[2] - quater[1]*quater[3]))/(quater[0]*quater[0] + quater[1]*quater[1] + quater[2]*quater[2] + quater[3]*quater[3]))
        euler[2] = math.atan2(2*(quater[1]*quater[2] + quater[0]*quater[3]),-quater[0]*quater[0] - quater[1]*quater[1] + quater[2]*quater[2] + quater[3]*quater[3])

        euler = map(math.degrees,euler)
        return euler


    def step(self):
        #Ejectuamos el filtro

        #Actualizamos los valores de los sensores
        self.accelRead()
        self.magnRead()
        self.gyroRead()
        #medimos tiempo transcurrido
        self.time_new = time.time()
        #corremos el filtro
        self.filter(self.time_new - self.time_old)
        #Actualizamos el tiempo
        self.time_old = self.time_new

    def getAttitude(self):
        #devuelve la estimacion actual del quaternion
        return self.SEq

    def getAttitudeEuler(self):
        #Devuelve los angulos de euler de la estimacion actual como [Pitch,Roll,Yaw]
        return self.quat2Euler(self.SEq)

    def getAngularVelocity(self):
        #Devuelve el vector en Rad/s de la velocidad angular compensada por el filtro de Madgwicks
        return self.gyro_data_fixed






def main():

    import time

    #Instanciamos la clase del filtro
    ahrs = MadgwicksFilter(init_quaternion = [0.5, 0.5, 0.5, 0.5])
    #We start the filter
    ahrs.begin()

    while(1):
        #We run one step of the filter
        ahrs.step()
        #We print the result
        attitude = ahrs.getAttitude()
        euler = ahrs.getAttitudeEuler()

        print("Pitch: {:+.2f}deg   Roll: {:+.2f}deg   Yaw: {:+.2f}deg    Quaternion:({:+.3f}, {:+.3f}, {:+.3f}, {:+.3f})".format(euler[0],euler[1],euler[2], attitude[0], attitude[1], attitude[2], attitude[3] ))

        #Esperamos un poco
        time.sleep(0.01)



if __name__ == '__main__':
    main()

