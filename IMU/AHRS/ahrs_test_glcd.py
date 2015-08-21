import ahrs
import time
import ssd1351


#Instanciamos la clase del filtro
ahrs = ahrs.MadgwicksFilter(init_quaternion = [0.5, 0.5, 0.5, 0.5])
#We start the filter
ahrs.begin()

#We instantiate the display
oled = ssd1351.SSD1351()
oled.begin()

#we try to stabilize the sistem
print"estabilizando"
for i in xrange(5000): ahrs.step()
print"5000 pasos listos"

# Imprimimos algunas cosas iniciales
oled.setCursor(4,4)
oled.write("Pitch: ")
oled.setCursor(4,6)
oled.write("Roll:  ")
oled.setCursor(4,8)
oled.write("Yaw:   ")


while(1):
    #We run one step of the filter
    ahrs.step()
    #We print the result
    attitude = ahrs.getAttitude()
    euler = ahrs.getAttitudeEuler()

    print("Pitch: {:+.2f}deg   Roll: {:+.2f}deg   Yaw: {:+.2f}deg    Quaternion:({:+.3f}, {:+.3f}, {:+.3f}, {:+.3f})".format(euler[0],euler[1],euler[2], attitude[0], attitude[1], attitude[2], attitude[3] ))

    #Display euler angles
    oled.setCursor(11,4)
    oled.write("{:+.2f}deg  ".format(euler[0]))

    oled.setCursor(11,6)
    oled.write("{:+.2f}deg  ".format(euler[1]))


    oled.setCursor(11,8)
    oled.write("{:+.2f}deg  ".format(euler[2]))

    #Display quaternion
    oled.setCursor(0,12)
    oled.write("Quat:\n[{:+.2f}, {:+.2f}, {:+.2f}, {:+.2f}]".format(attitude[0], attitude[1], attitude[2], attitude[3] ))


    #Esperamos un poco
    # time.sleep(0.01)
