#!/usr/bin/python

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import socket
import struct
import fcntl
import nmap



##########################################################################################################################
###                                           Funciones de Red                                                         ###
##########################################################################################################################


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

def get_network_config2():
    """ (Python 2) Funcion que devuelve un diccionario  con las interfaces de red de la computadora y sus respectivas direcciones
        ip. """
    interfaces = get_interfaces()
    ips = [get_ip_address2(ip) for ip in interfaces]
    return dict(zip(interfaces,ips))


def scan_ip(direccion):
    """ (Python 2 y 3) Funcion que recibe una direccion IP y luego usa NMAP para escanear dicha red y devuelve un diccionario con
    las ip resultantes y su labels (values y keys respectivamente).
    """
    nm = nmap.PortScanner()
    nm.scan(hosts=direccion+'/24',arguments='-sn')
    ipaddr = nm.all_hosts()
    labels = [nm[i].hostname() for i in nm.all_hosts()]
    scanned = dict(zip(labels,ipaddr))
    return scanned


##########################################################################################################################
###                                         Codigo                                                                     ###
##########################################################################################################################



window = 0
pitch = 0.0
roll = 0.0
yaw = 0.0

LightAmbient = 0.2, 0.2, 0.2, 1.0
LightDiffuse = 0.5, 0.5, 0.5, 1.0
LightPosition = 1.0, 0.0, 2.0, 1.0

#Intentamos ubicar la direccion del Raspberry PI
servidor = 'raspberrypi'

#Analizamos las interfaces de red actuales
inter_faces = get_network_config2()
inter_faces = {lab:ip for lab,ip in inter_faces.items() if ip != None}  #Limpiamos las interfaces que no estan conectadas
#Buscamos al raspberry pi en las redes actuales
if 'enp3s0' in inter_faces:
    host = scan_ip(inter_faces['enp3s0']).values()[0]
    print("Encontrado {} en {}, con direccion {}".format(servidor,'enp3s0',host))
else:
    for iface in inter_faces:                                                          #Revisamos todas las interzaces que no tienen una conexion
        red = scan_ip(inter_faces[iface])                                                   #Escaneamos cada interfaz para ver otras conexiones en LAN
        if servidor in red:                                                                 #Si el label del servidor que estamos buscando se encuentra en el resultado del escaneo
            host = red[servidor]                                                            #Devolvemos dicha direccion
            print("Encontrado {} en {}, con direccion {}".format(servidor,iface,host))      #Avisamos el resultado
            break                                                                           #Terminamos el loop de escaneos


#Intentamos conectar
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#host = "10.42.0.178"
port = 23322
s.connect((host,port))
print("Ya se obtuvo una conexion")


def read_line(sock):
    mensaje = ''
    while('\n' not in mensaje):
        mensaje += sock.recv(1)
    return mensaje[:-1]


def InitGL(Width, Height):              # We call this right after our OpenGL window is created.
    global LightAmbient, LightDiffuse, LightPosition
    glClearColor(0.0, 0.0, 0.0, 0.0)    # This Will Clear The Background Color To Black
    glClearDepth(1.0)                   # Enables Clearing Of The Depth Buffer
    glDepthFunc(GL_LESS)                # The Type Of Depth Test To Do
    glEnable(GL_DEPTH_TEST)             # Enables Depth Testing
    glShadeModel(GL_SMOOTH)             # Enables Smooth Color Shading
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()                    # Reset The Projection Matrix
                                        # Calculate The Aspect Ratio Of The Window
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)

    glMatrixMode(GL_MODELVIEW)

    glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient)
    glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse)
    glLightfv(GL_LIGHT1, GL_POSITION,LightPosition)
    glEnable(GL_LIGHT1)
    glEnable(GL_LIGHTING)
    glEnable(GL_COLOR_MATERIAL)

def ReSizeGLScene(Width, Height):
    if Height == 0:
        Height = 1

    glViewport(0, 0, Width, Height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

def DrawGLScene():
    global pitch, roll, yaw, s
    line = read_line(s)
    numeros = [float(i) for i in line.split(',')]

    yaw = float(numeros[2])
    pitch = float(numeros[0])
    roll = float(numeros[1])

    print("Pitch: {:+.2f}deg   Roll: {:+.2f}deg   Yaw: {:+.2f}deg".format(pitch,roll,yaw))

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glLoadIdentity()                    # Reset The View
    glTranslatef(0.0, 0.5, -6.0)
    glPushMatrix()
    glRotatef(yaw, 0.0, 1.0, 0.0)
    glRotatef(pitch, 0.0, 0.0, 1.0)
    glRotatef(roll, 1.0, 0.0, 0.0)

    # glRotatef(yaw, 0.0, 1.0, 0.0)
    # glRotatef(pitch, 1.0, 0.0, 0.0)
    # glRotatef(roll, 0.0, 0.0, 1.0)
    glColor3f(0.0, 0.8, 0.9)
    glutSolidTeapot(1.0)
    glPopMatrix()


    glutSwapBuffers()

def main():
    global window
    glutInit("")
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
    glutInitWindowSize(640, 480)
    glutInitWindowPosition(0, 0)
    window = glutCreateWindow("AHRS Tester")

    glutDisplayFunc (DrawGLScene)
    #glutFullScreen()
    glutIdleFunc(DrawGLScene)
    glutReshapeFunc (ReSizeGLScene)
    InitGL(640, 480)
    glutMainLoop()

print "Hit ESC key to quit."
main()
