import socket
import fcntl
import struct
import nmap

def get_ip_address(ifname):
    """ (Python 3) Funcion que recibe un string con el nombre de una interfaz de red y devuelve
        un string con la direccion IP de la interfaz, o None si dicha interfaz no
        tiene direccion IP asignada.

            get_ip_address('wlp2s0')
            '192.168.1.4'
                                       """
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', bytes(ifname,'ASCII')[:15])
        )[20:24])
    except:
        return None



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


# abrir archivo /proc/net/dev
# POP 2 veces del primer valor
# ifaces = [i[:i.find(':')] for i in archivo]

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


def get_network_config():
    """ (Python 3) Funcion que devuelve un diccionario  con las interfaces de red de la computadora y sus respectivas direcciones
        ip. """
    interfaces = get_interfaces()
    ips = [get_ip_address(ip) for ip in interfaces]
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


#LImpiar el diccionario, de valores vacios
#
# dic_limpio = {lab:ip for lab,ip in dictionary.items() if ip != None}



############################################################################################################################################




nm = nmap.PortScanner()
nm.scan(hosts='192.168.1.192/24',arguments='-sn')
nm.all_hosts()
# Out[19]:
# [u'192.168.1.1',
#  u'192.168.1.141',
#  u'192.168.1.173',
#  u'192.168.1.189',
#  u'192.168.1.22',
#  u'192.168.1.246',
#  u'192.168.1.4',
#  u'192.168.1.54',
#  u'192.168.1.61']
hola = [nm[i].hostname() for i in nm.all_hosts()]
# ['',
#  u'EPSON0BE763',
#  u'android-2b4df80d01eb5edb',
#  u'raspberrypi',
#  u'iPad-de-Roselis',
#  u'android-108fdef8f9f59d5d',
#  u'linux-xexk',
#  '',
#  u'iPhonedeGerardo']
