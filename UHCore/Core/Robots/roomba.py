#!/usr/bin/env python
import socket
import httplib
from time import sleep

from robot import Robot

# Roomba constants
COMMANDS = {
    'SAFE': 131,
    'FULL' : 132,
    'POWER' : 133,
    'SPOT' : 134,
    'CLEAN': 135,
    'MAX' : 136,
    'DRIVE' : 137,
    'MOTORS' : 138,
    'LEDS': 139,
    'SONG' : 140,
    'PLAY' : 141,
    'SENSORS' : 142,
    'DOCK' : 143,
}

# Sensor packets
SENSORS_0 = (chr(0), 26)
SENSORS_1 = (chr(0), 10)
SENSORS_2 = (chr(0), 6)
SENSORS_3 = (chr(0), 10)

# LED test sequence
#list = buffer(139, 25, 0, 128) #buffer takes 3 arguments at most
# motor buffer
#motorlist = buffer(138, 2)

# Drive constants.
RADIUS_TURN_IN_PLACE_CW = -1
RADIUS_TURN_IN_PLACE_CCW = 1
RADIUS_STRAIGHT = 32768
RADIUS_MAX = 2000

VELOCITY_MAX = 500  # mm/s
VELOCITY_SLOW = int(VELOCITY_MAX * 0.33)
VELOCITY_FAST = int(VELOCITY_MAX * 0.66)
VELOCITY_MIN = 25  # mm/s

WHEEL_SEPARATION = 298  # mm
WHEEL_SEPARATION_CREATE2 = 235  # mm (I'm measuring ~233/234)

SERIAL_TIMEOUT = 2  # Number of seconds to wait for reads. 2 is generous.
START_DELAY = 5  # Time it takes the Roomba/Create to boot.

class Roomba(Robot):
    """ Concrete implementation of the Robot interface for Roomba robots """

    def __init__(self, name, hostname):
        print "Roomba: %s" % hostname
        parts = hostname.split('://')
        if len(parts) > 1:
            proto = parts[0]
            hostname = parts[1]
        else:
            proto = 'http'
        if ':' in hostname:
            self._hostname, self._port = hostname.split(':')
        else:
            self._hostname = hostname
            self._port = 9001
        super(Roomba, self).__init__(name, SocketInterface)
        self._robIntInstance = SocketInterface(self._hostname, self._port)

    @property
    def _robInt(self):
        """Concrete implementation of the robot interface"""
        if self._robIntInstance == None:
            self._robIntInstance = self._robIntClass(self._hostname, self._port)
            
        return self._robIntInstance

class SocketInterface(object):
    def __init__(self, url, port):
        self._initialized = False
        self._hostname = url
        self._port = int(port)

    def __del__(self):
        #self._conn.close()
        pass

    def runFunction(self, funcName, kwargs):
        return 5
        
    def initComponent(self, name=None):
        print "Waking up roomba"
        #if not self._initialized:
        httpConnection = httplib.HTTPConnection(self._hostname)
        httpConnection.request('GET', '/rwr.cgi?exec=1')
        # Get the response
        resp = httpConnection.getresponse()
        body = resp.read()
        # status 200 == OK
        if resp.status != 200:
            # something went wrong
            raise Exception("Error calling action")
        self._initialized = True
        httpConnection.close()

    def getConnection(self):
        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        conn.connect((self._hostname, self._port))
        conn.settimeout(0.25)
        return conn

    def powerON(self, conn):
        conn.send(chr(142) + chr(5))
        try:
            mode = conn.recv(1)
            return True
        except:
            print "Waking up roomba"
            conn.send(chr(COMMANDS['POWER']))
            conn.send(chr(COMMANDS['SAFE']))
            return False
    
    def runComponent(self, name, value, mode=None, blocking=True):
        cmd = COMMANDS.get(name.upper(), None)
        if name is None:
            print "Invalid command: %s" % name
            return 5
        try:
            conn = self.getConnection()
            if not self.powerON(conn):
                # if power wasn't on, we need to close the connection
                # then wait for it to boot before sending the command
                conn.close()
                sleep(2)
                conn = self.getConnection()
            conn.send(chr(cmd))
            conn.close()
            return 3
        except Exception as e:
            print "RombaRC: %s" % e
            return 5

if __name__ == '__main__':
    print("Hello roowifi")
    robot = Roomba("10.0.0.1:9001")

