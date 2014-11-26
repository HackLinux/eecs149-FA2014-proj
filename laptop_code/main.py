import os
import sys
import time
import math
import bluetooth
import threading
import socket
import struct

class Cstruct(object):
    pass

class ARDrone(object):
    """A class for controlling the ARDrone 1.0 over wifi"""
    def __init__(self):
        super(ARDrone, self).__init__()

        self.dataOut = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dataIn = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.droneIP = "192.168.1.1"
        self.dronePort = 5556

    def CalibrateHorizontal(self):
        self.dataOut.sendto("AT*FTRIM=1\r", (self.droneIP, self.dronePort))

    def Takeoff(self):
        inputval = 0
        inputval |= 1<<9 #Take off bit
        inputval |= 0b1010101<< 18  #Because the drone demands it
        inputval |= 1<<28 #Drone demands this as well

        self.dataOut.sendto("AT*REF=1,%s\r" % str(inputval), (self.droneIP, self.dronePort))

    def Land(self):
        inputval = 0
        inputval |= 0b1010101<< 18  #Because the drone demands it
        inputval |= 1<<28 #Drone demands this as well

        self.dataOut.sendto("AT*REF=1,%s\r" % str(inputval), (self.droneIP, self.dronePort))

    def LEDTest(self):
        self.dataOut.sendto("AT*LED=1,0,1056964608,4\r", (self.droneIP, self.dronePort))

    def ResetEmergency(self):
        inputval = 0
        inputval |= 1<<8
        inputval |= 0b1010101<< 18  #Because the drone demands it
        inputval |= 1<<28 #Drone demands this as well

        self.dataOut.sendto("AT*REF=1,%s\r" % str(inputval), (self.droneIP, self.dronePort))

        

class Wiimote(object):
    """A class for controlling the wiimote asynchronously.
    Supports the accelerometer and button features only.

    This must use the older style of wiimotes:
    Nintendo RVL-CNT-01"""
    def __init__(self, address, xbias=0, ybias=0, zbias=0, xsensitivity=1.0, ysensitivity=1.0, zsensitivity=1.0):
        super(Wiimote, self).__init__()
        self.address = address
        self.connected = False
        self.pitch = 0
        self.roll = 0
        self.adc = Cstruct()
        self.adc.x = 0
        self.adc.y = 0
        self.adc.z = 0
        self.buttons = Cstruct()
        self.buttons.one = False
        self.buttons.two = False
        self.buttons.left = False
        self.buttons.right = False
        self.buttons.up = False
        self.buttons.down = False
        self.buttons.b = False
        self.buttons.a = False
        self.buttons.plus = False
        self.buttons.minus = False
        self.buttons.home = False
        

        self.xbias = xbias
        self.ybias = ybias
        self.zbias = zbias
        self.xsensitivity = xsensitivity
        self.ysensitivity = ysensitivity
        self.zsensitivity = zsensitivity

        self.dataInStream = None
        self.dataOutStream = None

        self.exitRequested = False

        self.receiveTimeout = 0
        self.responseTimeout = 1

    def connect(self):
        self.dataInStream = bluetooth.BluetoothSocket(bluetooth.L2CAP)
        self.dataOutStream = bluetooth.BluetoothSocket(bluetooth.L2CAP)

        print "Connecting to address: " + self.address
        self.dataInStream.connect((self.address, 0x13))
        self.dataOutStream.connect((self.address, 0x11))

        if self.dataInStream and self.dataOutStream:
            print "Connected!"

        self.connected = True

        # self.dataOutStream.send('\x52\x11\x05')
        # self.dataOutStream.send('\x52\x12\x00\x31')
        self.dataOutStream.send('\x52\x11\x60')
        self.dataOutStream.send('\x52\x11\x60')
        self.dataOutStream.send('\x52\x11\x60')

        self.dataOutStream.send('\x52\x12\x00\x31')

        self.dataInStream.settimeout(self.receiveTimeout)
        self.dataOutStream.settimeout(self.receiveTimeout)

        self.runThread = threading.Thread(target=self.ioLoop)
        self.runThread.start()

        


    def ioLoop(self):
        self.lastGoodDataTimestamp = time.time()
        buff = ''

        while self.connected and not self.exitRequested:
            try:
                buff = self.dataInStream.recv(10)
            except bluetooth.BluetoothError:
                pass

            if buff[:2] == '\xa1\x31':

                self.lastGoodDataTimestamp = time.time()
                # print "Read z value of: " + str(ord(buff[6]))
                # print "Z bias of: " + str(self.zbias)
                # print "Zsensitivity of: " + str(self.zsensitivity)
                self.adc.x = (ord(buff[4]) - self.xbias)/self.xsensitivity
                self.adc.y = (ord(buff[5]) - self.ybias)/self.ysensitivity
                self.adc.z = (ord(buff[6]) - self.zbias)/self.zsensitivity
                # print "Calculated Z: " + str(self.adc.z)
                self.buttons.left = ord(buff[2]) & 0x01
                self.buttons.right = ord(buff[2]) & 0x02
                self.buttons.down = ord(buff[2]) & 0x04
                self.buttons.up = ord(buff[2]) & 0x08
                self.buttons.plus = ord(buff[2]) & 0x10

                self.buttons.two = ord(buff[3]) & 0x01
                self.buttons.one = ord(buff[3]) & 0x02
                self.buttons.b = ord(buff[3]) & 0x04
                self.buttons.a = ord(buff[3]) & 0x08
                self.buttons.minus = ord(buff[3]) & 0x10
                self.buttons.home = ord(buff[3]) & 0x80

            elif self.responseTimeout < (time.time() - self.lastGoodDataTimestamp):
                self.connected = False
                self.dataOutStream.send('\x52\x12\x00\x31')

        self.onDisconnect()

    def onDisconnect(self):
        self.dataInStream.close()
        self.dataOutStream.close()


targetAddress = "00:5A:19:40:10:20"
calibFileName = ''.join(targetAddress.split(':')) + '.calib'
calibData = []

controller = Wiimote(targetAddress)
controller.connect()
time.sleep(1.5)
if not controller.connected:
    print "Controller ignored data requests, exiting"
    exit()

controller.exitRequested = True
exit()

if os.path.isfile(calibFileName):
    calibData = struct.unpack('hhhhhh', open(calibFileName, 'rb').read())
    if len(calibData) == 6:
        sys.stdout.write("A calibration file was found for this wiimote, would you like to use these values? (y/n):  ")
        decision = raw_input()
        if decision[0] != 'y':
            calibData = []
    else:
        calibData = []


if not calibData:
    sys.stdout.write("This wiimote has not been calibrated.  Would you like to calibrate it now? (y/n):  ")
    decision = raw_input()
    if decision[0] == 'y':
        sys.stdout.write("Stand the wiimote on it's end on a level surface, camera facing down.  Then press enter  ")
        raw_input()
        yM1 = controller.adc.y
        x01 = controller.adc.x
        z01 = controller.adc.z
        sys.stdout.write("Lay the wiimote on it's side on a level surface, plus-button-side down.  Then press enter")
        raw_input()
        y01 = controller.adc.y
        xM1 = controller.adc.x
        sys.stdout.write("Lay the wiimote on it's back on a level surface, battery-side down.  Then press enter")
        raw_input()
        zM1 = controller.adc.z

        xsensitivity = x01-xM1
        xbias = x01
        ysensitivity = y01-yM1
        ybias = y01
        zsensitivity = z01-zM1
        zbias = z01

        calibData = (xbias, xsensitivity, ybias, ysensitivity, zbias, zsensitivity)

        open(calibFileName, 'wb').write(struct.pack('hhhhhh', *calibData))
    else:
        calibData = [0, 1, 0, 1, 0, 1]

controller.xbias = calibData[0]
controller.ybias = calibData[2]
controller.zbias = calibData[4]
controller.xsensitivity = float(calibData[1])
controller.ysensitivity = float(calibData[3])
controller.zsensitivity = float(calibData[5])


drone = ARDrone()

while 1:
    if controller.buttons.home:
        drone.ResetEmergency()
        time.sleep(2)
    elif controller.buttons.plus:
        drone.Takeoff()
    elif controller.buttons.minus:
        drone.Land()
    elif controller.buttons.a:
        drone.CalibrateHorizontal()
    time.sleep(.05)
