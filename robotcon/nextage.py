# !/usr/bin/env python
import socket

class NxtSocket():
    """

    author: weiwei
    date: 20160309
    """

    def __init__(self):
        ip = '10.2.0.20'
        port = 50305
        self.__buffersize = 2048
        self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__s.connect((ip, port))
        self.__connected = False

    @property
    def consts(self):
        # read-only property
        return self.__connected

    def initialize(self):
        if not  self.__connected:
            self.__connected = self.__send('connect', 'connected!')
            if self.__connected:
                print "Robot is initialized!"
            else:
                print "Failed to initialize robot!"
        else:
            print "Robot already initialized!"

    def gooff(self):
        if self.__connected:
            self.__connected = self.__send('disconnect', 'disconnected!')
            print "Robot is back to off!"
        else:
            print "No initialized robot found!"

    def servoon(self):
        self.__send('servoon', 'servoison!')

    def movejnts15(self, nxjnts):
        """
        move all joints of the nextage robo

        :param nxjnts: the definition as robotsim/nextage/NxtRobot.__initjnts -> nparray 15
        :return: bool

        author: weiwei
        date: 20170309
        """

        bodyyaw = str(nxjnts[0])
        headyaw = str(nxjnts[1])
        headpitch = str(nxjnts[2])
        rgt1 = str(nxjnts[3])
        rgt2 = str(nxjnts[4])
        rgt3 = str(nxjnts[5])
        rgt4 = str(nxjnts[6])
        rgt5 = str(nxjnts[7])
        rgt6 = str(nxjnts[8])
        lft1 = str(nxjnts[9])
        lft2 = str(nxjnts[10])
        lft3 = str(nxjnts[11])
        lft4 = str(nxjnts[12])
        lft5 = str(nxjnts[13])
        lft6 = str(nxjnts[14])
        motiontime = str(3.0)
        sep = ','

        message = 'movejoints15' + sep + bodyyaw + sep + headyaw + sep + headpitch + sep + \
                  rgt1 + sep + rgt2 + sep + rgt3 + sep + rgt4 + sep + rgt5 + sep + rgt6 + sep \
                  + lft1 + sep + lft2 + sep + lft3 + sep + lft4 + sep + lft5 + sep + lft6 + sep \
                  + motiontime
        self.__send(message, 'joints15moved!')

    def __send(self, message, confirmmessage):
        """
        confirm pairs:
        connect connected!
        disconnect disconnected!
        goinitial wentinitial!

        :param message:
        :param confirmmessage:
        :return:
        author: weiwei
        date: 20170309
        """

        self.__s.send(message)
        data = self.__s.recv(self.__buffersize)
        if data == confirmmessage:
            return True
        else:
            return False

    def __del__(self):
        self.__s.close()

if __name__ == '__main__':
    nxts = NxtSocket()

    import time
    import numpy as np
    tic = time.clock()
    nxts.initialize()
    toc = time.clock()
    print toc-tic
    nxts.movejnts15(np.array([10,0,0,-15,0,-143,0,0,0,15,0,-143,0,0,0,0]))
    nxts.gooff()