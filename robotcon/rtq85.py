# !/usr/bin/env python
import math
import time
import socket
import numpy as np
from threading import _Timer

class RepeatingTimer(_Timer):
    """
    hearbeat function to maintain modbus tcp connection
    author: weiwei
    date: 20170310
    """

    def run(self):
        while not self.finished.is_set():
            self.function(*self.args, **self.kwargs)
            self.finished.wait(self.interval)

class Rtq85Socket():
    """

    author: weiwei
    date: 20160310
    """

    def __init__(self, handname = 'rgt'):
        ip = '10.2.0.21'
        if handname == 'lft':
            ip = '10.2.0.22'
        port = 502
        self.__buffersize = 2048
        self.__s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__s.connect((ip, port))
        self.__connected = False
        self.__heartbeater =None

    @property
    def consts(self):
        # read-only property
        return self.__connected

    def initialize(self):
        if not self.__connected:
            inithexstrlist = ['33', '9A', '00', '00', '00', '0D', '02', '10', '00', '00', '00', '03', \
                              '06', '01', '00', '00', '00', '00', '00']
            initrsphexstrlist = ['33', '9A', '00', '00', '00', '06', '02', '10', '00', '00', '00', '03']
            initchars = self.__hex2chars(inithexstrlist)
            initrspchars = self.__hex2chars(initrsphexstrlist)

            waithexstrlist = ['45', '33', '00', '00', '00', '06', '02', '04', '00', '00', '00', '01']
            waitrspcharsyes = ['45', '33', '00', '00', '00', '05', '02', '04', '02', '31', '00']
            waitchars = self.__hex2chars(waithexstrlist)
            waitrspchars = self.__hex2chars(waitrspcharsyes)
            print self.consts
            if self.__send(initchars, initrspchars):
                self.__connected = self.__send(waitchars, waitrspchars)
            if self.__connected:
                self.__heartbeater = RepeatingTimer(20.0, self.__hbtick)
                self.__heartbeater.start()
                print "Hand is initialized!"
            else:
                print "Failed to initialize hand!"
        else:
            print "Hand already initialized!"

    def gooff(self):
        if self.__connected:
            self.__connected = self.openhandto(85)
            print "Hand is back to open!"
        else:
            print "No initialized hand found!"

    def openhandto(self, position):
        """
        # uses modbus tcp; NOT RTU

        :param position: 0~85
        :return:
        """

        if position > 85 or position < 0:
            assert "Position out of range!"

        closetohexstrlist = ['34', 'AB', '00', '00', '00', '0D', '02', '10', '00', '00', '00', '03', '06', \
                        '09', '00', '00', 'FF', 'FF', '00']
        posscaled = int(math.floor((85.0-position)/85.0*255))
        print self.__unit82hexstr(posscaled)
        closetohexstrlist[16] = self.__unit82hexstr(posscaled)
        closetorsphexstrlist = ['34', 'AB', '00', '00', '00', '06', '02', '10', '00', '00', '00', '03']
        closetochars = self.__hex2chars(closetohexstrlist)
        closetorspchars = self.__hex2chars(closetorsphexstrlist)

        waithexstrlist = ['D6', '05', '00', '00', '00', '06', '02', '04', '00', '00', '00', '03']
        waitrspcharsyes = ['D6', '05', '00', '00', '00', '09', '02', '04', '06', 'F9', '00', '00']
        waitrspcharsyesobs = ['D6', '05', '00', '00', '00', '09', '02', '04', '06', 'B9', '00', '00']
        waitchars = self.__hex2chars(waithexstrlist)
        waitrspchars = self.__hex2chars(waitrspcharsyes)
        waitrspobshars = self.__hex2chars(waitrspcharsyesobs)
        if self.__send(closetochars, closetorspchars):
            if self.__send(waitchars, waitrspchars, waitrspobshars):
                print "Closed rtq85 to " + str(position)
            else:
                print "Failed to close rtq85 to "+str(position)

    def __send(self, message, confirmmessage, confirmmessageobs = None):
        """
        confirm pairs:
        connect connected!
        disconnect disconnected!
        goinitial wentinitial!

        :param message:
        :param confirmmessage:
        :param confirmmessageobs: the confirm message when obstacle exists
        :return:
        author: weiwei
        date: 20170309
        """

        data = ''
        datalength = len(confirmmessage)
        if confirmmessageobs is not None:
            datalengthobs = len(confirmmessageobs)
            while (data[0:datalength] != confirmmessage[0:datalength] and \
                               data[0:datalengthobs] != confirmmessageobs[0:datalengthobs]):
                self.__s.send(message)
                data = self.__s.recv(self.__buffersize)
                time.sleep(0.01)
                print "obs"
                print self.__str2hex(data)
                print self.__str2hex(confirmmessage)
            return True
        else:
            while data[0:datalength] != confirmmessage[0:datalength]:
                self.__s.send(message)
                data = self.__s.recv(self.__buffersize)
                time.sleep(0.01)
                print self.__str2hex(data)
                print self.__str2hex(confirmmessage)
            return True

    def __hex2chars(self, hexstrlist):
        """
        convert a list of hex string into unit8

        :param hexstrlist: ['xx', 'xx', ...]
        :return:

         author: weiwei
         date: 20170310
        """

        formatedhexstr = ''.join(hexstrlist)
        return formatedhexstr.decode('hex')

    def __str2hex(self, string):
        """
        convert each character in a string to hex

        :param hexstrlist: ['xx', 'xx', ...]
        :return:

         author: weiwei
         date: 20170310
        """

        return [x.encode('hex') for x in string]

    def __unit82hexstr(self, value):
        """
        convert one unit8 vlaue into a hexstr

        :param 15 -> 0f
        :return:

         author: weiwei
         date: 20170310
        """
        if value < 0 or value > 255:
            assert "Value is smaller than 0x00 or larger than 0xff!"
        strvalue = hex(value)[-2:]
        if strvalue[0] == 'x':
            strvalue = '0'+strvalue[1]
        return strvalue

    def __hbtick(self):
        """
        hearbeat function to maintain modbus tcp connection
        author: weiwei
        date: 20170310
        """

        readirhexstrlist = ['01', '00', '00', '00', '00', '06', '02', '04', '00', '00', '00', '06']
        readirrspcharsyes = ['01', '00', '00', '00', '00', '0f', '02', '04', '0C']
        readischars = self.__hex2chars(readirhexstrlist)
        readisrspchars = self.__hex2chars(readirrspcharsyes)
        self.__send(readischars, readisrspchars)

    def __del__(self):
        if self.__heartbeater is not None:
            self.__heartbeater.cancel()
        self.__s.close()

if __name__ == '__main__':
    rtq85s = Rtq85Socket(handname = 'lft')

    import time
    tic = time.clock()
    rtq85s.initialize()
    toc = time.clock()
    print toc-tic
    rtq85s.openhandto(20)