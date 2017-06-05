#! /user/bin/python
import math
from scipy.optimize import fsolve

class Sgllinkeuler(object):
    def __init__(self, linklen, linkm, dampc, deltat = 0.05):
        """
        initialize parameters

        :param linklen, length of link
        :param linkm, mass of link
        :param dampc, motor damping to the link yields
        author: weiwei
        date: 20170412
        """

        self.__g = 9810
        self.__mgl = linkm*self.__g*linklen/2.0
        self.__dampc = dampc
        self.__linkI = 1.0/12.0*linkm*linklen*linklen
        self.__deltat = deltat

        # np indicates n previous, npp indicates previous previous
        self.__thnp = 0
        self.__thnpp = 0

    def control(self, tau):
        """
        I referred to robot modeling and control
        when making this program

        :param deltaf:
        :return:

        author: weiwei
        date: 20170417
        """

        func = lambda th: self.__linkI*(th-self.__thnp-self.__thnp+self.__thnpp)/(self.__deltat*self.__deltat)+\
            self.__dampc*(th-self.__thnp)/self.__deltat+self.__mgl*math.sin(th)-tau
        # initial guess
        th_ig = 1
        th_solution = fsolve(func, th_ig)[0]
        # update previous values
        self.__thnpp = self.__thnp
        self.__thnp = th_solution

        return th_solution

if __name__ == '__main__':

    from panda3d.core import *
    import pandaplotutils.pandageom as pg
    import pandaplotutils.pandactrl as pc
    import numpy as np

    base = pc.World(camp = [200,0,200], lookatp = [0,0,0])

    linklen = 40.0
    linkm = .1
    dampc = 10000
    deltat = 0.01
    sgllink = Sgllinkeuler(linklen, linkm, dampc, deltat)
    linknp = []

    def updateshow(linknp, task):

        for link in linknp:
            link.detachNode()

        th = sgllink.control(10000)
        print th, math.degrees(th)
        arrownp = pg.plotArrow(base.render, spos = np.array([0,0,0]), epos = np.array([0, math.cos(th), math.sin(th)])*linklen)
        linknp.append(arrownp)
        # if time.clock() > 2:
        #     dfvalue = 0
        return task.again

    taskMgr.add(updateshow, 'updateshow', extraArgs = [linknp], appendTask = True)
    # taskMgr.doMethodLater(10, updateshow, 'updateshow', extraArgs = [ur5dualrobot, dfvalue, armid, robotnp, arrownp], appendTask = True)

    import time
    tic = time.clock()
    base.run()