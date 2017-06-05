#! /user/bin/python

class Impedance(object):
    def __init__(self, massm, dampc, springk, deltat = 0.05):
        """
        initialize parameters

        :param massm, mass + symbol m
        :param dampc, damp + symbol c
        :param springk, spring + symbol k
        author: weiwei
        date: 20170412
        """

        self.__massm = massm
        self.__dampc = dampc
        self.__springk = springk
        self.__deltat = deltat

        # np indicates n previous, npp indicates previous previous
        self.__deltaxnp = 0
        self.__deltaxnpp = 0

        self.__mdt2, self.__cdt, self.__mck = self.__prepareequations()

    @property
    def m(self):
        return self.__massm

    @m.setter
    def m(self, value):
        self.__massm = value
        self.__mdt2 = float(self.__massm)/float(self.__deltat*self.__deltat)
        self.__mck  =  self.__mdt2 + self.__cdt + self.__springk

    @property
    def c(self):
        return self.__dampc

    @c.setter
    def c(self, value):
        self.__dampc = value
        self.__cdt = float(self.__dampc)/float(self.__deltat)
        self.__mck  =  self.__mdt2 + self.__cdt + self.__springk

    @property
    def k(self):
        return self.__springk

    @k.setter
    def k(self, value):
        self.__springk = value
        self.__mck  =  self.__mdt2 + self.__cdt + self.__springk

    def __prepareequations(self):
        """
        prepare some equations to avoid repeated computation
        I referred to  http://www.kochi-tech.ac.jp/library/ron/2004/2004mech/1052001.pdf
        when making this program

        :return: [m/dt^2, c/dt, m/dt^2+c/dt+k]
        """

        mdt2 =  float(self.__massm)/float(self.__deltat*self.__deltat)
        cdt =  float(self.__dampc)/float(self.__deltat)
        mck =  mdt2 + cdt + self.__springk

        return [mdt2, cdt, mck]

    def control(self, deltaf):
        """
        I referred to  http://www.kochi-tech.ac.jp/library/ron/2004/2004mech/1052001.pdf
        when making this program

        :param deltaf:
        :return:

        author: weiwei
        date: 20170412
        """

        mdt2deltax = self.__mdt2*(2*self.__deltaxnp-self.__deltaxnpp)
        cdtdeltax = self.__cdt*self.__deltaxnp

        deltax = float(deltaf + mdt2deltax + cdtdeltax)/float(self.__mck)
        # update previous values
        self.__deltaxnpp = self.__deltaxnp
        self.__deltaxnp = deltax

        return deltax

if __name__ == '__main__':

    from panda3d.core import *
    import robotsim.ur5dual.ur5dual as u5d
    import robotsim.ur5dual.ur5dualik as u5dik
    import robotsim.ur5dual.ur5dualplot as u5p
    import pandaplotutils.pandageom as pg
    import pandaplotutils.pandactrl as pc
    import numpy as np
    from manipulation.grip.robotiq85 import rtq85nm
    handpkg = rtq85nm

    base = pc.World(camp = [3000,0,3000], lookatp = [0,0,700])
    ur5dualrobot = u5d.Ur5Dual()
    ur5dualrobot.goinitpose()

    armid = 'lft'
    arrownp = []
    robotnp = []
    dfvalue = 30
    m = .1
    c = .1
    k = .1
    t = .05
    impx = Impedance(m, c, k, t)
    impy = Impedance(m, c, k, t)
    impz = Impedance(m, c, k, t)
    inittcppos, inittcprot = ur5dualrobot.gettcp(armid)

    def updateshow(ur5dualrobot, dfvalue, armid, robotnp, arrownp, task):

        for robot in robotnp:
            robot.detachNode()
        for arrow in arrownp:
            arrow.detachNode()

        ur5dualmnp = u5p.genUr5dualmnp(ur5dualrobot, handpkg)
        ur5dualmnp.reparentTo(base.render)
        robotnp.append(ur5dualmnp)

        if time.clock() > 2:
            dfvalue = 0

        tcpmat4 = ur5dualrobot.gettcpframe(armid)
        dfvec  = tcpmat4.getRow3(0)*dfvalue
        arrownp.append(pg.plotArrow(base.render, tcpmat4.getRow3(3), tcpmat4.getRow3(3)+dfvec*30, thickness = 10, rgba = [.7,.2,.2,1]))
        deltax = impx.control(dfvec[0])
        deltay = impy.control(dfvec[1])
        deltaz = impz.control(dfvec[2])
        print deltax, deltay, deltaz
        tgtpos = inittcppos + np.array([deltax, deltay, deltaz])
        tgtrot = inittcprot

        armjntsgoal = u5dik.numik(ur5dualrobot, tgtpos, tgtrot, armid)
        if armjntsgoal is not None:
            ur5dualrobot.movearmfk(armjntsgoal, armid)

        return task.again

    taskMgr.add(updateshow, 'updateshow', extraArgs = [ur5dualrobot, dfvalue, armid, robotnp, arrownp], appendTask = True)
    # taskMgr.doMethodLater(10, updateshow, 'updateshow', extraArgs = [ur5dualrobot, dfvalue, armid, robotnp, arrownp], appendTask = True)

    import time
    tic = time.clock()
    base.run()