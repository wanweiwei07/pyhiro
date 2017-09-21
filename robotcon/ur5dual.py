# !/usr/bin/env python

import math
from utils import robotmath as rm
import numpy as np
import logging
from urx import Robot

class Ur5DualUrx():
    """
    urx 50, right arm 51, left arm 52

    author: weiwei
    date: 201670411
    """

    def __init__(self):
        iprgt = '10.2.0.50'
        iplft = '10.2.0.51'
        logging.basicConfig()
        self.__rgtarm = None
        # self.__rgtarm = Robot(iprgt)
        # self.__rgtarm.set_tcp((0, 0, 0, 0, 0, 0))
        # self.__rgtarm.set_payload(1.28)
        self.__lftarm = Robot(iplft, use_rt = True)
        self.__lftarm.set_tcp((0, 0, 0, 0, 0, 0))
        self.__lftarm.set_payload(.86)

        self.__lftarmbase = [0, 235.00, 965.00]
        self.__rgtarmbase = [0, -235.00, 965.00]
        self.__sqrt2o2 = math.sqrt(2.0)/2.0

    @property
    def rgtarm(self):
        # read-only property
        return self.__rgtarm

    def movejntssgl(self, joints, armid='rgt'):
        """

        :param joints: a 1-by-6 vector in degree
        :param armid:
        :return:

        author: weiwei
        date: 20170411
        """

        targetarm = self.__rgtarm
        if armid == 'lft':
            targetarm = self.__lftarm

        jointsrad = [math.radians(angdeg) for angdeg in joints]
        targetarm.movej(jointsrad, acc = 0.2, vel = 0.2, wait = True)

    def movejntsall(self, joints):
        """
        move all joints of the ur5 dual-arm robot
        NOTE that the two arms are moved sequentially
        use wait=False for simultaneous motion

        :param joints:  a 1-by-12 vector in degree, 6 for right, 6 for left
        :return: bool

        author: weiwei
        date: 20170411
        """

        jointsrad = [math.radians(angdeg) for angdeg in joints[0:6]]
        self.__rgtarm.movej(jointsrad)
        jointsrad = [math.radians(angdeg) for angdeg in joints[6:12]]
        self.__lftarm.movej(jointsrad)

    def gettcp(self, armid = 'rgt'):
        """
        get the tcp (x, y, z, rx, ry, rz) of the specified arm (in dual arm coordinate system)

        :param armid:
        :return:

        author: weiwei
        date: 20170413
        """

        targetarm = self.__rgtarm
        targetarmbase = self.__rgtarmbase
        if armid == 'lft':
            targetarm = self.__lftarm
            targetarmbase = self.__lftarmbase

        tpos = targetarm.getl(wait = True)
        print tpos
        rtpos = [0,0,0,0,0,0]
        rtpos[0] = -tpos[0]*1000.0
        rtpos[1] = (-tpos[2]+tpos[1])*self.__sqrt2o2*1000.0
        rtpos[2] = (-tpos[2]-tpos[1])*self.__sqrt2o2*1000.0
        if armid == 'lft':
            rtpos[0] = - rtpos[0]
            rtpos[1] = -rtpos[1]
            rtpos[2] = rtpos[2]

        rtpos[1] = rtpos[1] + targetarmbase[1]
        rtpos[2] = rtpos[2] + targetarmbase[2]
        return rtpos

    def getforcevec(self, armid='rgt'):
        """
        return force tcp in world coordinate system

        :return:

        author: weiwei
        date: 20170412
        """

        targetarm = self.__rgtarm
        if armid == 'lft':
            targetarm = self.__lftarm
        return targetarm.get_tcp_force()

if __name__ == '__main__':

    from panda3d.core import *
    import robotsim.ur5dual.ur5dual as u5d
    import robotsim.ur5dual.ur5dualik as u5dik
    import robotsim.ur5dual.ur5dualplot as u5p
    import pandaplotutils.pandageom as pg
    import pandaplotutils.pandactrl as pc
    import numpy as np
    from manipulation.grip.robotiq85 import rtq85nm
    import impedance as imp
    handpkg = rtq85nm

    base = pc.World(camp = [3000,0,3000], lookatp = [0,0,700])
    ur5dualrobot = u5d.Ur5DualRobot()
    ur5dualrobot.goinitpose()
    ur5u = Ur5DualUrx()

    # print ur5u.rgtarm.get_pose()
    # print ur5u.rgtarm.getj()
    # ur5u.movejnts6([0,0,0,0,0,0])
    # ur5u.movejnts6([-30,-90,120,0,120,0])

    # armid = 'lft'
    # # ur5u.movejntssgl(ur5dualrobot.initjnts[9:15], armid)
    # ur5dualrobot.movearmfk(ur5dualrobot.initjnts[9:15], armid)
    # tcprobot =  ur5u.gettcp(armid)
    # tcpsimrobot =  ur5dualrobot.lftarm[-1]['linkpos']
    # print tcprobot
    # print tcpsimrobot
    # ur5dualmnp = u5p.genUr5dualmnp(ur5dualrobot, handpkg)
    # ur5dualmnp.reparentTo(base.render)

    m = .01
    c = .01
    k = .01
    t = .05
    impx = imp.Impedance(m, c, k, t)
    impy = imp.Impedance(m, c, k, t)
    impz = imp.Impedance(m, c, k, t)

    arrownp = []
    robotnp = []

    counter = [0]
    fvalhandinit = [0,0,0]
    tcpinit = [np.array([0,0,0]), np.array([[1,0,0],[0,1,0],[0,0,1]])]
    tpose = [[0,0,0,0,0,0]]

    def updateshow(ur5u, ur5dualrobot, armid, robotnp, arrownp, counter, fvalhandinit, tcpinit, tpose, task):
        for robot in robotnp:
            robot.detachNode()
        for arrow in arrownp:
            arrow.detachNode()

        ur5dualmnp = u5p.genmnp(ur5dualrobot, handpkg)
        ur5dualmnp.reparentTo(base.render)
        robotnp.append(ur5dualmnp)

        force =  ur5u.getforcevec(armid = armid)
        basemat4 = Mat4.rotateMat(-145, Vec3(1,0,0))
        tcpmat4 = ur5dualrobot.gettcpframe(armid)
        fvecx = basemat4.getRow3(0)*force[0]
        fvecy = basemat4.getRow3(1)*force[1]
        fvecz = basemat4.getRow3(2)*force[2]
        fvecxhand = fvecx.project(tcpmat4.getRow3(0)) + fvecy.project(tcpmat4.getRow3(0)) + fvecz.project(tcpmat4.getRow3(0))
        fvecyhand = fvecx.project(tcpmat4.getRow3(1)) + fvecy.project(tcpmat4.getRow3(1)) + fvecz.project(tcpmat4.getRow3(1))
        fveczhand = fvecx.project(tcpmat4.getRow3(2)) + fvecy.project(tcpmat4.getRow3(2)) + fvecz.project(tcpmat4.getRow3(2))
        fvalxhand = fvecxhand.length()
        fvalyhand = fvecyhand.length()
        fvalzhand = fveczhand.length()
        fvecxworld = fvecx.project(Vec3(1,0,0)) + fvecy.project(Vec3(1,0,0)) + fvecz.project(Vec3(1,0,0))
        fvecyworld = fvecx.project(Vec3(0,1,0)) + fvecy.project(Vec3(0,1,0)) + fvecz.project(Vec3(0,1,0))
        fveczworld = fvecx.project(Vec3(0,0,1)) + fvecy.project(Vec3(0,0,1)) + fvecz.project(Vec3(0,0,1))
        fvalxworld = fvecxworld[0]
        fvalyworld = fvecyworld[1]
        fvalzworld = fveczworld[2]
        if counter[0] == 0:
            fvalhandinit[0] = fvalxhand
            fvalhandinit[1] = fvalyhand
            fvalhandinit[2] = fvalzhand
            tcpinit[0], tcpinit[1] = ur5dualrobot.gettcp(armid)
            counter[0] += 1
            task.again
        if fvalxhand > fvalhandinit[0]+20:
            deltax = impx.control(fvalxworld)
            arrownp.append(pg.plotArrow(base.render, tcpmat4.getRow3(3), tcpmat4.getRow3(3)+fvecxhand*50, thickness = 10, rgba = [.7,.2,.2,1]))
        else:
            deltax = impx.control(0)
        if fvalyhand > fvalhandinit[1]+20:
            deltay = impy.control(fvalyworld)
            arrownp.append(pg.plotArrow(base.render, tcpmat4.getRow3(3), tcpmat4.getRow3(3)+fvecyhand*50, thickness = 10, rgba = [.2,.7,.2,1]))
        else:
            deltay = impy.control(0)
        if fvalzhand > fvalhandinit[2]+20:
            deltaz = impz.control(fvalzworld)
            arrownp.append(pg.plotArrow(base.render, tcpmat4.getRow3(3), tcpmat4.getRow3(3)+fveczhand*50, thickness = 10, rgba = [.2,.2,.7,1]))
        else:
            deltaz = impz.control(0)
        print tpose[0]
        print deltax, deltay, deltaz

        tcpgoalpos = tcpinit[0] + np.array([deltax, deltay, deltaz])
        tcpgoalrot = tcpinit[1]

        armjntsgoal = u5dik.numik(ur5dualrobot, tcpgoalpos, tcpgoalrot, armid)
        if armjntsgoal is not None:
            ur5dualrobot.movearmfk(armjntsgoal, armid)
            # ur5u.movejntssgl(armjntsgoal.tolist(), armid)

        return task.again

    armid = 'lft'
    taskMgr.add(updateshow, 'updateshow', extraArgs = [ur5u, ur5dualrobot, armid, robotnp, arrownp, counter, fvalhandinit, tcpinit, tpose], appendTask = True)

    base.run()
