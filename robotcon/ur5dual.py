# !/usr/bin/env python

import math
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
        self.__rgtarm = Robot(iprgt)
        self.__rgtarm.set_tcp((0, 0, 0, 0, 0, 0))
        self.__rgtarm.set_payload(1.28)
        self.__lftarm = Robot(iplft)
        self.__lftarm.set_tcp((0, 0, 0, 0, 0, 0))
        self.__lftarm.set_payload(.86)

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
        targetarm.movej(jointsrad)

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

if __name__ == '__main__':
    ur5u = Ur5DualUrx()

    print ur5u.rgtarm.get_pose()
    print ur5u.rgtarm.getj()
    # ur5u.movejnts6([0,0,0,0,0,0])
    # ur5u.movejnts6([-30,-90,120,0,120,0])
    ur5u.movejnts6([30,-90,-120,180,-120,0], 'lft')