import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from panda3d.core import *
import copy
import math

import pandaplotutils.pandageom as pg

class NxtBall(object):
    """
    generate nxt5balls for quick collision detection

    NOTE: it is unnecessary to attach a mnp to render repeatedly
    once attached, it is always there. update the joint angles will change the attached model directly
    """

    def __init__(self):
        """
        load models

        author: weiwei
        date: 20180110
        """

        # b indicates ball, cn indicates collision node
        # self.__bodybcn = CollisionNode("robotbody")
        # self.__rgtupperbcn = CollisionNode("rgtupperarm")
        # self.__rgtlowerbcn = CollisionNode("rgtlowerbcn")
        # self.__rgthandbcn = CollisionNode("rgthandbcn")
        # self.__lftupperbcn = CollisionNode("lftupperarm")
        # self.__lftlowerbcn = CollisionNode("lftlowerbcn")
        # self.__lfthandbcn = CollisionNode("lfthandbcn")
        # self.__bodybcn = None
        # self.__rgtupperbcn = None
        # self.__rgtlowerbcn = None
        # self.__rgthandbcn = None
        # self.__lftupperbcn = None
        # self.__lftlowerbcn = None
        # self.__lfthandbcn = None
        self.__shownp = None

    def __genbcn(self, linklist, radius = 70.0, name = "autogen"):
        """
        gennerate the ball collision node for a link

        :param linkid:
        :param armid:
        :return:
        """

        radius = float(radius)
        balldist = radius

        tmpcolnode = CollisionNode(name)
        for link in linklist:
            spos = link['linkpos']
            epos = link['linkend']
            linklength = np.linalg.norm(epos - spos)
            linkvec = (epos - spos)/linklength
            nball = int(math.ceil(linklength / balldist))
            for i in range(1, nball):
                pos = spos+linkvec*i*(balldist)
                # if i == nball-1:
                #     pos = spos+linkvec*(i-.4)*(balldist)
                colsphere = CollisionSphere(pos[0], pos[1], pos[2], radius)
                tmpcolnode.addSolid(colsphere)
        return tmpcolnode

    def genbcnlist(self, nxtrobot):
        """
        generate the ball collision nodes of a robot

        :param nxtrobot: the NxtRobot object, see Nxtrobot.py
        :return: null

        author: weiwei
        date: 20180110
        """

        # body
        link0 = nxtrobot.rgtarm[0]
        link1 = nxtrobot.lftarm[0]
        bodybcn = self.__genbcn([link0, link1], radius = 30)

        # rgt arm
        # rgt upper arm
        link = nxtrobot.rgtarm[2]
        rgtupperbcn = self.__genbcn([link])
        # rgt lower arm
        link = nxtrobot.rgtarm[3]
        rgtlowerbcn = self.__genbcn([link])
        # rgt hand
        link = nxtrobot.rgtarm[6]
        rgthandbcn = self.__genbcn([link], radius = 80)

        # lft arm
        # lft upper arm
        link = nxtrobot.lftarm[2]
        lftupperbcn = self.__genbcn([link])
        # lft lower arm
        link = nxtrobot.lftarm[3]
        lftlowerbcn = self.__genbcn([link])
        # lft hand
        link = nxtrobot.lftarm[6]
        lfthandbcn = self.__genbcn([link], radius = 80)

        return [bodybcn, rgtupperbcn, rgtlowerbcn, rgthandbcn,
                lftupperbcn, lftlowerbcn, lfthandbcn]

    def showbcn(self, base, bcnlist):
        """
        show bcnlist to base

        :param bcnlist is in the form of
        [bodybcn, rgtupperbcn, rgtlowerbcn,rgthandbcn,
                lftupperbcn, lftlowerbcn, lfthandbcn]
        :return: null

        author: weiwei
        date: 20170615
        """

        self.unshowbcn()
        self.__shownp = NodePath("collision balls")
        for bcn in bcnlist:
            tst = self.__shownp.attachNewNode(bcn)
            tst.show()
        self.__shownp.reparentTo(base.render)

    def unshowbcn(self):
        """
        show bcnlist to base

        :param bcnlist is in the form of
        [bodybcn, rgtupperbcn, rgtlowerbcn,rgthandbcn,
                lftupperbcn, lftlowerbcn, lfthandbcn]
        :return: null

        author: weiwei
        date: 20170615
        """

        if self.__shownp is not None:
            self.__shownp.removeNode()
        self.__shownp = None


if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters
    import nxt
    from manipulation.grip.robotiq85 import rtq85nm
    import nxt
    import nxtmesh

    # loadPrcFileData("", "want-directtools #t")
    # loadPrcFileData("", "want-tk #t")

    base = pandactrl.World()

    nxtrobot = nxt.NxtRobot()
    nxtb = NxtBall()
    bcnlist = nxtb.genbcnlist(nxtrobot)
    nxtb.showbcn(base, bcnlist)
    handpkg = rtq85nm
    nxtmeshgen = nxtmesh.NxtMesh(handpkg)
    nxtmnp = nxtmeshgen.genmnp(nxtrobot)
    nxtmnp.reparentTo(base.render)

    base.run()

