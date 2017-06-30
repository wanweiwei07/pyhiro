import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from manipulation.grip.hrp5three import hrp5three
from panda3d.core import *
import copy
import math

import pandaplotutils.pandageom as pg

class Hrp5NBall(object):
    """
    generate hrp5nballs for quick collision detection

    NOTE: it is unnecessary to attach a mnp to render repeatedly
    once attached, it is always there. update the joint angles will change the attached model directly
    """

    def __init__(self):
        """
        load models

        author: weiwei
        date: 20170612
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

        :param hrp5nrobot:
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

    def genbcnlist(self, hrp5nrobot):
        """
        generate the ball collision nodes of a robot

        :param hrp5robot: the Hrp5Robot object, see Hrp5robot.py
        :return: null

        author: weiwei
        date: 20170615
        """

        # body
        link0 = hrp5nrobot.rgtarm[0]
        link1 = hrp5nrobot.lftarm[0]
        bodybcn = self.__genbcn([link0, link1], radius = 100)

        # rgt arm
        # rgt upper arm
        link = hrp5nrobot.rgtarm[4]
        rgtupperbcn = self.__genbcn([link])
        # rgt lower arm
        link = hrp5nrobot.rgtarm[5]
        rgtlowerbcn = self.__genbcn([link])
        # rgt hand
        link = hrp5nrobot.rgtarm[9]
        rgthandbcn = self.__genbcn([link], radius = 80)

        # lft arm
        # lft upper arm
        link = hrp5nrobot.lftarm[4]
        lftupperbcn = self.__genbcn([link])
        # lft lower arm
        link = hrp5nrobot.lftarm[5]
        lftlowerbcn = self.__genbcn([link])
        # lft hand
        link = hrp5nrobot.lftarm[9]
        lfthandbcn = self.__genbcn([link], radius = 80)

        return [bodybcn, rgtupperbcn, rgtlowerbcn,rgthandbcn,
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
    import hrp5n
    import hrp5nmesh
    from manipulation.grip.hrp5three import hrp5threenm

    # loadPrcFileData("", "want-directtools #t")
    # loadPrcFileData("", "want-tk #t")

    base = pandactrl.World()

    hrp5nrobot = hrp5n.Hrp5NRobot()
    hrp5nb = Hrp5NBall()
    bcnlist = hrp5nb.genbcnlist(hrp5nrobot)
    hrp5nb.showbcn(base, bcnlist)
    handpkg = hrp5threenm
    hrp5nmeshgen = hrp5nmesh.Hrp5NMesh(handpkg)
    hrp5nmnp = hrp5nmeshgen.genmnp(hrp5nrobot)
    hrp5nmnp.reparentTo(base.render)

    base.run()

