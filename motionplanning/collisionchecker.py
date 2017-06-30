import numpy as np
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
import utils.collisiondetection as cd
from panda3d.core import *
import pandaplotutils.pandageom as pg
import gc

class CollisionChecker(object):
    """
    check the collision of a robot

    """

    def __init__(self, robotmesh):
        """
        set up the collision checker

        :param robotmesh is an object of robotsim/robot.robotmesh

        author: weiwei
        date: 20170608
        """

        self.bulletworld = BulletWorld()
        self.robotmesh = robotmesh
        self.counter = 0

    def __isSglArmCollided(self, sglbullnodes):
        """
        check the self-collision of a single arm

        :param sglbullnodes: a list of bullnodes starting from arm base to end-effector
        :return:

        author: weiwei
        date: 20170608
        """

        # collisioncheck, the near three links are not examined
        nlink = len(sglbullnodes)
        for i in range(nlink):
            for k in range(i+3, nlink):
                result = self.bulletworld.contactTestPair(sglbullnodes[i], sglbullnodes[k])
                if result.getNumContacts():
                    return True
        return False

    def __isSglArmBdyCollided(self, sglbullnodes, bdybullnodes):
        """
        check the self-collision of a single arm

        :param sglbullnodes: a list of bullnodes starting from arm base to end-effector
        :param bdybullnodes: a list of bullnodes for robot body
        :return:

        author: weiwei
        date: 20170615
        """

        # collisioncheck, the near three links are not examined
        nlinkarm = len(sglbullnodes)
        nlinkbdy = len(bdybullnodes)
        for i in range(1, nlinkarm):
            for k in range(0, nlinkbdy):
                result = self.bulletworld.contactTestPair(sglbullnodes[i], bdybullnodes[k])
                if result.getNumContacts():
                    return True
        return False

    def __isDualArmCollided(self, sglbullnodesrgt, sglbullnodeslft):
        """
        check the self-collision of a single arm
        the current implementation only check hands

        :param sglbullnodesrgt: a list of bullnodes starting from base to end-effector
        :param sglbullnodeslft: a list of bullnodes starting from base to end-effector
        :return:

        author: weiwei
        date: 20170608
        """

        # collisioncheck, the near three links are not examined
        nlinkrgt = len(sglbullnodesrgt)
        nlinklft = len(sglbullnodeslft)
        for i in range(nlinkrgt-1, nlinkrgt):
            for k in range(nlinklft-1, nlinklft):
                result = self.bulletworld.contactTestPair(sglbullnodesrgt[i], sglbullnodeslft[k])
                if result.getNumContacts():
                    return True
        return False


    def isSelfCollided(self, robot):
        """
        check the collision of a single arm

        :param armid: 'lft' or 'rgt'
        :return:

        author: weiwei
        date: 20170608
        """

        dualmnps = self.robotmesh.genmnp_list(robot)
        # single arm check
        sglmnps = dualmnps[0]
        sglbullnodesrgt = []
        sglbullnodeslft = []
        # armbase is not examined
        for sglmnp in sglmnps[1:len(sglmnps)-1]:
            sglbullnode = cd.genCollisionMeshNp(sglmnp, basenodepath=None, name='autogen')
            sglbullnodesrgt.append(sglbullnode)
        # hand is multinp
        sglbullnode = cd.genCollisionMeshMultiNp(sglmnps[-1], basenodepath=None, name='autogen')
        sglbullnodesrgt.append(sglbullnode)
        if self.__isSglArmCollided(sglbullnodesrgt):
            print "right arm self collision!"
            return True
        else:
            sglmnps = dualmnps[1]
            # armbase is not examined
            for sglmnp in sglmnps[1:len(sglmnps)-1]:
                sglbullnode = cd.genCollisionMeshNp(sglmnp, basenodepath=None, name='autogen')
                sglbullnodeslft.append(sglbullnode)
            # hand is multinp
            sglbullnode = cd.genCollisionMeshMultiNp(sglmnps[-1], basenodepath=None, name='autogen')
            sglbullnodeslft.append(sglbullnode)
            if self.__isSglArmCollided(sglbullnodeslft):
                print "left arm self collision!"
                return True

        # dual arm check
        if self.__isDualArmCollided(sglbullnodesrgt, sglbullnodeslft):
            print "left-right arm self collision!"
            return True
        #
        # arm body check
        bodymnps = dualmnps[2]
        bdybullnodes = []
        if bodymnps:
            for bodymnp in bodymnps:
                bodybullnode = cd.genCollisionMeshMultiNp(bodymnp, basenodepath=None, name='autogen')
                bdybullnodes.append(bodybullnode)
        if self.__isSglArmBdyCollided(sglbullnodesrgt, bdybullnodes):
            print "right arm body self collision!"
            return True
        if self.__isSglArmBdyCollided(sglbullnodeslft, bdybullnodes):
            print "left right body arm self collision!"
            return True

        # for bullnode in sglbullnodesrgt[-4:]:
        #     self.bulletworld.attachRigidBody(bullnode)
        # for bullnode in sglbullnodeslft:
        #     self.bulletworld.attachRigidBody(bullnode)
        # for bullnode in bdybullnodes:
        #     self.bulletworld.attachRigidBody(bullnode)

        return False

    def isCollided(self, robot, obstaclelist):
        """
        simultaneously check self-collision and robot-obstacle collision
        this function should be faster than calling isselfcollided and isobstaclecollided one after another

        :param robot:
        :param obstaclelist:
        :return:

        author: weiwei
        date: 20170613
        """

        print self.counter
        if self.counter > 200:
            self.counter = 0
            self.bulletworld = BulletWorld()
            gc.collect()
        self.counter += 1

        dualmnps = self.robotmesh.genmnp_list(robot)
        sglbullnodesrgt = []
        sglbullnodeslft = []
        # rgt arm
        # single arm check
        sglmnps = dualmnps[0]
        # armbase is not examined
        for sglmnp in sglmnps[1:len(sglmnps)-1]:
            sglbullnode = cd.genCollisionMeshNp(sglmnp, basenodepath=None, name='autogen')
            sglbullnodesrgt.append(sglbullnode)
        # hand is multinp
        sglbullnode = cd.genCollisionMeshMultiNp(sglmnps[-1], basenodepath=None, name='autogen')
        sglbullnodesrgt.append(sglbullnode)
        # lft arm
        sglmnps = dualmnps[1]
        # armbase is not examined
        for sglmnp in sglmnps[1:len(sglmnps)-1]:
            sglbullnode = cd.genCollisionMeshNp(sglmnp, basenodepath=None, name='autogen')
            sglbullnodeslft.append(sglbullnode)
        # hand is multinp
        sglbullnode = cd.genCollisionMeshMultiNp(sglmnps[-1], basenodepath=None, name='autogen')
        sglbullnodeslft.append(sglbullnode)

        # arm obstacle check
        # only check hands
        nlinkrgt = len(sglbullnodesrgt)
        nlinklft = len(sglbullnodeslft)
        for obstaclemnp in obstaclelist:
            obstaclebullnode = cd.genCollisionMeshNp(obstaclemnp, basenodepath=None, name='autogen')
            for i in range(nlinkrgt - 1, nlinkrgt):
                result = self.bulletworld.contactTestPair(sglbullnodesrgt[i], obstaclebullnode)
                if result.getNumContacts():
                    print "rgtarm-obstacle collision!"
                    return True
            for i in range(nlinklft - 1, nlinklft):
                result = self.bulletworld.contactTestPair(sglbullnodeslft[i], obstaclebullnode)
                if result.getNumContacts():
                    print "lftarm-obstacle collision!"
                    return True
        #
        # # sgl arm check
        # if self.__isSglArmCollided(sglbullnodesrgt):
        #     print "right arm self collision!"
        #     return True
        # else:
        #     if self.__isSglArmCollided(sglbullnodeslft):
        #         print "left arm self collision!"
        #         return True
        #
        # # dual arm check
        # if self.__isDualArmCollided(sglbullnodesrgt, sglbullnodeslft):
        #     print "left-right arm self collision!"
        #     return True
        # #
        # # arm body check
        # bodymnps = dualmnps[2]
        # bdybullnodes = []
        # if bodymnps:
        #     for bodymnp in bodymnps:
        #         bodybullnode = cd.genCollisionMeshMultiNp(bodymnp, basenodepath=None, name='autogen')
        #         bdybullnodes.append(bodybullnode)
        # if self.__isSglArmBdyCollided(sglbullnodesrgt, bdybullnodes):
        #     print "right arm body self collision!"
        #     return True
        # if self.__isSglArmBdyCollided(sglbullnodeslft, bdybullnodes):
        #     print "left right body arm self collision!"
        #     return True

        # for bullnode in sglbullnodesrgt:
        #     self.bulletworld.attachRigidBody(bullnode)
        # for bullnode in sglbullnodeslft:
        #     self.bulletworld.attachRigidBody(bullnode)
        # for bullnode in bdybullnodes:
        #     self.bulletworld.attachRigidBody(bullnode)

        return False

if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    # from direct.filter.CommonFilters import CommonFilters
    # from robotsim.ur5dual import ur5dual
    # from robotsim.ur5dual import ur5dualplot
    from robotsim.hrp5n import hrp5n
    # from robotsim.hrp5n import hrp5nplot
    from robotsim.hrp5n import hrp5nmesh
    # from robotsim.nextage import nxt
    # from robotsim.nextage import nxtplot
    # from manipulation.grip.robotiq85 import rtq85nm
    from manipulation.grip.hrp5three import hrp5threenm

    # use the following two sentences to examine the scene
    # loadPrcFileData("", "want-directtools #t")
    # loadPrcFileData("", "want-tk #t")

    base = pandactrl.World(camp = [3000,0,3000], lookatp = [0,0,700])

    # robot = ur5dual.Ur5DualRobot()
    # handpkg = rtq85nm
    # robotplot = ur5dualplot

    robot = hrp5n.Hrp5NRobot()
    handpkg = hrp5threenm
    robotmesh = hrp5nmesh.Hrp5NMesh(handpkg)
    robotmnp = robotmesh.genmnp(robot)
    robotmnp.reparentTo(base.render)

    # robot = nxt.NxtRobot()
    # handpkg = rtq85nm
    # robotplot = nxtplot

    # robot.goinitpose()
    # robot.movearmfk([0.0,-20.0,-60.0,-70.0,-100.0,-30.0,0.0,0.0,0.0])
    # robot.movearmfk([0.0,-20.0,60.0,70.0,-100.0,30.0,0.0,0.0,0.0], armid = 'lft')
    # robotmnp = robotmesh.genmnp(robot)
    # robotmnp.reparentTo(base.render)

    # ur5dualmnp = ur5dualplot.genUr5dualmnp(robot, handpkg)
    # ur5dualmnp.reparentTo(base.render)
    pg.plotAxisSelf(base.render, Vec3(0,0,0))

    cdchecker = CollisionChecker(robotmesh)
    print cdchecker.isSelfCollided(robot)

    bullcldrnp = base.render.attachNewNode("bulletcollider")
    debugNode = BulletDebugNode('Debug')
    debugNode.showWireframe(True)
    debugNode.showConstraints(True)
    debugNode.showBoundingBoxes(False)
    debugNode.showNormals(False)
    debugNP = bullcldrnp.attachNewNode(debugNode)
    debugNP.show()

    cdchecker.bulletworld.setDebugNode(debugNP.node())

    def updateworld(world, task):
        world.doPhysics(globalClock.getDt())
        return task.cont
    base.taskMgr.add(updateworld, "updateworld", extraArgs=[cdchecker.bulletworld], appendTask=True)
    base.run()