import numpy as np
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
import utils.collisiondetection as cd
from panda3d.core import *
import pandaplotutils.pandageom as pg

class CollisionChecker(object):
    """
    check the collision of a robot

    """

    def __init__(self, robotplot):
        """
        set up the collision checker

        author: weiwei
        date: 20170608
        """

        self.bulletworld = BulletWorld()
        # robotplot is used to generate nodepaths for links
        self.robotplot = robotplot

    def __isSglArmCollided(self, sglbullnodes):
        """
        check the self-collision of a single arm

        :param sglbullnodes: a list of bullnodes starting from base to end-effector
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


    def isCollided(self, robot, handpkg):
        """
        check the collision of a single arm

        :param armid: 'lft' or 'rgt'
        :return:

        author: weiwei
        date: 20170608
        """

        dualmnps = self.robotplot.genmnplist(robot, handpkg)
        # single arm check
        sglmnps = dualmnps[0]
        sglbullnodes = []
        # armbase is not examined
        for sglmnp in sglmnps[1:len(sglmnps)-1]:
            sglbullnode = cd.genCollisionMeshMultiNp(sglmnp, basenodepath=None, name='autogen')
            sglbullnodes.append(sglbullnode)
        if self.__isSglArmCollided(sglbullnodes):
            print "right arm self collision!"
            return True
        else:
            sglmnps = dualmnps[1]
            sglbullnodes = []
        # armbase is not examined
            for sglmnp in sglmnps[1:]:
                sglbullnode = cd.genCollisionMeshMultiNp(sglmnp, basenodepath=None, name='autogen')
                sglbullnodes.append(sglbullnode)
            if self.__isSglArmCollided(sglbullnodes):
                print "left arm self collision!"
                return True

        # dual arm check
        rgtarmnp = NodePath("rgtarm")
        sglmnps = dualmnps[0]
        # armbase is not examined
        for sglmnp in sglmnps[1:]:
            sglmnp.reparentTo(rgtarmnp)
        lftarmnp = NodePath("lftarm")
        sglmnps = dualmnps[1]
        # armbase is not examined
        for sglmnp in sglmnps[1:]:
            sglmnp.reparentTo(lftarmnp)
        rgtbullnode = cd.genCollisionMeshMultiNp(rgtarmnp, basenodepath=None, name='autogen')
        lftbullnode = cd.genCollisionMeshMultiNp(lftarmnp, basenodepath=None, name='autogen')
        result = self.bulletworld.contactTestPair(rgtbullnode, lftbullnode)
        if result.getNumContacts():
            print "arm-arm collision!"
            return True

        # arm body check
        body = NodePath("body")
        bodymnps = dualmnps[2]
        if bodymnps:
            for bodymnp in bodymnps:
                bodymnp.reparentTo(body)
            bodybullnode = cd.genCollisionMeshMultiNp(body, basenodepath=None, name='autogen')
            result = self.bulletworld.contactTestPair(rgtbullnode, bodybullnode)
            if result.getNumContacts():
                print "rgtarm-body collision!"
                return True
            else:
                result = self.bulletworld.contactTestPair(lftbullnode, bodybullnode)
                if result.getNumContacts():
                    print "lftarm-body collision!"
                    return True

        self.bulletworld.attachRigidBody(rgtbullnode)
        self.bulletworld.attachRigidBody(lftbullnode)
        self.bulletworld.attachRigidBody(bodybullnode)

        return False

if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters
    from robotsim.ur5dual import ur5dual
    from robotsim.ur5dual import ur5dualplot
    from robotsim.hrp5n import hrp5n
    from robotsim.hrp5n import hrp5nplot
    from robotsim.nextage import nxt
    from robotsim.nextage import nxtplot
    from manipulation.grip.robotiq85 import rtq85nm
    from manipulation.grip.hrp5three import hrp5threenm

    base = pandactrl.World(camp = [3000,0,3000], lookatp = [0,0,700])

    robot = ur5dual.Ur5DualRobot()
    handpkg = rtq85nm
    robotplot = ur5dualplot

    robot = hrp5n.Hrp5NRobot()
    handpkg = hrp5threenm
    robotplot = hrp5nplot

    # robot = nxt.NxtRobot()
    # handpkg = rtq85nm
    # robotplot = nxtplot

    robot.goinitpose()
    robot.movearmfk([0.0,-20.0,-60.0,-70.0,-100.0,-30.0,0.0,0.0,0.0])
    robot.movearmfk([0.0,-20.0,60.0,70.0,-100.0,30.0,0.0,0.0,0.0], armid = 'lft')

    # ur5dualmnp = ur5dualplot.genUr5dualmnp(robot, handpkg)
    # ur5dualmnp.reparentTo(base.render)
    robotmnp = robotplot.genmnp(robot,handpkg)
    robotmnp.reparentTo(base.render)
    pg.plotAxisSelf(base.render, Vec3(0,0,0))

    cdchecker = CollisionChecker(robotplot)
    print cdchecker.isCollided(robot, handpkg)

    bullcldrnp = base.render.attachNewNode("bulletcollider")
    debugNode = BulletDebugNode('Debug')
    debugNode.showWireframe(True)
    debugNode.showConstraints(True)
    debugNode.showBoundingBoxes(False)
    debugNode.showNormals(False)
    debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()

    cdchecker.bulletworld.setDebugNode(debugNP.node())

    def updateworld(world, task):
        world.doPhysics(globalClock.getDt())
        return task.cont
    base.taskMgr.add(updateworld, "updateworld", extraArgs=[cdchecker.bulletworld], appendTask=True)
    base.run()