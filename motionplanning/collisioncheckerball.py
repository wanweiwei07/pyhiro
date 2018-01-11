import numpy as np
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
import utils.collisiondetection as cd
from panda3d.core import *
import pandaplotutils.pandageom as pg
import gc

class CollisionCheckerBall(object):
    """
    check the collision of a robot, using ball fitting

    """

    def __init__(self, robotball):
        """
        set up the collision checker

        :param robotball is an object of robotsim/robot.robotball

        author: weiwei
        date: 20170615
        """

        self.__robotball = robotball


    def isSelfCollided(self, robot):
        """
        check the collision of a single arm

        :param armid: 'lft' or 'rgt'
        :return:

        author: weiwei
        date: 20170615
        """

        dualbcnlist = self.__robotball.genbcnlist(robot)

        # arm body
        collisionnp = NodePath("collision nodepath")
        bodynp = collisionnp.attachNewNode(dualbcnlist[0])
        rgtupperarmnp = collisionnp.attachNewNode(dualbcnlist[1])
        rgtlowerarmnp = collisionnp.attachNewNode(dualbcnlist[2])
        rgthandnp = collisionnp.attachNewNode(dualbcnlist[3])
        lftupperarmnp = collisionnp.attachNewNode(dualbcnlist[4])
        lftlowerarmnp = collisionnp.attachNewNode(dualbcnlist[5])
        lfthandnp = collisionnp.attachNewNode(dualbcnlist[6])
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        ctrav.addCollider(bodynp, chan)
        ctrav.traverse(collisionnp)
        if chan.getNumEntries() > 0:
            print "arm-body collision"
            return True

        # dual arm
        collisionnp = NodePath("collision nodepath")
        rgtlowerarmnp = collisionnp.attachNewNode(dualbcnlist[2])
        rgthandnp = collisionnp.attachNewNode(dualbcnlist[3])
        lftlowerarmnp = collisionnp.attachNewNode(dualbcnlist[5])
        lfthandnp = collisionnp.attachNewNode(dualbcnlist[6])
        # rgt
        # hand
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        ctrav.addCollider(rgthandnp, chan)
        ctrav.traverse(collisionnp)
        if chan.getNumEntries() > 0:
            print "rgthand-arm collision"
            return True
        # lower arm
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        ctrav.addCollider(rgtlowerarmnp, chan)
        ctrav.traverse(collisionnp)
        if chan.getNumEntries() > 0:
            print "rgtlowerarm-arm collision"
            return True
        # lft
        # hand
        # only self-detection is needed
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        ctrav.addCollider(lfthandnp, chan)
        ctrav.traverse(lftlowerarmnp)
        if chan.getNumEntries() > 0:
            print "lfthand-arm collision"
            return True

        return False


    def isCollided(self, robot, obstaclelist = []):
        """
        check the collision of a single arm

        :param armid: 'lft' or 'rgt'
        :return:

        author: weiwei
        date: 20170615
        """

        dualbcnlist = self.__robotball.genbcnlist(robot)

        # arm body
        collisionnp = NodePath("collision nodepath")
        bodynp = collisionnp.attachNewNode(dualbcnlist[0])
        rgtupperarmnp = collisionnp.attachNewNode(dualbcnlist[1])
        rgtlowerarmnp = collisionnp.attachNewNode(dualbcnlist[2])
        rgthandnp = collisionnp.attachNewNode(dualbcnlist[3])
        lftupperarmnp = collisionnp.attachNewNode(dualbcnlist[4])
        lftlowerarmnp = collisionnp.attachNewNode(dualbcnlist[5])
        lfthandnp = collisionnp.attachNewNode(dualbcnlist[6])
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        ctrav.addCollider(bodynp, chan)
        ctrav.traverse(collisionnp)
        if chan.getNumEntries() > 0:
            print "arm-body collision"
            return True

        # dual arm
        collisionnp = NodePath("collision nodepath")
        rgtlowerarmnp = collisionnp.attachNewNode(dualbcnlist[2])
        rgthandnp = collisionnp.attachNewNode(dualbcnlist[3])
        lftlowerarmnp = collisionnp.attachNewNode(dualbcnlist[5])
        lfthandnp = collisionnp.attachNewNode(dualbcnlist[6])
        # rgt
        # hand
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        ctrav.addCollider(rgthandnp, chan)
        ctrav.traverse(collisionnp)
        if chan.getNumEntries() > 0:
            print "rgthand-arm collision"
            return True
        # lower arm
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        ctrav.addCollider(rgtlowerarmnp, chan)
        ctrav.traverse(collisionnp)
        if chan.getNumEntries() > 0:
            print "rgtlowerarm-arm collision"
            return True
        # lft
        # hand
        # only self-detection is needed
        ctrav = CollisionTraverser()
        chan = CollisionHandlerQueue()
        ctrav.addCollider(lfthandnp, chan)
        ctrav.traverse(lftlowerarmnp)
        if chan.getNumEntries() > 0:
            print "lfthand-arm collision"
            return True

        # obj robot
        collisionnp = NodePath("collision nodepath")
        bodynp = collisionnp.attachNewNode(dualbcnlist[0])
        rgtlowerarmnp = collisionnp.attachNewNode(dualbcnlist[2])
        rgthandnp = collisionnp.attachNewNode(dualbcnlist[3])
        lftlowerarmnp = collisionnp.attachNewNode(dualbcnlist[5])
        lfthandnp = collisionnp.attachNewNode(dualbcnlist[6])

        for obstacle in obstaclelist:
            bottomLeft, topRight = obstacle.getTightBounds()
            box = CollisionBox(bottomLeft, topRight)
            tmpcolnode = CollisionNode("auto gen")
            tmpcolnode.addSolid(box)
            obstaclenp = collisionnp.attachNewNode(tmpcolnode)
            ctrav = CollisionTraverser()
            chan = CollisionHandlerQueue()
            ctrav.addCollider(obstaclenp, chan)
            ctrav.traverse(collisionnp)
            if chan.getNumEntries() > 0:
                print "obstacle-arm collision"
                return True
            # obstaclenp.reparentTo(base.render)
            # obstaclenp.show()
            obstaclenp.removeNode()

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
    from robotsim.hrp5n import hrp5nball
    # from robotsim.nextage import nxt
    # from robotsim.nextage import nxtplot
    # from manipulation.grip.robotiq85 import rtq85nm
    from manipulation.grip.hrp5three import hrp5threenm

    base = pandactrl.World(camp = [3000,0,3000], lookatp = [0,0,700])

    robotpose = [0.0,20.0,60.0,-70.0,-100.0,-30.0,0.0,0.0,0.0]
    robotpose = [2.2080256451620497, 26.289390566725089, -22.514710701256242, 4.6334420464768309, -149.12158792410156, 10.508610971291665, -4.2442811014793724, 1.6655475605543084, 19.627905624666109]
    robot = hrp5n.Hrp5NRobot()
    robot.movearmfk(robotpose)
    handpkg = hrp5threenm
    robotmesh = hrp5nmesh.Hrp5NMesh(handpkg)
    robotball = hrp5nball.Hrp5NBall()

    robotmeshgen = hrp5nmesh.Hrp5NMesh(handpkg)
    robotmnp = robotmeshgen.genmnp(robot)
    # robotmnp.reparentTo(base.render)
    bcnlist = robotball.genbcnlist(robot)
    robotball.showbcn(base, bcnlist)

    cdchecker = CollisionCheckerBall(robotball)
    cdchecker.isSelfCollided(robot)
    base.run()