import numpy as np

from motionplanning import rrt
from motionplanning import rrtconnect as rrtc
import collisionchecker as cdck
from robotsim.hrp5n import hrp5n
from robotsim.hrp5n import hrp5nplot
from manipulation.grip.hrp5three import hrp5threenm
import pandaplotutils.pandactrl as pandactrl

def iscollidedfunc(point, obstacleList = [], robot = None, robotplot = None, handpkg = None):
    """
    check if a specific configuration is in collision

    :param point:
    :param obstacleList:
    :return:

    author: weiwei
    date: 20170609
    """

    robot.movearmfk(point)
    # isselfcollided
    cdchecker = cdck.CollisionChecker(robotplot)
    isselfcollided = cdchecker.isCollided(robot, handpkg)
    print point
    print isselfcollided

    if isselfcollided:
        return True
    else:
        return False

if __name__ == '__main__':
    base = pandactrl.World()

    robot = hrp5n.Hrp5NRobot()
    handpkg = hrp5threenm
    robotplot = hrp5nplot

    start = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    goal = [0.0,45.0,-20.0,0.0,-150.0,0.0,0.0,0.0,0.0]
    start = [0.0,45.0,-20.0,0.0,-150.0,0.0,0.0,0.0,0.0]
    goal = [0.0,90.0,-60.0,-70.0,-100.0,-30.0,0.0,0.0,0.0]

    jointlimits = [[robot.rgtarm[1]['rngmin'], robot.rgtarm[1]['rngmax']],
                   [robot.rgtarm[2]['rngmin'], robot.rgtarm[2]['rngmax']],
                   [robot.rgtarm[3]['rngmin'], robot.rgtarm[3]['rngmax']],
                   [robot.rgtarm[4]['rngmin'], robot.rgtarm[4]['rngmax']],
                   [robot.rgtarm[5]['rngmin'], robot.rgtarm[5]['rngmax']],
                   [robot.rgtarm[6]['rngmin'], robot.rgtarm[6]['rngmax']],
                   [robot.rgtarm[7]['rngmin'], robot.rgtarm[7]['rngmax']],
                   [robot.rgtarm[8]['rngmin'], robot.rgtarm[8]['rngmax']],
                   [robot.rgtarm[9]['rngmin'], robot.rgtarm[9]['rngmax']]]

    planner = rrt.RRT(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
                      jointlimits = jointlimits, expanddis = 10,
                      robot = robot, robotplot = robotplot, handpkg = handpkg)
    # planner = rrtc.RRTConnect(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
    #           jointlimits = jointlimits, expanddis = 10)

    # import time
    # tic = time.clock()
    # [path, sampledpoints] = planner.planning()
    # toc = time.clock()
    # print toc-tic
    #
    # for pose in path:
    #     robot.movearmfk(pose, armid = 'rgt')
    #     robotmnp = robotplot.genmnp_nm(robot, handpkg)
    #     robotmnp.reparentTo(base.render)

    robot.goinitpose()
    robotonscreen = [None]
    def updateshow(planner, robot, robotplot, robotonscreen, task):
        if robotonscreen[0] is not None:
            robotonscreen[0].detachNode()
        output = planner.planningcallback()
        if output == "collided":
            print planner.newnode
            return task.again
        if output == "done":
            path = planner.getpathcallback()
            for pose in path:
                robot.movearmfk(pose)
                robotmnp_nm = robotplot.genmnp(robot, handpkg)
                robotmnp_nm.reparentTo(base.render)
            return task.done
        if output == "continue":
            robot.movearmfk(planner.newnode)
            robotonscreen[0] = robotplot.genmnp(robot, handpkg)
            robot.goinitpose()
            robotonscreen[0].reparentTo(base.render)
            return task.again

    taskMgr.doMethodLater(.01, updateshow, "updateshow",
                          extraArgs=[planner, robot, robotplot, robotonscreen],
                          appendTask=True)

    base.run()