import motionplanning.collisionchecker as cdck
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.robotiq85 import rtq85nm
from motionplanning import rrt
from robotsim.nextage import nxt
from robotsim.nextage import nxtplot


def iscollidedfunc(point, obstaclelist = [], robot = None, robotplot = None, handpkg = None):
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
    # print point
    # print isselfcollided

    if isselfcollided:
        return True
    else:
        return False

def genmotionsequencearm(robot, robotplot, handpkg, start, goal, obstacleslist = []):
    """
    compute a sequence of robot poses

    :param robot:
    :param start:
    :param goal:
    :param obstacleslist:
    :return:
    """

    jointlimits = []
    for i in robot.targetjoints:
        jointlimits.append([robot.rgtarm[i]['rngmin'], robot.rgtarm[i]['rngmax']])
    planner = rrt.RRT(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
                      jointlimits = jointlimits, expanddis = 20, robot = robot,
                      robotplot = robotplot, handpkg = handpkg)
    import time
    tic = time.clock()
    [path, sampledpoints] = planner.planning()
    toc = time.clock()
    print toc-tic
    return [path, sampledpoints]

def genmotionsequencewaistarm(robot, robotplot, handpkg, start, goal, obstacleslist = []):
    """
    compute a sequence of robot poses

    :param robot:
    :param start:
    :param goal:
    :param obstacleslist:
    :return:
    """

    # the waist range is always
    jointlimits = [[-90.0, 90.0]]
    for i in robot.targetjoints:
        jointlimits.append([robot.rgtarm[i]['rngmin'], robot.rgtarm[i]['rngmax']])
    planner = rrt.RRT(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
                      jointlimits = jointlimits, expanddis = 20, robot = robot,
                      robotplot = robotplot, handpkg = handpkg)
    import time
    tic = time.clock()
    [path, sampledpoints] = planner.planning()
    toc = time.clock()
    print toc-tic
    return [path, sampledpoints]

if __name__ == '__main__':
    base = pandactrl.World()

    robot = nxt.NxtRobot()
    handpkg = rtq85nm
    robotplot = nxtplot

    start = [-15.0, 0.0, -143.0, 0.0, 0.0, 0.0]
    goal = [12.37707499, -61.8859855, -40.14386727, -76.90376033, 7.75123072, 75.05246743]

    [path, sampledpoints] = genmotionsequencearm(robot, robotplot, handpkg, start, goal)

    sampledpointsindex = [0]
    pathindex = [0]
    robotonscreen = [None]
    def updateshow(path, pathindex, sampledpoints, sampledpointsindex,
                   robot, robotplot, robotonscreen, task):
        if sampledpointsindex[0] <= len(sampledpoints)-1:
            if robotonscreen[0] is not None:
                robotonscreen[0].detachNode()
            print sampledpoints
            # fkrjnts = [sampledpoints[sampledpointsindex[0]][0][0], sampledpoints[sampledpointsindex[0]][0][1:]]
            # robot.movearmfkr(fkrjnts)
            robot.movearmfk(sampledpoints[sampledpointsindex[0]][0])
            robotonscreen[0] = robotplot.genmnp(robot, handpkg)
            robot.goinitpose()
            robotonscreen[0].reparentTo(base.render)
            sampledpointsindex[0] += 1
            return task.again
        else:
            for point in path:
                # robot.movearmfkr([point[0], point[1:]])
                robot.movearmfk(point)
                robotmnp = robotplot.genmnp(robot, handpkg)
                robot.goinitpose()
                robotmnp.reparentTo(base.render)
            return task.done

    taskMgr.doMethodLater(.01, updateshow, "updateshow",
                          extraArgs=[path, pathindex, sampledpoints, sampledpointsindex,
                                     robot, robotplot, robotonscreen],
                          appendTask=True)

    base.run()