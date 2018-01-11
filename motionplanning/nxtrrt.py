import motionplanning.collisioncheckerball as cdck
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.robotiq85 import rtq85nm
from motionplanning.rrt import ddrrtconnect as ddrrtc
from robotsim.nextage import nxt
from robotsim.nextage import nxtmesh
from robotsim.nextage import nxtball

def iscollidedfunc(point, obstaclelist = [], robot = None, cdchecker = None):
    """
    check if a specific configuration is in collision

    :param point:
    :param robot the object defined in robotsim/robot
    :param cdchecker: a collisionchecker object
    :return:

    author: weiwei
    date: 20180109
    """

    robot.movearmfk(point)
    isselfcollided = cdchecker.isCollided(robot, obstaclelist)
    robot.goinitpose()

    if isselfcollided:
        return True
    else:
        return False

if __name__ == '__main__':
    import robotsim.nextage.nxtplot as nxtplot

    base = pandactrl.World()

    robot = nxt.NxtRobot()
    handpkg = rtq85nm
    robotmesh = nxtmesh.NxtMesh(handpkg)
    robotball = nxtball.NxtBall()
    # cdchecker = cdck.CollisionChecker(robotmesh)
    cdchecker = cdck.CollisionCheckerBall(robotball)

    start = [50.0,0.0,-143.0,0.0,0.0,0.0]
    goal = [-15.0,0.0,-143.0,0.0,0.0,0.0]
    # plot init and goal
    robot.movearmfk(armjnts = start, armid = 'rgt')
    robotmesh.genmnp(robot).reparentTo(base.render)
    robot.movearmfk(armjnts = goal, armid = 'rgt')
    robotmesh.genmnp(robot).reparentTo(base.render)

    jointlimits = [[robot.rgtarm[1]['rngmin'], robot.rgtarm[1]['rngmax']],
                   [robot.rgtarm[2]['rngmin'], robot.rgtarm[2]['rngmax']],
                   [robot.rgtarm[3]['rngmin'], robot.rgtarm[3]['rngmax']],
                   [robot.rgtarm[4]['rngmin'], robot.rgtarm[4]['rngmax']],
                   [robot.rgtarm[5]['rngmin'], robot.rgtarm[5]['rngmax']],
                   [robot.rgtarm[6]['rngmin'], robot.rgtarm[6]['rngmax']]]
    import os
    from panda3d.core import *
    import pandaplotutils.pandageom as pg
    obsrotmat4 = Mat4(1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,355.252044678,-150.073120117,200.0000038147,1.0)
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "manipulation/grip", "objects", "tool.stl")
    objmnp = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    objmnp.setMat(obsrotmat4)
    objmnp.reparentTo(base.render)

    # planner = rrt.RRT(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
    #                   jointlimits = jointlimits, expanddis = 5,
    #                   robot = robot, cdchecker = cdchecker)
    # planner = rrtc.RRTConnect(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
    #           jointlimits = jointlimits, expanddis = 10, robot = robot, cdchecker = cdchecker)
    # planner = ddrrt.DDRRT(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
    #                   jointlimits = jointlimits, goalsamplerate=30, expanddis = 5, robot = robot,
    #                   cdchecker = cdchecker)
    #
    planner = ddrrtc.DDRRTConnect(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
                              jointlimits = jointlimits, starttreesamplerate=30, expanddis = 5, robot = robot,
                              cdchecker = cdchecker)

    import time
    tic = time.clock()
    [path, sampledpoints] = planner.planning(obstaclelist = [objmnp])
    toc = time.clock()
    print toc-tic
    #
    for pose in path:
        robot.movearmfk(pose, armid = 'rgt')
        robotstick = robotmesh.gensnp(nxtrobot = robot)
        robotstick.reparentTo(base.render)
    base.run()