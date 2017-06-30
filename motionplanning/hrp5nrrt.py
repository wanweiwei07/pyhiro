import collisioncheckerball as cdck
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.hrp5three import hrp5threenm
from motionplanning.rrt import ddrrtconnect as ddrrtc
from robotsim.hrp5n import hrp5n
from robotsim.hrp5n import hrp5nball
from robotsim.hrp5n import hrp5nmesh


def iscollidedfunc(point, obstaclelist = [], robot = None, cdchecker = None):
    """
    check if a specific configuration is in collision

    :param point:
    :param robot the object defined in robotsim/robot
    :param cdchecker: a collisionchecker object
    :return:

    author: weiwei
    date: 20170609
    """

    robot.movearmfk(point)
    iscollided = cdchecker.isCollided(robot, obstaclelist)
    robot.goinitpose()

    if iscollided:
        return True
    else:
        return False


if __name__ == '__main__':
    base = pandactrl.World()

    robot = hrp5n.Hrp5NRobot()
    handpkg = hrp5threenm
    robotmesh = hrp5nmesh.Hrp5NMesh(handpkg)
    robotball = hrp5nball.Hrp5NBall()
    # cdchecker = cdck.CollisionChecker(robotmesh)
    cdchecker = cdck.CollisionCheckerBall(robotball)

    start = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    goal = [0.0,45.0,-20.0,0.0,-150.0,0.0,0.0,0.0,0.0]
    start = [0.0,45.0,-20.0,0.0,-150.0,0.0,0.0,0.0,0.0]
    goal = [0.0,90.0,-60.0,-70.0,-100.0,-30.0,0.0,0.0,0.0]
    start = [0.0,45.0,-20.0,0.0,-150.0,0.0,0.0,0.0,0.0]
    goal = [0.0,-20.0,-60.0,-70.0,-100.0,-30.0,0.0,0.0,0.0]

    jointlimits = [[robot.rgtarm[1]['rngmin'], robot.rgtarm[1]['rngmax']],
                   [robot.rgtarm[2]['rngmin'], robot.rgtarm[2]['rngmax']],
                   [robot.rgtarm[3]['rngmin'], robot.rgtarm[3]['rngmax']],
                   [robot.rgtarm[4]['rngmin'], robot.rgtarm[4]['rngmax']],
                   [robot.rgtarm[5]['rngmin'], robot.rgtarm[5]['rngmax']],
                   [robot.rgtarm[6]['rngmin'], robot.rgtarm[6]['rngmax']],
                   [robot.rgtarm[7]['rngmin'], robot.rgtarm[7]['rngmax']],
                   [robot.rgtarm[8]['rngmin'], robot.rgtarm[8]['rngmax']],
                   [robot.rgtarm[9]['rngmin'], robot.rgtarm[9]['rngmax']]]
    import os
    from panda3d.core import *
    import pandaplotutils.pandageom as pg
    obsrotmat4 = Mat4(1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,405.252044678,-300.073120117,300.0000038147,1.0)
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
        robotmnp = robotmesh.genmnp_nm(robot)
        robotmnp.reparentTo(base.render)

    # robot.goinitpose()
    # robotonscreen = [None]
    # def updateshow(planner, robot, robotmesh, robotball, robotonscreen, task):
    #     if robotonscreen[0] is not None:
    #         robotonscreen[0].detachNode()
    #     output = planner.planningcallback(obstaclelist = [objmnp])
    #     if output == "collided":
    #         robot.movearmfk(planner.newpoint)
    #         robotonscreen[0] = robotmesh.genmnp(robot)
    #         bcnlist = robotball.genbcnlist(robot)
    #         robotball.showbcn(base, bcnlist)
    #         robot.goinitpose()
    #         robotonscreen[0].reparentTo(base.render)
    #         return task.again
    #     if output == "done":
    #         path = planner.getpathcallback()
    #         for pose in path:
    #             robot.movearmfk(pose)
    #             robotmnp_nm = robotmesh.genmnp(robot)
    #             robotmnp_nm.reparentTo(base.render)
    #             robotball.unshowbcn()
    #         return task.done
    #     if output == "continue":
    #         robot.movearmfk(planner.newpoint)
    #         robotonscreen[0] = robotmesh.genmnp(robot)
    #         bcnlist = robotball.genbcnlist(robot)
    #         robotball.showbcn(base, bcnlist)
    #         robot.goinitpose()
    #         robotonscreen[0].reparentTo(base.render)
    #         return task.again
    #
    # taskMgr.doMethodLater(.01, updateshow, "updateshow",
    #                       extraArgs=[planner, robot, robotmesh, robotball, robotonscreen],
    #                       appendTask=True)

    base.run()