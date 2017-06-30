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
    :param obstaclelist: a list of obstacle nodepath
    :return:

    author: weiwei
    date: 20170609
    """

    robot.movearmfk(point[0:9], armid = 'rgt')
    robot.movearmfk(point[9:18], armid = 'lft')
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
    cdchecker = cdck.CollisionCheckerBall(robotball)

    # start = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    # goal = [0.0,45.0,-20.0,0.0,-150.0,0.0,0.0,0.0,0.0]
    start = [0.0,45.0,-20.0,0.0,-150.0,0.0,0.0,0.0,0.0, 0.0,45.0,20.0,0.0,-150.0,0.0,0.0,0.0,0.0]
    goal = [0.0,-20.0,-60.0,-70.0,-100.0,-30.0,0.0,0.0,0.0, 0.0,-20.0,60.0,70.0,-100.0,30.0,0.0,0.0,0.0]

    jointlimits = [[robot.rgtarm[1]['rngmin'], robot.rgtarm[1]['rngmax']],
                   [robot.rgtarm[2]['rngmin'], robot.rgtarm[2]['rngmax']],
                   [robot.rgtarm[3]['rngmin'], robot.rgtarm[3]['rngmax']],
                   [robot.rgtarm[4]['rngmin'], robot.rgtarm[4]['rngmax']],
                   [robot.rgtarm[5]['rngmin'], robot.rgtarm[5]['rngmax']],
                   [robot.rgtarm[6]['rngmin'], robot.rgtarm[6]['rngmax']],
                   [robot.rgtarm[7]['rngmin'], robot.rgtarm[7]['rngmax']],
                   [robot.rgtarm[8]['rngmin'], robot.rgtarm[8]['rngmax']],
                   [robot.rgtarm[9]['rngmin'], robot.rgtarm[9]['rngmax']],
                   [robot.lftarm[1]['rngmin'], robot.lftarm[1]['rngmax']],
                   [robot.lftarm[2]['rngmin'], robot.lftarm[2]['rngmax']],
                   [robot.lftarm[3]['rngmin'], robot.lftarm[3]['rngmax']],
                   [robot.lftarm[4]['rngmin'], robot.lftarm[4]['rngmax']],
                   [robot.lftarm[5]['rngmin'], robot.lftarm[5]['rngmax']],
                   [robot.lftarm[6]['rngmin'], robot.lftarm[6]['rngmax']],
                   [robot.lftarm[7]['rngmin'], robot.lftarm[7]['rngmax']],
                   [robot.lftarm[8]['rngmin'], robot.lftarm[8]['rngmax']],
                   [robot.lftarm[9]['rngmin'], robot.lftarm[9]['rngmax']]]

    import os
    from panda3d.core import *
    import pandaplotutils.pandageom as pg
    obsrotmat4 = Mat4(1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,305.252044678,-400.073120117,165.0000038147,1.0)
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "manipulation/grip", "objects", "tool.stl")
    objmnp = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    objmnp.setMat(obsrotmat4)
    objmnp.reparentTo(base.render)

    # planner = rrt.RRT(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
    #                   jointlimits = jointlimits, expanddis = 10, robot = robot,
    #                   cdchecker = cdchecker)
    # planner = ddrrt.DDRRT(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
    #                   jointlimits = jointlimits, goalsamplerate=30, expanddis = 10, robot = robot,
    #                   cdchecker = cdchecker)
    #
    # robotonscreen = [None]
    # def updateshow(planner, robot, robotmesh, robotonscreen, task):
    #     if robotonscreen[0] is not None:
    #         robotonscreen[0].detachNode()
    #     output = planner.planningcallback(obstaclelist = [objmnp])
    #     if output == "collided":
    #         # print planner.randnode
    #         robot.movearmfk(planner.newpoint[0:9], armid = 'rgt')
    #         robot.movearmfk(planner.newpoint[9:18], armid = 'lft')
    #         robotonscreen[0] = robotmesh.genmnp(robot)
    #         robot.goinitpose()
    #         robotonscreen[0].reparentTo(base.render)
    #         return task.again
    #     if output == "done":
    #         path = planner.getpathcallback()
    #         for pose in [path[1], path[len(path)-1]]:
    #             robot.movearmfk(pose[0:9], armid = 'rgt')
    #             robot.movearmfk(pose[9:18], armid = 'lft')
    #             robotmnp_nm = robotmesh.genmnp(robot)
    #             robotmnp_nm.reparentTo(base.render)
    #         for pose in path[1:len(path)-1]:
    #             robot.movearmfk(pose[0:9], armid = 'rgt')
    #             robot.movearmfk(pose[9:18], armid = 'lft')
    #             robotmnp_nm = robotmesh.genmnp_nm(robot)
    #             robotmnp_nm.reparentTo(base.render)
    #         return task.done
    #     if output == "continue":
    #         robot.movearmfk(planner.newpoint[0:9], armid = 'rgt')
    #         robot.movearmfk(planner.newpoint[9:18], armid = 'lft')
    #         robotonscreen[0] = robotmesh.genmnp(robot)
    #         robot.goinitpose()
    #         robotonscreen[0].reparentTo(base.render)
    #         return task.again
    #
    # taskMgr.doMethodLater(.01, updateshow, "updateshow",
    #                       extraArgs=[planner, robot, robotmesh, robotonscreen],
    #                       appendTask=True)

    # planner = rrtc.RRTConnect(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
    #                           jointlimits = jointlimits, starttreesamplerate=30, expanddis = 10, robot = robot,
    #                           cdchecker = cdchecker)

    planner = ddrrtc.DDRRTConnect(start=start, goal=goal, iscollidedfunc = iscollidedfunc,
                              jointlimits = jointlimits, starttreesamplerate=30, expanddis = 20, robot = robot,
                              cdchecker = cdchecker)

    import time
    tic = time.clock()
    [path, sampledpoints] = planner.planning(obstaclelist = [objmnp])
    toc = time.clock()
    print toc-tic

    # for pose in path:
    #     robot.movearmfk(pose[0:9], armid = 'rgt')
    #     robot.movearmfk(pose[9:18], armid = 'lft')
    #     robotmnp = robotmesh.genmnp_nm(robot)
    #     robotmnp.reparentTo(base.render)

    import copy
    sampledpointsindex = [0]
    robotonscreen = [None]
    def updateshow(path, sampledpoints, sampledpointsindex,
                   robot, robotmesh, task):
        if sampledpointsindex[0] <= len(sampledpoints)-1:
            if robotonscreen[0] is not None:
                robotonscreen[0].detachNode()
            # print sampledpoints
            print sampledpointsindex[0], len(sampledpoints)
            robot.movearmfk(sampledpoints[sampledpointsindex[0]][0][0:9], armid = 'rgt')
            robot.movearmfk(sampledpoints[sampledpointsindex[0]][0][9:18], armid = 'lft')
            robotonscreen[0] = robotmesh.genmnp(robot)
            robotonscreen[0].reparentTo(base.render)
            robot.goinitpose()
            sampledpointsindex[0] += 1
            return task.again
        else:
            for point in path:
                robot.movearmfk(point[0:9], armid = 'rgt')
                robot.movearmfk(point[9:18], armid = 'lft')
                robotmnp = copy.deepcopy(robotmesh.genmnp(robot))
                robot.goinitpose()
                robotmnp.reparentTo(base.render)
            return task.done

    taskMgr.add(updateshow, "updateshow",
                          extraArgs=[path, sampledpoints, sampledpointsindex,
                                     robot, robotmesh],
                          appendTask=True)

    base.run()