import math
import numpy as np
import exceptions as ep
import utils.robotmath as rm

def eubik(pos, armid="rgt"):
    """
    compute the euristic waist rotation
    ew = euristic waist

    :param pos: object position
    :return: waistangle in degree

    author: weiwei
    date: 20170410
    """

    anglecomponent1 = 0
    try:
        anglecomponent1 = math.asin(80/np.linalg.norm(pos[0:2]))
    except:
        pass
    waistangle = (math.atan2(pos[1], pos[0]) + anglecomponent1)*180/math.pi
    if armid=="lft":
        waistangle = 180.0-2*math.atan2(pos[0], pos[1])*180.0/math.pi-waistangle
    return waistangle

def jacobian(hrp5nrobot, armid="rgt"):
    """
    compute the jacobian matrix of the last 6 dof for rgt or lft arm

    :param hrp5robot: see the hrp5.Hrp5Robot class
    :param armid: a string indicating "rgt" or "lft"
    :return: armjac a 6-by-6 ndarray

    author: weiwei
    date: 20161202
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    armlj = hrp5nrobot.rgtarm
    if armid == "lft":
        armlj = hrp5nrobot.lftarm

    armjac = np.zeros((6,len(hrp5nrobot.targetjoints)))

    counter = 0
    for i in hrp5nrobot.targetjoints:
        a = np.dot(armlj[i]["rotmat"], armlj[i]["rotax"])
        armjac[:, counter] = np.append(np.cross(a, armlj[hrp5nrobot.targetjoints[-1]]["linkpos"]-armlj[i]["linkpos"]), a)
        counter += 1

    return armjac


def manipulability(hrp5nrobot, armid="rgt"):
    """
    compute the yasukawa manipulability of rgt or lft arm (lst 6-dof)

    :param hrp5robot: see the hrp5.Hrp5Robot class
    :param armid: a string indicating "rgt" or "lft"
    :return:
    """

    armjac = jacobian(hrp5nrobot, armid)
    return math.sqrt(np.linalg.det(np.dot(armjac, armjac.transpose())))


def tcperror(hrp5nrobot, tgtpos, tgtrot, armid="rgt"):
    """
    compute the error of a specified (rgt or lft) tool point center to its goal

    :param hrp5nrobot: see the hrp5robot.Hrp5Robot class
    :param armid: a string indicating "rgt" or "lft"
    :param tgtpos: the position of the goal
    :param tgtrot: the rotation of the goal
    :return: a 1-by-6 vector where the first three indicates the displacement in pos,
                the second three indictes the displacement in rot

    author: weiwei
    date: 20161205
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    armlj = hrp5nrobot.rgtarm
    if armid == "lft":
        armlj = hrp5nrobot.lftarm

    deltapos = tgtpos - armlj[hrp5nrobot.targetjoints[-1]]["linkend"]
    deltarot = np.dot(tgtrot, armlj[hrp5nrobot.targetjoints[-1]]["rotmat"].transpose())

    anglesum = np.trace(deltarot)
    if anglesum is 3:
        deltaw = np.array([0,0,0])
    else:
        nominator = anglesum - 1
        if nominator > 2:
            nominator = 2
        if nominator < -2:
            nominator = -2
        theta = math.acos(nominator / 2.0)
        if theta == 0:
            deltaw = np.array([0, 0, 0])
        else:
            deltaw = (theta / (2 * math.sin(theta))) * (np.array([deltarot[2, 1] - deltarot[1, 2], \
                                                                  deltarot[0, 2] - deltarot[2, 0], \
                                                                  deltarot[1, 0] - deltarot[0, 1]]))

    return np.append(deltapos, deltaw)


def numik(hrp5nrobot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik numerically for the specified armid

    :param hrp5robot: see hrp5robot.Hrp5Robot class
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot: the orientation of the goal, 3-by-3 numpyndarray
    :param armid: a string "rgt" or "lft" indicating the arm that will be solved
    :return: armjnts: a 1-by-6 numpy ndarray

    author: weiwei
    date: 20161205
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    # stablizer
    steplength = 5
    steplengthinc = 10
    armjntssave = hrp5nrobot.getarmjnts(armid)
    armjntsiter = armjntssave.copy()
    errnormlast = 0.0
    nlocalencountered = 0
    for i in range(100):
        armjac = jacobian(hrp5nrobot, armid)
        if np.linalg.matrix_rank(armjac) == 6:
            err = tcperror(hrp5nrobot, tgtpos, tgtrot, armid)
            dq = steplength * (np.linalg.lstsq(armjac, err))[0]
        else:
            print "The Jacobian Matrix of the specified arm is at singularity"
            # hrp5nrobot.movearmfk(armjntssave, armid)
            # return None
            break
        print np.linalg.norm(err)
        # if np.linalg.norm(err)<1e-4:
        errnorm = np.linalg.norm(err)
        if errnorm < 1:
            print 'goal reached', armjntsiter
            print "number of iteration ", i
            # if hrp5nrobot.chkrng(armjntsiter, armid):
            armjntsreturn = hrp5nrobot.getarmjnts(armid)
            hrp5nrobot.movearmfk(armjntssave, armid)
            return armjntsreturn
            # else:
            #     # import hrp5nplot
            #     # import manipulation.grip.hrp5three.hrp5three as handpkg
            #     # # hrp5nplot.plotstick(base.render, hrp5nrobot)
            #     # hrp5nmnp = hrp5nplot.genmnp(hrp5nrobot, handpkg)
            #     # hrp5nmnp.reparentTo(base.render)
            #     # print "out of range"
            #     # hrp5nrobot.movearmfk(armjntssave, armid)
            #     # return None
            #     break
        else:
            # todo dq definition
            # judge local minima
            if abs(errnorm - errnormlast) < 1e-3:
                nlocalencountered += 1
                print "local minima at iteration", i
                print "n local encountered", nlocalencountered
                steplength = 3
                steplengthinc = 7
                if nlocalencountered > 2:
                    break
            else:
                if steplength < 50:
                    steplength = steplength + steplengthinc
            armjntsiter += dq
            armjntsiter = rm.cvtRngPM180(armjntsiter)
            # print armjntsiter
            # for i in range(armjntsiter.shape[0]):
            #     armjntsiter[i] = armjntsiter[i]%360
            # print armjntsiter
            # the robot may encounter overrange errors in the first few iterations
            # use i<50 to avoid these errors
            # if hrp5nrobot.chkrng(armjntsiter, armid) or i < 30:
            #     # print armjntsiter
            #     hrp5nrobot.movearmfk(armjntsiter, armid)
            #     # import hrp5plot
            #     # hrp5plot.plotstick(base.render, hrp5robot)
            #     # hrp5mnp = hrp5plot.genHrp5mnp(hrp5robot)
            #     # hrp5mnp.reparentTo(base.render)
            #     # nxtmnp = nxtplot.genNxtmnp(nxtrobot)
            #     # nxtmnp.reparentTo(base.render)
            #     # import hrp5nplot
            #     # import manipulation.grip.hrp5three.hrp5three as handpkg
            #     # # hrp5nplot.plotstick(base.render, hrp5nrobot)
            #     # hrp5nmnp = hrp5nplot.genHrp5Nmnp_nm(hrp5nrobot, handpkg)
            #     # hrp5nmnp.reparentTo(base.render)
            # else:
            #     # import hrp5plot
            #     # hrp5plot.plotstick(base.render, hrp5robot)
            #     hrp5nrobot.movearmfk(armjntssave, armid)
            #     return None
            bdragged, jntangles = hrp5nrobot.chkrngdrag(armjntsiter, armid)
            armjntsiter[:] = jntangles[:]
            print jntangles
            hrp5nrobot.movearmfk(jntangles, armid)
            import hrp5nplot
            hrp5nplot.plotstick(base.render, hrp5nrobot)
        errnormlast = errnorm
        print errnorm
    import manipulation.grip.hrp5three.hrp5three as handpkg
    hrp5nmnp = hrp5nplot.genmnp(hrp5nrobot, handpkg)
    hrp5nmnp.reparentTo(base.render)
    hrp5nrobot.movearmfk(armjntssave, armid)
    return None

def numikr(hrp5nrobot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik of the specified arm, waist is included (r means redundant)

    :param hrp5robot:
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot:
    :param armid:
    :return: [waist, shoulder, 1-by-6 armjnts]

    author: weiwei
    date: 20161216, sapporo
    """

    # anglewai means the init angle of waist
    anglewai = hrp5nrobot.getjntwaist()
    anglewa = eubik(tgtpos, armid)
    hrp5nrobot.movewaist(anglewa)
    armjnts = numik(hrp5nrobot, tgtpos, tgtrot, armid)
    hrp5nrobot.movewaist(anglewai)
    if armjnts is None:
        return None
    else:
        return [anglewa, armjnts]

if __name__=="__main__":
    pos = [300,300,0]
    print eubik(pos)

    try:
        print math.asin(145/np.linalg.norm(pos[0:1]))
    except:
        print "nontriangle"