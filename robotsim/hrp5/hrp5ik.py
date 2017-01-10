import math
import numpy as np
import exceptions as ep


def eurgtbik(pos):
    """
    compute the euristic waist angles for the first three joints?
    ew = euristic waist

    :param pos: object position
    :return: waistangle in degree

    author: weiwei
    date: 20161202
    """

    anglecomponent1 = 0
    try:
        anglecomponent1 = math.asin(145/np.linalg.norm(pos[0:1]))
    except:
        pass
    waistangle = (math.atan2(pos[1], pos[0]) + anglecomponent1)*180/math.pi
    return waistangle


def jacobian(hrp5robot, armid="rgt"):
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

    armlj = hrp5robot.rgtarm
    if armid == "lft":
        armlj = hrp5robot.lftarm

    armjac = np.zeros((6,6))

    counter = 0
    for i in hrp5robot.sixjoints:
        a = np.dot(armlj[i]["rotmat"], armlj[i]["rotax"])
        armjac[:, counter] = np.append(np.cross(a, armlj[hrp5robot.sixjoints[-1]]["linkpos"]-armlj[i]["linkpos"]), a)
        counter += 1

    return armjac


def manipulability(hrp5robot, armid="rgt"):
    """
    compute the yasukawa manipulability of rgt or lft arm (lst 6-dof)

    :param hrp5robot: see the hrp5.Hrp5Robot class
    :param armid: a string indicating "rgt" or "lft"
    :return:
    """

    armjac = jacobian(hrp5robot, armid)
    return math.sqrt(np.linalg.det(np.dot(armjac, armjac.transpose())))


def tcperror(hrp5robot, tgtpos, tgtrot, armid="rgt"):
    """
    compute the error of a specified (rgt or lft) tool point center to its goal

    :param hrp5robot: see the hrp5robot.Hrp5Robot class
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

    armlj = hrp5robot.rgtarm
    if armid == "lft":
        armlj = hrp5robot.lftarm

    deltapos = tgtpos - armlj[hrp5robot.sixjoints[-1]]["linkend"]
    deltarot = np.dot(tgtrot, armlj[hrp5robot.sixjoints[-1]]["rotmat"].transpose())

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


def numik(hrp5robot, tgtpos, tgtrot, armid="rgt"):
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
    steplength = 30
    armjntssave = hrp5robot.getarmjnts6(armid)
    print armjntssave
    armjntsiter = armjntssave.copy()
    for i in range(500):
        armjac = jacobian(hrp5robot, armid)
        if abs(np.linalg.det(armjac))>1e-6:
            err = tcperror(hrp5robot, tgtpos, tgtrot, armid)
            dq = steplength*(np.linalg.solve(armjac, err))
        else:
            print "The Jacobian Matrix of the specified arm is at singularity"
            return None
        if np.linalg.norm(err)<1e-4:
            armjntsreturn = hrp5robot.getarmjnts6(armid)
            hrp5robot.movearmfk6(armjntssave, armid)
            return armjntsreturn
        else:
            # todo dq definition
            armjntsiter += dq
            # the robot may encounter overrange errors in the first few iterations
            # use i<50 to avoid these errors
            if hrp5robot.chkrng6(armjntsiter) or i < 50:
                # print armjntsiter
                hrp5robot.movearmfk6(armjntsiter, armid)
                # import hrp5plot
                # hrp5plot.plotstick(base.render, hrp5robot)
                # hrp5mnp = hrp5plot.genHrp5mnp(hrp5robot)
                # hrp5mnp.reparentTo(base.render)
                # nxtmnp = nxtplot.genNxtmnp(nxtrobot)
                # nxtmnp.reparentTo(base.render)
            else:
                import hrp5plot
                hrp5plot.plotstick(base.render, hrp5robot)
                return None


if __name__=="__main__":
    pos = [300,300,0]
    print eurgtbik(pos)

    try:
        print math.asin(145/np.linalg.norm(pos[0:1]))
    except:
        print "nontriangle"