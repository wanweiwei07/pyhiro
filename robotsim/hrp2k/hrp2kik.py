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

def jacobian(hrp2krobot, armid="rgt"):
    """
    compute the jacobian matrix of the last 6 dof for rgt or lft arm

    :param hrp2krobot: see the hrp2k.Hrp2KRobot class
    :param armid: a string indicating "rgt" or "lft"
    :return: armjac a 6-by-6 ndarray

    author: weiwei
    date: 20170626
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    armlj = hrp2krobot.rgtarm
    if armid == "lft":
        armlj = hrp2krobot.lftarm

    armjac = np.zeros((6,len(hrp2krobot.targetjoints)))

    counter = 0
    for i in hrp2krobot.targetjoints:
        a = np.dot(armlj[i]["rotmat"], armlj[i]["rotax"])
        armjac[:, counter] = np.append(np.cross(a, armlj[hrp2krobot.targetjoints[-1]]["linkpos"]-armlj[i]["linkpos"]), a)
        counter += 1

    return armjac


def manipulability(hrp2krobot, armid="rgt"):
    """
    compute the yasukawa manipulability of rgt or lft arm (lst 6-dof)

    :param hrp2krobot: see the hrp2.Hrp2KRobot class
    :param armid: a string indicating "rgt" or "lft"
    :return:
    """

    armjac = jacobian(hrp2krobot, armid)
    return math.sqrt(np.linalg.det(np.dot(armjac, armjac.transpose())))


def tcperror(hrp2krobot, tgtpos, tgtrot, armid="rgt"):
    """
    compute the error of a specified (rgt or lft) tool point center to its goal

    :param hrp2krobot: see the hrp2krobot.Hrp2KRobot class
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

    armlj = hrp2krobot.rgtarm
    if armid == "lft":
        armlj = hrp2krobot.lftarm

    deltapos = tgtpos - armlj[hrp2krobot.targetjoints[-1]]["linkend"]
    deltarot = np.dot(tgtrot, armlj[hrp2krobot.targetjoints[-1]]["rotmat"].transpose())

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

    # print tgtpos, armlj[hrp2krobot.targetjoints[-1]]["linkend"], deltapos, deltarot
    return np.append(deltapos, deltaw)


def numik(hrp2krobot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik numerically for the specified armid

    :param hrp2krobot: see hrp2krobot.Hrp2KRobot class
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
    armjntssave = hrp2krobot.getarmjnts(armid)
    armjntsiter = armjntssave.copy()
    for i in range(500):
        armjac = jacobian(hrp2krobot, armid)
        if np.linalg.matrix_rank(armjac) == 6:
            err = tcperror(hrp2krobot, tgtpos, tgtrot, armid)
            dq = steplength*(np.linalg.lstsq(armjac, err))[0]
        else:
            print "The Jacobian Matrix of the specified arm is at singularity"
            hrp2krobot.movearmfk(armjntssave, armid)
            return None
        if np.linalg.norm(err)<1e-4:
            if hrp2krobot.chkrng(armjntsiter, armid):
                armjntsreturn = hrp2krobot.getarmjnts(armid)
                hrp2krobot.movearmfk(armjntssave, armid)
                return armjntsreturn
            else:
                hrp2krobot.movearmfk(armjntssave, armid)
                return None
        else:
            # todo dq definition
            armjntsiter += dq
            armjntsiter = rm.cvtRngPM180(armjntsiter)
            if hrp2krobot.chkrng(armjntsiter, armid) or i < 30:
                hrp2krobot.movearmfk(armjntsiter, armid)
                # import hrp2kplot
                # from manipulation.grip.robotiq85 import rtq85nm
                # handpkg = rtq85nm
                # hrp2kplot.plotstick(base.render, hrp2krobot)
                # print hrp2krobot.rgtarm[hrp2krobot.targetjoints[-1]]['linkend']
                # hrp2kmnp_nm = hrp2kplot.genmnp_nm(hrp2krobot, handpkg)
                # hrp2kmnp_nm.setColor(.7,.2,0.7,.1)
                # hrp2kmnp_nm.reparentTo(base.render)
            else:
                hrp2krobot.movearmfk(armjntssave, armid)
                return None
    hrp2krobot.movearmfk(armjntssave, armid)
    return None

def numikr(hrp2krobot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik of the specified arm, waist is included (r means redundant)

    :param hrp2krobot:
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot:
    :param armid:
    :return: [waist, shoulder, 1-by-6 armjnts]

    author: weiwei
    date: 20161216, sapporo
    """

    anglewi = hrp2krobot.getjntwaist()
    armjntb = eubik(tgtpos, armid)
    hrp2krobot.movewaist(armjntb)
    armjnts6 = numik(hrp2krobot, tgtpos, tgtrot, armid)
    if armjnts6 is None:
        hrp2krobot.movewaist(anglewi)
        return None
    else:
        hrp2krobot.movewaist(anglewi)
        armjnts7 = [armjntb, armjnts6]
        return armjnts7


if __name__=="__main__":
    pos = [300,300,0]
    print eubik(pos)

    try:
        print math.asin(145/np.linalg.norm(pos[0:1]))
    except:
        print "nontriangle"