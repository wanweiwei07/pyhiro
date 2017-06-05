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

    # waist is always 0
    waistangle = 0
    return waistangle

def jacobian(ur5dualrobot, armid="rgt"):
    """
    compute the jacobian matrix of the targetjoints of rgt or lft arm

    :param ur5dualrobot: see the ur5.ur5dual class
    :param armid: a string indicating "rgt" or "lft"
    :return: armjac a 6-by-x matrix

    author: weiwei
    date: 20161202
    """

    if armid!="rgt" and armid!="lft":
        raise ep.ValueError

    armlj = ur5dualrobot.rgtarm
    if armid == "lft":
        armlj = ur5dualrobot.lftarm

    armjac = np.zeros((6,len(ur5dualrobot.targetjoints)))

    counter = 0
    for i in ur5dualrobot.targetjoints:
        a = np.dot(armlj[i]["rotmat"], armlj[i]["rotax"])
        armjac[:, counter] = np.append(np.cross(a, armlj[ur5dualrobot.targetjoints[-1]]["linkpos"]-armlj[i]["linkpos"]), a)
        counter += 1

    return armjac


def manipulability(ur5dualrobot, armid="rgt"):
    """
    compute the yasukawa manipulability of rgt or lft

    :param hrp5robot: see the hrp5.Hrp5Robot class
    :param armid: a string indicating "rgt" or "lft"
    :return:
    """

    armjac = jacobian(ur5dualrobot, armid)
    return math.sqrt(np.linalg.det(np.dot(armjac, armjac.transpose())))


def tcperror(ur5dualrobot, tgtpos, tgtrot, armid="rgt"):
    """
    compute the error of a specified (rgt or lft) tool point center to its goal

    :param ur5dualrobot: see the ur5.ur5dual class
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

    armlj = ur5dualrobot.rgtarm
    if armid == "lft":
        armlj = ur5dualrobot.lftarm

    deltapos = tgtpos - armlj[ur5dualrobot.targetjoints[-1]]["linkend"]
    deltarot = np.dot(tgtrot, armlj[ur5dualrobot.targetjoints[-1]]["rotmat"].transpose())

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


def numik(ur5dualrobot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik numerically for the specified armid

    :param ur5dualrobot: see the ur5.ur5dual class
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
    armjntssave = ur5dualrobot.getarmjnts(armid)
    armjntsiter = armjntssave.copy()
    for i in range(500):
        armjac = jacobian(ur5dualrobot, armid)
        if np.linalg.matrix_rank(armjac) == 6:
            err = tcperror(ur5dualrobot, tgtpos, tgtrot, armid)
            dq = steplength*(np.linalg.lstsq(armjac, err))[0]
        else:
            print "The Jacobian Matrix of the specified arm is at singularity"
            ur5dualrobot.movearmfk(armjntssave, armid)
            return None
        if np.linalg.norm(err)<1e-4:
            if ur5dualrobot.chkrng(armjntsiter, armid):
                armjntsreturn = ur5dualrobot.getarmjnts(armid)
                ur5dualrobot.movearmfk(armjntssave, armid)
                return armjntsreturn
            else:
                ur5dualrobot.movearmfk(armjntssave, armid)
                return None
        else:
            # todo dq definition
            armjntsiter += dq
            armjntsiter = rm.cvtRngPM360(armjntsiter)
            # print dq
            # print armjntsiter
            if ur5dualrobot.chkrng(armjntsiter, armid) or i < 150:
                # print armjntsiter
                ur5dualrobot.movearmfk(armjntsiter, armid)
                # import ur5dualplot
                # import manipulation.grip.hrp5three.hrp5three as handpkg
                # ur5dualplot.plotstick(base.render, ur5dualrobot)
                # ur5mnp = ur5dualplot.genUr5dualmnp(ur5dualrobot, handpkg)
                # ur5mnp.reparentTo(base.render)
            else:
                ur5dualrobot.movearmfk(armjntssave, armid)
                return None
    return None

def numikr(ur5dualrobot, tgtpos, tgtrot, armid="rgt"):
    """
    solve the ik of the specified arm, waist is included (r means redundant)

    :param ur5dualrobot:
    :param tgtpos: the position of the goal, 1-by-3 numpy ndarray
    :param tgtrot:
    :param armid:
    :return: [waist, shoulder, 1-by-6 armjnts]

    author: weiwei
    date: 20161216, sapporo
    """

    # anglewai means the init angle of waist
    anglewai = ur5dualrobot.getjntwaist()
    anglewa = eubik(tgtpos, armid)
    ur5dualrobot.movewaist(anglewa)
    armjnts = numik(ur5dualrobot, tgtpos, tgtrot, armid)
    ur5dualrobot.movewaist(anglewai)
    if armjnts is None:
        return None
    else:
        return [anglewa, armjnts]

def numikreltcp(ur5dualrobot, deltax, deltay, deltaz, armid = 'rgt'):
    """
    add deltax, deltay, deltaz to the tcp
    tcp is link[-1].linkpos
    since the function is relative, moving link[-1].linkend is the same

    :param deltax: float
    :param deltay:
    :param deltaz:
    :return:

    author: weiwei
    date: 20170412
    """

    armlj = ur5dualrobot.rgtarm
    if armid == "lft":
        armlj = ur5dualrobot.lftarm

    tgtpos = armlj[-1]['linkend']
    tgtrot = armlj[-1]['rotmat']
    newtgtpos = tgtpos + np.array([deltax, deltay, deltaz])
    return numik(ur5dualrobot, newtgtpos, tgtrot, armid)

if __name__=="__main__":
    pos = [300,300,0]
    print eubik(pos)

    try:
        print math.asin(145/np.linalg.norm(pos[0:1]))
    except:
        print "nontriangle"