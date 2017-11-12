import numpy as np
import exceptions as ep
import utils.robotmath as rm
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
import ur5dualik

class Ur5DualRobot():
    def __init__(self):
        # initialize the UR5 dual-arm robot
        self.__name = 'ur5dual'
        # initjnts has 15 elements where the first three are for the waist and head
        # the first three are dummy since the realrobot does not have those joints
        # after that, the first six are for the right arm
        # the remaining 6 are for the left arm
        # self.__initjnts = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
        self.__initjnts = np.array([0,0,0,-30,-90,120,-145,-90,0,30,-90,-120,-45,90,0]);
        self.__rgtarm = self.__initrgtlj()
        self.__lftarm = self.__initlftlj()
        self.__targetjoints = [1,2,3,4,5,6]
        self.__base = self.__rgtarm[0]
        self.goinitpose()

    @property
    def name(self):
        # read-only property
        return self.__name

    @property
    def initjnts(self):
        # read-only property
        return self.__initjnts

    @property
    def rgtarm(self):
        # read-only property
        return self.__rgtarm

    @property
    def lftarm(self):
        # read-only property
        return self.__lftarm

    @property
    def base(self):
        # read-only property
        return self.__base

    @property
    def sixjoints(self):
        # read-only property
        return self.__sixjoints

    @property
    def targetjoints(self):
        # read-only property
        return self.__targetjoints

    def gettcpframe(self, armid = 'rgt'):
        """
        get the local frame of tcp in Mat4 format

        :param armid:
        author: weiwei
        date: 20170412
        """

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        return pg.cvtMat4(armlj[-1]['rotmat'], armlj[-1]['linkpos'])

    def movewaist(self, rotangle=0):
        """
        rotate the base of the robot

        :param rotangle: in degree
        :return: null

        author: weiwei
        date: 20170410
        """

        # setting right and left 0 is for compatibility
        # # right arm
        # self.rgtarm[0]['rotangle'] = rotangle
        # self.rgtarm[0]['rotmat'] = rm.rodrigues(self.rgtarm[0]['rotax'], self.rgtarm[0]['rotangle'])
        # self.rgtarm[0]['linkend'] = np.squeeze(np.dot(self.rgtarm[0]['rotmat'], self.rgtarm[0]['linkvec'].reshape((-1,))))+self.rgtarm[0]['linkpos']
        #
        # # left arm
        # self.lftarm[0]['rotangle'] = rotangle
        # self.lftarm[0]['rotmat'] = rm.rodrigues(self.lftarm[0]['rotax'], self.lftarm[0]['rotangle'])
        # self.lftarm[0]['linkend'] = np.squeeze(np.dot(self.lftarm[0]['rotmat'], self.lftarm[0]['linkvec'].reshape((-1,))))+self.lftarm[0]['linkpos']

        self.__updatefk(self.rgtarm)
        self.__updatefk(self.lftarm)

    def movearmfk(self, armjnts, armid="rgt"):
        """
        move the joints of armlj specified by targetjoints using forward kinematics, waist is not included

        :param armjnts: a 1-by-6 ndarray where each element indicates the angle of a joint (in degree)
        :param armid: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
        :return: null

        author: weiwei
        date: 20161205
        """

        if armid != "rgt" and armid != "lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            armlj[i]['rotangle'] = armjnts[counter]
            counter += 1
        self.__updatefk(armlj)

    def movearmfkr(self, armjnts, armid ="rgt"):
        """
        move the redundant joints of armlj using forward kinematics

        :param armjnts: [waistrot, shoulderrot, armjnts6] all in angle
        :param armid: a string indicating the rgtlj or lftlj robot structure, could "lft" or "rgt"
        :return: null

        author: weiwei
        date: 20170112
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            armlj[i]['rotangle'] = armjnts[1][counter]
            counter += 1

        self.movewaist(armjnts[0])

    def movealljnts(self, robotjnts):
        """
        move all joints of the robot

        :param robotjnts: the same definition as self.initjntss
        :return: null

        author: weiwei
        date: 20170411
        """

        # right arm
        i = 1
        while i != -1:
            self.rgtarm[i]['rotangle'] = robotjnts[i+2]
            i = self.rgtarm[i]['child']
        # left arm
        i = 1
        while i != -1:
            self.lftarm[i]['rotangle'] = robotjnts[i+8]
            i = self.lftarm[i]['child']

        self.__updatefk(self.rgtarm)
        self.__updatefk(self.lftarm)

    def goinitpose(self):
        """
        move the robot to initial pose

        :return: null

        author: weiwei
        date: 20161202, tsukuba
        """

        self.movealljnts(self.initjnts)

    def gettcp(self, armid = 'rgt'):
        """
        get the tcppos, and tcprot of the speficied armid

        :return: [tcppos, tcprot] in nparray

        author: weiwei
        date: 20170412
        """

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        tcppos = armlj[-1]['linkend']
        tcprot = armlj[-1]['rotmat']
        return [tcppos, tcprot]

    def getarmjnts(self, armid="rgt"):
        """
        get the target joints of the specified armid

        :param armid:
        :return: armjnts: a 1-by-x numpy ndarray

        author: weiwei
        date: 20161205, tsukuba
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        armjnts = np.zeros(len(self.__targetjoints))
        counter = 0
        for i in self.__targetjoints:
            armjnts[counter] = armlj[i]['rotangle']
            counter += 1

        return armjnts

    def getjntwaist(self):
        """
        get the rot angle of robot waist

        :return: waistangle in degree

         author: weiwei
         date: 20170112
        """

        return self.base['rotangle']

    def chkrng(self, armjnts, armid="rgt"):
        """
        check if the given armjnts is inside the oeprating range of the speificed armid
        this function doesn't check the waist

        :param armjnts: a 1-by-x numpy ndarray indicating the targejoints of a manipulator
        :param armid: a string "rgt" or "lft"
        :return: True or False indicating inside the range or not

        author: weiwei
        date: 20161205
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        for i in self.__targetjoints:
            if armjnts[counter] < armlj[i]["rngmin"] or armjnts[counter] > armlj[i]["rngmax"]:
                print "Joint "+ str(i) + " of the " + armid + " arm is out of range"
                print "Angle is " + str(armjnts[counter])
                print "Range is (" + str(armlj[i]["rngmin"]) + ", " + str(armlj[i]["rngmax"]) + ")"
                return False
            counter += 1

        return True

    def __initrgtlj(self):
        """
        Init rgt arm links and joints

        ## note
        x is facing forward, y is facing left, z is facing upward
        each element of rgtlj is a dictionary
        rgtlj[i]['linkpos'] indicates the position of a link
        rgtlj[i]['linkvec'] indicates the vector of a link that points from start to end
        rgtlj[i]['rotmat'] indicates the frame of this link
        rgtlj[i]['rotax'] indicates the rotation axis of the link
        rgtlj[i]['rotangle'] indicates the rotation angle of the link around the rotax
        rgtlj[i]['linkend'] indicates the end position of the link (passively computed)

        ## more note:
        rgtlj[1]['linkpos'] is the position of the first joint
        rgtlj[i]['linkend'] is the same as rgtlj[i+1]['linkpos'],
        I am keeping this value for the eef (end-effector)

        ## even more note:
        joint is attached to the linkpos of a link
        for the first link, the joint is fixed and the rotax = 0,0,0

        ## inherentR is only available at the first link

        :return:
        rgtlj:
            a list of dictionaries with each dictionary holding name, mother, child,
        linkpos (link position), linkvec (link vector), rotmat (orientation), rotax (rotation axis of a joint),
        rotangle (rotation angle of the joint around rotax)
        linkend (the end position of a link) is computed using rotmat, rotax, linkpos, and linklen
        lj means link and joint, the joint attached to the link is at the linkend

        author: weiwei
        date: 20161202, tsukuba
        """

        # create a arm with 6 joints
        rgtlj = [dict() for i in range(7)]
        rngsafemargin = 0

        # the 0th link and joint
        rgtlj[0]['name'] = 'link0'
        rgtlj[0]['mother'] = -1
        rgtlj[0]['child'] = 1
        rgtlj[0]['linkpos'] = np.array([0,0,0])
        rgtlj[0]['linkvec'] = np.array([0, -237.50, 954.39])+np.dot(rm.rodrigues([1,0,0],135), np.array([0,0,89.159]))
        rgtlj[0]['rotax'] = np.array([0,0,1])
        rgtlj[0]['rotangle'] = 0
        rgtlj[0]['rotmat'] = np.eye(3)
        rgtlj[0]['linkend'] = np.dot(rgtlj[0]['rotmat'], rgtlj[0]['linkvec'])+rgtlj[0]['linkpos']

        # the 1st joint and link
        rgtlj[1]['name'] = 'link1'
        rgtlj[1]['mother'] = 0
        rgtlj[1]['child'] = 2
        rgtlj[1]['linkpos'] = rgtlj[0]['linkend']
        rgtlj[1]['linkvec'] = np.array([0,135.85,0])
        rgtlj[1]['rotax'] = np.array([0,0,1])
        rgtlj[1]['rotangle'] = 0
        rgtlj[1]['inherentR'] = rm.rodrigues([1,0,0],135)
        rgtlj[1]['rotmat'] = np.dot(np.dot(rgtlj[0]['rotmat'], rgtlj[1]['inherentR']), \
                                    rm.rodrigues(rgtlj[1]['rotax'], rgtlj[1]['rotangle']))
        rgtlj[1]['linkend'] = np.dot(rgtlj[1]['rotmat'], rgtlj[1]['linkvec'])+rgtlj[1]['linkpos']
        rgtlj[1]['rngmin'] = -(360-rngsafemargin)
        rgtlj[1]['rngmax'] = +(360-rngsafemargin)

        # the 2nd joint and link
        rgtlj[2]['name'] = 'link2'
        rgtlj[2]['mother'] = 1
        rgtlj[2]['child'] = 3
        rgtlj[2]['linkpos'] = rgtlj[1]['linkend']
        rgtlj[2]['linkvec'] = np.array([0,-119.70,425.00])
        rgtlj[2]['rotax'] = np.array([0,1,0])
        rgtlj[2]['rotangle'] = 0
        rgtlj[2]['inherentR'] = rm.rodrigues([0,1,0],90)
        rgtlj[2]['rotmat'] = np.dot(np.dot(rgtlj[1]['rotmat'], rgtlj[2]['inherentR']), \
                                    rm.rodrigues(rgtlj[2]['rotax'], rgtlj[2]['rotangle']))
        rgtlj[2]['linkend'] = np.dot(rgtlj[2]['rotmat'], rgtlj[2]['linkvec'])+rgtlj[2]['linkpos']
        rgtlj[2]['rngmin'] = -(360-rngsafemargin)
        rgtlj[2]['rngmax'] = +(360-rngsafemargin)

        # the 3rd joint and link
        rgtlj[3]['name'] = 'link3'
        rgtlj[3]['mother'] = 2
        rgtlj[3]['child'] = 4
        rgtlj[3]['linkpos'] = rgtlj[2]['linkend']
        rgtlj[3]['linkvec'] = np.array([0,0,392.25])
        rgtlj[3]['rotax'] = np.array([0,1,0])
        rgtlj[3]['rotangle'] = 0
        rgtlj[3]['rotmat'] = np.dot(rgtlj[2]['rotmat'], rm.rodrigues(rgtlj[3]['rotax'], rgtlj[3]['rotangle']))
        rgtlj[3]['linkend'] = np.dot(rgtlj[3]['rotmat'], rgtlj[3]['linkvec'])+rgtlj[3]['linkpos']
        rgtlj[3]['rngmin'] = -(360-rngsafemargin)
        rgtlj[3]['rngmax'] = +(360-rngsafemargin)

        # the 4th joint and link
        rgtlj[4]['name'] = 'link4'
        rgtlj[4]['mother'] = 3
        rgtlj[4]['child'] = 5
        rgtlj[4]['linkpos'] = rgtlj[3]['linkend']
        rgtlj[4]['linkvec'] = np.array([0,93.00,0])
        rgtlj[4]['rotax'] = np.array([0,1,0])
        rgtlj[4]['rotangle'] = 0
        rgtlj[4]['inherentR'] = rm.rodrigues([0,1,0], 90)
        rgtlj[4]['rotmat'] = np.dot(np.dot(rgtlj[3]['rotmat'], rgtlj[4]['inherentR']), \
                                    rm.rodrigues(rgtlj[4]['rotax'], rgtlj[4]['rotangle']))
        rgtlj[4]['linkend'] = np.dot(rgtlj[4]['rotmat'], rgtlj[4]['linkvec'])+rgtlj[4]['linkpos']
        rgtlj[4]['rngmin'] = -(360-rngsafemargin)
        rgtlj[4]['rngmax'] = +(360-rngsafemargin)

        # the 5th joint and link
        rgtlj[5]['name'] = 'link5'
        rgtlj[5]['mother'] = 4
        rgtlj[5]['child'] = 6
        rgtlj[5]['linkpos'] = rgtlj[4]['linkend']
        rgtlj[5]['linkvec'] = np.array([0,82.30,94.65])
        rgtlj[5]['rotax'] = np.array([0,0,1])
        rgtlj[5]['rotangle'] = 0
        rgtlj[5]['rotmat'] = np.dot(rgtlj[4]['rotmat'], rm.rodrigues(rgtlj[5]['rotax'], rgtlj[5]['rotangle']))
        rgtlj[5]['linkend'] = np.dot(rgtlj[5]['rotmat'], rgtlj[5]['linkvec'])+rgtlj[5]['linkpos']
        rgtlj[5]['rngmin'] = -(360-rngsafemargin)
        rgtlj[5]['rngmax'] = +(360-rngsafemargin)

        # the 6th joint and link
        rgtlj[6]['name'] = 'link6'
        rgtlj[6]['mother'] = 5
        rgtlj[6]['child'] = -1
        rgtlj[6]['linkpos'] = rgtlj[5]['linkend']
        # for hrp5three
        # rgtlj[6]['linkvec'] = np.array([-172.7,0,0])
        # for rtq85
        rgtlj[6]['linkvec'] = np.array([-13.5-145.0,0,0])
        rgtlj[6]['rotax'] = np.array([1,0,0])
        rgtlj[6]['rotangle'] = 0
        rgtlj[6]['inherentR'] = rm.rodrigues([0,0,1],-90)
        rgtlj[6]['rotmat'] = np.dot(np.dot(rgtlj[5]['rotmat'], rgtlj[6]['inherentR']), \
                                    rm.rodrigues(rgtlj[6]['rotax'], rgtlj[6]['rotangle']))
        rgtlj[6]['linkend'] = np.dot(rgtlj[6]['rotmat'], rgtlj[6]['linkvec'])+rgtlj[6]['linkpos']
        rgtlj[6]['rngmin'] = -(360-rngsafemargin)
        rgtlj[6]['rngmax'] = +(360-rngsafemargin)

        return rgtlj

    def __initlftlj(self):
        """
        Init hrp5's lft arm links and joints

        ## note
        x is facing forward, y is facing left, z is facing upward
        each element of lftlj is a dictionary
        lftlj[i]['linkpos'] indicates the position of a link
        lftlj[i]['linkvec'] indicates the vector of a link that points from start to end
        lftlj[i]['rotmat'] indicates the frame of this link
        lftlj[i]['rotax'] indicates the rotation axis of the link
        lftlj[i]['rotangle'] indicates the rotation angle of the link around the rotax
        lftlj[i]['linkend'] indicates the end position of the link (passively computed)

        ## more note:
        lftlj[1]['linkpos'] is the position of the first joint
        lftlj[i]['linkend'] is the same as lftlj[i+1]['linkpos'],
        I am keeping this value for the eef (end-effector)

        ## even more note:
        joint is attached to the linkpos of a link
        for the first link, the joint is fixed and the rotax = 0,0,0

        ## inherentR is only available at the first link

        :return:
        lftlj:
            a list of dictionaries with each dictionary holding name, mother, child,
        linkpos (link position), linkvec (link vector), rotmat (orientation), rotax (rotation axis of a joint),
        rotangle (rotation angle of the joint around rotax)
        linkend (the end position of a link) is computed using rotmat, rotax, linkpos, and linklen
        lj means link and joint, the joint attached to the link is at the linkend

        author: weiwei
        date: 20161202, tsukuba
        """

        # create a arm with 6 joints
        lftlj = [dict() for i in range(7)]
        rngsafemargin = 0

        # the 0th link and joint
        lftlj[0]['name'] = 'link0'
        lftlj[0]['mother'] = -1
        lftlj[0]['child'] = 1
        lftlj[0]['linkpos'] = np.array([0,0,0])
        lftlj[0]['linkvec'] = np.array([0, 237.50, 954.39])+np.dot(rm.rodrigues([1,0,0], -135), np.array([0,0,89.159]))
        lftlj[0]['rotax'] = np.array([0,0,1])
        lftlj[0]['rotangle'] = 0
        lftlj[0]['rotmat'] = np.eye(3)
        lftlj[0]['linkend'] = np.dot(lftlj[0]['rotmat'], lftlj[0]['linkvec'])+lftlj[0]['linkpos']

        # the 1st joint and link
        lftlj[1]['name'] = 'link1'
        lftlj[1]['mother'] = 0
        lftlj[1]['child'] = 2
        lftlj[1]['linkpos'] = lftlj[0]['linkend']
        lftlj[1]['linkvec'] = np.array([0,135.85,0])
        lftlj[1]['rotax'] = np.array([0,0,1])
        lftlj[1]['rotangle'] = 0
        lftlj[1]['inherentR'] = np.dot(rm.rodrigues([0,0,1],180), rm.rodrigues([1,0,0],135))
        lftlj[1]['rotmat'] = np.dot(np.dot(lftlj[0]['rotmat'], lftlj[1]['inherentR']), \
                                    rm.rodrigues(lftlj[1]['rotax'], lftlj[1]['rotangle']))
        lftlj[1]['linkend'] = np.dot(lftlj[1]['rotmat'], lftlj[1]['linkvec'])+lftlj[1]['linkpos']
        lftlj[1]['rngmin'] = -(360-rngsafemargin)
        lftlj[1]['rngmax'] = +(360-rngsafemargin)

        # the 2nd joint and link
        lftlj[2]['name'] = 'link2'
        lftlj[2]['mother'] = 1
        lftlj[2]['child'] = 3
        lftlj[2]['linkpos'] = lftlj[1]['linkend']
        lftlj[2]['linkvec'] = np.array([0,-119.70,425.00])
        lftlj[2]['rotax'] = np.array([0,1,0])
        lftlj[2]['rotangle'] = 0
        lftlj[2]['inherentR'] = rm.rodrigues([0,1,0],90)
        lftlj[2]['rotmat'] = np.dot(np.dot(lftlj[1]['rotmat'], lftlj[2]['inherentR']), \
                                    rm.rodrigues(lftlj[2]['rotax'], lftlj[2]['rotangle']))
        lftlj[2]['linkend'] = np.dot(lftlj[2]['rotmat'], lftlj[2]['linkvec'])+lftlj[2]['linkpos']
        lftlj[2]['rngmin'] = -(360-rngsafemargin)
        lftlj[2]['rngmax'] = +(360-rngsafemargin)

        # the 3rd joint and link
        lftlj[3]['name'] = 'link3'
        lftlj[3]['mother'] = 2
        lftlj[3]['child'] = 4
        lftlj[3]['linkpos'] = lftlj[2]['linkend']
        lftlj[3]['linkvec'] = np.array([0,0,392.25])
        lftlj[3]['rotax'] = np.array([0,1,0])
        lftlj[3]['rotangle'] = 0
        lftlj[3]['rotmat'] = np.dot(lftlj[2]['rotmat'], rm.rodrigues(lftlj[3]['rotax'], lftlj[3]['rotangle']))
        lftlj[3]['linkend'] = np.dot(lftlj[3]['rotmat'], lftlj[3]['linkvec'])+lftlj[3]['linkpos']
        lftlj[3]['rngmin'] = -(360-rngsafemargin)
        lftlj[3]['rngmax'] = +(360-rngsafemargin)

        # the 4th joint and link
        lftlj[4]['name'] = 'link4'
        lftlj[4]['mother'] = 3
        lftlj[4]['child'] = 5
        lftlj[4]['linkpos'] = lftlj[3]['linkend']
        lftlj[4]['linkvec'] = np.array([0,93.00,0])
        lftlj[4]['rotax'] = np.array([0,1,0])
        lftlj[4]['rotangle'] = 0
        lftlj[4]['inherentR'] = rm.rodrigues([0,1,0], 90)
        lftlj[4]['rotmat'] = np.dot(np.dot(lftlj[3]['rotmat'], lftlj[4]['inherentR']), \
                                    rm.rodrigues(lftlj[4]['rotax'], lftlj[4]['rotangle']))
        lftlj[4]['linkend'] = np.dot(lftlj[4]['rotmat'], lftlj[4]['linkvec'])+lftlj[4]['linkpos']
        lftlj[4]['rngmin'] = -(360-rngsafemargin)
        lftlj[4]['rngmax'] = +(360-rngsafemargin)

        # the 5th joint and link
        lftlj[5]['name'] = 'link5'
        lftlj[5]['mother'] = 4
        lftlj[5]['child'] = 6
        lftlj[5]['linkpos'] = lftlj[4]['linkend']
        lftlj[5]['linkvec'] = np.array([0,82.30,94.65])
        lftlj[5]['rotax'] = np.array([0,0,1])
        lftlj[5]['rotangle'] = 0
        lftlj[5]['rotmat'] = np.dot(lftlj[4]['rotmat'], rm.rodrigues(lftlj[5]['rotax'], lftlj[5]['rotangle']))
        lftlj[5]['linkend'] = np.dot(lftlj[5]['rotmat'], lftlj[5]['linkvec'])+lftlj[5]['linkpos']
        lftlj[5]['rngmin'] = -(360-rngsafemargin)
        lftlj[5]['rngmax'] = +(360-rngsafemargin)

        # the 6th joint and link
        lftlj[6]['name'] = 'link6'
        lftlj[6]['mother'] = 5
        lftlj[6]['child'] = -1
        lftlj[6]['linkpos'] = lftlj[5]['linkend']
        # for hrp5three
        # lftlj[6]['linkvec'] = np.array([-172.7,0,0])
        # for rtq85
        lftlj[6]['linkvec'] = np.array([-13.5-145.0,0,0])
        lftlj[6]['rotax'] = np.array([1,0,0])
        lftlj[6]['rotangle'] = 0
        lftlj[6]['inherentR'] = rm.rodrigues([0,0,1],-90)
        lftlj[6]['rotmat'] = np.dot(np.dot(lftlj[5]['rotmat'], lftlj[6]['inherentR']), \
                                    rm.rodrigues(lftlj[6]['rotax'], lftlj[6]['rotangle']))
        lftlj[6]['linkend'] = np.dot(lftlj[6]['rotmat'], lftlj[6]['linkvec'])+lftlj[6]['linkpos']
        lftlj[6]['rngmin'] = -(360-rngsafemargin)
        lftlj[6]['rngmax'] = +(360-rngsafemargin)

        return lftlj

    def __updatefk(self, armlj):
        """
        Update the structure of hrp5's arm links and joints (single)
        Note that this function should not be called explicitly
        It is called automatically by functions like movexxx

        :param armlj: the rgtlj or lftlj robot structure
        :return: null

        author: weiwei
        date: 20161202
        """

        i = 1
        while i != -1:
            j = armlj[i]['mother']
            armlj[i]['linkpos'] = armlj[j]['linkend']
            if i != 3 and i != 5:
                armlj[i]['rotmat'] = np.dot(np.dot(armlj[j]['rotmat'], armlj[i]['inherentR']),
                                            rm.rodrigues(armlj[i]['rotax'], armlj[i]['rotangle']))
            else:
                armlj[i]['rotmat'] = np.dot(armlj[j]['rotmat'], rm.rodrigues(armlj[i]['rotax'], armlj[i]['rotangle']))
            armlj[i]['linkend'] = np.squeeze(
                np.dot(armlj[i]['rotmat'], armlj[i]['linkvec'].reshape((-1, 1))).reshape((1, -1))) + armlj[i]['linkpos']
            i = armlj[i]['child']
        return armlj

    def numik(self, objpos, objrot, armid="rgt"):
        return ur5dualik.numik(self, objpos, objrot, armid)

    def numikr(self, objpos, objrot, armid="rgt"):
        return ur5dualik.numikr(self, objpos, objrot, armid)

if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters

    base = pandactrl.World(camp = [3000,0,3000], lookatp = [0,0,700])

    ur5dualrobot = Ur5DualRobot()
    ur5dualrobot.goinitpose()

    import ur5dualplot
    # ur5dualplot.plotstick(base.render, ur5dualrobot)

    # from manipulation.grip.hrp5three import hrp5threenm
    # handpkg = hrp5threenm
    from manipulation.grip.robotiq85 import rtq85nm
    handpkg = rtq85nm
    ur5dualmnp = ur5dualplot.genmnp(ur5dualrobot, handpkg)
    # ur5dualmnp.reparentTo(base.render)
    # pg.plotAxisSelf(base.render, Vec3(0,0,0))
    #
    objpos = np.array([400,200,400])
    # objrot = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    objrot = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
    # objrot = np.array([[-1,0,0],[0,1,0],[0,0,-1]])
    # objrotmat4 = pg.npToMat4(objrot)
    # objrotmat4 = objrotmat4*Mat4.rotateMat(30, Vec3(1,0,0))
    # objrot = pg.mat3ToNp(objrotmat4.getUpper3())
    # objpos = np.array([401.67913818,-644.12841797,0])
    # objrot = np.array([[1.93558640e-06,-8.36298645e-01,5.48274219e-01],
    #                     [1.93560686e-06,-5.48274219e-01,-8.36298645e-01],
    #                     [1.00000000e+00,2.67997166e-06,5.57513317e-07]])
    # lfthnd
    # objpos = np.array([180,130,100])
    # objrot = np.array([[0,0,-1],[1,0,0],[0,-1,0]])
    armid="rgt"
    armid="lft"
    # armjntsgoal = hrp5robot.numikr(objpos, objrot, armid)
    # if armjntsgoal is not None:
    #     hrp5robot.movearmfkr(armjntsgoal, armid)
    #     hrp5plot.plotstick(base.render, hrp5robot)
    #     hrp5mnp = hrp5plot.genHrp5mnp(hrp5robot)
    #     hrp5mnp.reparentTo(base.render)
    armjntsgoal = ur5dualrobot.numik(objpos, objrot, armid)
    if armjntsgoal is not None:
        ur5dualrobot.movearmfk(armjntsgoal, armid)
        # ur5dualplot.plotstick(base.render, ur5dualrobot)
        ur5mnp = ur5dualplot.genmnp(ur5dualrobot, handpkg)
        ur5mnp.reparentTo(base.render)

    # goal hand
    # from manipulation.grip.robotiq85 import rtq85nm
    # hrp5robotrgthnd = rtq85nm.Rtq85NM()
    # hrp5robotrgthnd.setColor([1,0,0,.3])
    # hrp5robotrgtarmlj9_rotmat = pandageom.cvtMat4(objrot, objpos+objrot[:,0]*130)
    # pg.plotAxisSelf(base.render, objpos, hrp5robotrgtarmlj9_rotmat)
    # hrp5robotrgthnd.setMat(hrp5robotrgtarmlj9_rotmat)
    # hrp5robotrgthnd.reparentTo(base.render)
    #
    # angle = nxtik.eurgtbik(objpos)
    # nxtrobot.movewaist(angle)
    # armjntsgoal=nxtik.numik(nxtrobot, objpos, objrot)
    #
    # # nxtplot.plotstick(base.render, nxtrobot)
    # pandamat4=Mat4()
    # pandamat4.setRow(3,Vec3(0,0,250))
    # # pg.plotAxis(base.render, pandamat4)
    # # nxtplot.plotmesh(base, nxtrobot)
    # # pandageom.plotAxis(base.render, pandageom.cvtMat4(nxtrobot.rgtarm[6]['rotmat'], nxtrobot.rgtarm[6]['linkpos']))
    # pg.plotDumbbell(base.render, objpos, objpos, rgba = [1,0,0,1])
    pg.plotAxisSelf(base.render, objpos, pg.npToMat4(objrot))
    # pg.plotArrow(base.render, hrp5robot.rgtarm[8]['linkpos'], hrp5robot.rgtarm[8]['linkpos']+hrp5robot.rgtarm[8]['rotax']*1000)
    #
    # # nxtrobot.movearmfk6(armjntsgoal)
    # # nxtmnp = nxtplot.genNxtmnp_nm(nxtrobot, plotcolor=[1,0,0,1])
    # # nxtmnp.reparentTo(base.render)

    base.run()