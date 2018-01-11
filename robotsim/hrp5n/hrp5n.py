import numpy as np
import exceptions as ep
import utils.robotmath as rm
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
import hrp5nik

class Hrp5NRobot():
    def __init__(self):
        # initjnts has 21 elements where the first three are for the base and head,
        # the remaining 18 are for each of the two 9-dof arms
        self.__name = 'hrp5n'
        # initjnts[0] = waist, 0,0 = head, 0,45,-20,...-30,0,0 = rgt, 0,45,20,...,30,0,0 = lft
        self.__initjnts = np.array([0,0,0,0,45,-50,-45,-150,45,0,0,0,0,45,50,45,-150,-45,0,0,0]);
        # self.__initjnts = np.array([0,0,0,0,45,-20,0,-75,0,-30,0,0,0,45,20,0,-75,0,30,0,0]);
        # self.__initjnts = np.array([0,0,0,45,-20,0,-64,82,-27,109,-111,0,45,20,0,-75,0,30,0,0]);
        # self.__initjnts = np.array([0,0,0,45,-20,0,-68,77,-32,109,-122,0,45,20,0,-75,0,30,0,0]);
        # self.__initjnts = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
        self.__rgtarm = self.__initrgtlj()
        self.__lftarm = self.__initlftlj()
        # define the target joints for ik
        # waist 0 should not be in the list
        # self.__targetjoints = [2,3,4,5,6,9]
        self.__targetjoints = [1,2,3,4,5,6,7,8,9]
        # self.__sixjoints = [2,4,5,6,7,9]
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
    def initrgtjntsr(self):
        # read-only property
        return np.array([self.__initjnts[i] for i in [0,3,4,5,6,7,8,9,10,11]])

    @property
    def initlftjntsr(self):
        # read-only property
        return  np.array([self.__initjnts[i] for i in [0,12,13,14,15,16,17,18,19,20]])

    @property
    def initrgtjnts(self):
        # read-only property
        return np.array([self.__initjnts[i] for i in [3,4,5,6,7,8,9,10,11]])

    @property
    def initlftjnts(self):
        # read-only property
        return  np.array([self.__initjnts[i] for i in [12,13,14,15,16,17,18,19,20]])

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
    def targetjoints(self):
        # read-only property
        return self.__targetjoints

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

        # create a arm with 10 joints
        rgtlj = [dict() for i in range(10)]
        rngsafemargin = 5

        # the 0th link and joint
        rgtlj[0]['name'] = 'link0'
        rgtlj[0]['mother'] = -1
        rgtlj[0]['child'] = 1
        rgtlj[0]['linkpos'] = np.array([0,0,0])
        rgtlj[0]['linkvec'] = np.array([32.0, -80.0, 542.0])
        rgtlj[0]['rotax'] = np.array([0,0,1])
        rgtlj[0]['rotangle'] = 0
        rgtlj[0]['rotmat'] = np.eye(3)
        rgtlj[0]['linkend'] = np.dot(rgtlj[0]['rotmat'], rgtlj[0]['linkvec'])+rgtlj[0]['linkpos']

        # the 1st joint and link
        rgtlj[1]['name'] = 'link1'
        rgtlj[1]['mother'] = 0
        rgtlj[1]['child'] = 2
        rgtlj[1]['linkpos'] = rgtlj[0]['linkend']
        rgtlj[1]['linkvec'] = np.array([0,-130,0])
        rgtlj[1]['rotax'] = np.array([0,0,1])
        rgtlj[1]['rotangle'] = 0
        rgtlj[1]['rotmat'] = np.dot(rgtlj[0]['rotmat'], rm.rodrigues(rgtlj[1]['rotax'], rgtlj[1]['rotangle']))
        rgtlj[1]['linkend'] = np.dot(rgtlj[1]['rotmat'], rgtlj[1]['linkvec'])+rgtlj[1]['linkpos']
        rgtlj[1]['rngmin'] = -(20-rngsafemargin)
        rgtlj[1]['rngmax'] = +(60-rngsafemargin)

        # the 2nd joint and link
        rgtlj[2]['name'] = 'link2'
        rgtlj[2]['mother'] = 1
        rgtlj[2]['child'] = 3
        rgtlj[2]['linkpos'] = rgtlj[1]['linkend']
        rgtlj[2]['linkvec'] = np.array([0,0,0])
        rgtlj[2]['rotax'] = np.array([0,1,0])
        rgtlj[2]['rotangle'] = 0
        rgtlj[2]['rotmat'] = np.dot(rgtlj[1]['rotmat'], rm.rodrigues(rgtlj[2]['rotax'], rgtlj[2]['rotangle']))
        rgtlj[2]['linkend'] = np.dot(rgtlj[2]['rotmat'], rgtlj[2]['linkvec'])+rgtlj[2]['linkpos']
        rgtlj[2]['rngmin'] = -(200-rngsafemargin)
        rgtlj[2]['rngmax'] = +(70-rngsafemargin)

        # the 3rd joint and link
        rgtlj[3]['name'] = 'link3'
        rgtlj[3]['mother'] = 2
        rgtlj[3]['child'] = 4
        rgtlj[3]['linkpos'] = rgtlj[2]['linkend']
        rgtlj[3]['linkvec'] = np.array([0,-20,0])
        rgtlj[3]['rotax'] = np.array([1,0,0])
        rgtlj[3]['rotangle'] = 0
        rgtlj[3]['rotmat'] = np.dot(rgtlj[2]['rotmat'], rm.rodrigues(rgtlj[3]['rotax'], rgtlj[3]['rotangle']))
        rgtlj[3]['linkend'] = np.dot(rgtlj[3]['rotmat'], rgtlj[3]['linkvec'])+rgtlj[3]['linkpos']
        rgtlj[3]['rngmin'] = -(95-rngsafemargin)
        rgtlj[3]['rngmax'] = +(25-rngsafemargin)

        # the 4th joint and link
        rgtlj[4]['name'] = 'link4'
        rgtlj[4]['mother'] = 3
        rgtlj[4]['child'] = 5
        rgtlj[4]['linkpos'] = rgtlj[3]['linkend']
        rgtlj[4]['linkvec'] = np.array([50,0,-295.804])
        rgtlj[4]['rotax'] = np.array([0,0,1])
        rgtlj[4]['rotangle'] = 0
        rgtlj[4]['rotmat'] = np.dot(rgtlj[3]['rotmat'], rm.rodrigues(rgtlj[4]['rotax'], rgtlj[4]['rotangle']))
        rgtlj[4]['linkend'] = np.dot(rgtlj[4]['rotmat'], rgtlj[4]['linkvec'])+rgtlj[4]['linkpos']
        rgtlj[4]['rngmin'] = -(92-rngsafemargin)
        rgtlj[4]['rngmax'] = +(120-rngsafemargin)

        # the 5th joint and link
        rgtlj[5]['name'] = 'link5'
        rgtlj[5]['mother'] = 4
        rgtlj[5]['child'] = 6
        rgtlj[5]['linkpos'] = rgtlj[4]['linkend']
        rgtlj[5]['linkvec'] = np.array([-50,0,-295.804])
        rgtlj[5]['rotax'] = np.array([0,1,0])
        rgtlj[5]['rotangle'] = 0
        rgtlj[5]['rotmat'] = np.dot(rgtlj[4]['rotmat'], rm.rodrigues(rgtlj[5]['rotax'], rgtlj[5]['rotangle']))
        rgtlj[5]['linkend'] = np.dot(rgtlj[5]['rotmat'], rgtlj[5]['linkvec'])+rgtlj[5]['linkpos']
        rgtlj[5]['rngmin'] = -(180-rngsafemargin)
        rgtlj[5]['rngmax'] = -(17.2-rngsafemargin)

        # the 6th joint and link
        rgtlj[6]['name'] = 'link6'
        rgtlj[6]['mother'] = 5
        rgtlj[6]['child'] = 7
        rgtlj[6]['linkpos'] = rgtlj[5]['linkend']
        rgtlj[6]['linkvec'] = np.array([0,0,0])
        rgtlj[6]['rotax'] = np.array([0,0,1])
        rgtlj[6]['rotangle'] = 0
        rgtlj[6]['rotmat'] = np.dot(rgtlj[5]['rotmat'], rm.rodrigues(rgtlj[6]['rotax'], rgtlj[6]['rotangle']))
        rgtlj[6]['linkend'] = np.dot(rgtlj[6]['rotmat'], rgtlj[6]['linkvec'])+rgtlj[6]['linkpos']
        rgtlj[6]['rngmin'] = -(122-rngsafemargin)
        rgtlj[6]['rngmax'] = +(182-rngsafemargin)

        # the 7th joint and link
        rgtlj[7]['name'] = 'link7'
        rgtlj[7]['mother'] = 6
        rgtlj[7]['child'] = 8
        rgtlj[7]['linkpos'] = rgtlj[6]['linkend']
        rgtlj[7]['linkvec'] = np.array([0,0,0])
        rgtlj[7]['rotax'] = np.array([1,0,0])
        rgtlj[7]['rotangle'] = 0
        rgtlj[7]['rotmat'] = np.dot(rgtlj[6]['rotmat'], rm.rodrigues(rgtlj[7]['rotax'], rgtlj[7]['rotangle']))
        rgtlj[7]['linkend'] = np.dot(rgtlj[7]['rotmat'], rgtlj[7]['linkvec'])+rgtlj[7]['linkpos']
        rgtlj[7]['rngmin'] = -(47-rngsafemargin)
        rgtlj[7]['rngmax'] = +(70-rngsafemargin)
        # rgtlj[7]['rngmin'] = -(87-rngsafemargin)
        # rgtlj[7]['rngmax'] = +(120-rngsafemargin)


        # the 8th joint and link
        rgtlj[8]['name'] = 'link8'
        rgtlj[8]['mother'] = 7
        rgtlj[8]['child'] = 9
        rgtlj[8]['linkpos'] = rgtlj[7]['linkend']
        rgtlj[8]['linkvec'] = np.array([0,7.004,-67.249])
        rgtlj[8]['rotax'] = np.array([0,1,0])
        rgtlj[8]['rotangle'] = 0
        rgtlj[8]['rotmat'] = np.dot(rgtlj[7]['rotmat'], rm.rodrigues(rgtlj[8]['rotax'], rgtlj[8]['rotangle']))
        rgtlj[8]['linkend'] = np.dot(rgtlj[8]['rotmat'], rgtlj[8]['linkvec'])+rgtlj[8]['linkpos']
        rgtlj[8]['rngmin'] = -(42-rngsafemargin)
        rgtlj[8]['rngmax'] = +(42-rngsafemargin)
        # rgtlj[8]['rngmin'] = -(92-rngsafemargin)
        # rgtlj[8]['rngmax'] = +(92-rngsafemargin)

        # the 9th joint and link
        rgtlj[9]['name'] = 'link9'
        rgtlj[9]['mother'] = 8
        rgtlj[9]['child'] = -1
        rgtlj[9]['linkpos'] = rgtlj[8]['linkend']
        rgtlj[9]['linkvec'] = np.array([-172.7,0,0])
        rgtlj[9]['rotax'] = np.array([1,0,0])
        rgtlj[9]['rotangle'] = 0
        # make sure x direction faces at ee, z directions faces downward
        # see the definition of the coordinates in rtq85 (execute the file)
        rgtlj[9]['inherentR'] = np.dot(rm.rodrigues([1,0,0],45),rm.rodrigues([0,1,0],-90))
        rgtlj[9]['rotmat'] = np.dot(np.dot(rgtlj[8]['rotmat'], rgtlj[9]['inherentR']), \
                                    rm.rodrigues(rgtlj[9]['rotax'], rgtlj[9]['rotangle']))
        rgtlj[9]['linkend'] = np.dot(rgtlj[9]['rotmat'], rgtlj[9]['linkvec'])+rgtlj[9]['linkpos']
        rgtlj[9]['rngmin'] = -(300-rngsafemargin)
        rgtlj[9]['rngmax'] = +(300-rngsafemargin)

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

        # create a arm with 10 joints
        lftlj = [dict() for i in range(10)]
        rngsafemargin = 5

        # the 0th link and joint
        lftlj[0]['name'] = 'link0'
        lftlj[0]['mother'] = -1
        lftlj[0]['child'] = 1
        lftlj[0]['linkpos'] = np.array([0,0,0])
        lftlj[0]['linkvec'] = np.array([32, 80, 542])
        lftlj[0]['rotax'] = np.array([0,0,1])
        lftlj[0]['rotangle'] = 0
        lftlj[0]['rotmat'] = np.eye(3)
        lftlj[0]['linkend'] = np.dot(lftlj[0]['rotmat'], lftlj[0]['linkvec'])+lftlj[0]['linkpos']

        # the 1st joint and link
        lftlj[1]['name'] = 'link1'
        lftlj[1]['mother'] = 0
        lftlj[1]['child'] = 2
        lftlj[1]['linkpos'] = lftlj[0]['linkend']
        lftlj[1]['linkvec'] = np.array([0,130,0])
        lftlj[1]['rotax'] = np.array([0,0,1])
        lftlj[1]['rotangle'] = 0
        lftlj[1]['rotmat'] = np.dot(lftlj[0]['rotmat'], rm.rodrigues(lftlj[1]['rotax'], lftlj[1]['rotangle']))
        lftlj[1]['linkend'] = np.dot(lftlj[1]['rotmat'], lftlj[1]['linkvec'])+lftlj[1]['linkpos']
        lftlj[1]['rngmin'] = -(60-rngsafemargin)
        lftlj[1]['rngmax'] = +(20-rngsafemargin)

        # the 2nd joint and link
        lftlj[2]['name'] = 'link2'
        lftlj[2]['mother'] = 1
        lftlj[2]['child'] = 3
        lftlj[2]['linkpos'] = lftlj[1]['linkend']
        lftlj[2]['linkvec'] = np.array([0,0,0])
        lftlj[2]['rotax'] = np.array([0,1,0])
        lftlj[2]['rotangle'] = 0
        lftlj[2]['rotmat'] = np.dot(lftlj[1]['rotmat'], rm.rodrigues(lftlj[2]['rotax'], lftlj[2]['rotangle']))
        lftlj[2]['linkend'] = np.dot(lftlj[2]['rotmat'], lftlj[2]['linkvec'])+lftlj[2]['linkpos']
        lftlj[2]['rngmin'] = -(200-rngsafemargin)
        lftlj[2]['rngmax'] = +(70-rngsafemargin)

        # the 3rd joint and link
        lftlj[3]['name'] = 'link3'
        lftlj[3]['mother'] = 2
        lftlj[3]['child'] = 4
        lftlj[3]['linkpos'] = lftlj[2]['linkend']
        lftlj[3]['linkvec'] = np.array([0,20,0])
        lftlj[3]['rotax'] = np.array([1,0,0])
        lftlj[3]['rotangle'] = 0
        lftlj[3]['rotmat'] = np.dot(lftlj[2]['rotmat'], rm.rodrigues(lftlj[3]['rotax'], lftlj[3]['rotangle']))
        lftlj[3]['linkend'] = np.dot(lftlj[3]['rotmat'], lftlj[3]['linkvec'])+lftlj[3]['linkpos']
        lftlj[3]['rngmin'] = -(25-rngsafemargin)
        lftlj[3]['rngmax'] = +(95-rngsafemargin)

        # the 4th joint and link
        lftlj[4]['name'] = 'link4'
        lftlj[4]['mother'] = 3
        lftlj[4]['child'] = 5
        lftlj[4]['linkpos'] = lftlj[3]['linkend']
        lftlj[4]['linkvec'] = np.array([50,0,-295.804])
        lftlj[4]['rotax'] = np.array([0,0,1])
        lftlj[4]['rotangle'] = 0
        lftlj[4]['rotmat'] = np.dot(lftlj[3]['rotmat'], rm.rodrigues(lftlj[4]['rotax'], lftlj[4]['rotangle']))
        lftlj[4]['linkend'] = np.dot(lftlj[4]['rotmat'], lftlj[4]['linkvec'])+lftlj[4]['linkpos']
        lftlj[4]['rngmin'] = -(120-rngsafemargin)
        lftlj[4]['rngmax'] = +(92-rngsafemargin)

        # the 5th joint and link
        lftlj[5]['name'] = 'link5'
        lftlj[5]['mother'] = 4
        lftlj[5]['child'] = 6
        lftlj[5]['linkpos'] = lftlj[4]['linkend']
        lftlj[5]['linkvec'] = np.array([-50,0,-295.804])
        lftlj[5]['rotax'] = np.array([0,1,0])
        lftlj[5]['rotangle'] = 0
        lftlj[5]['rotmat'] = np.dot(lftlj[4]['rotmat'], rm.rodrigues(lftlj[5]['rotax'], lftlj[5]['rotangle']))
        lftlj[5]['linkend'] = np.dot(lftlj[5]['rotmat'], lftlj[5]['linkvec'])+lftlj[5]['linkpos']
        lftlj[5]['rngmin'] = -(180-rngsafemargin)
        lftlj[5]['rngmax'] = -(17.2-rngsafemargin)

        # the 6th joint and link
        lftlj[6]['name'] = 'link6'
        lftlj[6]['mother'] = 5
        lftlj[6]['child'] = 7
        lftlj[6]['linkpos'] = lftlj[5]['linkend']
        lftlj[6]['linkvec'] = np.array([0,0,0])
        lftlj[6]['rotax'] = np.array([0,0,1])
        lftlj[6]['rotangle'] = 0
        lftlj[6]['rotmat'] = np.dot(lftlj[5]['rotmat'], rm.rodrigues(lftlj[6]['rotax'], lftlj[6]['rotangle']))
        lftlj[6]['linkend'] = np.dot(lftlj[6]['rotmat'], lftlj[6]['linkvec'])+lftlj[6]['linkpos']
        lftlj[6]['rngmin'] = -(182-rngsafemargin)
        lftlj[6]['rngmax'] = +(122-rngsafemargin)

        # the 7th joint and link
        lftlj[7]['name'] = 'link7'
        lftlj[7]['mother'] = 6
        lftlj[7]['child'] = 8
        lftlj[7]['linkpos'] = lftlj[6]['linkend']
        lftlj[7]['linkvec'] = np.array([0,0,0])
        lftlj[7]['rotax'] = np.array([1,0,0])
        lftlj[7]['rotangle'] = 0
        lftlj[7]['rotmat'] = np.dot(lftlj[6]['rotmat'], rm.rodrigues(lftlj[7]['rotax'], lftlj[7]['rotangle']))
        lftlj[7]['linkend'] = np.dot(lftlj[7]['rotmat'], lftlj[7]['linkvec'])+lftlj[7]['linkpos']
        lftlj[7]['rngmin'] = -(70-rngsafemargin)
        lftlj[7]['rngmax'] = +(47-rngsafemargin)

        # the 8th joint and link
        lftlj[8]['name'] = 'link8'
        lftlj[8]['mother'] = 7
        lftlj[8]['child'] = 9
        lftlj[8]['linkpos'] = lftlj[7]['linkend']
        lftlj[8]['linkvec'] = np.array([0,-7.004,-67.249])
        lftlj[8]['rotax'] = np.array([0,1,0])
        lftlj[8]['rotangle'] = 0
        lftlj[8]['rotmat'] = np.dot(lftlj[7]['rotmat'], rm.rodrigues(lftlj[8]['rotax'], lftlj[8]['rotangle']))
        lftlj[8]['linkend'] = np.dot(lftlj[8]['rotmat'], lftlj[8]['linkvec'])+lftlj[8]['linkpos']
        lftlj[8]['rngmin'] = -(42-rngsafemargin)
        lftlj[8]['rngmax'] = +(42-rngsafemargin)

        # the 9th joint and link
        lftlj[9]['name'] = 'link9'
        lftlj[9]['mother'] = 8
        lftlj[9]['child'] = -1
        lftlj[9]['linkpos'] = lftlj[8]['linkend']
        lftlj[9]['linkvec'] = np.array([-172.7,0,0])
        lftlj[9]['rotax'] = np.array([1,0,0])
        lftlj[9]['rotangle'] = 0
        # make sure x direction faces at ee, z directions faces downward
        # see the definition of the coordinates in rtq85 (execute the file)
        lftlj[9]['inherentR'] = np.dot(rm.rodrigues([1,0,0],-45),rm.rodrigues([0,1,0],-90))
        lftlj[9]['rotmat'] = np.dot(np.dot(lftlj[8]['rotmat'], lftlj[9]['inherentR']), \
                                    rm.rodrigues(lftlj[9]['rotax'], lftlj[9]['rotangle']))
        lftlj[9]['linkend'] = np.dot(lftlj[9]['rotmat'], lftlj[9]['linkvec'])+lftlj[9]['linkpos']
        lftlj[9]['rngmin'] = -(300-rngsafemargin)
        lftlj[9]['rngmax'] = +(300-rngsafemargin)

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
            if i == 9:
                armlj[i]['rotmat'] = np.dot(np.dot(armlj[j]['rotmat'], armlj[i]['inherentR']),
                                            rm.rodrigues(armlj[i]['rotax'], armlj[i]['rotangle']))
            else:
                armlj[i]['rotmat'] = np.dot(armlj[j]['rotmat'], rm.rodrigues(armlj[i]['rotax'], armlj[i]['rotangle']))
            armlj[i]['linkend'] = np.squeeze(
                np.dot(armlj[i]['rotmat'], armlj[i]['linkvec'].reshape((-1, 1))).reshape((1, -1))) + armlj[i]['linkpos']
            i = armlj[i]['child']
        return armlj

    def movewaist(self, rotangle=0):
        """
        rotate the base of the robot

        :param rotangle: in degree
        :return: null

        author: weiwei
        date: 20170410
        """

        # right arm
        self.rgtarm[0]['rotangle'] = rotangle
        self.rgtarm[0]['rotmat'] = rm.rodrigues(self.rgtarm[0]['rotax'], self.rgtarm[0]['rotangle'])
        self.rgtarm[0]['linkend'] = np.squeeze(np.dot(self.rgtarm[0]['rotmat'], self.rgtarm[0]['linkvec'].reshape((-1,))))+self.rgtarm[0]['linkpos']

        # left arm
        self.lftarm[0]['rotangle'] = rotangle
        self.lftarm[0]['rotmat'] = rm.rodrigues(self.lftarm[0]['rotax'], self.lftarm[0]['rotangle'])
        self.lftarm[0]['linkend'] = np.squeeze(np.dot(self.lftarm[0]['rotmat'], self.lftarm[0]['linkvec'].reshape((-1,))))+self.lftarm[0]['linkpos']

        self.__updatefk(self.rgtarm)
        self.__updatefk(self.lftarm)

    def movearmfk(self, armjnts, armid="rgt"):
        """
        move the joints of armlj specified by targetjoints using forward kinematics, waist is not included

        :param armjnts: a 1-by-n ndarray where each element indicates the angle of a joint (in degree)
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
        move all joints of the  robo

        :param robotjnts: the definition as self.initjntss
        :return: null

        author: weiwei
        date: 20161202
        """

        narmjoints = len(self.__targetjoints)
        # right arm
        i = 1
        while i != -1:
            self.rgtarm[i]['rotangle'] = robotjnts[i+2]
            i = self.rgtarm[i]['child']
        # left arm
        i = 1
        while i != -1:
            self.lftarm[i]['rotangle'] = robotjnts[i+2+narmjoints]
            i = self.lftarm[i]['child']

        # self.__updatefk(self.rgtarm)
        # self.__updatefk(self.lftarm)
        self.movewaist(robotjnts[0])

    def goinitpose(self):
        """
        move the robot to initial pose

        :return: null

        author: weiwei
        date: 20161202, tsukuba
        """

        self.movealljnts(self.initjnts)

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

        :param armjnts: a 1-by-6 numpy ndarray indicating the targejoints of a manipulator
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
                # print "Joint "+ str(i) + " of the " + armid + " arm is out of range"
                # print "Angle is " + str(armjnts[counter])
                # print "Range is (" + str(armlj[i]["rngmin"]) + ", " + str(armlj[i]["rngmax"]) + ")"
                return False
            counter += 1

        return True

    def chkrngdrag(self, armjnts, armid="rgt"):
        """
        check if the given armjnts is inside the oeprating range of the speificed armid
        this function doesn't check the waist
        The joint angles out of range will be pulled back to their maxima

        :param armjnts: a 1-by-6 numpy ndarray indicating the targejoints of a manipulator
        :param armid: a string "rgt" or "lft"
        :return: Two parameters, one is true or false indicating if the joint angles are inside the range or not
                The other is the joint angles after draggin.
                If the joints were not dragged, the same joint angles will be returned

        author: weiwei
        date: 20161205
        """

        if armid!="rgt" and armid!="lft":
            raise ep.ValueError

        armlj = self.rgtarm
        if armid == "lft":
            armlj = self.lftarm

        counter = 0
        bdragged = True
        jntanglesdrag = []
        for i in self.__targetjoints:
            if armjnts[counter] < armlj[i]["rngmin"]:
                # print "Joint "+ str(i) + " of the " + armid + " arm is out of range"
                # print "Angle is " + str(armjnts[counter])
                # print "Range is (" + str(armlj[i]["rngmin"]) + ", " + str(armlj[i]["rngmax"]) + ")"
                bdragged = True
                jntanglesdrag.append(armlj[i]["rngmin"])
            elif armjnts[counter] > armlj[i]["rngmax"]:
                bdragged = True
                jntanglesdrag.append(armlj[i]["rngmax"])
            else:
                bdragged = False
                jntanglesdrag.append(armjnts[counter])

            counter += 1

        return bdragged, jntanglesdrag

    def numik(self, objpos, objrot, armid="rgt"):
        return hrp5nik.numik(self, objpos, objrot, armid)

    def numikr(self, objpos, objrot, armid="rgt"):
        return hrp5nik.numikr(self, objpos, objrot, armid)

    def gencollisionmodel(self):
        """
        generate capsule based collision model for quick collision detection

        :return: capcdnode a dictionary with 'rgt' and 'lft'
        author: weiwei
        date: 20170615
        """

        # bodycapcdnode = CollisionNode("body")
        # rgtarmcapcdnode = CollisionNode("rgtarm")
        # lftarmcapcdnode = CollisionNode("lftarm")
        # capcdnode['rgt'] = []
        # capcdnode['lft'] = []
        # # rgt arm
        # i = 0
        # while i != -1:
        #     spos=self.rgtarm[i]['linkpos']
        #     epos=self.rgtarm[i]['linkend']
        #     radius = np.linalg.norm(epos-spos)/3.0
        #     capcdnode['rgt'].append(CollisionTube(spos[0], spos[1], spos[2], epos[0], epos[1], epos[2], radius))
        #     i = self.rgtarm[i]['child']
        # # lft arm
        # i = 0
        # while i != -1:
        #     spos=self.lftarm[i]['linkpos']
        #     epos=self.lftarm[i]['linkend']
        #     radius = np.linalg.norm(epos-spos)/3.0
        #     capcdnode['lft'].append(CollisionTube(spos[0], spos[1], spos[2], epos[0], epos[1], epos[2], radius))
        #     i = self.lftarm[i]['child']

        # return capcdnode
        pass

if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters
    import hrp5nmesh
    from manipulation.grip.hrp5three import hrp5threenm
    from manipulation.grip.robotiq85 import rtq85nm

    # loadPrcFileData("", "want-directtools #t")
    # loadPrcFileData("", "want-tk #t")

    base = pandactrl.World()

    hrp5nrobot = Hrp5NRobot()
    # hrp5nrobot.goinitpose()

    import hrp5nplot
    hrp5nplot.plotstick(base.render, hrp5nrobot)

    handpkg = hrp5threenm
    # handpkg = rtq85nm
    hrp5nmeshgen = hrp5nmesh.Hrp5NMesh(handpkg)
    hrp5nmnp = hrp5nmeshgen.genmnp(hrp5nrobot)
    hrp5nmnp.reparentTo(base.render)

    objpos = np.array([500,-400,305])
    objrot = np.array([[-1,0,0],[0,1,0],[0,0,-1]])
    # plot goal axis
    pg.plotAxisSelf(nodepath = base.render, spos = objpos, pandamat4 = pg.cvtMat4(objrot))

    # objrot = np.array([[0.125178158283, 0.00399381108582, 0.992126166821], [0.98617619276, -0.109927728772, -0.123984932899], [0.108567006886, 0.993931531906, -0.0176991540939]]).T
    # objrot = np.array([[0,1,0],[0,0,1],[1,0,0]])
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
    armjntsgoal = hrp5nrobot.numikr(objpos, objrot, armid)
    if armjntsgoal is not None:
        hrp5nrobot.movearmfkr(armjntsgoal, armid)
        hrp5nmnp = hrp5nmeshgen.genmnp(hrp5nrobot)
        # hrp5nmnp.reparentTo(base.render)


    startrotmat4 = Mat4(1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,305.252044678,-400.073120117,-55.0000038147,1.0)

    startrotmat4 = Mat4(1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,405.252044678,-199.926864624,-55.0000038147,1.0)
    import os
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "../manipulation/grip", "objects", "tool.stl")
    objmnp = pg.genObjmnp(objpath, color = Vec4(.7,.7,0,1))
    objmnp.setMat(startrotmat4)
    # objmnp.reparentTo(base.render)
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
    # pg.plotAxisSelf(base.render, objpos, pg.npToMat4(objrot))
    # pg.plotArrow(base.render, hrp5robot.rgtarm[8]['linkpos'], hrp5robot.rgtarm[8]['linkpos']+hrp5robot.rgtarm[8]['rotax']*1000)
    #
    # # nxtrobot.movearmfk6(armjntsgoal)
    # # nxtmnp = nxtplot.genNxtmnp_nm(nxtrobot, plotcolor=[1,0,0,1])
    # # nxtmnp.reparentTo(base.render)

    base.run()