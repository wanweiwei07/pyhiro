#!/usr/bin/python

import os
import itertools

import MySQLdb as mdb
import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from manipulation.grip.hrp5three import hrp5threenm
from panda3d.bullet import BulletWorld
from panda3d.core import *

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
from manipulation.grip import freetabletopplacement as tp
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from robotsim.nextage import nxt
from robotsim.hrp5 import hrp5
from robotsim.hrp5n import hrp5n
from robotsim.hrp2k import hrp2k
from database import dbaccess as db
import trimesh
import time
import pickle

class FloatingPoses(object):
    """
    like freetabletopplacement and tableplacements
    manipulation.floatingposes corresponds to freegrip
    floatingposes doesn't take into account
    the position and orientation of the object
    it is "free" in position and rotation. No obstacles were considered.
    In contrast, each item in regrasp.floatingposes
    has different position and orientation
    it is at a specific pose in the workspace
    To clearly indicate the difference, "free" is attached
    to the front of "freegrip"
    "s" is attached to the end of "floatingposes"
    """

    def __init__(self, objpath, gdb, handpkg, base):
        """
        initialization

        :param objpath: path of the object
        :param base: for the loadFreeAirGrip --> genHandPairs functions (for collision detection)

        author: weiwei
        date: 20161215, tsukuba
        """

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()
        self.hand0 = handpkg.newHandNM(hndcolor=[1,0,0,.1])
        self.hand1 = handpkg.newHandNM(hndcolor=[0,0,1,.1])

        self.objtrimesh = trimesh.load_mesh(objpath)
        self.objnp = pg.packpandanp(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]
        # mat4list is nested list, floatingposemat4 is list (the flat version of mat4lsit), they are essentially the same
        self.mat4list = []
        self.floatingposemat4 = []
        # gridsfloatingposemat4s is self.floatingposemat4 translated to different grids
        self.gridsfloatingposemat4s = []
        self.icos = None

        self.floatinggripids = []
        self.floatinggripmat4s = []
        self.floatinggripcontacts = []
        self.floatinggripnormals = []
        self.floatinggripjawwidth = []
        self.floatinggripidfreeair = []

        self.bulletworld = BulletWorld()
        self.handpairList = []

        self.gdb = gdb

        self.__loadFreeAirGrip(base)

        # for IK
        self.floatinggripikfeas_rgt = []
        self.floatinggripikfeas_handx_rgt = []
        self.floatinggripikfeas_lft = []
        self.floatinggripikfeas_handx_lft = []

        # for ik-feasible pairs
        self.floatinggrippairsids = []
        self.floatinggrippairshndmat4s = []
        self.floatinggrippairscontacts = []
        self.floatinggrippairsnormals = []
        self.floatinggrippairsjawwidths = []
        self.floatinggrippairsidfreeairs = []

        self.__genPandaRotmat4()

    def __loadFreeAirGrip(self, base):
        """
        load self.freegripids, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: weiwei
        date: 20170110
        """

        freeairgripdata = self.gdb.loadFreeAirGrip(self.dbobjname, handname = self.handpkg.getHandName())
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripids = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]

    def __genPandaRotmat4(self, icolevel=1, angles=[0,45,90,135,180,225,270,315]):
        """
        generate panda3d rotmat4 using icospheres and rotationaangle each origin-vertex vector of the icosphere

        :param icolevel, the default value 1 = 42vertices
        :param angles, 8 directions by default
        :return: a list of [pandarotmat4, ...] size of the inner list is size of the angles

        author: weiwei
        date: 20170221, tsukuba
        """

        self.mat4list = []
        self.icos = trimesh.creation.icosphere(icolevel)
        initmat4 = self.objnp.getMat()
        for vert in self.icos.vertices:
            self.mat4list.append([])
            self.objnp.lookAt(vert[0], vert[1], vert[2])
            ytozmat4 = Mat4.rotateMat(-90, self.objnp.getMat().getRow3(0))
            newobjmat4 = self.objnp.getMat()*ytozmat4
            for angle in angles:
                tmppandamat4 = Mat4.rotateMat(angle, newobjmat4.getRow3(2))
                tmppandamat4 = newobjmat4*tmppandamat4
                self.mat4list[-1].append(tmppandamat4)
            self.objnp.setMat(initmat4)
        self.floatingposemat4 = [e for l in self.mat4list for e in l]

    def genHandPairs(self, base, loadser=False):
        self.handpairList = []
        if loadser is False:
            # hand0 = self.handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
            # hand1 = self.handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
            pairidlist = list(itertools.combinations(range(len(self.freegripids)), 2))
            print len(pairidlist)/5000+1
            for i in range(100,len(pairidlist),len(pairidlist)/5000+1):
            # for i0, i1 in pairidlist:
                i0, i1 = pairidlist[i]
                print i, len(pairidlist)
                self.hand0.setMat(pandanpmat4 = self.freegriprotmats[i0])
                self.hand0.setJawwidth(self.freegripjawwidth[i0])
                self.hand1.setMat(pandanpmat4 = self.freegriprotmats[i1])
                self.hand1.setJawwidth(self.freegripjawwidth[i1])
                hndbullnodei0 = cd.genCollisionMeshMultiNp(self.hand0.handnp, base.render)
                hndbullnodei1 = cd.genCollisionMeshMultiNp(self.hand1.handnp, base.render)
                result = self.bulletworld.contactTestPair(hndbullnodei0, hndbullnodei1)
                if not result.getNumContacts():
                    self.handpairList.append([self.freegripids[i0], self.freegripids[i1]])
            pickle.dump(self.handpairList, open("tmp.pickle", mode="wb"))
        else:
            self.handpairList = pickle.load(open("tmp.pickle", mode="rb"))


    def genFPandGs(self, grids, icolevel=1, angles=[0,45,90,135,180,225,270,315]):
        """
        genterate floating poses and their grasps, this function leverages genPandaRotmat4 and transformGrips

        :param icolevel
        :param angles see genPandaRotmat4
        :return:

        author: weiwei
        date: 20170221
        """

        self.gridsfloatingposemat4s = []
        self.__genPandaRotmat4(icolevel, angles)
        for gridposition in grids:
            for posemat4 in self.floatingposemat4:
                tmpposemat4 = Mat4(posemat4)
                tmpposemat4.setRow(3, Vec3(gridposition[0], gridposition[1], gridposition[2]))
                self.gridsfloatingposemat4s.append(tmpposemat4)
        self.transformGrips()

    def saveToDB(self):
        """
        save the floatingposes and their grasps to the database

        :return:

        author: weiwei
        date: 20170221
        """

        sql = "SELECT * FROM floatingposes, object WHERE floatingposes.idobject = object.idobject \
                        AND object.name LIKE '%s'" % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) == 0:
            # the gridsfloatingposes for the self.dbobjname is not saved
            sql = "INSERT INTO floatingposes(rotmat, idobject) VALUES "
            for i in range(len(self.gridsfloatingposemat4s)):
                # print i, "/", len(self.gridsfloatingposemat4s)
                sql += "('%s', (SELECT idobject FROM object WHERE name LIKE '%s')), " % \
                       (dc.mat4ToStr(self.gridsfloatingposemat4s[i]), self.dbobjname)
            sql = sql[:-2] + ";"
            self.gdb.execute(sql)

        sql = "SELECT * FROM floatinggrips,floatingposes,object,freeairgrip,hand WHERE \
                        floatinggrips.idfloatingposes = floatingposes.idfloatingposes AND \
                        floatingposes.idobject = object.idobject AND \
                        object.name LIKE '%s' AND floatinggrips.idfreeairgrip=freeairgrip.idfreeairgrip AND \
                        freeairgrip.idhand = hand.idhand AND hand.name LIKE '%s'" % (self.dbobjname, self.handpkg.getHandName())
        result = self.gdb.execute(sql)
        if len(result) == 0:
            for i in range(len(self.gridsfloatingposemat4s)):
                sql = "SELECT floatingposes.idfloatingposes FROM floatingposes,object WHERE \
                        floatingposes.idobject = object.idobject AND \
                        floatingposes.rotmat LIKE '%s' AND \
                        object.name LIKE '%s'" % (dc.mat4ToStr(self.gridsfloatingposemat4s[i]), self.dbobjname)
                result = self.gdb.execute(sql)[0]
                if len(result) != 0:
                    idfloatingposes = result[0]
                    sql = "SELECT * FROM floatinggrips WHERE idfloatingposes = %d" % idfloatingposes
                    result = self.gdb.execute(sql)
                    if len(result) == 0:
                        if len(self.floatinggripmat4s[i]) != 0:
                            sql = "INSERT INTO floatinggrips(contactpoint0, contactpoint1, contactnormal0, contactnormal1, \
                                    rotmat, jawwidth, idfloatingposes, idfreeairgrip) VALUES "
                            for j in range(len(self.floatinggripmat4s[i])):
                                # print "gripposes", i, "/", len(self.gridsfloatingposemat4s)
                                # print  "grips", j, "/", len(self.floatinggripmat4s[i])
                                cct0 = self.floatinggripcontacts[i][j][0]
                                cct1 = self.floatinggripcontacts[i][j][1]
                                cctn0 = self.floatinggripnormals[i][j][0]
                                cctn1 = self.floatinggripnormals[i][j][1]
                                sql += "('%s', '%s', '%s', '%s', '%s', '%s', %d, %d), " % \
                                       (dc.v3ToStr(cct0), dc.v3ToStr(cct1), dc.v3ToStr(cctn0), dc.v3ToStr(cctn1), \
                                        dc.mat4ToStr(self.floatinggripmat4s[i][j]), str(self.floatinggripjawwidth[i][j]), \
                                        idfloatingposes, self.floatinggripidfreeair[i][j])
                            sql = sql[:-2] + ";"
                            self.gdb.execute(sql)

    def loadFromDB(self):
        """
        load the floatingposes and their grasps from the database

        :return:

        author: weiwei
        date: 20170227
        """

        self.gridsfloatingposemat4s = []
        self.floatinggripids = []
        self.floatinggripmat4s = []
        self.floatinggripcontacts = []
        self.floatinggripnormals = []
        self.floatinggripjawwidth = []
        self.floatinggripidfreeair = []
        sql = "SELECT * FROM floatingposes, object WHERE floatingposes.idobject = object.idobject \
                        AND object.name LIKE '%s'" % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) != 0:
            for resultrow in result:
                self.gridsfloatingposemat4s.append(dc.strToMat4(resultrow[1]))
                idfloatingposes = resultrow[0]
                sql = "SELECT floatinggrips.idfloatinggrips, floatinggrips.contactpoint0, \
                        floatinggrips.contactpoint1, floatinggrips.contactnormal0, \
                        floatinggrips.contactnormal1, floatinggrips.rotmat, floatinggrips.jawwidth, \
                        floatinggrips.idfreeairgrip FROM floatinggrips,freeairgrip,hand WHERE \
                        floatinggrips.idfreeairgrip = freeairgrip.idfreeairgrip AND \
                        freeairgrip.idhand = hand.idhand AND floatinggrips.idfloatingposes = %d AND \
                        hand.name = '%s'" % (idfloatingposes, self.handpkg.getHandName())
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    floatinggripids = []
                    floatinggripmat4s = []
                    floatinggripcontacts = []
                    floatinggripnormals = []
                    floatinggripjawwidths = []
                    floatinggripidfreeairs = []
                    for resultrow in result:
                        cct0 = dc.strToV3(resultrow[1])
                        cct1 = dc.strToV3(resultrow[2])
                        cctn0 = dc.strToV3(resultrow[3])
                        cctn1 = dc.strToV3(resultrow[4])
                        floatinggripids.append(int(resultrow[0]))
                        floatinggripmat4s.append(dc.strToMat4(resultrow[5]))
                        floatinggripcontacts.append([cct0, cct1])
                        floatinggripnormals.append([cctn0, cctn1])
                        floatinggripjawwidths.append(float(resultrow[6]))
                        floatinggripidfreeairs.append(int(resultrow[7]))
                    self.floatinggripids.append(floatinggripids)
                    self.floatinggripmat4s.append(floatinggripmat4s)
                    self.floatinggripcontacts.append(floatinggripcontacts)
                    self.floatinggripnormals.append(floatinggripnormals)
                    self.floatinggripjawwidth.append(floatinggripjawwidths)
                    self.floatinggripidfreeair.append(floatinggripidfreeairs)
                else:
                    print 'Plan floating grips first!'
                    assert(False)
        else:
            assert('No object found!')

    def transformGrips(self):
        """
        transform the freeair grips to all rotmat4 in self.gridsfloatingposemat4s

        :return:

        author: weiwei
        date: 20170221, tsukuba
        """

        self.floatinggripmat4s = []
        self.floatinggripcontacts = []
        self.floatinggripnormals = []
        self.floatinggripjawwidth = []
        self.floatinggripidfreeair = []
        for icomat4 in self.gridsfloatingposemat4s:
            floatinggrips = self.transformGripsOnePose(icomat4)
            self.floatinggripmat4s.append(floatinggrips[0])
            self.floatinggripcontacts.append(floatinggrips[1])
            self.floatinggripnormals.append(floatinggrips[2])
            self.floatinggripjawwidth.append(floatinggrips[3])
            self.floatinggripidfreeair.append(floatinggrips[4])


    def transformGripsOnePose(self, rotmat4):
        """
        transform the freeair grips to one specific rotmat4

        :param rotmat4:
        :return: [floatinggripmat4s, floatinggripcontacts, floatinggripnormals, floatinggripjawwidths, floatinggripidfreeairs]
        each element in the list is also a list
        """

        floatinggripmat4s = []
        floatinggripcontacts = []
        floatinggripnormals = []
        floatinggripjawwidths = []
        floatinggripidfreeairs = []
        for i, gripmat4 in enumerate(self.freegriprotmats):
            floatinggripmat4 = gripmat4 * rotmat4
            cct0 = rotmat4.xformPoint(self.freegripcontacts[i][0])
            cct1 = rotmat4.xformPoint(self.freegripcontacts[i][1])
            cctn0 = rotmat4.xformPoint(self.freegripnormals[i][0])
            cctn1 = rotmat4.xformPoint(self.freegripnormals[i][1])
            floatinggripjawwidth = self.freegripjawwidth[i]
            floatinggripidfreeair = self.freegripids[i]
            floatinggripmat4s.append(floatinggripmat4)
            floatinggripcontacts.append([cct0, cct1])
            floatinggripnormals.append([cctn0, cctn1])
            floatinggripjawwidths.append(floatinggripjawwidth)
            floatinggripidfreeairs.append(floatinggripidfreeair)
        return [floatinggripmat4s, floatinggripcontacts, floatinggripnormals,
                floatinggripjawwidths, floatinggripidfreeairs]

    def showIcomat4s(self, nodepath):
        """
        show the pandamat4s generated by genPandaRotmat4

        :param nodepath, where np to repreantTo, usually base.render
        :return:

        author: weiwei
        date: 20170221, tsukuba
        """

        for i, icomat4list in enumerate(self.mat4list):
            vert = self.icos.vertices*100
            spos = Vec3(vert[i][0], vert[i][1], vert[i][2])
            for icomat4 in icomat4list:
                pg.plotAxisSelf(nodepath, spos, icomat4, length=100, thickness=3)

    def updateDBwithFGPairs(self, loadser = False):
        """
        compute the floatinggrippairs using freegrippairs and
        save the floatinggripspairs into Database

        :param loadser whether use serialized data for handpairlist
        :return:

        author: weiwei
        date: 20170301
        """

        if len(self.handpairList) == 0:
            self.genHandPairs(base, loadser)
        tic = time.clock()
        for fpid in range(len(self.gridsfloatingposemat4s)):
            toc = time.clock()
            print toc-tic
            if fpid != 0:
                print "remaining time", (toc-tic)*len(self.gridsfloatingposemat4s)/fpid-(toc-tic)
            print fpid, len(self.gridsfloatingposemat4s)
            # gen correspondence between freeairgripid and index
            # indfgoffa means index of floatinggrips whose freeairgripid are xxx
            indfgoffa = {}
            for i in range(len(self.floatinggripidfreeair[fpid])):
                indfgoffa[self.floatinggripidfreeair[fpid][i]] = i
            # handidpair_indfg is the pairs using index of floatinggrips
            handidpair_indfg = []
            for handidpair in self.handpairList:
                handidpair_indfg.append([indfgoffa[handidpair[0]], indfgoffa[handidpair[1]]])
                # if handidpair_indfg[0] is right, 1 is left
                sql = "INSERT IGNORE INTO floatinggripspairs VALUES (%d, %d)" % \
                        (self.floatinggripids[fpid][handidpair_indfg[-1][0]], self.floatinggripids[fpid][handidpair_indfg[-1][1]])
                self.gdb.execute(sql)
                # if 1 is right, 0 is left
                sql = "INSERT IGNORE INTO floatinggripspairs VALUES (%d, %d)" % \
                        (self.floatinggripids[fpid][handidpair_indfg[-1][1]], self.floatinggripids[fpid][handidpair_indfg[-1][0]])
                self.gdb.execute(sql)

    def loadIKFeasibleFGPairsFromDB(self, robot):
        """
        load the IK FeasibleFGPairs
        :return:

        author: weiwei
        date: 20170301
        """

        self.loadFromDB()
        self.loadIKFromDB(robot)

        idrobot = self.gdb.loadIdRobot(robot)
        idarmrgt = self.gdb.loadIdArm('rgt')
        idarmlft = self.gdb.loadIdArm('lft')

        self.floatinggrippairsids = []
        self.floatinggrippairshndmat4s = []
        self.floatinggrippairscontacts = []
        self.floatinggrippairsnormals = []
        self.floatinggrippairsjawwidths = []
        self.floatinggrippairsidfreeairs = []
        for fpid in range(len(self.gridsfloatingposemat4s)):
            sql = "SELECT floatingposes.idfloatingposes FROM floatingposes, object WHERE floatingposes.idobject = object.idobject \
                            AND object.name LIKE '%s' AND floatingposes.rotmat LIKE '%s'" % \
                  (self.dbobjname, dc.mat4ToStr(self.gridsfloatingposemat4s[fpid]))
            result = self.gdb.execute(sql)
            if len(result) != 0:
                idfloatingposes = result[0][0]
                floatinggrippairsids = []
                floatinggrippairshndmat4s = []
                floatinggrippairscontacts = []
                floatinggrippairsnormals = []
                floatinggrippairsjawwidths = []
                floatinggrippairsidfreeairs = []
                sql = "SELECT floatinggripspairs.idfloatinggrips0, floatinggripspairs.idfloatinggrips1, \
                        fg0.contactpoint0, fg0.contactpoint1, fg0.contactnormal0, fg0.contactnormal1, fg0.rotmat, \
                        fg0.jawwidth, fg0.idfreeairgrip, \
                        fg1.contactpoint0, fg1.contactpoint1, fg1.contactnormal0, fg1.contactnormal1, fg1.rotmat, \
                        fg1.jawwidth, fg1.idfreeairgrip FROM floatinggripspairs, floatinggrips fg0, floatinggrips fg1, \
                        ikfloatinggrips ikfg0, ikfloatinggrips ikfg1  WHERE \
                        floatinggripspairs.idfloatinggrips0 = fg0.idfloatinggrips AND \
                        floatinggripspairs.idfloatinggrips1 = fg1.idfloatinggrips AND \
                        fg0.idfloatingposes = %d AND fg1.idfloatingposes = %d AND \
                        fg0.idfloatinggrips = ikfg0.idfloatinggrips AND ikfg0.feasibility like 'True' AND ikfg0.feasibility_handx like 'True' AND \
                        ikfg0.idrobot = %d AND ikfg0.idarm = %d AND \
                        fg1.idfloatinggrips = ikfg1.idfloatinggrips AND ikfg1.feasibility like 'True' AND ikfg1.feasibility_handx like 'True' AND \
                        ikfg1.idrobot = %d AND ikfg1.idarm = %d" % (idfloatingposes, idfloatingposes, idrobot, idarmrgt, idrobot, idarmlft)
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    for resultrow in result:
                        floatinggrippairsids.append([resultrow[0], resultrow[1]])
                        floatinggrippairshndmat4s.append([dc.strToMat4(resultrow[6]), dc.strToMat4(resultrow[13])])
                        rgtcct0 = dc.strToV3(resultrow[2])
                        rgtcct1 = dc.strToV3(resultrow[3])
                        lftcct0 = dc.strToV3(resultrow[9])
                        lftcct1 = dc.strToV3(resultrow[10])
                        floatinggrippairscontacts.append([[rgtcct0, rgtcct1], [lftcct0, lftcct1]])
                        rgtcctn0 = dc.strToV3(resultrow[4])
                        rgtcctn1 = dc.strToV3(resultrow[5])
                        lftcctn0 = dc.strToV3(resultrow[11])
                        lftcctn1 = dc.strToV3(resultrow[12])
                        floatinggrippairsnormals.append([[rgtcctn0, rgtcctn1], [lftcctn0, lftcctn1]])
                        floatinggrippairsjawwidths.append([float(resultrow[7]), float(resultrow[14])])
                        floatinggrippairsidfreeairs.append([int(resultrow[8]), int(resultrow[15])])
                self.floatinggrippairsids.append(floatinggrippairsids)
                self.floatinggrippairshndmat4s.append(floatinggrippairshndmat4s)
                self.floatinggrippairscontacts.append(floatinggrippairscontacts)
                self.floatinggrippairsnormals.append(floatinggrippairsnormals)
                self.floatinggrippairsjawwidths.append(floatinggrippairsjawwidths)
                self.floatinggrippairsidfreeairs.append(floatinggrippairsidfreeairs)
        # for i,pairs in enumerate(self.floatinggrippairsids):
        #     print i
        #     print pairs

    def updateDBwithIK(self, robot):
        """
        compute the IK feasible grasps of each pose

        :return:
        """

        # load retraction distances
        rethandx, retworldz, retworlda, worldz = self.gdb.loadIKRet()
        # load idrobot
        idrobot = self.gdb.loadIdRobot(robot)

        self.floatinggripikfeas_rgt = []
        self.floatinggripikfeas_handx_rgt = []
        self.floatinggripikfeas_lft = []
        self.floatinggripikfeas_handx_lft = []
        tic = time.clock()
        for fpid in range(len(self.gridsfloatingposemat4s)):
            toc = time.clock()
            print toc-tic
            if fpid != 0:
                print "remaining time", (toc-tic)*len(self.gridsfloatingposemat4s)/fpid-(toc-tic)
            print fpid, len(self.gridsfloatingposemat4s)
            ### right hand
            armname = 'rgt'
            feasibility = []
            feasibility_handx = []
            for i, hndrotmat4 in enumerate(self.floatinggripmat4s[fpid]):
                feasibility.append('False')
                feasibility_handx.append('False')
                fpgsfgrcenter = (self.floatinggripcontacts[fpid][i][0]+self.floatinggripcontacts[fpid][i][1])/2
                fpgsfgrcenternp = pg.v3ToNp(fpgsfgrcenter)
                fpgsrotmat3np = pg.mat3ToNp(hndrotmat4.getUpper3())
                handx =  hndrotmat4.getRow3(0)
                # check the angle between handx and minus y
                minusworldy = Vec3(0,-1,0)
                if Vec3(handx).angleDeg(minusworldy) < 60:
                    fpgsfgrcenternp_handx = pg.v3ToNp(fpgsfgrcenter+handx*rethandx)
                    if robot.numik(fpgsfgrcenternp, fpgsrotmat3np, armname) is not None:
                        feasibility[i] = 'True'
                    if robot.numik(fpgsfgrcenternp_handx, fpgsrotmat3np, armname) is not None:
                        feasibility_handx[i] = 'True'
            self.floatinggripikfeas_rgt.append(feasibility)
            self.floatinggripikfeas_handx_rgt.append(feasibility_handx)
            ### left hand
            armname = 'lft'
            feasibility = []
            feasibility_handx = []
            for i, hndrotmat4 in enumerate(self.floatinggripmat4s[fpid]):
                feasibility.append('False')
                feasibility_handx.append('False')
                fpgsfgrcenter = (self.floatinggripcontacts[fpid][i][0]+self.floatinggripcontacts[fpid][i][1])/2
                fpgsfgrcenternp = pg.v3ToNp(fpgsfgrcenter)
                fpgsrotmat3np = pg.mat3ToNp(hndrotmat4.getUpper3())
                handx =  hndrotmat4.getRow3(0)
                # check the angle between handx and minus y
                plusworldy = Vec3(0,1,0)
                if Vec3(handx).angleDeg(plusworldy) < 60:
                    fpgsfgrcenternp_handx = pg.v3ToNp(fpgsfgrcenter+handx*rethandx)
                    if robot.numik(fpgsfgrcenternp, fpgsrotmat3np, armname) is not None:
                        feasibility[i] = 'True'
                    if robot.numik(fpgsfgrcenternp_handx, fpgsrotmat3np, armname) is not None:
                        feasibility_handx[i] = 'True'
            self.floatinggripikfeas_lft.append(feasibility)
            self.floatinggripikfeas_handx_lft.append(feasibility_handx)

        self.saveIKtoDB(idrobot)

    def saveIKtoDB(self, idrobot):
        """
        saveupdated IK to DB
        this function is separated from updateDBwithIK for illustration

        :return:
        """

        for fpid in range(len(self.gridsfloatingposemat4s)):
            # right arm
            idarm = self.gdb.loadIdArm('rgt')
            for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
                sql = "INSERT IGNORE INTO ikfloatinggrips(idrobot, idarm, idfloatinggrips, feasibility, \
                      feasibility_handx) VALUES (%d, %d, %d, '%s', '%s')" \
                      % (idrobot, idarm, idfloatinggrips, self.floatinggripikfeas_rgt[fpid][i],
                         self.floatinggripikfeas_handx_rgt[fpid][i])
                gdb.execute(sql)
            # left arm
            idarm = self.gdb.loadIdArm('lft')
            for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
                sql = "INSERT IGNORE INTO ikfloatinggrips(idrobot, idarm, idfloatinggrips, feasibility, \
                      feasibility_handx) VALUES (%d, %d, %d, '%s', '%s')" \
                      % (idrobot, idarm, idfloatinggrips, self.floatinggripikfeas_lft[fpid][i],
                         self.floatinggripikfeas_handx_lft[fpid][i])
                gdb.execute(sql)

    def loadIKFromDB(self, robot):
        """
        load the feasibility of IK from db

        :param robot:
        :return:

        author: weiwei
        date: 20170301
        """

        idrobot = self.gdb.loadIdRobot(robot)

        self.floatinggripikfeas_rgt = []
        self.floatinggripikfeas_handx_rgt = []
        self.floatinggripikfeas_lft = []
        self.floatinggripikfeas_handx_lft = []
        for fpid in range(len(self.gridsfloatingposemat4s)):
            # right arm
            feasibility = []
            feasibility_handx = []
            idarm = self.gdb.loadIdArm('rgt')
            for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
                feasibility.append('False')
                feasibility_handx.append('False')
                sql = "SELECT feasibility, feasibility_handx FROM ikfloatinggrips WHERE \
                        idrobot = %d AND idarm = %d and idfloatinggrips = %d" % (idrobot, idarm, idfloatinggrips)
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    feasibility[i] = result[0][0]
                    feasibility_handx[i] = result[0][1]
            self.floatinggripikfeas_rgt.append(feasibility)
            self.floatinggripikfeas_handx_rgt.append(feasibility_handx)
            # left arm
            feasibility = []
            feasibility_handx = []
            idarm = self.gdb.loadIdArm('lft')
            for i, idfloatinggrips in enumerate(self.floatinggripids[fpid]):
                feasibility.append('False')
                feasibility_handx.append('False')
                sql = "SELECT feasibility, feasibility_handx FROM ikfloatinggrips WHERE \
                        idrobot = %d AND idarm = %d and idfloatinggrips = %d" % (idrobot, idarm, idfloatinggrips)
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    feasibility[i] = result[0][0]
                    feasibility_handx[i] = result[0][1]
            self.floatinggripikfeas_lft.append(feasibility)
            self.floatinggripikfeas_handx_lft.append(feasibility_handx)

    def plotOneFPandG(self, parentnp, fpid=0):
        """
        plot the objpose and grasps at a specific rotmat4

        :param fpid: the index of self.floatingposemat4
        :return:

        author: weiwei
        date: 20170221, tsukuba
        """

        objnp = pg.packpandanp(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        objnp.setMat(self.gridsfloatingposemat4s[fpid])
        objnp.reparentTo(parentnp)
        for i, hndrotmat4 in enumerate(self.gridsfloatingposemat4s[fpid]):
            if i == 7:
                # show grasps
                hand0 = self.handpkg.newHandNM(hndcolor=[0, 1, 0, 1])
                hand0.setMat(pandanpmat4 = hndrotmat4)
                hand0.setJawwidth(self.floatinggripjawwidth[fpid][i])
                hand0.reparentTo(parentnp)
                print self.handpairList
                for handidpair in self.handpairList:
                    if handidpair[0] == self.floatinggripidfreeair[fpid][i]:
                        pairedilist = [i1 for i1 in range(len(self.floatinggripidfreeair[fpid]))
                                       if self.floatinggripidfreeair[fpid][i1]==handidpair[1]]
                        print pairedilist
                        i1 = pairedilist[0]
                        # if self.floatinggripikfeas_lft[fpid][i1] == 'True':
                        hand1 = self.handpkg.newHandNM(hndcolor=[0, 1, 1, 1])
                        hndrotmat4 = self.floatinggripmat4s[fpid][i1]
                        hand1.setMat(pandanpmat4 = hndrotmat4)
                        hand1.setJawwidth(self.floatinggripjawwidth[fpid][i1])
                        hand1.reparentTo(parentnp)
                    if handidpair[1] == self.floatinggripidfreeair[fpid][i]:
                        pairedilist = [i1 for i1 in range(len(self.floatinggripidfreeair[fpid]))
                                       if self.floatinggripidfreeair[fpid][i1]==handidpair[0]]
                        print pairedilist
                        i1 = pairedilist[0]
                        # if self.floatinggripikfeas_lft[fpid][i1] == 'True':
                        hand1 = self.handpkg.newHandNM(hndcolor=[0, 1, 1, 1])
                        hndrotmat4 = self.floatinggripmat4s[fpid][i1]
                        hand1.setMat(pandanpmat4 = hndrotmat4)
                        hand1.setJawwidth(self.floatinggripjawwidth[fpid][i1])
                        hand1.reparentTo(parentnp)

    def plotOneFPandGPairs(self, parentnp, fpid=0):
        """
        plot the gpairss at a specific rotmat4

        :param fpid: the index of self.floatingposemat4
        :return:

        author: weiwei
        date: 20170301, tsukuba
        """

        objnp = pg.packpandanp(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        objnp.setMat(self.gridsfloatingposemat4s[fpid])
        objnp.setColor(Vec4(.7,0.3,0,1))
        objnp.reparentTo(parentnp)
        print self.floatinggrippairshndmat4s[fpid]
        for i, hndrotmat4pair in enumerate(self.floatinggrippairshndmat4s[fpid]):
            # if i == 9:
            # show grasps
            hand0 = self.handpkg.newHandNM(hndcolor=[1, 0, 0, .5])
            hand0mat4 = Mat4(hndrotmat4pair[0])
            # h0row3 = hand0mat4.getRow3(3)
            # h0row3[2] = h0row3[2]+i*20.0
            # hand0mat4.setRow(3, h0row3[2])
            hand0.setMat(pandanpmat4 = hand0mat4)
            hand0.setJawwidth(self.floatinggrippairsjawwidths[fpid][i][0])
            hand0.reparentTo(parentnp)
            hand1 = self.handpkg.newHandNM(hndcolor=[0, .0, 1, .5])
            hand1mat4 = Mat4(hndrotmat4pair[1])
            # h1row3 = hand1mat4.getRow3(3)
            # h1row3[2] = h1row3[2]+i*20.0
            # hand1mat4.setRow(3, h1row3[2])
            hand1.setMat(pandanpmat4 = hand1mat4)
            hand1.setJawwidth(self.floatinggrippairsjawwidths[fpid][i][1])
            hand1.reparentTo(parentnp)

if __name__=="__main__":

    # show in panda3d
    from direct.showbase.ShowBase import ShowBase
    from panda3d.core import *
    from panda3d.bullet import BulletSphereShape
    import pandaplotutils.pandageom as pandageom
    import pandaplotutils.pandactrl as pandactrl
    from direct.filter.CommonFilters import CommonFilters

    base = pandactrl.World(lookatp=[0,0,0])

    # handpkg = rtq85nm
    handpkg = hrp5threenm
    gdb = db.GraspDB()

    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "ttube.stl")
    objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "tool2.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "sandpart.stl")
    # objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "planerearstay.stl")
    fpose = FloatingPoses(objpath, gdb, handpkg, base)
    # fpose.showIcomat4s(base.render)

    # usage:
    # genFPandGs generate floatingposes and grips
    # saveToDB save the generated FP and Gs into database
    # updateDBwithFGPairs compute the FGpairs without considering ik
    # updateDBwithIK compute the ik feasible FGs
    # loadIKFeasibleFGPairsFromDB load DBFGpairs and DBwithIK and compute the IK-feasible FGpairs

    grids = []
    for x in range(400,401,100):
        for y in [0]:
            for z in range(150,151,100):
                grids.append([x,y,z])
    fpose.genFPandGs(grids)
    fpose.saveToDB()
    fpose.loadFromDB()
    fpose.updateDBwithFGPairs()
    # nxtrobot = nxt.NxtRobot()
    # hrp5nrobot = hrp5n.Hrp5NRobot()
    hrp2k = hrp2k.Hrp2KRobot()
    # fpose.updateDBwithIK(robot=nxtrobot)
    # fpose.updateDBwithIK(robot=hrp5nrobot)
    fpose.updateDBwithIK(robot=hrp2k)
    # for i in range(1,len(fpose.gridsfloatingposemat4s),len(fpose.floatingposemat4)):
    #     fpose.plotOneFPandG(base.render, i)
    # fpose.loadIKFeasibleFGPairsFromDB(robot=nxtrobot)
    # fpose.loadIKFeasibleFGPairsFromDB(robot=hrp5nrobot)
    fpose.loadIKFeasibleFGPairsFromDB(robot=hrp2k)
    # fpose.plotOneFPandG(base.render, 0)4
    fpose.plotOneFPandGPairs(base.render, 0)

    # poseid = [0]

    # def updateshow(poseid, task):
    #     gbnpcollection = base.render.findAllMatches("**/+GeomNode")
    #     for gbnp in gbnpcollection:
    #         gbnp.detachNode()
    #     if poseid[0] >= len(fpose.gridsfloatingposemat4s):
    #         poseid[0] = 0
    #     for i in range(poseid[0],len(fpose.gridsfloatingposemat4s),len(fpose.floatingposemat4)):
    #         fpose.plotOneFPandG(base.render, i)
    #     poseid[0] = poseid[0]+1
    #     return task.again
    #
    # taskMgr.doMethodLater(.2, updateshow, "updateshow", extraArgs=[poseid], appendTask=True)

    base.run()