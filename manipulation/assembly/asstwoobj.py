#!/usr/bin/python

import os
import itertools
import trimesh
import pandaplotutils.pandageom as pg
from utils import collisiondetection as cd
from manipulation.grip.freegrip import FreeAirGrip as Fag

from panda3d.core import *
from utils import dbcvt as dc
from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
from manipulation.grip.robotiq85 import rtq85nm
from panda3d.bullet import BulletWorld
import time
from robotsim.nextage import nxt

class TwoObjAss(object):
    """
    the class uses genAvailableFAGs to compute grasps for the assembly of two objects
    different rotations could be saved to database
    """

    def __init__(self, base, obj0path, obj0Mat4, obj1path, obj1Mat4, assDirect1to0, gdb, handpkg):
        """
        initiliazation
        obj0 will be manipulated by rgt hand, obj1 will be manipulated by lft hand
        # see genAvailableFAGs function for the details of parameters

        :param base:
        :param obj0path:
        :param obj0Mat4:
        :param obj1path:
        :param obj1Mat4:
        :param assDirect1to0: the assembly direction to assemble object 1 to object 0 (with length)
        :param gdb:
        :param handpkg:
        """

        self.base = base
        self.obj0path = obj0path
        self.obj0Mat4 = obj0Mat4
        self.dbobj0name = os.path.splitext(os.path.basename(obj0path))[0]
        self.obj1path = obj1path
        self.obj1Mat4 = obj1Mat4
        self.dbobj1name = os.path.splitext(os.path.basename(obj1path))[0]
        self.assDirect1to0 = assDirect1to0
        self.gdb = gdb
        self.handpkg = handpkg

        # define the free ass0grip and ass1grip
        # definition: objgrips: [assgripcontacts, assgripnormals, assgriprotmat4s, assgripjawwidth, assgripidfreeair]
        self.obj0grips = []
        self.obj1grips = []

        # ico means they corresponds to ico rotmats
        self.gridsfloatingposemat4s = []

        # right grips for each gridsfloatingposemat4
        self.icoass0gripids = []
        self.icoass0gripcontacts = []
        self.icoass0gripnormals = []
        self.icoass0griprotmat4s = []
        self.icoass0gripjawwidth = []
        self.icoass0gripidfreeair = []

        # left grips for each gridsfloatingposemat4
        self.icoass1gripids = []
        self.icoass1gripcontacts = []
        self.icoass1gripnormals = []
        self.icoass1griprotmat4s = []
        self.icoass1gripjawwidth = []
        self.icoass1gripidfreeair = []

        # ik related
        self.icoass0gripsik = []
        self.icoass0gripsik_retassdir = []
        self.icoass1gripsik = []
        self.icoass1gripsik_retassdir = []

        # handpairList
        self.handpairList = []

        self.bulletworld = BulletWorld()

        # ik feasible pairs for each gridsfloatingposemat4
        self.icoassgrippairsids = []
        self.icoassgrippairscontacts = []
        self.icoassgrippairsnormals = []
        self.icoassgrippairshndmat4s = []
        self.icoassgrippairsjawwidths = []
        self.icoassgrippairsidfreeairs = []

    def genXAssPandGs(self, grids, icolevel=1, angles=[0,45,90,135,180,225,270,315]):
        """
        genterate xass poses and their grasps, this function leverages genPandaRotmat4 and transformGrips

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

        if len(self.obj0grips) == 0 or len(self.obj1grips) == 0:
            self.__genFreeAssGrips()
        self.__transformGrips()

    def saveToDB(self):
        """

        :return:
        """

        self.icoass0gripids = []
        self.icoass1gripids = []
        # save to assembly table, 0-right, 1-left
        idassembly = self.gdb.loadIdAssembly(self.dbobj0name, self.obj0Mat4, self.dbobj1name, self.obj1Mat4, self.assDirect1to0)
        # check if rotation already exist
        sql = "SELECT * FROM assemblyx WHERE assemblyx.idassembly = %d" % idassembly
        result = self.gdb.execute(sql)
        if len(result) == 0:
            # save rotation and grasps
            # if assemblyx do not exist, assemblyxgrip0/1 dont exist
            for i, poserotmat4 in enumerate(self.gridsfloatingposemat4s):
                sql = "INSERT INTO assemblyx(idassembly, rotmat) VALUES (%d, '%s')" % (idassembly, dc.mat4ToStr(poserotmat4))
                idassemblyx = self.gdb.execute(sql)
                # object 0
                self.icoass0gripids.append([])
                for j, hndrotmat4 in enumerate(self.icoass0griprotmat4s[i]):
                    strcct0 = dc.v3ToStr(self.icoass0gripcontacts[i][j][0])
                    strcct1 = dc.v3ToStr(self.icoass0gripcontacts[i][j][1])
                    strcctn0 = dc.v3ToStr(self.icoass0gripnormals[i][j][0])
                    strcctn1 = dc.v3ToStr(self.icoass0gripnormals[i][j][1])
                    strrotmat = dc.mat4ToStr(hndrotmat4)
                    strjawwidth = str(self.icoass0gripjawwidth[i][j])
                    idfreeairgrip = self.icoass0gripidfreeair[i][j]
                    sql = "INSERT INTO assemblyxgrips0(contactpoint0, contactpoint1, contactnormal0, contactnormal1, \
                            rotmat, jawwidth, idassemblyx, idfreeairgrip) VALUES ('%s', '%s', '%s', '%s', '%s', '%s', %d, \
                            %d)" % (strcct0, strcct1, strcctn0, strcctn1, strrotmat, strjawwidth, idassemblyx, idfreeairgrip)
                    idaxg = self.gdb.execute(sql)
                    self.icoass0gripids[-1].append(idaxg)
                # object 1
                self.icoass1gripids.append([])
                for j, hndrotmat4 in enumerate(self.icoass1griprotmat4s[i]):
                    strcct0 = dc.v3ToStr(self.icoass1gripcontacts[i][j][0])
                    strcct1 = dc.v3ToStr(self.icoass1gripcontacts[i][j][1])
                    strcctn0 = dc.v3ToStr(self.icoass1gripnormals[i][j][0])
                    strcctn1 = dc.v3ToStr(self.icoass1gripnormals[i][j][1])
                    strrotmat = dc.mat4ToStr(hndrotmat4)
                    strjawwidth = str(self.icoass1gripjawwidth[i][j])
                    idfreeairgrip = self.icoass1gripidfreeair[i][j]
                    sql = "INSERT INTO assemblyxgrips1(contactpoint0, contactpoint1, contactnormal0, contactnormal1, \
                            rotmat, jawwidth, idassemblyx, idfreeairgrip) VALUES ('%s', '%s', '%s', '%s', '%s', '%s', %d, \
                            %d)" % (strcct0, strcct1, strcctn0, strcctn1, strrotmat, strjawwidth, idassemblyx, idfreeairgrip)
                    idaxg = self.gdb.execute(sql)
                    self.icoass1gripids[-1].append(idaxg)
        # TODO: Have to delete assemblyx when grasps are deleted
        # TODO: write algorithms to enable insertion of grasps using idassemblyx

    def loadFromDB(self):
        """

        :return:
        """

        self.gridsfloatingposemat4s = []

        self.icoass0gripids = []
        self.icoass0gripcontacts = []
        self.icoass0gripnormals = []
        self.icoass0griprotmat4s = []
        self.icoass0gripjawwidth = []
        self.icoass0gripidfreeair = []

        self.icoass1gripids = []
        self.icoass1gripcontacts = []
        self.icoass1gripnormals = []
        self.icoass1griprotmat4s = []
        self.icoass1gripjawwidth = []
        self.icoass1gripidfreeair = []

        idassembly = self.gdb.loadIdAssembly(self.dbobj0name, self.obj0Mat4, self.dbobj1name, self.obj1Mat4, self.assDirect1to0)
        # check if rotation already exist
        sql = "SELECT assemblyx.idassemblyx, assemblyx.rotmat FROM assemblyx WHERE assemblyx.idassembly = %d" % idassembly
        result = self.gdb.execute(sql)
        if len(result) != 0:
            for resultrow in result:
                poserotmat4 = dc.strToMat4(resultrow[1])
                self.gridsfloatingposemat4s.append(poserotmat4)
                idassemblyx = resultrow[0]
                # g0
                sql = "SELECT * FROM assemblyxgrips0 WHERE idassemblyx = %d" % idassemblyx
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    self.icoass0gripids.append([])
                    self.icoass0gripcontacts.append([])
                    self.icoass0gripnormals.append([])
                    self.icoass0griprotmat4s.append([])
                    self.icoass0gripjawwidth.append([])
                    self.icoass0gripidfreeair.append([])
                    for resultrow in result:
                        self.icoass0gripids[-1].append(resultrow[0])
                        cct0 = dc.strToV3(resultrow[1])
                        cct1 = dc.strToV3(resultrow[2])
                        cctn0 = dc.strToV3(resultrow[3])
                        cctn1 = dc.strToV3(resultrow[4])
                        self.icoass0gripcontacts[-1].append([cct0, cct1])
                        self.icoass0gripnormals[-1].append([cctn0, cctn1])
                        self.icoass0griprotmat4s[-1].append(dc.strToMat4(resultrow[5]))
                        self.icoass0gripjawwidth[-1].append(float(resultrow[6]))
                        self.icoass0gripidfreeair[-1].append(int(resultrow[8]))
                # g1
                sql = "SELECT * FROM assemblyxgrips1 WHERE idassemblyx = %d" % idassemblyx
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    self.icoass1gripids.append([])
                    self.icoass1gripcontacts.append([])
                    self.icoass1gripnormals.append([])
                    self.icoass1griprotmat4s.append([])
                    self.icoass1gripjawwidth.append([])
                    self.icoass1gripidfreeair.append([])
                    for resultrow in result:
                        self.icoass1gripids[-1].append(resultrow[0])
                        cct0 = dc.strToV3(resultrow[1])
                        cct1 = dc.strToV3(resultrow[2])
                        cctn0 = dc.strToV3(resultrow[3])
                        cctn1 = dc.strToV3(resultrow[4])
                        self.icoass1gripcontacts[-1].append([cct0, cct1])
                        self.icoass1gripnormals[-1].append([cctn0, cctn1])
                        self.icoass1griprotmat4s[-1].append(dc.strToMat4(resultrow[5]))
                        self.icoass1gripjawwidth[-1].append(float(resultrow[6]))
                        self.icoass1gripidfreeair[-1].append(int(resultrow[8]))

    def updateDBwithHandPairs(self):
        """
        compute the assgrippairs using assfreegrippairs and
        save the assgrippairs into Database

        :return:

        author: weiwei
        date: 20170307
        """

        self.__genHandPairs(base)

        tic = time.clock()
        for fpid in range(len(self.gridsfloatingposemat4s)):
            toc = time.clock()
            print toc-tic
            if fpid != 0:
                print "remaining time", (toc-tic)*len(self.gridsfloatingposemat4s)/fpid-(toc-tic)
            print fpid, len(self.gridsfloatingposemat4s)
            # gen correspondence between freeairgripid and index
            # indagoffa means index of assgrips whose freeairgripid are xxx
            indagoffa0 = {}
            for i in range(len(self.icoass0gripidfreeair[fpid])):
                indagoffa0[self.icoass0gripidfreeair[fpid][i]] = i
            indagoffa1 = {}
            for i in range(len(self.icoass1gripidfreeair[fpid])):
                indagoffa1[self.icoass1gripidfreeair[fpid][i]] = i
            # handidpair_indag is the pairs using index of floatinggrips
            handidpair_indag = []
            for handidpair in self.handpairList:
                handidpair_indag.append([indagoffa0[handidpair[0]], indagoffa1[handidpair[1]]])
                sql = "INSERT IGNORE INTO assemblyxgrippairs VALUES (%d, %d)" % \
                        (self.icoass0gripids[fpid][handidpair_indag[-1][0]], self.icoass1gripids[fpid][handidpair_indag[-1][1]])
                self.gdb.execute(sql)

    def updateDBwithIK(self, robot):
        """
        compute the IK feasible grasps of each pose

        :return:
        """

        # load idrobot
        idrobot = self.gdb.loadIdRobot(robot)

        self.assgikfeas0 = []
        self.assgikfeas0_retassdir = []
        self.assgikfeas1= []
        self.assgikfeas1_retassdir = []
        tic = time.clock()
        for fpid, apmat in enumerate(self.gridsfloatingposemat4s):
            assdir1to0 = apmat.xformVec(self.assDirect1to0)
            assdir0to1 = apmat.xformVec(-self.assDirect1to0)
            toc = time.clock()
            print toc-tic
            if fpid != 0:
                print "remaining time", (toc-tic)*len(self.gridsfloatingposemat4s)/fpid-(toc-tic)
            print fpid, len(self.gridsfloatingposemat4s)
            ### right hand # rgt = 0
            armname = 'rgt'
            assgikfeas0 = []
            assgikfeas0_retassdir = []
            for i, hndrotmat4 in enumerate(self.icoass0griprotmat4s[fpid]):
                assgikfeas0.append('False')
                assgikfeas0_retassdir.append('False')
                assgsfgrcenter = (self.icoass0gripcontacts[fpid][i][0]+self.icoass0gripcontacts[fpid][i][1])/2
                assgsfgrcenternp = pg.v3ToNp(assgsfgrcenter)
                assgsrotmat3np = pg.mat3ToNp(hndrotmat4.getUpper3())
                handx =  hndrotmat4.getRow3(0)
                # check the angle between handx and minus y
                minusworldy = Vec3(0,-1,0)
                if Vec3(handx).angleDeg(minusworldy) < 60:
                    assgsfgrcenternp_retassdir = pg.v3ToNp(assgsfgrcenter+assdir1to0)
                    if robot.numik(assgsfgrcenternp, assgsrotmat3np, armname) is not None:
                        assgikfeas0[i] = 'True'
                    if robot.numik(assgsfgrcenternp_retassdir, assgsrotmat3np, armname) is not None:
                        assgikfeas0_retassdir[i] = 'True'
            self.icoass0gripsik.append(assgikfeas0)
            self.icoass0gripsik_retassdir.append(assgikfeas0_retassdir)
            ### left hand
            armname = 'lft'
            assgikfeas1 = []
            assgikfeas1_retassdir = []
            for i, hndrotmat4 in enumerate(self.icoass1griprotmat4s[fpid]):
                assgikfeas1.append('False')
                assgikfeas1_retassdir.append('False')
                assgsfgrcenter = (self.icoass1gripcontacts[fpid][i][0]+self.icoass1gripcontacts[fpid][i][1])/2
                assgsfgrcenternp = pg.v3ToNp(assgsfgrcenter)
                assgsrotmat3np = pg.mat3ToNp(hndrotmat4.getUpper3())
                handx =  hndrotmat4.getRow3(0)
                # check the angle between handx and minus y
                plusworldy = Vec3(0,1,0)
                if Vec3(handx).angleDeg(plusworldy) < 60:
                    assgsfgrcenternp_retassdir = pg.v3ToNp(assgsfgrcenter+assdir0to1)
                    if robot.numik(assgsfgrcenternp, assgsrotmat3np, armname) is not None:
                        assgikfeas1[i] = 'True'
                    if robot.numik(assgsfgrcenternp_retassdir, assgsrotmat3np, armname) is not None:
                        assgikfeas1_retassdir[i] = 'True'
            self.icoass1gripsik.append(assgikfeas1)
            self.icoass1gripsik_retassdir.append(assgikfeas1_retassdir)

        self.__saveIKtoDB(idrobot)

    def loadIKFeasibleAGPairsFromDB(self, robot):
        """
        load the IK FeasibleAGPairs AG -> assgrippairs
        :return:

        author: weiwei
        date: 20170301
        """

        self.loadFromDB()
        self.__loadIKFromDB(robot)

        idrobot = self.gdb.loadIdRobot(robot)
        idarmrgt = self.gdb.loadIdArm('rgt')
        idarmlft = self.gdb.loadIdArm('lft')

        self.icoassgrippairsids = []
        self.icoassgrippairscontacts = []
        self.icoassgrippairsnormals = []
        self.icoassgrippairshndmat4s = []
        self.icoassgrippairsjawwidths = []
        self.icoassgrippairsidfreeairs = []
        idassembly = self.gdb.loadIdAssembly(self.dbobj0name, self.obj0Mat4, self.dbobj1name, self.obj1Mat4, self.assDirect1to0)
        for fpid in range(len(self.gridsfloatingposemat4s)):
            sql = "SELECT assemblyx.idassemblyx FROM assemblyx WHERE assemblyx.idassembly = %d AND \
                    assemblyx.rotmat LIKE '%s'" % (idassembly, dc.mat4ToStr(self.gridsfloatingposemat4s[fpid]))
            result = self.gdb.execute(sql)
            if len(result) != 0:
                idfloatingposes = result[0][0]
                icoassgrippairsids = []
                icoassgrippairscontacts = []
                icoassgrippairsnormals = []
                icoassgrippairshndmat4s = []
                icoassgrippairsjawwidths = []
                icoassgrippairsidfreeairs = []
                sql = "SELECT assemblyxgrippairs.idassemblyxgrips0, assemblyxgrippairs.idassemblyxgrips1, \
                        assemblyxgrips0.contactpoint0, assemblyxgrips0.contactpoint1, assemblyxgrips0.contactnormal0, \
                        assemblyxgrips0.contactnormal1, assemblyxgrips0.rotmat, \
                        assemblyxgrips0.jawwidth, assemblyxgrips0.idfreeairgrip, \
                        assemblyxgrips1.contactpoint0, assemblyxgrips1.contactpoint1, assemblyxgrips1.contactnormal0, \
                        assemblyxgrips1.contactnormal1, assemblyxgrips1.rotmat, \
                        assemblyxgrips1.jawwidth, assemblyxgrips1.idfreeairgrip \
                        FROM assemblyxgrippairs, assemblyxgrips0, assemblyxgrips1, \
                        ikassemblyxgrips0, ikassemblyxgrips1 WHERE \
                        assemblyxgrippairs.idassemblyxgrips0 = assemblyxgrips0.idassemblyxgrips0 AND \
                        assemblyxgrippairs.idassemblyxgrips1 = assemblyxgrips1.idassemblyxgrips1 AND \
                        assemblyxgrips0.idassemblyx = %d AND assemblyxgrips1.idassemblyx = %d AND \
                        assemblyxgrips0.idassemblyxgrips0 = ikassemblyxgrips0.idassemblyxgrips0 AND \
                        ikassemblyxgrips0.feasibility like 'True' AND ikassemblyxgrips0.feasibility_assdir like 'True' AND \
                        ikassemblyxgrips0.idrobot = %d AND ikassemblyxgrips0.idarm = %d AND \
                        assemblyxgrips1.idassemblyxgrips1 = ikassemblyxgrips1.idassemblyxgrips1 AND \
                        ikassemblyxgrips1.feasibility like 'True' AND ikassemblyxgrips1.feasibility_assdir like 'True' AND \
                        ikassemblyxgrips1.idrobot = %d AND ikassemblyxgrips1.idarm = %d" % \
                        (idfloatingposes, idfloatingposes, idrobot, idarmrgt, idrobot, idarmlft)
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    for resultrow in result:
                        icoassgrippairsids.append([resultrow[0], resultrow[1]])
                        rgtcct0 = dc.strToV3(resultrow[2])
                        rgtcct1 = dc.strToV3(resultrow[3])
                        lftcct0 = dc.strToV3(resultrow[9])
                        lftcct1 = dc.strToV3(resultrow[10])
                        icoassgrippairscontacts.append([[rgtcct0, rgtcct1], [lftcct0, lftcct1]])
                        rgtcctn0 = dc.strToV3(resultrow[4])
                        rgtcctn1 = dc.strToV3(resultrow[5])
                        lftcctn0 = dc.strToV3(resultrow[11])
                        lftcctn1 = dc.strToV3(resultrow[12])
                        icoassgrippairsnormals.append([[rgtcctn0, rgtcctn1], [lftcctn0, lftcctn1]])
                        icoassgrippairshndmat4s.append([dc.strToMat4(resultrow[6]), dc.strToMat4(resultrow[13])])
                        icoassgrippairsjawwidths.append([float(resultrow[7]), float(resultrow[14])])
                        icoassgrippairsidfreeairs.append([int(resultrow[8]), int(resultrow[15])])
                self.icoassgrippairsids.append(icoassgrippairsids)
                self.icoassgrippairscontacts.append(icoassgrippairscontacts)
                self.icoassgrippairsnormals.append(icoassgrippairsnormals)
                self.icoassgrippairshndmat4s.append(icoassgrippairshndmat4s)
                self.icoassgrippairsjawwidths.append(icoassgrippairsjawwidths)
                self.icoassgrippairsidfreeairs.append(icoassgrippairsidfreeairs)

    def __genHandPairs(self, base):
        self.handpairList = []
        self.__genFreeAssGrips()
        ass0gripcontacts, ass0gripnormals, ass0griprotmat4s, ass0gripjawwidth, ass0gripidfreeair = self.obj0grips
        ass1gripcontacts, ass1gripnormals, ass1griprotmat4s, ass1gripjawwidth, ass1gripidfreeair = self.obj1grips
        hand0 = self.handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
        hand1 = self.handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
        i0list = range(len(ass0gripidfreeair))
        i1list = range(len(ass1gripidfreeair))
        pairidlist = list(itertools.product(i0list, i1list))
        print len(pairidlist)/10000+1
        for i in range(0,len(pairidlist),len(pairidlist)/10000+1):
            # for i0, i1 in pairidlist:
            i0, i1 = pairidlist[i]
            print i, len(pairidlist)
            hand0.setMat(ass0griprotmat4s[i0])
            hand0.setJawwidth(ass0gripjawwidth[i0])
            hand1.setMat(ass1griprotmat4s[i1])
            hand1.setJawwidth(ass1gripjawwidth[i1])
            hndbullnodei0 = cd.genCollisionMeshMultiNp(hand0.handnp, base.render)
            hndbullnodei1 = cd.genCollisionMeshMultiNp(hand1.handnp, base.render)
            result = self.bulletworld.contactTestPair(hndbullnodei0, hndbullnodei1)
            if not result.getNumContacts():
                self.handpairList.append([ass0gripidfreeair[i0], ass1gripidfreeair[i1]])
        # print ass0gripidfreeair
        # print self.icoass0gripidfreeair[0]
        # print ass1gripidfreeair
        # print self.icoass1gripidfreeair[0]
        # print self.handpairList
        # assert "compare"

    def __saveIKtoDB(self, idrobot):
        """
        saveupdated IK to DB
        this function is separated from updateDBwithIK for illustration

        :return:
        """

        for fpid in range(len(self.gridsfloatingposemat4s)):
            # right arm
            idarm = self.gdb.loadIdArm('rgt')
            for i, idicoass0grip in enumerate(self.icoass0gripids[fpid]):
                sql = "INSERT IGNORE INTO ikassemblyxgrips0(idrobot, idarm, idassemblyxgrips0, feasibility, \
                      feasibility_assdir) VALUES (%d, %d, %d, '%s', '%s')" \
                      % (idrobot, idarm, idicoass0grip, self.icoass0gripsik[fpid][i],
                         self.icoass0gripsik_retassdir[fpid][i])
                gdb.execute(sql)
            # left arm
            idarm = self.gdb.loadIdArm('lft')
            for i, idicoass1grip in enumerate(self.icoass1gripids[fpid]):
                sql = "INSERT IGNORE INTO ikassemblyxgrips1(idrobot, idarm, idassemblyxgrips1, feasibility, \
                      feasibility_assdir) VALUES (%d, %d, %d, '%s', '%s')" \
                      % (idrobot, idarm, idicoass1grip, self.icoass1gripsik[fpid][i],
                         self.icoass1gripsik_retassdir[fpid][i])
                gdb.execute(sql)

    def __loadIKFromDB(self, robot):
        """
        load the feasibility of IK from db

        :param robot:
        :return:

        author: weiwei
        date: 20170307
        """

        idrobot = self.gdb.loadIdRobot(robot)

        self.assgikfeas0 = []
        self.assgikfeas0_retassdir = []
        self.assgikfeas1= []
        self.assgikfeas1_retassdir = []
        for fpid in range(len(self.gridsfloatingposemat4s)):
            # right arm
            assgikfeas0 = []
            assgikfeas0_retassdir = []
            idarm = self.gdb.loadIdArm('rgt')
            for i, idassgrips in enumerate(self.icoass0gripids[fpid]):
                assgikfeas0.append('False')
                assgikfeas0_retassdir.append('False')
                sql = "SELECT feasibility, feasibility_assdir FROM ikassemblyxgrips0 WHERE \
                        idrobot = %d AND idarm = %d and idassemblyxgrips0 = %d" % (idrobot, idarm, idassgrips)
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    assgikfeas0[i] = result[0][0]
                    assgikfeas0_retassdir[i] = result[0][1]
                else:
                    assert "Compute ik of the freeassemblygrips first!"
            self.assgikfeas0.append(assgikfeas0)
            self.assgikfeas0_retassdir.append(assgikfeas0_retassdir)
            # left arm
            assgikfeas1 = []
            assgikfeas1_retassdir = []
            idarm = self.gdb.loadIdArm('lft')
            for i, idassgrips in enumerate(self.icoass1gripids[fpid]):
                assgikfeas1.append('False')
                assgikfeas1_retassdir.append('False')
                sql = "SELECT feasibility, feasibility_assdir FROM ikassemblyxgrips1 WHERE \
                        idrobot = %d AND idarm = %d and idassemblyxgrips1 = %d" % (idrobot, idarm, idassgrips)
                result = self.gdb.execute(sql)
                if len(result) != 0:
                    assgikfeas1[i] = result[0][0]
                    assgikfeas1_retassdir[i] = result[0][1]
                else:
                    assert "Compute ik of the freeassemblygrips first!"
            self.assgikfeas1.append(assgikfeas1)
            self.assgikfeas1_retassdir.append(assgikfeas1_retassdir)

    def __genPandaRotmat4(self, icolevel=1, angles=[0, 45, 90, 135, 180, 225, 270, 315]):
        """
        generate panda3d rotmat4 using icospheres and rotationaangle each origin-vertex vector of the icosphere

        :param icolevel, the default value 1 = 42vertices
        :param angles, 8 directions by default
        :return: a list of [pandarotmat4, ...] size of the inner list is size of the angles

        author: weiwei
        date: 20170221, tsukuba (copied from regrasp/floatingposes.py)
        """

        objnp = pg.genObjmnp(self.obj0path)
        mat4list = []
        self.icos = trimesh.creation.icosphere(icolevel)
        initmat4 = objnp.getMat()
        for vert in self.icos.vertices:
            mat4list.append([])
            objnp.lookAt(vert[0], vert[1], vert[2])
            ytozmat4 = Mat4.rotateMat(-90, objnp.getMat().getRow3(0))
            newobjmat4 = objnp.getMat() * ytozmat4
            for angle in angles:
                tmppandamat4 = Mat4.rotateMat(angle, newobjmat4.getRow3(2))
                tmppandamat4 = newobjmat4 * tmppandamat4
                mat4list[-1].append(tmppandamat4)
            objnp.setMat(initmat4)
        self.floatingposemat4 = [e for l in mat4list for e in l]

    def __genFreeAssGrips(self):
        self.obj0grips = []
        self.obj1grips = []
        self.obj0grips, self.obj1grips = \
            genAvailableFAGs(self.base, self.obj0path, self.obj0Mat4,
                             self.obj1path, self.obj1Mat4, self.gdb, self.handpkg)

    def __transformGripsOnePose(self, objgrips, rotmat4):
        """
        transform the freeair grips to one specific rotmat4

        :param objgrips: [assgripcontacts, assgripnormals, assgriprotmat4s, assgripjawwidth, assgripidfreeair]
        :param rotmat4:
        :return: [xassgripcontacts, xassgripnormals, xassgriprotmat4s, xassgripjawwidth, xassgripidfreeair]
        each element in the list is also a list

        author: weiwei
        date: 20170307
        """

        assgripcontacts, assgripnormals, assgriprotmat4s, assgripjawwidth, assgripidfreeair = objgrips
        xassgripcontacts = []
        xassgripnormals = []
        xassgriprotmat4s = []
        xassgripjawwidth = []
        xassgripidfreeair = []
        for i, gripmat4 in enumerate(assgriprotmat4s):
            floatinggripmat4 = gripmat4 * rotmat4
            cct0 = rotmat4.xformPoint(assgripcontacts[i][0])
            cct1 = rotmat4.xformPoint(assgripcontacts[i][1])
            cctn0 = rotmat4.xformPoint(assgripnormals[i][0])
            cctn1 = rotmat4.xformPoint(assgripnormals[i][1])
            xassgripcontacts.append([cct0, cct1])
            xassgripnormals.append([cctn0, cctn1])
            xassgriprotmat4s.append(floatinggripmat4)
            xassgripjawwidth.append(assgripjawwidth[i])
            xassgripidfreeair.append(assgripidfreeair[i])
        return [xassgripcontacts, xassgripnormals, xassgriprotmat4s, xassgripjawwidth, xassgripidfreeair]

    def __transformGrips(self):
        """
        transform the freeass grips to all rotmat4 in self.gridsfloatingposemat4s

        :return:

        author: weiwei
        date: 20170221, tsukuba (copied from regrasp/floatingposes.py)
        """

        self.icoass0gripcontacts = []
        self.icoass0gripnormals = []
        self.icoass0griprotmat4s = []
        self.icoass0gripjawwidth = []
        self.icoass0gripidfreeair = []
        for icomat4 in self.gridsfloatingposemat4s:
            xassgrips = self.__transformGripsOnePose(self.obj0grips, icomat4)
            self.icoass0gripcontacts.append(xassgrips[0])
            self.icoass0gripnormals.append(xassgrips[1])
            self.icoass0griprotmat4s.append(xassgrips[2])
            self.icoass0gripjawwidth.append(xassgrips[3])
            self.icoass0gripidfreeair.append(xassgrips[4])

        self.icoass1gripcontacts = []
        self.icoass1gripnormals = []
        self.icoass1griprotmat4s = []
        self.icoass1gripjawwidth = []
        self.icoass1gripidfreeair = []
        for icomat4 in self.gridsfloatingposemat4s:
            xassgrips = self.__transformGripsOnePose(self.obj1grips, icomat4)
            self.icoass1gripcontacts.append(xassgrips[0])
            self.icoass1gripnormals.append(xassgrips[1])
            self.icoass1griprotmat4s.append(xassgrips[2])
            self.icoass1gripjawwidth.append(xassgrips[3])
            self.icoass1gripidfreeair.append(xassgrips[4])


def genAvailableFAGsSgl(base, basepath, baseMat4, objpath, objMat4, gdb, handpkg):
    """
    find the collision freeairgrips of objpath without considering rotation

    :param base: panda base
    :param basepath: the path of base object
    :param objpath: the path of obj object, the object to be assembled
    :param baseMat4, objMat4: all in world coordinates, not relative
    :param gdb: grasp db
    :param handpkg: hand package
    :return: [assgripcontacts, assgripnormals, assgriprotmat4s, assgripjawwidth, assgripidfreeair]

    author: weiwei
    date: 20170307
    """

    bulletworld = BulletWorld()

    robothand = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])

    basetrimesh = trimesh.load_mesh(basepath)
    basenp = pg.packpandanp(basetrimesh.vertices, basetrimesh.face_normals, basetrimesh.faces)
    basenp.setMat(baseMat4)
    basebullnode = cd.genCollisionMeshNp(basenp, base.render)

    bulletworld.attachRigidBody(basebullnode)

    dbobjname = os.path.splitext(os.path.basename(objpath))[0]
    objfag = Fag(gdb, dbobjname, handpkg)

    assgripcontacts = []
    assgripnormals = []
    assgriprotmat4s = []
    assgripjawwidth = []
    assgripidfreeair = []
    for i, rotmat in enumerate(objfag.freegriprotmats):
        assgrotmat = rotmat*objMat4
        robothand.setMat(assgrotmat)
        # detect the collisions when hand is open!
        robothand.setJawwidth(robothand.jawwidthopen)
        hndbullnode = cd.genCollisionMeshMultiNp(robothand.handnp)
        result0 = bulletworld.contactTest(hndbullnode)
        robothand.setJawwidth(objfag.freegripjawwidth[i])
        hndbullnode = cd.genCollisionMeshMultiNp(robothand.handnp)
        result1 = bulletworld.contactTest(hndbullnode)
        if (not result0.getNumContacts()) and (not result1.getNumContacts()):
            cct0 = objMat4.xformPoint(objfag.freegripcontacts[i][0])
            cct1 = objMat4.xformPoint(objfag.freegripcontacts[i][1])
            cctn0 = objMat4.xformPoint(objfag.freegripnormals[i][0])
            cctn1 = objMat4.xformPoint(objfag.freegripnormals[i][1])
            assgripcontacts.append([cct0, cct1])
            assgripnormals.append([cctn0, cctn1])
            assgriprotmat4s.append(assgrotmat)
            assgripjawwidth.append(objfag.freegripjawwidth[i])
            assgripidfreeair.append(objfag.freegripids[i])

    return [assgripcontacts, assgripnormals, assgriprotmat4s, assgripjawwidth, assgripidfreeair]

def genAvailableFAGs(base, obj0path, obj0Mat4, obj1path, obj1Mat4, gdb, handpkg):
    """
    find the collision freeairgrips of objpath without considering rotation
    # the collision between obj1 grasps and obj2 grasps will be checked after finding a path

    :param base: panda base
    :param obj0path: the path of obj0 object
    :param obj1path: the path of obj1 object
    :param obj0Mat4, obj1Mat4: all in world coordinates, not relative
    :param gdb: grasp db
    :param handpkg: hand package
    :return: [obj0grips, obj1grips] where objxgrasps =
            [assgripcontacts, assgripnormals, assgriprotmat4s, assgripjawwidth, assgripidfreeair]

    author: weiwei
    date: 20170307
    """

    # obj0
    obj0grips = genAvailableFAGsSgl(base, obj1path, obj1Mat4, obj0path, obj0Mat4, gdb, handpkg)
    # obj1
    obj1grips = genAvailableFAGsSgl(base, obj0path, obj0Mat4, obj1path, obj1Mat4, gdb, handpkg)
    return [obj0grips, obj1grips]

if __name__ == '__main__':

    base = pandactrl.World(lookatp=[0,0,0])

    nxtrobot = nxt.NxtRobot()

    handpkg = rtq85nm
    gdb = db.GraspDB()

    this_dir, this_filename = os.path.split(__file__)
    obj0path = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "planefrontstay.stl")
    obj0Mat4 = Mat4.identMat()
    obj1path = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "planewheel.stl")
    obj1Mat4 = Mat4(obj0Mat4)
    obj1Mat4.setCell(3,1,32)
    obj1Mat4.setCell(3,2,10)

    obj0trimesh = trimesh.load_mesh(obj0path)
    obj0np = pg.packpandanp(obj0trimesh.vertices, obj0trimesh.face_normals, obj0trimesh.faces)
    obj0np.setMat(obj0Mat4)
    obj0np.setColor(1,.7,0.3)
    obj1trimesh = trimesh.load_mesh(obj1path)
    obj1np = pg.packpandanp(obj1trimesh.vertices, obj1trimesh.face_normals, obj1trimesh.faces)
    obj1np.setMat(obj1Mat4)
    obj1np.setColor(0.3,.3,0.3)

    # obj0np.reparentTo(base.render)
    # obj1np.reparentTo(base.render)
    #
    # obj0grips, obj1grips = genAvailableFAGs(base, obj0path, obj0Mat4, obj1path, obj1Mat4, gdb, handpkg)
    #
    # ass0gripcontacts, ass0gripnormals, ass0griprotmat4s, ass0gripjawwidth, ass0gripidfreeair = obj0grips
    # ass1gripcontacts, ass1gripnormals, ass1griprotmat4s, ass1gripjawwidth, ass1gripidfreeair = obj1grips
    # for i, assgriprotmat in enumerate(ass1griprotmat4s):
    #     if i%3 == 0:
    #         robothand = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
    #         robothand.setMat(assgriprotmat)
    #         robothand.setJawwidth(ass1gripjawwidth[i])
    #         robothand.setColor([.7,.7,.2,1])
    #         robothand.reparentTo(base.render)

    assdirect1to0 = Vec3(0,-70,0)
    toass = TwoObjAss(base, obj0path, obj0Mat4, obj1path, obj1Mat4, assdirect1to0, gdb, handpkg)
    grids = []
    for x in range(400,401,100):
        for y in [0]:
            for z in range(400,401,100):
                grids.append([x,y,z])
    toass.genXAssPandGs(grids)
    toass.saveToDB()
    toass.loadFromDB()
    toass.updateDBwithHandPairs()
    toass.updateDBwithIK(nxtrobot)
    toass.loadIKFeasibleAGPairsFromDB(nxtrobot)
    print len(toass.icoass0gripcontacts)
    print len(toass.icoass1gripcontacts)

    # # g0
    # ass0gripcontacts = toass.icoass0gripcontacts[poseind]
    # ass0gripnormals = toass.icoass0gripnormals[poseind]
    # ass0griprotmat4s = toass.icoass0griprotmat4s[poseind]
    # ass0gripjawwidth = toass.icoass0gripjawwidth[poseind]
    # ass0gripidfreeair = toass.icoass0gripidfreeair[poseind]
    # # g1
    # ass1gripcontacts = toass.icoass1gripcontacts[poseind]
    # ass1gripnormals = toass.icoass1gripnormals[poseind]
    # ass1griprotmat4s = toass.icoass1griprotmat4s[poseind]
    # ass1gripjawwidth = toass.icoass1gripjawwidth[poseind]
    # ass1gripidfreeair = toass.icoass1gripidfreeair[poseind]
    # for i, assgriprotmat in enumerate(ass0griprotmat4s):
    #     if i%1 == 0:
    #         robothand = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
    #         robothand.setMat(assgriprotmat)
    #         robothand.setJawwidth(ass0gripjawwidth[i])
    #         robothand.setColor([.2,.7,.7,1])
    #         robothand.reparentTo(base.render)
    # pg.plotAxis(base.render)

    asspindl = [0]

    obj = [None]
    objnode = NodePath('obj')
    nassp = len(toass.gridsfloatingposemat4s)
    xassrotmat4 = toass.gridsfloatingposemat4s[asspindl[0]]
    obj0np.setMat(obj0Mat4*xassrotmat4)
    obj1np.setMat(obj1Mat4*xassrotmat4)
    obj0np.reparentTo(objnode)
    obj1np.reparentTo(objnode)
    obj[0] = objnode

    hnd = [None]
    hndnode = NodePath('hnd')
    for pairind, hndrotmat4pair in enumerate(toass.icoassgrippairshndmat4s[asspindl[0]]):
        hndrotmat40 = hndrotmat4pair[0]
        hndrotmat41 = hndrotmat4pair[1]
        jawwidth0 = toass.icoassgrippairsjawwidths[asspindl[0]][pairind][0]
        jawwidth1 = toass.icoassgrippairsjawwidths[asspindl[0]][pairind][1]
        robothand0 = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
        robothand0.setMat(hndrotmat40)
        robothand0.setJawwidth(jawwidth0)
        robothand0.setColor([.7,.7,.2,1])
        robothand1 = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
        robothand1.setMat(hndrotmat41)
        robothand1.setJawwidth(jawwidth1)
        robothand1.setColor([.2,.7,.7,1])
        robothand0.reparentTo(hndnode)
        robothand1.reparentTo(hndnode)
        asspmat = toass.gridsfloatingposemat4s[asspindl[0]]
        assdirect1to0x = asspmat.xformVec(assdirect1to0)
        assdirect0to1x = asspmat.xformVec(-assdirect1to0)
        hndrotmat40.setRow(3, hndrotmat4pair[0].getRow3(3)+assdirect1to0x)
        hndrotmat41.setRow(3, hndrotmat4pair[1].getRow3(3)+assdirect0to1x)
        jawwidth0x = 85
        jawwidth1x = 85
        robothand0x = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
        robothand0x.setMat(hndrotmat40)
        robothand0x.setJawwidth(jawwidth0x)
        robothand0x.setColor([.7,.7,.2,.5])
        robothand1x = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
        robothand1x.setMat(hndrotmat41)
        robothand1x.setJawwidth(jawwidth1x)
        robothand1x.setColor([.2,.7,.7,.5])
        robothand0x.reparentTo(hndnode)
        robothand1x.reparentTo(hndnode)
    hnd[0] = hndnode

    obj[0].reparentTo(base.render)
    hnd[0].reparentTo(base.render)

    def updateshow(asspindl, obj, hnd, nassp, task):
        if asspindl[0] < nassp:
            if obj[0] is not None:
                obj[0].detachNode()
            if hnd[0] is not None:
                hnd[0].detachNode()
            # npc = base.render.findAllMatches("**/+GeomNode")
            # for np in npc:
            #     np.detachNode()
            objnode = NodePath('obj')
            xassrotmat4 = toass.gridsfloatingposemat4s[asspindl[0]]
            obj0np.setMat(obj0Mat4*xassrotmat4)
            obj1np.setMat(obj1Mat4*xassrotmat4)
            obj0np.reparentTo(objnode)
            obj1np.reparentTo(objnode)
            obj[0] = objnode
            hndnode = NodePath('hnd')
            for pairind, hndrotmat4pair in enumerate(toass.icoassgrippairshndmat4s[asspindl[0]]):
                hndrotmat40 = hndrotmat4pair[0]
                hndrotmat41 = hndrotmat4pair[1]
                jawwidth0 = toass.icoassgrippairsjawwidths[asspindl[0]][pairind][0]
                jawwidth1 = toass.icoassgrippairsjawwidths[asspindl[0]][pairind][1]
                robothand0 = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
                robothand0.setMat(hndrotmat40)
                robothand0.setJawwidth(jawwidth0)
                robothand0.setColor([.7,.7,.2,1])
                robothand1 = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
                robothand1.setMat(hndrotmat41)
                robothand1.setJawwidth(jawwidth1)
                robothand1.setColor([.2,.7,.7,1])
                robothand0.reparentTo(hndnode)
                robothand1.reparentTo(hndnode)
                asspmat = toass.gridsfloatingposemat4s[asspindl[0]]
                assdirect1to0x = asspmat.xformVec(assdirect1to0)
                assdirect0to1x = asspmat.xformVec(-assdirect1to0)
                hndrotmat40.setRow(3, hndrotmat4pair[0].getRow3(3)+assdirect1to0x)
                hndrotmat41.setRow(3, hndrotmat4pair[1].getRow3(3)+assdirect0to1x)
                jawwidth0x = 85
                jawwidth1x = 85
                robothand0x = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
                robothand0x.setMat(hndrotmat40)
                robothand0x.setJawwidth(jawwidth0x)
                robothand0x.setColor([.7,.7,.2,.5])
                robothand1x = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])
                robothand1x.setMat(hndrotmat41)
                robothand1x.setJawwidth(jawwidth1x)
                robothand1x.setColor([.2,.7,.7,.5])
                robothand0x.reparentTo(hndnode)
                robothand1x.reparentTo(hndnode)
            hnd[0] = hndnode
            obj[0].reparentTo(base.render)
            hnd[0].reparentTo(base.render)
            # base.win.saveScreenshot(Filename(str(asspindl[0])+'.jpg'))
            asspindl[0] += 1
        else:
            asspindl[0] = 0
        return task.cont
        return task.again
    taskMgr.doMethodLater(1, updateshow, "updateshow",
                          extraArgs = [asspindl, obj, hnd, nassp],
                          appendTask = True)

    base.run()

