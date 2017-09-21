#!/usr/bin/python

import os
import itertools

import MySQLdb as mdb
import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from panda3d.bullet import BulletWorld
from panda3d.core import *
from shapely.geometry import LinearRing
from shapely.geometry import Point
from shapely.geometry import Polygon

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
import trimesh
from pandaplotutils import pandageom as pg
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from mpl_toolkits.mplot3d import art3d as mc3d
from operator import add
from robotsim.nextage import nxt
from robotsim.hrp5 import hrp5
from robotsim.hrp5n import hrp5n
from database import dbaccess as db

import networkx as nx
import math
import random
import floatingposes

# regriptppfp means regrip using tabletop placements and floating poses
class RegripTppFp():

    def __init__(self, objpath, robot, handpkg, gdb, base, tableheight = -55):
        """

        :param objpath:
        :param robot:
        :param handpkg:
        :param gdb:
        :param base:
        :param tableheight: offset from 0
        """

        self.handpkg = handpkg
        self.robot = robot
        self.objtrimesh=trimesh.load_mesh(objpath)

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # regg = regrip graph
        self.regg = nx.Graph()

        self.ndiscreterot = 0
        self.nplacements = 0
        self.globalgripids = []

        # for removing the grasps at start and goal
        self.robothand = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])

        # plane to remove hand
        self.bulletworld = BulletWorld()
        self.planebullnode = cd.genCollisionPlane(offset = tableheight)
        self.bulletworld.attachRigidBody(self.planebullnode)

        # add tabletop plane model to bulletworld
        this_dir, this_filename = os.path.split(__file__)
        ttpath = Filename.fromOsSpecific(os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "supports", "tabletop.egg"))
        self.ttnodepath = NodePath("tabletop")
        ttl = loader.loadModel(ttpath)
        ttl.setPos(0, 0, tableheight)
        ttl.instanceTo(self.ttnodepath)

        self.startrgtnodeids = []
        self.startlftnodeids = []
        self.goalrgtnodeids = []
        self.goallftnodeids = []
        self.shortestpaths = None

        self.gdb = gdb
        self.robot = robot

        # load retraction distances
        self.rethandx, self.retworldz, self.retworlda, self.worldz = self.gdb.loadIKRet()
        # worlda is default for the general grasps on table top
        # for assembly at start and goal, worlda is computed by assembly planner
        self.worlda = Vec3(0,0,1)

        self.floatingposes = floatingposes.FloatingPoses(objpath, gdb, handpkg, base)
        self.floatingposes.loadIKFeasibleFGPairsFromDB(robot)

        # loadfreeairgrip
        self.__loadFreeAirGrip()
        self.__buildGraphs(armname = "rgt")
        self.__buildGraphs(armname = "lft")
        self.__bridgeGraph()

        # shortestpaths
        self.directshortestpaths_startrgtgoalrgt = []
        self.directshortestpaths_startrgtgoallft = []
        self.directshortestpaths_startlftgoalrgt = []
        self.directshortestpaths_startlftgoallft = []

        # for fast path plot
        self.gnodesplotpos = {}

    def __loadFreeAirGrip(self):
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

    def __buildGraphs(self, armname = "rgt"):
        """
        load tabletopgrips
        retraction distance are also loaded from database

        :param robot: an robot defined in robotsim.hrp5 or robotsim.nextage
        :param gdb: an object of the database.GraspDB class
        :param idarm: value = 1 "lft" or 2 "rgt", which arm to use
        :return:

        author: weiwei
        date: 20170112
        """

        # load idarm
        idarm = self.gdb.loadIdArm(armname)

        # get the global grip ids
        # and prepare the global edges
        # for each globalgripid, find all its tabletopids (pertaining to placements)
        globalidsedges = {}
        for ggid in self.freegripids:
            globalidsedges[str(ggid)] = []
            self.globalgripids.append(ggid)
        sql = "SELECT tabletopplacements.idtabletopplacements, angle.value, \
                tabletopplacements.idfreetabletopplacement, tabletopplacements.tabletopposition, \
                tabletopplacements.rotmat FROM \
                tabletopplacements,freetabletopplacement,angle,object WHERE \
                tabletopplacements.idangle=angle.idangle AND \
                tabletopplacements.idfreetabletopplacement=freetabletopplacement.idfreetabletopplacement AND \
                freetabletopplacement.idobject=object.idobject AND \
                object.name LIKE '%s' AND angle.value IN (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0)" \
                % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) != 0:
            tpsrows = np.array(result)
            # nubmer of discreted rotation
            self.angles = list(set(map(float, tpsrows[:,1])))
            # for plotting
            self.fttpsids = list(set(map(int, tpsrows[:,2])))
            self.nfttps = len(self.fttpsids)

            idrobot = self.gdb.loadIdRobot(self.robot)
            for i, idtps in enumerate(tpsrows[:,0]):
                sql = "SELECT tabletopgrips.idtabletopgrips, tabletopgrips.contactpnt0, tabletopgrips.contactpnt1, \
                        tabletopgrips.rotmat, tabletopgrips.jawwidth, tabletopgrips.idfreeairgrip \
                        FROM tabletopgrips,ik,freeairgrip,hand WHERE tabletopgrips.idfreeairgrip = freeairgrip.idfreeairgrip AND \
                        freeairgrip.idhand = hand.idhand AND\
                        tabletopgrips.idtabletopgrips=ik.idtabletopgrips AND \
                        tabletopgrips.idtabletopplacements = %d AND ik.idrobot=%d AND \
                        ik.feasibility='True' AND ik.feasibility_handx='True' AND ik.feasibility_handxworldz='True' \
                        AND ik.feasibility_worlda='True' AND ik.feasibility_worldaworldz='True' AND ik.idarm = %d AND hand.name LIKE '%s'" \
                        % (int(idtps), idrobot, idarm, self.handpkg.getHandName())
                resultttgs = self.gdb.execute(sql)
                if len(resultttgs)==0:
                    continue
                localidedges = []
                for ttgsrow in resultttgs:
                    ttgsid = int(ttgsrow[0])
                    ttgscct0 = dc.strToV3(ttgsrow[1])
                    ttgscct1 = dc.strToV3(ttgsrow[2])
                    ttgsrotmat = dc.strToMat4(ttgsrow[3])
                    ttgsjawwidth = float(ttgsrow[4])
                    ttgsidfreeair = int(ttgsrow[5])
                    ttgsfgrcenter = (ttgscct0+ttgscct1)/2
                    # if ttgsfgrcenter[1] < 200 or ttgsfgrcenter[1] > -200:
                    #     continue
                    handx = ttgsrotmat.getRow3(0)
                    ttgsfgrcenterhandx = ttgsfgrcenter + handx*self.rethandx
                    ttgsfgrcenterhandxworldz = ttgsfgrcenterhandx + self.worldz*self.retworldz
                    ttgsfgrcenterworlda = ttgsfgrcenter + self.worlda*self.retworlda
                    ttgsfgrcenterworldaworldz = ttgsfgrcenterworlda+ self.worldz*self.retworldz
                    ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                    ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
                    ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
                    ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                    ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
                    ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                    objrotmat4 = dc.strToMat4(tpsrows[:,4][i])
                    objrotmat4worlda = Mat4(objrotmat4)
                    objrotmat4worlda.setRow(3, objrotmat4.getRow3(3)+self.worlda*self.retworlda)
                    objrotmat4worldaworldz = Mat4(objrotmat4worlda)
                    objrotmat4worldaworldz.setRow(3, objrotmat4worlda.getRow3(3)+self.worldz*self.retworldz)
                    self.regg.add_node(armname+str(ttgsid), fgrcenter=ttgsfgrcenternp,
                                       fgrcenterhandx = ttgsfgrcenternp_handx,
                                       fgrcenterhandxworldz = ttgsfgrcenternp_handxworldz,
                                       fgrcenterworlda = ttgsfgrcenternp_worlda,
                                       fgrcenterworldaworldz = ttgsfgrcenternp_worldaworldz,
                                       jawwidth=ttgsjawwidth, hndrotmat3np=ttgsrotmat3np,
                                       globalgripid = ttgsidfreeair, freetabletopplacementid = int(tpsrows[:,2][i]),
                                       tabletopplacementrotmat = objrotmat4,
                                       tabletopplacementrotmathandx = objrotmat4,
                                       tabletopplacementrotmathandxworldz = objrotmat4,
                                       tabletopplacementrotmatworlda = objrotmat4worlda,
                                       tabletopplacementrotmatworldaworldz = objrotmat4worldaworldz,
                                       angle = float(tpsrows[:,1][i]), tabletopposition = dc.strToV3(tpsrows[:,3][i]))
                    globalidsedges[str(ttgsidfreeair)].append(armname+str(ttgsid))
                    localidedges.append(armname+str(ttgsid))
                # print list(itertools.combinations(ttgrows[:,0], 2))
                for edge in list(itertools.combinations(localidedges, 2)):
                    self.regg.add_edge(*edge, weight=1, edgetype = 'transit')
            if len(globalidsedges) == 0:
                raise ValueError("Plan tabletopgrips first!")

            # add floatingposes
            for fpind, objrotmat4 in enumerate(self.floatingposes.gridsfloatingposemat4s):
                for pairind, hndrotmat4pair in enumerate(self.floatingposes.floatinggrippairshndmat4s[fpind]):
                    iele = 0
                    if armname == 'lft':
                        iele = 1
                    fpgid = self.floatingposes.floatinggrippairsids[fpind][pairind][iele]
                    fpgidfreeair = self.floatingposes.floatinggrippairsidfreeairs[fpind][pairind][iele]
                    ccts = self.floatingposes.floatinggrippairscontacts[fpind][pairind][iele]
                    hndrotmat4 = hndrotmat4pair[iele]
                    fpgfgrcenter = (Vec3(ccts[0][0],ccts[0][1],ccts[0][2])+Vec3(ccts[1][0],ccts[1][1],ccts[1][2]))/2
                    fpgfgrcenterhandx = fpgfgrcenter+hndrotmat4.getRow3(0)*self.rethandx
                    fpgfgrcenternp = pg.v3ToNp(fpgfgrcenter)
                    fpgfgrcenterhandxnp = pg.v3ToNp(fpgfgrcenterhandx)
                    jawwidth =self.floatingposes.floatinggrippairsjawwidths[fpind][pairind][iele]
                    hndrotmat3np = pg.mat3ToNp(hndrotmat4.getUpper3())
                    fprotmat4 = objrotmat4
                    self.regg.add_node('ho'+armname+str(fpgid), fgrcenter = fpgfgrcenternp,
                                       fgrcenterhandx = fpgfgrcenterhandxnp, jawwidth = jawwidth,
                                       hndrotmat3np = hndrotmat3np, floatingposerotmat4 = fprotmat4,
                                       floatingposerotmat4handx = fprotmat4, floatingposeind = fpind,
                                       floatingposegrippairind = pairind, globalgripid = fpgidfreeair)
                    globalidsedges[str(fpgidfreeair)].append('ho'+armname+str(fpgid))
            for globalidedgesid in globalidsedges:
                for edge in list(itertools.combinations(globalidsedges[globalidedgesid], 2)):
                    self.regg.add_edge(*edge, weight=1, edgetype = 'transfer')

    def __bridgeGraph(self):
        for fpind, objrotmat4 in enumerate(self.floatingposes.gridsfloatingposemat4s):
            for pairind, hndrotmat4pair in enumerate(self.floatingposes.floatinggrippairshndmat4s[fpind]):
                fpgid0 = self.floatingposes.floatinggrippairsids[fpind][pairind][0]
                fpgid1 = self.floatingposes.floatinggrippairsids[fpind][pairind][1]
                self.regg.add_edge('horgt'+str(fpgid0), 'holft'+str(fpgid1), weight = 1, edgetype = 'handovertransit')

    def __addstartgoal(self, startrotmat4, goalrotmat4, base):
        """
        add start and goal for the regg

        :param startrotmat4 and goalrotmat4: both are 4by4 panda3d matrix
        :return:

        author: weiwei
        date: 20161216, sapporo
        """

        ### start rgt
        # the node id of a globalgripid in startnode
        nodeidofglobalidinstartrgt= {}
        # the startnodeids is also for quick access
        self.startrgtnodeids = []
        self.ttnodepath.reparentTo(base.render)
        for j, rotmat in enumerate(self.freegriprotmats):
            ttgsrotmat = rotmat * startrotmat4
            # for collision detection, we move the object back to x=0,y=0
            ttgsrotmatx0y0 = Mat4(startrotmat4)
            ttgsrotmatx0y0.setCell(3,0,0)
            ttgsrotmatx0y0.setCell(3,1,0)
            ttgsrotmatx0y0 = rotmat * ttgsrotmatx0y0
            # check if the hand collide with tabletop
            tmphnd = self.robothand
            # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
            initmat = tmphnd.getMat()
            initjawwidth = tmphnd.jawwidth
            # set jawwidth to 80 to avoid collision with surrounding obstacles
            # set to gripping with is unnecessary
            # tmprtq85.setJawwidth(self.freegripjawwidth[j])
            tmphnd.setJawwidth(80)
            tmphnd.setMat(pandanpmat4 = ttgsrotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            if not result.getNumContacts():
                ttgscct0=startrotmat4.xformPoint(self.freegripcontacts[j][0])
                ttgscct1=startrotmat4.xformPoint(self.freegripcontacts[j][1])
                ttgsfgrcenter = (ttgscct0+ttgscct1)/2
                handx = ttgsrotmat.getRow3(0)
                ttgsfgrcenterhandx = ttgsfgrcenter + handx*self.rethandx
                # handxworldz is not necessary for start
                # ttgsfgrcenterhandxworldz = ttgsfgrcenterhandx + self.worldz*self.retworldz
                ttgsfgrcenterworlda = ttgsfgrcenter + self.worlda*self.retworlda
                ttgsfgrcenterworldaworldz = ttgsfgrcenterworlda+ self.worldz*self.retworldz
                ttgsjawwidth = self.freegripjawwidth[j]
                ttgsidfreeair = self.freegripids[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
                # handxworldz is not necessary for start
                # ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
                ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                ikc = self.robot.numikr(ttgsfgrcenternp, ttgsrotmat3np, armid = 'rgt')
                ikcx = self.robot.numikr(ttgsfgrcenternp_handx, ttgsrotmat3np, armid = 'rgt')
                ikca = self.robot.numikr(ttgsfgrcenternp_worlda, ttgsrotmat3np, armid = 'rgt')
                ikcaz = self.robot.numikr(ttgsfgrcenternp_worldaworldz, ttgsrotmat3np, armid = 'rgt')
                if (ikc is not None) and (ikcx is not None) and (ikca is not None) and (ikcaz is not None):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = startrotmat4.getRow3(3)
                    startrotmat4worlda = Mat4(startrotmat4)
                    startrotmat4worlda.setRow(3, startrotmat4.getRow3(3)+self.worlda*self.retworlda)
                    startrotmat4worldaworldz = Mat4(startrotmat4worlda)
                    startrotmat4worldaworldz.setRow(3, startrotmat4worlda.getRow3(3)+self.worldz*self.retworldz)
                    self.regg.add_node('startrgt'+str(j), fgrcenter=ttgsfgrcenternp,
                                       fgrcenterhandx = ttgsfgrcenternp_handx,
                                       fgrcenterhandxworldz = 'na',
                                       fgrcenterworlda = ttgsfgrcenternp_worlda,
                                       fgrcenterworldaworldz = ttgsfgrcenternp_worldaworldz,
                                       jawwidth=ttgsjawwidth, hndrotmat3np=ttgsrotmat3np,
                                       globalgripid = ttgsidfreeair, freetabletopplacementid = 'na',
                                       tabletopplacementrotmat = startrotmat4,
                                       tabletopplacementrotmathandx = startrotmat4,
                                       tabletopplacementrotmathandxworldz = 'na',
                                       tabletopplacementrotmatworlda = startrotmat4worlda,
                                       tabletopplacementrotmatworldaworldz = startrotmat4worldaworldz,
                                       angle = 'na', tabletopposition = tabletopposition)
                    nodeidofglobalidinstartrgt[ttgsidfreeair]='startrgt'+str(j)
                    self.startrgtnodeids.append('startrgt'+str(j))
                    # tmprtq85.reparentTo(base.render)
            tmphnd.setMat(pandanpmat4 = initmat)
            tmphnd.setJawwidth(initjawwidth)

        if len(self.startrgtnodeids) == 0:
            print "No available starting grip for right hand!"

        ### start lft
        # the node id of a globalgripid in startnode
        nodeidofglobalidinstartlft= {}
        # the startnodeids is also for quick access
        self.startlftnodeids = []
        self.ttnodepath.reparentTo(base.render)
        for j, rotmat in enumerate(self.freegriprotmats):
            ttgsrotmat = rotmat * startrotmat4
            # for collision detection, we move the object back to x=0,y=0
            ttgsrotmatx0y0 = Mat4(startrotmat4)
            ttgsrotmatx0y0.setCell(3,0,0)
            ttgsrotmatx0y0.setCell(3,1,0)
            ttgsrotmatx0y0 = rotmat * ttgsrotmatx0y0
            # check if the hand collide with tabletop
            tmphnd = self.robothand
            # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
            initmat = tmphnd.getMat()
            initjawwidth = tmphnd.jawwidth
            # set jawwidth to 80 to avoid collision with surrounding obstacles
            # set to gripping with is unnecessary
            # tmprtq85.setJawwidth(self.freegripjawwidth[j])
            tmphnd.setJawwidth(80)
            tmphnd.setMat(pandanpmat4 = ttgsrotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            if not result.getNumContacts():
                ttgscct0=startrotmat4.xformPoint(self.freegripcontacts[j][0])
                ttgscct1=startrotmat4.xformPoint(self.freegripcontacts[j][1])
                ttgsfgrcenter = (ttgscct0+ttgscct1)/2
                handx = ttgsrotmat.getRow3(0)
                ttgsfgrcenterhandx = ttgsfgrcenter + handx*self.rethandx
                # handxworldz is not necessary for start
                # ttgsfgrcenterhandxworldz = ttgsfgrcenterhandx + self.worldz*self.retworldz
                ttgsfgrcenterworlda = ttgsfgrcenter + self.worlda*self.retworlda
                ttgsfgrcenterworldaworldz = ttgsfgrcenterworlda+ self.worldz*self.retworldz
                ttgsjawwidth = self.freegripjawwidth[j]
                ttgsidfreeair = self.freegripids[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
                # handxworldz is not necessary for start
                # ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
                ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                ikc = self.robot.numikr(ttgsfgrcenternp, ttgsrotmat3np, armid = 'lft')
                ikcx = self.robot.numikr(ttgsfgrcenternp_handx, ttgsrotmat3np, armid = 'lft')
                ikca = self.robot.numikr(ttgsfgrcenternp_worlda, ttgsrotmat3np, armid = 'lft')
                ikcaz = self.robot.numikr(ttgsfgrcenternp_worldaworldz, ttgsrotmat3np, armid = 'lft')
                if (ikc is not None) and (ikcx is not None) and (ikca is not None) and (ikcaz is not None):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = startrotmat4.getRow3(3)
                    startrotmat4worlda = Mat4(startrotmat4)
                    startrotmat4worlda.setRow(3, startrotmat4.getRow3(3)+self.worlda*self.retworlda)
                    startrotmat4worldaworldz = Mat4(startrotmat4worlda)
                    startrotmat4worldaworldz.setRow(3, startrotmat4worlda.getRow3(3)+self.worldz*self.retworldz)
                    self.regg.add_node('startlft'+str(j), fgrcenter=ttgsfgrcenternp,
                                       fgrcenterhandx = ttgsfgrcenternp_handx,
                                       fgrcenterhandxworldz = 'na',
                                       fgrcenterworlda = ttgsfgrcenternp_worlda,
                                       fgrcenterworldaworldz = ttgsfgrcenternp_worldaworldz,
                                       jawwidth=ttgsjawwidth, hndrotmat3np=ttgsrotmat3np,
                                       globalgripid = ttgsidfreeair, freetabletopplacementid = 'na',
                                       tabletopplacementrotmat = startrotmat4,
                                       tabletopplacementrotmathandx = startrotmat4,
                                       tabletopplacementrotmathandxworldz = 'na',
                                       tabletopplacementrotmatworlda = startrotmat4worlda,
                                       tabletopplacementrotmatworldaworldz = startrotmat4worldaworldz,
                                       angle = 'na', tabletopposition = tabletopposition)
                    nodeidofglobalidinstartlft[ttgsidfreeair]='startlft'+str(j)
                    self.startlftnodeids.append('startlft'+str(j))
                    # tmprtq85.reparentTo(base.render)
            tmphnd.setMat(pandanpmat4 = initmat)
            tmphnd.setJawwidth(initjawwidth)

        if len(self.startlftnodeids) == 0:
            print "No available starting grip for left hand!"

        if len(self.startrgtnodeids) == 0 and len(self.startlftnodeids) == 0:
            raise ValueError("No available starting grip!")

        # add edge right
        # add start transit edge
        for edge in list(itertools.combinations(self.startrgtnodeids, 2)):
            self.regg.add_edge(*edge, weight = 1, edgetype = 'startrgttransit')
        # add start transfer edge
        for reggnode, reggnodedata in self.regg.nodes(data=True):
            if reggnode.startswith('rgt') or reggnode.startswith('horgt'):
                globalgripid = reggnodedata['globalgripid']
                if globalgripid in nodeidofglobalidinstartrgt.keys():
                    startnodeid = nodeidofglobalidinstartrgt[globalgripid]
                    self.regg.add_edge(startnodeid, reggnode, weight=1, edgetype = 'startrgttransfer')

        # add edge left
        # add start transit edge
        for edge in list(itertools.combinations(self.startlftnodeids, 2)):
            self.regg.add_edge(*edge, weight = 1, edgetype = 'startlfttransit')
        # add start transfer edge
        for reggnode, reggnodedata in self.regg.nodes(data=True):
            if reggnode.startswith('lft') or reggnode.startswith('holft'):
                globalgripid = reggnodedata['globalgripid']
                if globalgripid in nodeidofglobalidinstartlft.keys():
                    startnodeid = nodeidofglobalidinstartlft[globalgripid]
                    self.regg.add_edge(startnodeid, reggnode, weight=1, edgetype = 'startlfttransfer')


        ### goal rgt
        # the node id of a globalgripid in goalnode, for quick setting up edges
        nodeidofglobalidingoalrgt = {}
        # the goalnodeids is also for quick access
        self.goalrgtnodeids = []
        for j, rotmat in enumerate(self.freegriprotmats):
            ttgsrotmat = rotmat * goalrotmat4
            # for collision detection, we move the object back to x=0,y=0
            ttgsrotmatx0y0 = Mat4(goalrotmat4)
            ttgsrotmatx0y0.setCell(3,0,0)
            ttgsrotmatx0y0.setCell(3,1,0)
            ttgsrotmatx0y0 = rotmat * ttgsrotmatx0y0
            # check if the hand collide with tabletop
            tmphnd = self.robothand
            # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
            initmat = tmphnd.getMat()
            initjawwidth = tmphnd.jawwidth
            tmphnd.setJawwidth(self.freegripjawwidth[j])
            tmphnd.setMat(pandanpmat4 = ttgsrotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            if not result.getNumContacts():
                ttgscct0=goalrotmat4.xformPoint(self.freegripcontacts[j][0])
                ttgscct1=goalrotmat4.xformPoint(self.freegripcontacts[j][1])
                ttgsfgrcenter = (ttgscct0+ttgscct1)/2
                handx =  ttgsrotmat.getRow3(0)
                ttgsfgrcenterhandx = ttgsfgrcenter + handx*self.rethandx
                ttgsfgrcenterhandxworldz = ttgsfgrcenterhandx + self.worldz*self.retworldz
                ttgsfgrcenterworlda = ttgsfgrcenter + self.worlda*self.retworlda
                ttgsfgrcenterworldaworldz = ttgsfgrcenterworlda+ self.worldz*self.retworldz
                ttgsjawwidth = self.freegripjawwidth[j]
                ttgsidfreeair = self.freegripids[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
                ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
                ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                ikc = self.robot.numikr(ttgsfgrcenternp, ttgsrotmat3np, armid = 'rgt')
                ikcx = self.robot.numikr(ttgsfgrcenternp_handx, ttgsrotmat3np, armid = 'rgt')
                ikcxz = self.robot.numikr(ttgsfgrcenternp_handxworldz, ttgsrotmat3np, armid = 'rgt')
                ikca = self.robot.numikr(ttgsfgrcenternp_worlda, ttgsrotmat3np, armid = 'rgt')
                ikcaz = self.robot.numikr(ttgsfgrcenternp_worldaworldz, ttgsrotmat3np, armid = 'rgt')
                if (ikc is not None) and (ikcx is not None) and (ikcxz is not None) \
                        and (ikca is not None) and (ikcaz is not None):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = goalrotmat4.getRow3(3)
                    goalrotmat4worlda = Mat4(goalrotmat4)
                    goalrotmat4worlda.setRow(3, goalrotmat4.getRow3(3)+self.worlda*self.retworlda)
                    goalrotmat4worldaworldz = Mat4(goalrotmat4worlda)
                    goalrotmat4worldaworldz.setRow(3, goalrotmat4worlda.getRow3(3)+self.worldz*self.retworldz)
                    self.regg.add_node('goalrgt'+str(j), fgrcenter=ttgsfgrcenternp,
                                       fgrcenterhandx = ttgsfgrcenternp_handx,
                                       fgrcenterhandxworldz = ttgsfgrcenternp_handxworldz,
                                       fgrcenterworlda = ttgsfgrcenternp_worlda,
                                       fgrcenterworldaworldz = ttgsfgrcenternp_worldaworldz,
                                       jawwidth=ttgsjawwidth, hndrotmat3np=ttgsrotmat3np,
                                       globalgripid = ttgsidfreeair, freetabletopplacementid = 'na',
                                       tabletopplacementrotmat = goalrotmat4,
                                       tabletopplacementrotmathandx = goalrotmat4,
                                       tabletopplacementrotmathandxworldz = goalrotmat4,
                                       tabletopplacementrotmatworlda = goalrotmat4worlda,
                                       tabletopplacementrotmatworldaworldz = goalrotmat4worldaworldz,
                                       angleid = 'na', tabletopposition = tabletopposition)
                    nodeidofglobalidingoalrgt[ttgsidfreeair]='goalrgt'+str(j)
                    self.goalrgtnodeids.append('goalrgt'+str(j))
            tmphnd.setMat(pandanpmat4 = initmat)
            tmphnd.setJawwidth(initjawwidth)

        if len(self.goalrgtnodeids) == 0:
            print "No available goal grip for right hand!"

        ### goal lft
        # the node id of a globalgripid in goalnode, for quick setting up edges
        nodeidofglobalidingoallft = {}
        # the goalnodeids is also for quick access
        self.goallftnodeids = []
        for j, rotmat in enumerate(self.freegriprotmats):
            ttgsrotmat = rotmat * goalrotmat4
            # for collision detection, we move the object back to x=0,y=0
            ttgsrotmatx0y0 = Mat4(goalrotmat4)
            ttgsrotmatx0y0.setCell(3,0,0)
            ttgsrotmatx0y0.setCell(3,1,0)
            ttgsrotmatx0y0 = rotmat * ttgsrotmatx0y0
            # check if the hand collide with tabletop
            tmphnd = self.robothand
            # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
            initmat = tmphnd.getMat()
            initjawwidth = tmphnd.jawwidth
            tmphnd.setJawwidth(self.freegripjawwidth[j])
            tmphnd.setMat(pandanpmat4 = ttgsrotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            if not result.getNumContacts():
                ttgscct0=goalrotmat4.xformPoint(self.freegripcontacts[j][0])
                ttgscct1=goalrotmat4.xformPoint(self.freegripcontacts[j][1])
                ttgsfgrcenter = (ttgscct0+ttgscct1)/2
                handx =  ttgsrotmat.getRow3(0)
                ttgsfgrcenterhandx = ttgsfgrcenter + handx*self.rethandx
                ttgsfgrcenterhandxworldz = ttgsfgrcenterhandx + self.worldz*self.retworldz
                ttgsfgrcenterworlda = ttgsfgrcenter + self.worlda*self.retworlda
                ttgsfgrcenterworldaworldz = ttgsfgrcenterworlda+ self.worldz*self.retworldz
                ttgsjawwidth = self.freegripjawwidth[j]
                ttgsidfreeair = self.freegripids[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
                ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
                ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                ikc = self.robot.numikr(ttgsfgrcenternp, ttgsrotmat3np, armid = 'lft')
                ikcx = self.robot.numikr(ttgsfgrcenternp_handx, ttgsrotmat3np, armid = 'lft')
                ikcxz = self.robot.numikr(ttgsfgrcenternp_handxworldz, ttgsrotmat3np, armid = 'lft')
                ikca = self.robot.numikr(ttgsfgrcenternp_worlda, ttgsrotmat3np, armid = 'lft')
                ikcaz = self.robot.numikr(ttgsfgrcenternp_worldaworldz, ttgsrotmat3np, armid = 'lft')
                if (ikc is not None) and (ikcx is not None) and (ikcxz is not None) \
                        and (ikca is not None) and (ikcaz is not None):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = goalrotmat4.getRow3(3)
                    goalrotmat4worlda = Mat4(goalrotmat4)
                    goalrotmat4worlda.setRow(3, goalrotmat4.getRow3(3)+self.worlda*self.retworlda)
                    goalrotmat4worldaworldz = Mat4(goalrotmat4worlda)
                    goalrotmat4worldaworldz.setRow(3, goalrotmat4worlda.getRow3(3)+self.worldz*self.retworldz)
                    self.regg.add_node('goallft'+str(j), fgrcenter=ttgsfgrcenternp,
                                       fgrcenterhandx = ttgsfgrcenternp_handx,
                                       fgrcenterhandxworldz = ttgsfgrcenternp_handxworldz,
                                       fgrcenterworlda = ttgsfgrcenternp_worlda,
                                       fgrcenterworldaworldz = ttgsfgrcenternp_worldaworldz,
                                       jawwidth=ttgsjawwidth, hndrotmat3np=ttgsrotmat3np,
                                       globalgripid = ttgsidfreeair, freetabletopplacementid = 'na',
                                       tabletopplacementrotmat = goalrotmat4,
                                       tabletopplacementrotmathandx = goalrotmat4,
                                       tabletopplacementrotmathandxworldz = goalrotmat4,
                                       tabletopplacementrotmatworlda = goalrotmat4worlda,
                                       tabletopplacementrotmatworldaworldz = goalrotmat4worldaworldz,
                                       angleid = 'na', tabletopposition = tabletopposition)
                    nodeidofglobalidingoallft[ttgsidfreeair]='goallft'+str(j)
                    self.goallftnodeids.append('goallft'+str(j))
            tmphnd.setMat(pandanpmat4 = initmat)
            tmphnd.setJawwidth(initjawwidth)

        if len(self.goallftnodeids) == 0:
            print "No available goal grip for left hand!"

        # add edge right
        # add goal transit edges
        for edge in list(itertools.combinations(self.goalrgtnodeids, 2)):
            self.regg.add_edge(*edge, weight = 1, edgetype = 'goalrgttransit')
        # add goal transfer edge
        for reggnode, reggnodedata in self.regg.nodes(data=True):
            if reggnode.startswith('rgt') or reggnode.startswith('horgt'):
                globalgripid = reggnodedata['globalgripid']
                if globalgripid in nodeidofglobalidingoalrgt.keys():
                    goalnodeid = nodeidofglobalidingoalrgt[globalgripid]
                    self.regg.add_edge(goalnodeid, reggnode, weight=1, edgetype = 'goalrgttransfer')

        # add edge left
        # add goal transit edges
        for edge in list(itertools.combinations(self.goallftnodeids, 2)):
            self.regg.add_edge(*edge, weight = 1, edgetype = 'goallfttransit')
        # add goal transfer edge
        for reggnode, reggnodedata in self.regg.nodes(data=True):
            if reggnode.startswith('lft') or reggnode.startswith('holft'):
                globalgripid = reggnodedata['globalgripid']
                if globalgripid in nodeidofglobalidingoallft.keys():
                    goalnodeid = nodeidofglobalidingoallft[globalgripid]
                    self.regg.add_edge(goalnodeid, reggnode, weight=1, edgetype = 'goallfttransfer')

        # add start to goal direct edges rgt-rgt
        for startnodeid in self.startrgtnodeids:
            for goalnodeid in self.goalrgtnodeids:
                # startnodeggid = start node global grip id
                startnodeggid = self.regg.node[startnodeid]['globalgripid']
                goalnodeggid = self.regg.node[goalnodeid]['globalgripid']
                if startnodeggid == goalnodeggid:
                    self.regg.add_edge(startnodeid, goalnodeid, weight=1, edgetype = 'startgoalrgttransfer')

        # add start to goal direct edges lft-lft
        for startnodeid in self.startlftnodeids:
            for goalnodeid in self.goallftnodeids:
                # startnodeggid = start node global grip id
                startnodeggid = self.regg.node[startnodeid]['globalgripid']
                goalnodeggid = self.regg.node[goalnodeid]['globalgripid']
                if startnodeggid == goalnodeggid:
                    self.regg.add_edge(startnodeid, goalnodeid, weight=1, edgetype = 'startgoallfttransfer')

    def findshortestpath(self, startrotmat4, goalrotmat4, base, bagain = False):
        if bagain == False:
            self.__addstartgoal(startrotmat4, goalrotmat4, base)

        # startrgt goalrgt
        if len(self.startrgtnodeids) > 0 and len(self.goalrgtnodeids) > 0:
            startgrip = self.startrgtnodeids[0]
            goalgrip = self.goalrgtnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
            self.directshortestpaths_startrgtgoalrgt = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path)-1:
                            continue
                        else:
                            self.directshortestpaths_startrgtgoalrgt.append(path[i-1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startrgtgoalrgt[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startrgtgoalrgt[-1]=self.directshortestpaths_startrgtgoalrgt[-1][:i+1]
                            break
            except:
                assert('No startrgt goalrgt path')

        # startrgt goallft
        if len(self.startrgtnodeids) > 0 and len(self.goallftnodeids) > 0:
            startgrip = self.startrgtnodeids[0]
            goalgrip = self.goallftnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
            self.directshortestpaths_startrgtgoallft = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path)-1:
                            continue
                        else:
                            self.directshortestpaths_startrgtgoallft.append(path[i-1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startrgtgoallft[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startrgtgoallft[-1]=self.directshortestpaths_startrgtgoallft[-1][:i+1]
                            break
            except:
                assert('No startrgt goallft path')

        # startlft goalrgt
        if len(self.startlftnodeids) > 0 and len(self.goalrgtnodeids) > 0:
            startgrip = self.startlftnodeids[0]
            goalgrip = self.goalrgtnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
            self.directshortestpaths_startlftgoalrgt = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path)-1:
                            continue
                        else:
                            self.directshortestpaths_startlftgoalrgt.append(path[i-1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startlftgoalrgt[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startlftgoalrgt[-1]=self.directshortestpaths_startlftgoalrgt[-1][:i+1]
                            break
            except:
                assert('No startlft goalrgt path')

        # startlft goallft
        if len(self.startlftnodeids) > 0 and len(self.goallftnodeids) > 0:
            startgrip = self.startlftnodeids[0]
            goalgrip = self.goallftnodeids[0]
            self.shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
            self.directshortestpaths_startlftgoallft = []
            try:
                for path in self.shortestpaths:
                    for i, pathnode in enumerate(path):
                        if pathnode.startswith('start') and i < len(path)-1:
                            continue
                        else:
                            self.directshortestpaths_startlftgoallft.append(path[i-1:])
                            break
                    for i, pathnode in enumerate(self.directshortestpaths_startlftgoallft[-1]):
                        if i > 0 and pathnode.startswith('goal'):
                            self.directshortestpaths_startlftgoallft[-1]=self.directshortestpaths_startlftgoallft[-1][:i+1]
                            break
            except:
                assert('No startlft goallft path')

    def removeBadNodes(self, nodelist):
        """
        remove the invalidated nodes to prepare for a new plan

        :param nodelist: a list of invalidated nodes
        :return:

        author: weiwei
        date: 20170920
        """

        self.regg.remove_nodes_from(nodelist)

    def plotgraph(self, pltfig):
        """
        plot the graph without start and goal

        :param pltfig: the matplotlib object
        :return:

        author: weiwei
        date: 20161217, sapporos
        """

        # biggest circle: grips; big circle: rotation; small circle: placements
        radiusplacement = 30
        radiusrot = 6
        radiusgrip = 1
        xyplacementspos = {}
        xydiscreterotspos = {}
        self.xyzglobalgrippos = {}
        for i, ttpsid in enumerate(self.fttpsids):
            xydiscreterotspos[ttpsid]={}
            self.xyzglobalgrippos[ttpsid]={}
            xypos = [radiusplacement*math.cos(2*math.pi/self.nfttps*i),
                     radiusplacement*math.sin(2*math.pi/self.nfttps*i)]
            xyplacementspos[ttpsid] = xypos
            for j, anglevalue in enumerate(self.angles):
                self.xyzglobalgrippos[ttpsid][anglevalue]={}
                xypos = [radiusrot*math.cos(math.radians(anglevalue)), radiusrot*math.sin(math.radians(anglevalue))]
                xydiscreterotspos[ttpsid][anglevalue] = \
                    [xyplacementspos[ttpsid][0]+xypos[0], xyplacementspos[ttpsid][1]+xypos[1]]
                for k, globalgripid in enumerate(self.globalgripids):
                    xypos = [radiusgrip*math.cos(2*math.pi/len(self.globalgripids)* k),
                             radiusgrip*math.sin(2*math.pi/len(self.globalgripids)*k)]
                    self.xyzglobalgrippos[ttpsid][anglevalue][globalgripid]=\
                        [xydiscreterotspos[ttpsid][anglevalue][0]+xypos[0],
                         xydiscreterotspos[ttpsid][anglevalue][1]+xypos[1], 0]

        # for start and goal grasps poses:
        self.xyzglobalgrippos_startgoal={}
        for k, globalgripid in enumerate(self.globalgripids):
            xypos = [radiusgrip * math.cos(2 * math.pi / len(self.globalgripids) * k),
                     radiusgrip * math.sin(2 * math.pi / len(self.globalgripids) * k)]
            self.xyzglobalgrippos_startgoal[globalgripid] = [xypos[0],xypos[1],0]

        # for handover
        nfp = len(self.floatingposes.gridsfloatingposemat4s)
        xdist = 10
        x = range(300,501,xdist)
        y = range(-50,50,100*xdist/nfp)

        transitedges = []
        transferedges = []
        hotransitedges = []
        hotransferedges = []
        startrgttransferedges = []
        startlfttransferedges = []
        goalrgttransferedges = []
        goallfttransferedges = []
        startgoalrgttransferedges = []
        startgoallfttransferedges = []
        startrgttransitedges = []
        goalrgttransitedges = []
        startlfttransitedges = []
        goallfttransitedges = []
        for nid0, nid1, reggedgedata in self.regg.edges(data=True):
            xyzpos0 = [0,0,0]
            xyzpos1 = [0,0,0]
            if (reggedgedata['edgetype'] is 'transit') or (reggedgedata['edgetype'] is 'transfer'):
                if nid0.startswith('ho'):
                    fpind0 = self.regg.node[nid0]['floatingposeind']
                    fpgpind0 = self.regg.node[nid0]['floatingposegrippairind']
                    nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind0])
                    xpos = x[fpind0 % len(x)]
                    ypos = y[fpind0/len(x)]
                    xyzpos0 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind0)+xpos,
                             radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind0)+ypos, 0]
                    if nid0.startswith('horgt'):
                        xyzpos0[1] = xyzpos0[1]-100
                    if nid0.startswith('holft'):
                        xyzpos0[1] = xyzpos0[1]+100
                else:
                    fttpid0 = self.regg.node[nid0]['freetabletopplacementid']
                    anglevalue0 = self.regg.node[nid0]['angle']
                    ggid0 = self.regg.node[nid0]['globalgripid']
                    tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                    xyzpos0 = map(add, self.xyzglobalgrippos[fttpid0][anglevalue0][ggid0],
                                  [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                    if nid0.startswith('rgt'):
                        xyzpos0[1] = xyzpos0[1]-800
                    if nid0.startswith('lft'):
                        xyzpos0[1] = xyzpos0[1]+800
                if nid1.startswith('ho'):
                    fpind1 = self.regg.node[nid1]['floatingposeind']
                    fpgpind1 = self.regg.node[nid1]['floatingposegrippairind']
                    nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind1])
                    xpos = x[fpind1 % len(x)]
                    ypos = y[fpind1/len(x)]
                    xyzpos1 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind1)+xpos,
                             radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind1)+ypos, 0]
                    if nid1.startswith('horgt'):
                        xyzpos1[1] = xyzpos1[1]-100
                    if nid1.startswith('holft'):
                        xyzpos1[1] = xyzpos1[1]+100
                else:
                    fttpid1 = self.regg.node[nid1]['freetabletopplacementid']
                    anglevalue1 = self.regg.node[nid1]['angle']
                    ggid1 = self.regg.node[nid1]['globalgripid']
                    tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                    xyzpos1 = map(add, self.xyzglobalgrippos[fttpid1][anglevalue1][ggid1],
                                  [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                    if nid1.startswith('rgt'):
                        xyzpos1[1] = xyzpos1[1]-800
                    if nid1.startswith('lft'):
                        xyzpos1[1] = xyzpos1[1]+800
                # 3d
                # if reggedgedata['edgetype'] is 'transit':
                #     transitedges.append([xyzpos0, xyzpos1])
                # if reggedgedata['edgetype'] is 'transfer':
                #     transferedges.append([xyzpos0, xyzpos1])
                #2d
                # move the basic graph to x+600
                xyzpos0[0] = xyzpos0[0]+600
                xyzpos1[0] = xyzpos1[0]+600
                if reggedgedata['edgetype'] is 'transit':
                    transitedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'] is 'transfer':
                    if nid0.startswith('ho') or nid1.startswith('ho'):
                        hotransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                    else:
                        transferedges.append([xyzpos0[:2], xyzpos1[:2]])
            elif (reggedgedata['edgetype'] is 'handovertransit'):
                fpind0 = self.regg.node[nid0]['floatingposeind']
                fpgpind0 = self.regg.node[nid0]['floatingposegrippairind']
                nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind0])
                xpos = x[fpind0 % len(x)]
                ypos = y[fpind0/len(x)]
                xyzpos0 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind0)+xpos,
                         radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind0)+ypos, 0]
                if nid0.startswith('horgt'):
                    xyzpos0[1] = xyzpos0[1]-100
                if nid0.startswith('holft'):
                    xyzpos0[1] = xyzpos0[1]+100
                fpind1 = self.regg.node[nid1]['floatingposeind']
                fpgpind1 = self.regg.node[nid1]['floatingposegrippairind']
                nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind1])
                xpos = x[fpind1 % len(x)]
                ypos = y[fpind1/len(x)]
                xyzpos1 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind1)+xpos,
                         radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind1)+ypos, 0]
                if nid1.startswith('horgt'):
                    xyzpos1[1] = xyzpos1[1]-100
                if nid1.startswith('holft'):
                    xyzpos1[1] = xyzpos1[1]+100
                # move the basic graph to x+600
                xyzpos0[0] = xyzpos0[0]+600
                xyzpos1[0] = xyzpos1[0]+600
                hotransitedges.append([xyzpos0[:2], xyzpos1[:2]])
            elif reggedgedata['edgetype'].endswith('transit'):
                gid0 = self.regg.node[nid0]['globalgripid']
                gid1 = self.regg.node[nid1]['globalgripid']
                tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                xyzpos0 = map(add, self.xyzglobalgrippos_startgoal[gid0],
                              [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                xyzpos1 = map(add, self.xyzglobalgrippos_startgoal[gid1],
                              [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                if reggedgedata['edgetype'] is 'startrgttransit':
                    startrgttransitedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'] is 'goalrgttransit':
                    goalrgttransitedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'] is 'startlfttransit':
                    startlfttransitedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'] is 'goallfttransit':
                    goallfttransitedges.append([xyzpos0[:2], xyzpos1[:2]])
            elif reggedgedata['edgetype'].endswith('transfer'):
                if nid0.startswith('ho'):
                    fpind0 = self.regg.node[nid0]['floatingposeind']
                    fpgpind0 = self.regg.node[nid0]['floatingposegrippairind']
                    nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind0])
                    xpos = x[fpind0 % len(x)]
                    ypos = y[fpind0/len(x)]
                    xyzpos0 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind0)+xpos,
                             radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind0)+ypos, 0]
                    if nid0.startswith('horgt'):
                        xyzpos0[1] = xyzpos0[1]-100
                    if nid0.startswith('holft'):
                        xyzpos0[1] = xyzpos0[1]+100
                    xyzpos0[0] = xyzpos0[0]+600
                elif nid0.startswith('rgt') or nid0.startswith('lft'):
                    fttpid0 = self.regg.node[nid0]['freetabletopplacementid']
                    anglevalue0 = self.regg.node[nid0]['angle']
                    ggid0 = self.regg.node[nid0]['globalgripid']
                    tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                    xyzpos0 = map(add, self.xyzglobalgrippos[fttpid0][anglevalue0][ggid0],
                                  [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                    if nid0.startswith('rgt'):
                        xyzpos0[1] = xyzpos0[1]-800
                    if nid0.startswith('lft'):
                        xyzpos0[1] = xyzpos0[1]+800
                    xyzpos0[0] = xyzpos0[0]+600
                else:
                    gid0 = self.regg.node[nid0]['globalgripid']
                    tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                    xyzpos0 = map(add, self.xyzglobalgrippos_startgoal[gid0],
                                  [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                if nid1.startswith('ho'):
                    fpind1 = self.regg.node[nid1]['floatingposeind']
                    fpgpind1 = self.regg.node[nid1]['floatingposegrippairind']
                    nfpgp = len(self.floatingposes.floatinggrippairshndmat4s[fpind1])
                    xpos = x[fpind1 % len(x)]
                    ypos = y[fpind1/len(x)]
                    xyzpos1 = [radiusgrip * math.cos(2 * math.pi / nfpgp * fpgpind1)+xpos,
                             radiusgrip * math.sin(2 * math.pi / nfpgp * fpgpind1)+ypos, 0]
                    if nid1.startswith('horgt'):
                        xyzpos1[1] = xyzpos1[1]-100
                    if nid1.startswith('holft'):
                        xyzpos1[1] = xyzpos1[1]+100
                    xyzpos1[0] = xyzpos1[0]+600
                elif nid1.startswith('lft') or nid1.startswith('rgt'):
                    fttpid1 = self.regg.node[nid1]['freetabletopplacementid']
                    anglevalue1 = self.regg.node[nid1]['angle']
                    ggid1 = self.regg.node[nid1]['globalgripid']
                    tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                    xyzpos1 = map(add, self.xyzglobalgrippos[fttpid1][anglevalue1][ggid1],
                                  [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                    if nid1.startswith('rgt'):
                        xyzpos1[1] = xyzpos1[1]-800
                    if nid1.startswith('lft'):
                        xyzpos1[1] = xyzpos1[1]+800
                    xyzpos1[0] = xyzpos1[0]+600
                else:
                    ggid1 = self.regg.node[nid1]['globalgripid']
                    tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                    xyzpos1 = map(add, self.xyzglobalgrippos_startgoal[ggid1],
                                  [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                if reggedgedata['edgetype'].startswith('startgoalrgt'):
                    startgoalrgttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'].startswith('startgoallft'):
                    startgoallfttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'].startswith('startrgt'):
                    startrgttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'].startswith('startlft'):
                    startlfttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'].startswith('goalrgt'):
                    goalrgttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'].startswith('goallft'):
                    goallfttransferedges.append([xyzpos0[:2], xyzpos1[:2]])

            self.gnodesplotpos[nid0] = xyzpos0[:2]
            self.gnodesplotpos[nid1] = xyzpos1[:2]
        #3d
        # transitec = mc3d.Line3DCollection(transitedges, colors=[0,1,1,1], linewidths=1)
        # transferec = mc3d.Line3DCollection(transferedges, colors=[0,0,0,.1], linewidths=1)
        #2d
        transitec = mc.LineCollection(transitedges, colors=[0,1,1,1], linewidths=1)
        transferec = mc.LineCollection(transferedges, colors=[0,0,0,.1], linewidths=1)
        hotransitec = mc.LineCollection(hotransitedges, colors=[1,0,1,.1], linewidths=1)
        hotransferec = mc.LineCollection(hotransferedges, colors=[.5,.5,0,.03], linewidths=1)
        # transfer
        startrgttransferec = mc.LineCollection(startrgttransferedges, colors=[.7,0,0,.3], linewidths=1)
        startlfttransferec = mc.LineCollection(startlfttransferedges, colors=[.3,0,0,.3], linewidths=1)
        goalrgttransferec = mc.LineCollection(goalrgttransferedges, colors=[0,0,.7,.3], linewidths=1)
        goallfttransferec = mc.LineCollection(goallfttransferedges, colors=[0,0,.3,.3], linewidths=1)
        startgoalrgttransferec = mc.LineCollection(startgoalrgttransferedges, colors=[0,0,.7,.3], linewidths=1)
        startgoallfttransferec = mc.LineCollection(startgoallfttransferedges, colors=[0,0,.3,.3], linewidths=1)
        # transit
        startrgttransitec = mc.LineCollection(startrgttransitedges, colors=[0,.5,1,.3], linewidths=1)
        startlfttransitec = mc.LineCollection(startlfttransitedges, colors=[0,.2,.4,.3], linewidths=1)
        goalrgttransitec = mc.LineCollection(goalrgttransitedges, colors=[0,.5,1,.3], linewidths=1)
        goallfttransitec = mc.LineCollection(goallfttransitedges, colors=[0,.2,.4,.3], linewidths=1)

        ax = pltfig.add_subplot(111)
        ax.add_collection(transferec)
        ax.add_collection(transitec)
        ax.add_collection(hotransferec)
        ax.add_collection(hotransitec)
        ax.add_collection(startrgttransferec)
        ax.add_collection(startlfttransferec)
        ax.add_collection(goalrgttransferec)
        ax.add_collection(goallfttransferec)
        ax.add_collection(startgoalrgttransferec)
        ax.add_collection(startgoallfttransferec)
        # ax.add_collection(startrgttransitec)
        # ax.add_collection(startlfttransitec)
        # ax.add_collection(goalrgttransitec)
        # ax.add_collection(goallfttransitec)

        # for reggnode, reggnodedata in self.regg.nodes(data=True):
        #     placementid =  reggnodedata['placementid']
        #     angleid = reggnodedata['angleid']
        #     globalgripid = reggnodedata['globalgripid']
        #    tabletopposition = reggnodedata['tabletopposition']
        #     xyzpos = map(add, xyzglobalgrippos[placementid][angleid][str(globalgripid)],[tabletopposition[0], tabletopposition[1], tabletopposition[2]])
        #     plt.plot(xyzpos[0], xyzpos[1], 'ro')

    def plotshortestpath(self, pltfig, id = 0, choice = 'startrgtgoalrgt'):
        """
        plot the shortest path

        about transit and transfer:
        The tabletoppositions of start and goal are the local zero of the mesh model
        in contrast, the tabletoppositions of the other nodes in the graph are the local zero of the supporting facet
        if tabletopposition start == tabletop position goal
        there are two possibilities:
        1) start and goal are the same, then it is transit
        2) start and goal are different, then it is tranfer
        Note that start and the second will never be the same since they are in different coordinate systems.
        It is reasonable since the shortest path will never let the start go to the same position again.
        if the second item is not the goal, the path between the first and second items is
        sure to be a transfer path

        :param id: which path to plot
        :param choice: startrgtgoalrgt/startrgtgoallft/startlftgoalrgt/startlftgoallft
        :return:

        author: weiwei
        date: 20170302
        """

        directshortestpaths = []
        if choice is 'startrgtgoalrgt':
            directshortestpaths = self.directshortestpaths_startrgtgoalrgt
        elif choice is 'startrgtgoallft':
            directshortestpaths = self.directshortestpaths_startrgtgoallft
        elif choice is 'startlftgoalrgt':
            directshortestpaths = self.directshortestpaths_startlftgoalrgt
        elif choice is 'startlftgoallft':
            directshortestpaths = self.directshortestpaths_startlftgoallft
        for i,path in enumerate(directshortestpaths):
            if i == id:
                pathedgestransit = []
                pathedgestransfer = []
                pathlength = len(path)
                for pnidx in range(pathlength-1):
                    nid0 = path[pnidx]
                    nid1 = path[pnidx+1]
                    if self.regg[nid0][nid1]['edgetype'].endswith('transit'):
                        pathedgestransit.append([self.gnodesplotpos[nid0], self.gnodesplotpos[nid1]])
                    if self.regg[nid0][nid1]['edgetype'].endswith('transfer'):
                        pathedgestransfer.append([self.gnodesplotpos[nid0], self.gnodesplotpos[nid1]])
                pathtransitec = mc.LineCollection(pathedgestransit, colors=[.5, 1, 0, 1], linewidths=5)
                pathtransferec = mc.LineCollection(pathedgestransfer, colors=[0, 1, 0, 1], linewidths=5)

                ax = pltfig.gca()
                ax.add_collection(pathtransitec)
                ax.add_collection(pathtransferec)

    def plotgraphp3d(self, base):
        """
        draw the graph in panda3d

        :param base:
        :return:

        author: weiwei
        date: 20161216, osaka itami airport
        """

        # big circle: rotation; small circle: placements
        radiusplacement = 30
        radiusrot = 6
        radiusgrip = 1
        xyplacementspos = []
        xydiscreterotspos = []
        xyzglobalgrippos = []
        for i in range(self.nplacements):
            xydiscreterotspos.append([])
            xyzglobalgrippos.append([])
            xypos = [radiusplacement*math.cos(2*math.pi/self.nplacements*i), radiusplacement*math.sin(2*math.pi/self.nplacements*i)]
            xyplacementspos.append(xypos)
            for j in range(self.ndiscreterot):
                xyzglobalgrippos[-1].append({})
                xypos = [radiusrot*math.cos(2*math.pi/self.ndiscreterot* j), radiusrot*math.sin(2*math.pi/self.ndiscreterot * j)]
                xydiscreterotspos[-1].append([xyplacementspos[-1][0]+xypos[0],xyplacementspos[-1][1]+xypos[1]])
                for k, globalgripid in enumerate(self.globalgripids):
                    xypos = [radiusgrip*math.cos(2*math.pi/len(self.globalgripids)* k), radiusgrip*math.sin(2*math.pi/len(self.globalgripids)*k)]
                    xyzglobalgrippos[-1][-1][globalgripid]=[xydiscreterotspos[-1][-1][0]+xypos[0],xydiscreterotspos[-1][-1][1]+xypos[1], 0]

        transitedges = []
        transferedges = []
        for nid0, nid1, reggedgedata in self.regg.edges(data=True):
            fttpid0 = self.regg.node[nid0]['freetabletopplacementid']
            anglevalue0 = self.regg.node[nid0]['angle']
            gid0 = self.regg.node[nid0]['globalgripid']
            fttpid1 = self.regg.node[nid1]['freetabletopplacementid']
            angelvalue1 = self.regg.node[nid1]['angle']
            gid1 = self.regg.node[nid1]['globalgripid']
            tabletopposition0 = self.regg.node[nid0]['tabletopposition']
            tabletopposition1 = self.regg.node[nid1]['tabletopposition']
            xyzpos0 = map(add, xyzglobalgrippos[fttpid0][anglevalue0][str(gid0)],
                          [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
            xyzpos1 = map(add, xyzglobalgrippos[fttpid1][angelvalue1][str(gid1)],
                          [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
            # 3d
            if reggedgedata['edgetype'] is 'transit':
                transitedges.append([xyzpos0, xyzpos1])
            if reggedgedata['edgetype'] is 'transfer':
                transferedges.append([xyzpos0, xyzpos1])
        #3d
        transitecnp = pg.makelsnodepath(transitedges, rgbacolor=[0,1,1,1])
        transferecnp = pg.makelsnodepath(transferedges, rgbacolor=[0,0,0,.1])

        transitecnp.reparentTo(base.render)
        transferecnp.reparentTo(base.render)


if __name__=='__main__':
    gdb = db.GraspDB()

    handpkg = rtq85nm
    hrp5nrobot = hrp5n.Hrp5Robot()
    nxtrobot = nxt.NxtRobot()

    base = pandactrl.World(camp=[700,300,600], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "ttube.stl")
    regrip = RegripTppFp(objpath, nxtrobot, handpkg, gdb, base)

    # startrotmat4 = Mat4(-0.70710670948,-0.70710682869,0.0,0.0,
    #                     0.70710682869,-0.70710670948,0.0,0.0,
    #                     0.0,-0.0,-1.0,0.0,
    #                     300.004150391,-599.998901367,38.0383911133,1.0)
    # # goalrotmat4 = Mat4(0.707106769085,-0.707106769085,0.0,0.0,
    # #                    4.32978030171e-17,4.32978030171e-17,-1.0,0.0,
    # #                    0.707106769085,0.707106769085,6.12323426293e-17,0.0,
    # #                    499.998474121,400.003753662,44.9995155334,1.0)
    # goalrotmat4 = Mat4(-0.707106769085,-0.70710682869,0.0,0.0,
    #                    4.32978063259e-17,-4.32978030171e-17,-1.0,0.0,
    #                    0.70710682869,-0.707106769085,6.12323426293e-17,0.0,
    #                    400.003753662,600.001525879,44.9995155334,1.0)
    #
    # regrip.findshortestpath(startrotmat4, goalrotmat4, base)
    startrotmat4 = Mat4(1.0,0.0,0.0,0.0,
                        0.0,6.12323426293e-17,-1.0,0.0,
                        0.0,1.0,6.12323426293e-17,0.0,
                        399.996276855,-399.998413086,44.9995155334,1.0)
    goalrotmat4 = Mat4(1.0,0.0,0.0,0.0,
                       0.0,6.12323426293e-17,-1.0,0.0,
                       0.0,1.0,6.12323426293e-17,0.0,
                       499.996276855,200.001571655,44.9995155334,1.0)

    regrip.findshortestpath(startrotmat4, goalrotmat4, base)
    pltfig = plt.figure()
    regrip.plotgraph(pltfig)
    # regrip.plotshortestpath(pltfig, choice = 'startrgtgoalrgt')

    plt.axis("equal")
    plt.show()
    # regrip.plotgraphp3d(base)

    base.run()