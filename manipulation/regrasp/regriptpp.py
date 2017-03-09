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
from robotsim.nextage import nextage
from robotsim.hrp5 import hrp5
from database import dbaccess as db

import networkx as nx
import math
import random

# regriptpp means regrip using tabletop placements
class RegripTpp():

    def __init__(self, objpath, robot, gdb):
        self.objtrimesh=trimesh.load_mesh(objpath)

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # regg = regrip graph
        self.regg = nx.Graph()

        self.ndiscreterot = 0
        self.nplacements = 0
        self.globalgripids = []

        # for removing the grasps at start and goal
        self.rtq85hnd = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, .1])

        # plane to remove hand
        self.bulletworld = BulletWorld()
        self.planebullnode = cd.genCollisionPlane()
        self.bulletworld.attachRigidBody(self.planebullnode)

        # add tabletop plane model to bulletworld
        this_dir, this_filename = os.path.split(__file__)
        ttpath = Filename.fromOsSpecific(os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "supports", "tabletop.egg"))
        self.ttnodepath = NodePath("tabletop")
        ttl = loader.loadModel(ttpath)
        ttl.instanceTo(self.ttnodepath)

        self.startnodeids = None
        self.goalnodeids = None
        self.shortestpaths = None

        self.gdb = gdb
        self.robot = robot

        # load retraction distances
        self.rethandx, self.retworldz, self.retworlda, self.worldz = self.gdb.loadIKRet()
        # worlda is default for the general grasps on table top
        # for assembly at start and goal, worlda is computed by assembly planner
        self.worlda = Vec3(0,0,1)

        # loadfreeairgrip
        self.__loadFreeAirGrip()
        self.__loadGripsToBuildGraph()

    def __loadFreeAirGrip(self):
        """
        load self.freegripid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: weiwei
        date: 20170110
        """

        freeairgripdata = self.gdb.loadFreeAirGrip(self.dbobjname)
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripid = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]

    def __loadGripsToBuildGraph(self, armname = "rgt"):
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
        idarm = gdb.loadIdArm(armname)

        # get the global grip ids
        # and prepare the global edges
        # for each globalgripid, find all its tabletopids (pertaining to placements)
        globalidsedges = {}
        sql = "SELECT idfreeairgrip FROM freeairgrip,object WHERE freeairgrip.idobject=object.idobject AND \
                object.name LIKE '%s'" % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) == 0:
            raise ValueError("Plan freeairgrip first!")
        for ggid in result:
            globalidsedges[str(ggid[0])] = []
            self.globalgripids.append(ggid[0])
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
                sql = "SELECT tabletopgrips.idtabletopgrips, contactpnt0, contactpnt1, rotmat, jawwidth, idfreeairgrip \
                        FROM tabletopgrips,ik WHERE tabletopgrips.idtabletopgrips=ik.idtabletopgrips AND \
                        tabletopgrips.idtabletopplacements = %d AND ik.idrobot=%d AND \
                        ik.feasibility='True' AND ik.feasibility_handx='True' AND ik.feasibility_handxworldz='True' \
                        AND ik.feasibility_worlda='True' AND ik.feasibility_worldaworldz='True' AND ik.idarm = %d" \
                        % (int(idtps), idrobot, idarm)
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
                    self.regg.add_node(ttgsid, fgrcenter=ttgsfgrcenternp,
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
                    globalidsedges[str(ttgsidfreeair)].append(ttgsid)
                    localidedges.append(ttgsid)
                # print list(itertools.combinations(ttgrows[:,0], 2))
                for edge in list(itertools.combinations(localidedges, 2)):
                    self.regg.add_edge(*edge, weight=1, edgetype = 'transit')
            if len(globalidsedges) == 0:
                raise ValueError("Plan tabletopgrips first!")
            for globalidedgesid in globalidsedges:
                for edge in list(itertools.combinations(globalidsedges[globalidedgesid], 2)):
                    self.regg.add_edge(*edge, weight=1, edgetype = 'transfer')

    def __addstartgoal(self, startrotmat4, goalrotmat4, base):
        """
        add start and goal for the regg

        :param startrotmat4 and goalrotmat4: both are 4by4 panda3d matrix
        :return:

        author: weiwei
        date: 20161216, sapporo
        """

        ### start
        # the node id of a globalgripid in startnode
        nodeidofglobalidinstart= {}
        # the startnodeids is also for quick access
        self.startnodeids = []
        self.ttnodepath.reparentTo(base.render)
        for j, rotmat in enumerate(self.freegriprotmats):
            ttgsrotmat = rotmat * startrotmat4
            # for collision detection, we move the object back to x=0,y=0
            ttgsrotmatx0y0 = Mat4(startrotmat4)
            ttgsrotmatx0y0.setCell(3,0,0)
            ttgsrotmatx0y0.setCell(3,1,0)
            ttgsrotmatx0y0 = rotmat * ttgsrotmatx0y0
            # check if the hand collide with tabletop
            tmprtq85 = self.rtq85hnd
            # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
            initmat = tmprtq85.getMat()
            initjawwidth = tmprtq85.jawwidth
            # set jawwidth to 80 to avoid collision with surrounding obstacles
            # set to gripping with is unnecessary
            # tmprtq85.setJawwidth(self.freegripjawwidth[j])
            tmprtq85.setJawwidth(80)
            tmprtq85.setMat(ttgsrotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmprtq85.rtq85np)
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
                ttgsidfreeair = self.freegripid[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
                # handxworldz is not necessary for start
                # ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
                ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                ikc = self.robot.numikr(ttgsfgrcenternp, ttgsrotmat3np)
                ikcx = self.robot.numikr(ttgsfgrcenternp_handx, ttgsrotmat3np)
                ikca = self.robot.numikr(ttgsfgrcenternp_worlda, ttgsrotmat3np)
                ikcaz = self.robot.numikr(ttgsfgrcenternp_worldaworldz, ttgsrotmat3np)
                if (ikc is not None) and (ikcx is not None) and (ikca is not None) and (ikcaz is not None):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = startrotmat4.getRow3(3)
                    startrotmat4worlda = Mat4(startrotmat4)
                    startrotmat4worlda.setRow(3, startrotmat4.getRow3(3)+self.worlda*self.retworlda)
                    startrotmat4worldaworldz = Mat4(startrotmat4worlda)
                    startrotmat4worldaworldz.setRow(3, startrotmat4worlda.getRow3(3)+self.worldz*self.retworldz)
                    self.regg.add_node('start'+str(j), fgrcenter=ttgsfgrcenternp,
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
                    nodeidofglobalidinstart[ttgsidfreeair]='start'+str(j)
                    self.startnodeids.append('start'+str(j))
                    # tmprtq85.reparentTo(base.render)
            tmprtq85.setMat(initmat)
            tmprtq85.setJawwidth(initjawwidth)

        if len(self.startnodeids) == 0:
            raise ValueError("No available starting grip!")

        # add start transit edge
        for edge in list(itertools.combinations(self.startnodeids, 2)):
            self.regg.add_edge(*edge, weight = 1, edgetype = 'starttransit')
        # add start transfer edge
        for reggnode, reggnodedata in self.regg.nodes(data=True):
            if isinstance(reggnode, int):
                globalgripid = reggnodedata['globalgripid']
                if globalgripid in nodeidofglobalidinstart.keys():
                    startnodeid = nodeidofglobalidinstart[globalgripid]
                    self.regg.add_edge(startnodeid, reggnode, weight=1, edgetype = 'starttransfer')

        ### goal
        # the node id of a globalgripid in goalnode, for quick setting up edges
        nodeidofglobalidingoal= {}
        # the goalnodeids is also for quick access
        self.goalnodeids = []
        for j, rotmat in enumerate(self.freegriprotmats):
            ttgsrotmat = rotmat * goalrotmat4
            # for collision detection, we move the object back to x=0,y=0
            ttgsrotmatx0y0 = Mat4(goalrotmat4)
            ttgsrotmatx0y0.setCell(3,0,0)
            ttgsrotmatx0y0.setCell(3,1,0)
            ttgsrotmatx0y0 = rotmat * ttgsrotmatx0y0
            # check if the hand collide with tabletop
            tmprtq85 = self.rtq85hnd
            # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
            initmat = tmprtq85.getMat()
            initjawwidth = tmprtq85.jawwidth
            tmprtq85.setJawwidth(self.freegripjawwidth[j])
            tmprtq85.setMat(ttgsrotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmprtq85.rtq85np)
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
                ttgsidfreeair = self.freegripid[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsfgrcenternp_handx = pg.v3ToNp(ttgsfgrcenterhandx)
                ttgsfgrcenternp_handxworldz = pg.v3ToNp(ttgsfgrcenterhandxworldz)
                ttgsfgrcenternp_worlda = pg.v3ToNp(ttgsfgrcenterworlda)
                ttgsfgrcenternp_worldaworldz = pg.v3ToNp(ttgsfgrcenterworldaworldz)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                ikc = self.robot.numikr(ttgsfgrcenternp, ttgsrotmat3np)
                ikcx = self.robot.numikr(ttgsfgrcenternp_handx, ttgsrotmat3np)
                ikcxz = self.robot.numikr(ttgsfgrcenternp_handxworldz, ttgsrotmat3np)
                ikca = self.robot.numikr(ttgsfgrcenternp_worlda, ttgsrotmat3np)
                ikcaz = self.robot.numikr(ttgsfgrcenternp_worldaworldz, ttgsrotmat3np)
                if (ikc is not None) and (ikcx is not None) and (ikcxz is not None) \
                        and (ikca is not None) and (ikcaz is not None):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = goalrotmat4.getRow3(3)
                    goalrotmat4worlda = Mat4(goalrotmat4)
                    goalrotmat4worlda.setRow(3, goalrotmat4.getRow3(3)+self.worlda*self.retworlda)
                    goalrotmat4worldaworldz = Mat4(goalrotmat4worlda)
                    goalrotmat4worldaworldz.setRow(3, goalrotmat4worlda.getRow3(3)+self.worldz*self.retworldz)
                    self.regg.add_node('goal'+str(j), fgrcenter=ttgsfgrcenternp,
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
                    nodeidofglobalidingoal[ttgsidfreeair]='goal'+str(j)
                    self.goalnodeids.append('goal'+str(j))
            tmprtq85.setMat(initmat)
            tmprtq85.setJawwidth(initjawwidth)

        if len(self.goalnodeids) == 0:
            raise ValueError("No available goal grip!")

        # add goal transit edges
        for edge in list(itertools.combinations(self.goalnodeids, 2)):
            self.regg.add_edge(*edge, weight = 1, edgetype = 'goaltransit')
        # add goal transfer edge
        for reggnode, reggnodedata in self.regg.nodes(data=True):
            if isinstance(reggnode, int):
                globalgripid = reggnodedata['globalgripid']
                if globalgripid in nodeidofglobalidingoal.keys():
                    goalnodeid = nodeidofglobalidingoal[globalgripid]
                    self.regg.add_edge(goalnodeid, reggnode, weight=1, edgetype = 'goaltransfer')

        # add start to goal direct edges
        for startnodeid in self.startnodeids:
            for goalnodeid in self.goalnodeids:
                # startnodeggid = start node global grip id
                startnodeggid = self.regg.node[startnodeid]['globalgripid']
                goalnodeggid = self.regg.node[goalnodeid]['globalgripid']
                if startnodeggid == goalnodeggid:
                    self.regg.add_edge(startnodeid, goalnodeid, weight=1, edgetype = 'startgoaltransfer')

    def findshortestpath(self, startrotmat4, goalrotmat4, base):
        self.__addstartgoal(startrotmat4, goalrotmat4, base)

        # startgrip = random.select(self.startnodeids)
        # goalgrip = random.select(self.goalnodeids)
        startgrip = self.startnodeids[0]
        goalgrip = self.goalnodeids[0]
        self.shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
        self.directshortestpaths = []
        # directshortestpaths removed the repeated start and goal transit
        for path in self.shortestpaths:
            for i, pathnode in enumerate(path):
                if isinstance(pathnode, str) and i < len(path)-1:
                    continue
                else:
                    self.directshortestpaths.append(path[i-1:])
                    break
            for i, pathnode in enumerate(self.directshortestpaths[-1]):
                if i > 0 and isinstance(pathnode, str):
                    self.directshortestpaths[-1]=self.directshortestpaths[-1][:i+1]
                    break

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
        self.xyzlobalgrippos={}
        for k, globalgripid in enumerate(self.globalgripids):
            xypos = [radiusgrip * math.cos(2 * math.pi / len(self.globalgripids) * k),
                     radiusgrip * math.sin(2 * math.pi / len(self.globalgripids) * k)]
            self.xyzlobalgrippos[globalgripid] = [xypos[0],xypos[1],0]

        transitedges = []
        transferedges = []
        starttransferedges = []
        goaltransferedges = []
        starttransitedges = []
        goaltransitedges = []
        for nid0, nid1, reggedgedata in self.regg.edges(data=True):
            if (reggedgedata['edgetype'] is 'transit') or (reggedgedata['edgetype'] is 'transfer'):
                fttpid0 = self.regg.node[nid0]['freetabletopplacementid']
                anglevalue0 = self.regg.node[nid0]['angle']
                ggid0 = self.regg.node[nid0]['globalgripid']
                fttpid1 = self.regg.node[nid1]['freetabletopplacementid']
                anglevalue1 = self.regg.node[nid1]['angle']
                ggid1 = self.regg.node[nid1]['globalgripid']
                tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                xyzpos0 = map(add, self.xyzglobalgrippos[fttpid0][anglevalue0][ggid0],
                              [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                xyzpos1 = map(add, self.xyzglobalgrippos[fttpid1][anglevalue1][ggid1],
                              [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                # 3d
                # if reggedgedata['edgetype'] is 'transit':
                #     transitedges.append([xyzpos0, xyzpos1])
                # if reggedgedata['edgetype'] is 'transfer':
                #     transferedges.append([xyzpos0, xyzpos1])
                #2d
                if reggedgedata['edgetype'] is 'transit':
                    transitedges.append([xyzpos0[:2], xyzpos1[:2]])
                if reggedgedata['edgetype'] is 'transfer':
                    transferedges.append([xyzpos0[:2], xyzpos1[:2]])
            else:
                if (reggedgedata['edgetype'] is 'starttransit') or (reggedgedata['edgetype'] is 'goaltransit'):
                    gid0 = self.regg.node[nid0]['globalgripid']
                    gid1 = self.regg.node[nid1]['globalgripid']
                    tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                    tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                    xyzpos0 = map(add, self.xyzlobalgrippos[gid0],
                                  [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                    xyzpos1 = map(add, self.xyzlobalgrippos[gid1],
                                  [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                    if reggedgedata['edgetype'] is 'starttransit':
                        starttransitedges.append([xyzpos0[:2], xyzpos1[:2]])
                    if reggedgedata['edgetype'] is 'goaltransit':
                        goaltransitedges.append([xyzpos0[:2], xyzpos1[:2]])
                else:
                    # start or goal transfer
                    if isinstance(nid0, int):
                        fttpid0 = self.regg.node[nid0]['freetabletopplacementid']
                        anglevalue0 = self.regg.node[nid0]['angle']
                        gid0 = self.regg.node[nid0]['globalgripid']
                        tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                        gid1 = self.regg.node[nid1]['globalgripid']
                        tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                        xyzpos0 = map(add, self.xyzglobalgrippos[fttpid0][anglevalue0][gid0],
                                      [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                        xyzpos1 = map(add, self.xyzlobalgrippos[gid1],
                                      [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                        if nid1[:4] == 'goal':
                            goaltransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                        else:
                            starttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                    if isinstance(nid1, int):
                        gid0 = self.regg.node[nid0]['globalgripid']
                        tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                        fttpid1 = self.regg.node[nid1]['freetabletopplacementid']
                        anglevalue1 = self.regg.node[nid1]['angle']
                        gid1 = self.regg.node[nid1]['globalgripid']
                        tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                        xyzpos0 = map(add, self.xyzlobalgrippos[gid0],
                                      [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                        xyzpos1 = map(add, self.xyzglobalgrippos[fttpid1][anglevalue1][gid1],
                                      [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                        if nid0[:4] == 'goal':
                            goaltransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                        else:
                            starttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
        #3d
        # transitec = mc3d.Line3DCollection(transitedges, colors=[0,1,1,1], linewidths=1)
        # transferec = mc3d.Line3DCollection(transferedges, colors=[0,0,0,.1], linewidths=1)
        #2d
        transitec = mc.LineCollection(transitedges, colors=[0,1,1,1], linewidths=1)
        transferec = mc.LineCollection(transferedges, colors=[0,0,0,.1], linewidths=1)
        starttransferec = mc.LineCollection(starttransferedges, colors=[1,0,0,.3], linewidths=1)
        goaltransferec = mc.LineCollection(goaltransferedges, colors=[0,0,1,.3], linewidths=1)
        starttransitec = mc.LineCollection(starttransitedges, colors=[.5,0,0,.3], linewidths=1)
        goaltransitec = mc.LineCollection(goaltransitedges, colors=[0,0,.5,.3], linewidths=1)

        ax = pltfig.add_subplot(111)
        ax.add_collection(transferec)
        ax.add_collection(transitec)
        ax.add_collection(starttransferec)
        ax.add_collection(goaltransferec)
        ax.add_collection(starttransitec)
        ax.add_collection(goaltransitec)

        # for reggnode, reggnodedata in self.regg.nodes(data=True):
        #     placementid =  reggnodedata['placementid']
        #     angleid = reggnodedata['angleid']
        #     globalgripid = reggnodedata['globalgripid']
        #    tabletopposition = reggnodedata['tabletopposition']
        #     xyzpos = map(add, xyzglobalgrippos[placementid][angleid][str(globalgripid)],[tabletopposition[0], tabletopposition[1], tabletopposition[2]])
        #     plt.plot(xyzpos[0], xyzpos[1], 'ro')

    def plotshortestpath(self, pltfig, id=None):
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

        :param id:
        :return:
        """

        for i,path in enumerate(self.directshortestpaths):
            if i is 0:
                pathedgestransit = []
                pathedgestransfer = []
                pathlength = len(path)
                for pnidx in range(pathlength-1):
                    nid0 = path[pnidx]
                    nid1 = path[pnidx+1]
                    if pnidx == 0 and pnidx+1 == pathlength-1:
                        gid0 = self.regg.node[nid0]['globalgripid']
                        tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                        gid1 = self.regg.node[nid1]['globalgripid']
                        tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                        xyzpos0 = map(add, self.xyzlobalgrippos[gid0],
                                      [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                        xyzpos1 = map(add, self.xyzlobalgrippos[gid1],
                                      [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                        if tabletopposition0 == tabletopposition1:
                            pathedgestransit.append([xyzpos0[:2], xyzpos1[:2]])
                        else:
                            pathedgestransfer.append([xyzpos0[:2], xyzpos1[:2]])
                    else:
                        if pnidx == 0:
                            # this is sure to be transfer
                            gid0 = self.regg.node[nid0]['globalgripid']
                            tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                            fttpid1 = self.regg.node[nid1]['freetabletopplacementid']
                            anglevalue1 = self.regg.node[nid1]['angle']
                            gid1 = self.regg.node[nid1]['globalgripid']
                            tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                            xyzpos0 = map(add, self.xyzlobalgrippos[gid0],
                                          [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                            xyzpos1 = map(add, self.xyzglobalgrippos[fttpid1][anglevalue1][gid1], [
                                tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                            pathedgestransfer.append([xyzpos0[:2], xyzpos1[:2]])
                        if pnidx+1 == pathlength-1:
                            # also definitely transfer
                            fttpid0 = self.regg.node[nid0]['freetabletopplacementid']
                            anglevalue0 = self.regg.node[nid0]['angle']
                            gid0 = self.regg.node[nid0]['globalgripid']
                            tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                            gid1 = self.regg.node[nid1]['globalgripid']
                            tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                            xyzpos0 = map(add, self.xyzglobalgrippos[fttpid0][anglevalue0][gid0],
                                          [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                            xyzpos1 = map(add, self.xyzlobalgrippos[gid1],
                                          [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                            pathedgestransfer.append([xyzpos0[:2], xyzpos1[:2]])
                        if pnidx > 0 and pnidx+1 < pathlength-1:
                            fttpid0 = self.regg.node[nid0]['freetabletopplacementid']
                            anglevalue0 = self.regg.node[nid0]['angle']
                            gid0 = self.regg.node[nid0]['globalgripid']
                            tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                            fttpid1 = self.regg.node[nid1]['freetabletopplacementid']
                            anglevalue1 = self.regg.node[nid1]['angle']
                            gid1 = self.regg.node[nid1]['globalgripid']
                            tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                            xyzpos0 = map(add, self.xyzglobalgrippos[fttpid0][anglevalue0][gid0],
                                          [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                            xyzpos1 = map(add, self.xyzglobalgrippos[fttpid1][anglevalue1][gid1],
                                          [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                            if tabletopposition0 == tabletopposition1:
                                pathedgestransit.append([xyzpos0[:2], xyzpos1[:2]])
                            else:
                                pathedgestransfer.append([xyzpos0[:2], xyzpos1[:2]])
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

    hrp5robot = hrp5.Hrp5Robot()
    nxtrobot = nextage.NxtRobot()
    handpkg = rtq85nm


    base = pandactrl.World(camp=[700,300,600], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planewheel.stl")
    regrip = RegripTpp(objpath, nxtrobot, gdb)

    startrotmat4 =  Mat4(1.0,0.0,0.0,0.0,\
                     0.0,0.0,1.0,0.0,\
                     0.0,-1.0,0.0,0.0,\
                     350,400,0.0,1.0)
    # goalrotmat4 = Mat4(0.707106769085,-0.707106769085,0.0,0.0,
    #                    4.32978030171e-17,4.32978030171e-17,-1.0,0.0,
    #                    0.707106769085,0.707106769085,6.12323426293e-17,0.0,
    #                    499.998474121,400.003753662,44.9995155334,1.0)
    goalrotmat4 =  Mat4(1.0,0.0,0.0,0.0,\
                     0.0,0.0,1.0,0.0,\
                     0.0,-1.0,0.0,0.0,\
                     350,400,0.0,1.0)

    regrip.findshortestpath(startrotmat4, goalrotmat4, base)

    pltfig = plt.figure()
    regrip.plotgraph(pltfig)
    regrip.plotshortestpath(pltfig)

    plt.axis("equal")
    plt.show()
    # regrip.plotgraphp3d(base)

    base.run()