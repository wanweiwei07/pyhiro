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
from robotsim.nextage import nxtik
from robotsim.nextage import nextage

import networkx as nx
import math
import random

# regriptpp means regrip using tabletop placements
class RegripTpp():

    def __init__(self, objpath):
        print objpath
        self.objtrimesh=trimesh.load_mesh(objpath)

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]
        self.freegripid = []
        self.freegripcontacts = []
        self.freegripnormals = []
        self.freegriprotmats = []
        self.freegripjawwidth = []
        # access to db
        db = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
        cursor = db.cursor()
        sql = "select idfreeairgrip, contactpnt0, contactpnt1, contactnormal0, contactnormal1, rotmat, jawwidth from freeairgrip \
                where objname like '%s'" % self.dbobjname
        try:
            cursor.execute(sql)
        except mdb.Error as e:
            print "MySQL Error [%d]: %s" % (e.args[0], e.args[1])
            db.rollback()
            raise mdb.Error
        data = list(cursor.fetchall())
        for i in range(len(data)):
            self.freegripid.append(data[i][0])
            self.freegripcontacts.append([dc.strToV3(data[i][1]), dc.strToV3(data[i][2])])
            self.freegripnormals.append([dc.strToV3(data[i][3]), dc.strToV3(data[i][4])])
            self.freegriprotmats.append(dc.strToMat4(data[i][5]))
            self.freegripjawwidth.append(float(data[i][6]))
        db.close()

        # regg = regrip graph
        self.regg = nx.Graph()

        self.ndiscreterot = 0
        self.nplacements = 0
        self.globalgripids = []

        # for removing the grasps at start and goal
        self.rtq85hnd = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, .1])

        # use two bulletworld, one for the ray, the other for the tabletop
        self.bulletworld = BulletWorld()
        # add tabletop plane model to bulletworld
        # tt = tabletop
        this_dir, this_filename = os.path.split(__file__)
        ttpath = Filename.fromOsSpecific(os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "supports", "tabletop.egg"))
        self.ttnodepath = NodePath("tabletop")
        ttl = loader.loadModel(ttpath)
        ttl.instanceTo(self.ttnodepath)
        self.ttbullnode = cd.genCollisionMeshNp(self.ttnodepath)
        self.bulletworld.attachRigidBody(self.ttbullnode)

        self.startnodeids = None
        self.goalnodeids = None
        self.shortestpaths = None

    def loadgripstobuildgraph(self):
        # connect to database
        db = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
        cursor = db.cursor()
        # get the global grip ids
        # and prepare the global edges
        # for each globalgripid, find all its tabletopids (pertaining to placements)
        globalidsedges = {}
        result = cursor.execute("SELECT idfreeairgrip FROM freeairgrip WHERE objname LIKE '%s'" % self.dbobjname)
        if result:
            ggrprows = np.array(list(cursor.fetchall()))
            for ggid in ggrprows:
                globalidsedges[str(ggid[0])] = []
                self.globalgripids.append(ggid[0])
        else:
            print "cannot find the table of global grips"
        result = cursor.execute("SELECT idtabletopplacements, angleid, placementid, tabletopposition FROM tabletopplacements WHERE objname LIKE '%s' and angleid in (0)" % self.dbobjname)
        if result:
            tpsrows = np.array(list(cursor.fetchall()))
            # nubmer of discreted rotation
            self.angleids = list(set(map(int, tpsrows[:,1])))
            self.placementids = list(set(map(int, tpsrows[:,2])))
            self.ndiscreterot = len(self.angleids)
            self.nplacements = len(self.placementids)

            for i, tpsid in enumerate(tpsrows[:,0]):
                resultg = cursor.execute("SELECT idtabletopgrip, contactpnt0, contactpnt1, rotmat, jawwidth, "
                                         "idfreeairgrip FROM tabletopgrip WHERE idtabletopplacements = %d and ikfeasible like 'True'" % int(tpsid))
                if resultg:
                    ttgrows = np.array(list(cursor.fetchall()))
                    localidedges = []
                    for ttgrow in ttgrows:
                        ttgid = int(ttgrow[0])
                        ttgcct0 = dc.strToV3(ttgrow[1])
                        ttgcct1 = dc.strToV3(ttgrow[2])
                        ttgrotmat = dc.strToMat4(ttgrow[3])
                        ttgjawwidth = float(ttgrow[4])
                        ttgidfreeair = int(ttgrow[5])
                        ttgfgrcenter = (ttgcct0+ttgcct1)/2
                        self.regg.add_node(ttgid, fgrcenter=ttgfgrcenter, jawwidth=ttgjawwidth, hndrotmat4=ttgrotmat,
                                           globalgripid = ttgidfreeair, placementid = int(tpsrows[:,2][i]),
                                           angleid = int(tpsrows[:,1][i]), tabletopposition = dc.strToV3(tpsrows[:,3][i]))
                        globalidsedges[str(ttgidfreeair)].append(ttgid)
                        localidedges.append(ttgid)
                    # print list(itertools.combinations(ttgrows[:,0], 2))
                    for edge in list(itertools.combinations(localidedges, 2)):
                        self.regg.add_edge(*edge, weight = 1, edgetype = 'transit')
            for globalidedgesid in globalidsedges:
                for edge in list(itertools.combinations(globalidsedges[globalidedgesid], 2)):
                    self.regg.add_edge(*edge, weight=1, edgetype = 'transfer')
        db.close()

    def addstartgoal(self, startrotmat4, goalrotmat4, robot, base):
        """
        add start and goal for the regg

        :param startrotmat4 and goalrotmat4: both are 4by4 matrix
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
            tmprtq85.setJawwidth(self.freegripjawwidth[j])
            tmprtq85.setMat(ttgsrotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmprtq85.rtq85np)
            self.bulletworld.attachRigidBody(hndbullnode)
            result = self.bulletworld.contactTest(self.ttbullnode)
            # print result.getNumContacts()
            self.bulletworld.removeRigidBody(hndbullnode)
            if not result.getNumContacts():
                ttgscct0=ttgsrotmat.xformPoint(self.freegripcontacts[j][0])
                ttgscct1=ttgsrotmat.xformPoint(self.freegripcontacts[j][1])
                ttgsfgrcenter = (ttgscct0+ttgscct1)/2
                ttgsjawwidth = self.freegripjawwidth[j]
                ttgsidfreeair = self.freegripid[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                if nxtik.numikr(robot, ttgsfgrcenternp, ttgsrotmat3np):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = startrotmat4.getRow3(3)
                    self.regg.add_node('start'+str(j), fgrcenter=ttgsfgrcenter, jawwidth=ttgsjawwidth, hndrotmat4=ttgsrotmat,
                                       globalgripid = ttgsidfreeair, placementid = 'na',
                                       angleid = 'na', tabletopposition = tabletopposition)
                    nodeidofglobalidinstart[ttgsidfreeair]='start'+str(j)
                    self.startnodeids.append('start'+str(j))
                    # tmprtq85.reparentTo(base.render)
            tmprtq85.setMat(initmat)
            tmprtq85.setJawwidth(initjawwidth)

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
            self.bulletworld.attachRigidBody(hndbullnode)
            result = self.bulletworld.contactTest(self.ttbullnode)
            # print result.getNumContacts()
            self.bulletworld.removeRigidBody(hndbullnode)
            if not result.getNumContacts():
                ttgscct0=ttgsrotmat.xformPoint(self.freegripcontacts[j][0])
                ttgscct1=ttgsrotmat.xformPoint(self.freegripcontacts[j][1])
                ttgsfgrcenter = (ttgscct0+ttgscct1)/2
                ttgsjawwidth = self.freegripjawwidth[j]
                ttgsidfreeair = self.freegripid[j]
                ttgsfgrcenternp = pg.v3ToNp(ttgsfgrcenter)
                ttgsrotmat3np = pg.mat3ToNp(ttgsrotmat.getUpper3())
                if nxtik.numikr(robot, ttgsfgrcenternp, ttgsrotmat3np):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = goalrotmat4.getRow3(3)
                    self.regg.add_node('goal'+str(j), fgrcenter=ttgsfgrcenter, jawwidth=ttgsjawwidth, hndrotmat4=ttgsrotmat,
                                       globalgripid = ttgsidfreeair, placementid = 'na',
                                       angleid = 'na', tabletopposition = tabletopposition)
                    nodeidofglobalidingoal[ttgsidfreeair]='goal'+str(j)
                    self.goalnodeids.append('goal'+str(j))
            tmprtq85.setMat(initmat)
            tmprtq85.setJawwidth(initjawwidth)

        # add goal transit edges
        for edge in list(itertools.combinations(self.goalnodeids, 2)):
            self.regg.add_edge(*edge, weight = 1, edgetype = 'goaltransit')
        # add start transfer edge
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


    def findshortestpath(self):
        # startgrip = random.select(self.startnodeids)
        # goalgrip = random.select(self.goalnodeids)
        startgrip = self.startnodeids[0]
        goalgrip = self.goalnodeids[0]
        self.shortestpaths = nx.all_shortest_paths(self.regg, source = startgrip, target = goalgrip)
        print self.shortestpaths

    def plotgraph(self, pltfig):
        """
        plot the graph without start and goal

        :param pltfig: the matplotlib object
        :return:

        author: weiwei
        date: 20161217, sapporos
        """

        # big circle: rotation; small circle: placements
        radiusplacement = 30
        radiusrot = 6
        radiusgrip = 1
        xyplacementspos = {}
        xydiscreterotspos = {}
        self.xyzglobalgrippos = {}
        print self.angleids
        # placementid and angleid are fixed (not from the database autoincrease
        # use them to directly set and access positions
        # we could use select conditions to selectively choose some placements and angles
        # globalgripid is autoincreased in the database
        # it is not allowed to access its subset using select conditions
        for i, placementid in enumerate(self.placementids):
            xydiscreterotspos[placementid]={}
            self.xyzglobalgrippos[placementid]={}
            xypos = [radiusplacement*math.cos(2*math.pi/self.nplacements*placementid), radiusplacement*math.sin(2*math.pi/self.nplacements*placementid)]
            xyplacementspos[placementid] = xypos
            for j, angleid in enumerate(self.angleids):
                self.xyzglobalgrippos[placementid][angleid]={}
                xypos = [radiusrot*math.cos(2*math.pi/self.ndiscreterot* angleid), radiusrot*math.sin(2*math.pi/self.ndiscreterot * angleid)]
                xydiscreterotspos[placementid][angleid] = [xyplacementspos[placementid][0]+xypos[0], xyplacementspos[placementid][1]+xypos[1]]
                for k, globalgripid in enumerate(self.globalgripids):
                    xypos = [radiusgrip*math.cos(2*math.pi/len(self.globalgripids)* k), radiusgrip*math.sin(2*math.pi/len(self.globalgripids)*k)]
                    self.xyzglobalgrippos[placementid][angleid][globalgripid]=[xydiscreterotspos[placementid][angleid][0]+xypos[0],xydiscreterotspos[placementid][angleid][1]+xypos[1], 0]
        # for start and goal grasps poses:
        self.xyzlobalgrippos={}
        for k, globalgripid in enumerate(self.globalgripids):
            xypos = [radiusgrip * math.cos(2 * math.pi / len(self.globalgripids) * k),
                     radiusgrip * math.sin(2 * math.pi / len(self.globalgripids) * k)]
            self.xyzlobalgrippos[globalgripid] = [xypos[0],xypos[1], 0]

        transitedges = []
        transferedges = []
        starttransferedges = []
        goaltransferedges = []
        starttransitedges = []
        goaltransitedges = []
        for nid0, nid1, reggedgedata in self.regg.edges(data=True):
            if (reggedgedata['edgetype'] is 'transit') or (reggedgedata['edgetype'] is 'transfer'):
                pid0 = self.regg.node[nid0]['placementid']
                aid0 = self.regg.node[nid0]['angleid']
                gid0 = self.regg.node[nid0]['globalgripid']
                pid1 = self.regg.node[nid1]['placementid']
                aid1 = self.regg.node[nid1]['angleid']
                gid1 = self.regg.node[nid1]['globalgripid']
                tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                xyzpos0 = map(add, self.xyzglobalgrippos[pid0][aid0][gid0], [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                xyzpos1 = map(add, self.xyzglobalgrippos[pid1][aid1][gid1], [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
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
                    xyzpos0 = map(add, self.xyzlobalgrippos[gid0], [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                    xyzpos1 = map(add, self.xyzlobalgrippos[gid1], [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                    if reggedgedata['edgetype'] is 'starttransit':
                        starttransitedges.append([xyzpos0[:2], xyzpos1[:2]])
                    if reggedgedata['edgetype'] is 'goaltransit':
                        goaltransitedges.append([xyzpos0[:2], xyzpos1[:2]])
                else:
                    # start or goal transfer
                    print nid0,nid1
                    if isinstance(nid0, int):
                        pid0 = self.regg.node[nid0]['placementid']
                        aid0 = self.regg.node[nid0]['angleid']
                        gid0 = self.regg.node[nid0]['globalgripid']
                        tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                        gid1 = self.regg.node[nid1]['globalgripid']
                        tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                        xyzpos0 = map(add, self.xyzglobalgrippos[pid0][aid0][gid0], [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                        xyzpos1 = map(add, self.xyzlobalgrippos[gid1], [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                        if nid1[:4] == 'goal':
                            goaltransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                        else:
                            starttransferedges.append([xyzpos0[:2], xyzpos1[:2]])
                    if isinstance(nid1, int):
                        gid0 = self.regg.node[nid0]['globalgripid']
                        tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                        pid1 = self.regg.node[nid1]['placementid']
                        aid1 = self.regg.node[nid1]['angleid']
                        gid1 = self.regg.node[nid1]['globalgripid']
                        tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                        xyzpos0 = map(add, self.xyzlobalgrippos[gid0], [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                        xyzpos1 = map(add, self.xyzglobalgrippos[pid1][aid1][gid1], [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
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

        directshortestpaths = []
        for path in self.shortestpaths:
            for i, pathnode in enumerate(path):
                if isinstance(pathnode, str) and i < len(path)-1:
                    continue
                else:
                    directshortestpaths.append(path[i-1:])
                    break
            for i, pathnode in enumerate(directshortestpaths[-1]):
                if i > 0 and isinstance(pathnode, str):
                    directshortestpaths[-1]=directshortestpaths[-1][:i+1]
                    break
        print directshortestpaths
        for i,path in enumerate(directshortestpaths):
            print path
            if i is 0:
                pathedgestransit = []
                pathedgestransfer = []
                pathlength = len(path)
                for pnidx in range(pathlength-1):
                    nid0 = path[pnidx]
                    nid1 = path[pnidx+1]
                    if pnidx == 0 and pnidx+1 == pathlength-1:
                        pid0 = self.regg.node[nid0]['placementid']
                        gid0 = self.regg.node[nid0]['globalgripid']
                        tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                        pid1 = self.regg.node[nid1]['placementid']
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
                            pid1 = self.regg.node[nid1]['placementid']
                            aid1 = self.regg.node[nid1]['angleid']
                            gid1 = self.regg.node[nid1]['globalgripid']
                            tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                            xyzpos0 = map(add, self.xyzlobalgrippos[gid0], [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                            xyzpos1 = map(add, self.xyzglobalgrippos[pid1][aid1][gid1], [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                            pathedgestransfer.append([xyzpos0[:2], xyzpos1[:2]])
                        if pnidx+1 == pathlength-1:
                            # also definitely transfer
                            pid0 = self.regg.node[nid0]['placementid']
                            aid0 = self.regg.node[nid0]['angleid']
                            gid0 = self.regg.node[nid0]['globalgripid']
                            tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                            gid1 = self.regg.node[nid1]['globalgripid']
                            tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                            xyzpos0 = map(add, self.xyzglobalgrippos[pid0][aid0][gid0], [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                            xyzpos1 = map(add, self.xyzlobalgrippos[gid1], [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                            pathedgestransfer.append([xyzpos0[:2], xyzpos1[:2]])
                        if pnidx > 0 and pnidx+1 < pathlength-1:
                            pid0 = self.regg.node[nid0]['placementid']
                            aid0 = self.regg.node[nid0]['angleid']
                            gid0 = self.regg.node[nid0]['globalgripid']
                            tabletopposition0 = self.regg.node[nid0]['tabletopposition']
                            pid1 = self.regg.node[nid1]['placementid']
                            aid1 = self.regg.node[nid1]['angleid']
                            gid1 = self.regg.node[nid1]['globalgripid']
                            tabletopposition1 = self.regg.node[nid1]['tabletopposition']
                            xyzpos0 = map(add, self.xyzglobalgrippos[pid0][aid0][gid0], [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
                            xyzpos1 = map(add, self.xyzglobalgrippos[pid1][aid1][gid1], [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
                            if tabletopposition0 == tabletopposition1:
                                pathedgestransit.append([xyzpos0[:2], xyzpos1[:2]])
                            else:
                                pathedgestransfer.append([xyzpos0[:2], xyzpos1[:2]])

                #plot
                print pathedgestransit
                print pathedgestransfer
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
            pid0 = self.regg.node[nid0]['placementid']
            aid0 = self.regg.node[nid0]['angleid']
            gid0 = self.regg.node[nid0]['globalgripid']
            pid1 = self.regg.node[nid1]['placementid']
            aid1 = self.regg.node[nid1]['angleid']
            gid1 = self.regg.node[nid1]['globalgripid']
            tabletopposition0 = self.regg.node[nid0]['tabletopposition']
            tabletopposition1 = self.regg.node[nid1]['tabletopposition']
            xyzpos0 = map(add, xyzglobalgrippos[pid0][aid0][str(gid0)], [tabletopposition0[0], tabletopposition0[1], tabletopposition0[2]])
            xyzpos1 = map(add, xyzglobalgrippos[pid1][aid1][str(gid1)], [tabletopposition1[0], tabletopposition1[1], tabletopposition1[2]])
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
    nxtrobot = nextage.NxtRobot()
    nxtrobot.goinitpose()

    base = pandactrl.World(camp=[700,300,600], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "ttube.stl")
    regrip = RegripTpp(objpath)
    regrip.loadgripstobuildgraph()

    startrotmat4 = Mat4(-0.70710670948,-0.70710682869,0.0,0.0,
                        0.70710682869,-0.70710670948,0.0,0.0,
                        0.0,-0.0,-1.0,0.0,
                        300.004150391,-599.998901367,38.0383911133,1.0)
    # goalrotmat4 = Mat4(0.707106769085,-0.707106769085,0.0,0.0,
    #                    4.32978030171e-17,4.32978030171e-17,-1.0,0.0,
    #                    0.707106769085,0.707106769085,6.12323426293e-17,0.0,
    #                    499.998474121,400.003753662,44.9995155334,1.0)
    goalrotmat4 = Mat4(-0.707106769085,-0.70710682869,0.0,0.0,4.32978063259e-17,-4.32978030171e-17,-1.0,0.0,0.70710682869,-0.707106769085,6.12323426293e-17,0.0,400.003753662,600.001525879,44.9995155334,1.0)

    regrip.addstartgoal(startrotmat4, goalrotmat4, nxtrobot, base)
    regrip.findshortestpath()

    pltfig = plt.figure()
    regrip.plotgraph(pltfig)
    regrip.plotshortestpath(pltfig)

    plt.axis("equal")
    plt.show()
    # regrip.plotgraphp3d(base)

    base.run()