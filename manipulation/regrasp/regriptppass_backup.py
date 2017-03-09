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
import floatingposes
from manipulation.assembly.asstwoobj import TwoObjAss as Toa

class GraphTpp(object):

    def __init__(self, objpath, robot, handpkg, gdb, armname):
        self.objtrimesh=trimesh.load_mesh(objpath)

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # regg = regrip graph
        self.regg = nx.Graph()

        self.gdb = gdb
        self.robot = robot
        self.handpkg = handpkg

        # load retraction distances
        self.rethandx, self.retworldz, self.retworlda, self.worldz = self.gdb.loadIKRet()
        # worlda is default for the general grasps on table top
        # for assembly at start and goal, worlda is computed by assembly planner
        self.worlda = Vec3(0,0,1)

        self.globalgripids = []
        self.fttpsids = []
        self.nfttps = 0
        self.gnodesplotpos = {}

        self.gdb = gdb
        self.robot = robot
        self.handpkg = handpkg

        self.__loadGripsToBuildGraph(armname)

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
                sql = "SELECT tabletopgrips.idtabletopgrips, tabletopgrips.contactpnt0, tabletopgrips.contactpnt1, \
                       tabletopgrips.rotmat, tabletopgrips.jawwidth, tabletopgrips.idfreeairgrip \
                       FROM tabletopgrips,ik,freeairgrip,hand WHERE tabletopgrips.idfreeairgrip = freeairgrip.idfreeairgrip AND \
                       freeairgrip.idhand = hand.idhand AND\
                       tabletopgrips.idtabletopgrips=ik.idtabletopgrips AND \
                       tabletopgrips.idtabletopplacements = %d AND ik.idrobot=%d AND \
                       ik.feasibility='True' AND ik.feasibility_handx='True' AND ik.feasibility_handxworldz='True' \
                       AND ik.feasibility_worlda='True' AND ik.feasibility_worldaworldz='True' AND ik.idarm = %d \
                       AND hand.name LIKE '%s'" \
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
            for globalidedgesid in globalidsedges:
                for edge in list(itertools.combinations(globalidsedges[globalidedgesid], 2)):
                    self.regg.add_edge(*edge, weight=1, edgetype = 'transfer')

        # gen plot pos
        # biggest circle: grips; big circle: rotation; small circle: placements
        radiusplacement = 30
        radiusrot = 6
        radiusgrip = 1
        xyplacementspos = {}
        xydiscreterotspos = {}
        xyzglobalgrippos = {}
        for i, ttpsid in enumerate(self.fttpsids):
            xydiscreterotspos[ttpsid] = {}
            xyzglobalgrippos[ttpsid] = {}
            xypos = [radiusplacement * math.cos(2 * math.pi / self.nfttps * i),
                     radiusplacement * math.sin(2 * math.pi / self.nfttps * i)]
            xyplacementspos[ttpsid] = xypos
            for j, anglevalue in enumerate(self.angles):
                xyzglobalgrippos[ttpsid][anglevalue] = {}
                xypos = [radiusrot * math.cos(anglevalue), radiusrot * math.sin(anglevalue)]
                xydiscreterotspos[ttpsid][anglevalue] = \
                    [xyplacementspos[ttpsid][0] + xypos[0], xyplacementspos[ttpsid][1] + xypos[1]]
                for k, globalgripid in enumerate(self.globalgripids):
                    xypos = [radiusgrip * math.cos(2 * math.pi / len(self.globalgripids) * k),
                             radiusgrip * math.sin(2 * math.pi / len(self.globalgripids) * k)]
                    xyzglobalgrippos[ttpsid][anglevalue][globalgripid] = \
                        [xydiscreterotspos[ttpsid][anglevalue][0] + xypos[0],
                         xydiscreterotspos[ttpsid][anglevalue][1] + xypos[1], 0]
        for nid in self.regg.nodes():
            fttpid = self.regg.node[nid]['freetabletopplacementid']
            anglevalue = self.regg.node[nid]['angle']
            ggid = self.regg.node[nid]['globalgripid']
            tabletopposition = self.regg.node[nid]['tabletopposition']
            xyzpos = map(add, xyzglobalgrippos[fttpid][anglevalue][ggid],
                          [tabletopposition[0], tabletopposition[1], tabletopposition[2]])
            self.gnodesplotpos[nid] = xyzpos[:2]

    def plotGraph(self, pltax, offset = [0,0]):
        """

        :param pltax:
        :param offset: where to plot the graph
        :return:
        """

        # add offset
        for i in self.gnodesplotpos.keys():
            self.gnodesplotpos[i] = map(add, self.gnodesplotpos[i], offset)

        transitedges = []
        transferedges = []
        for nid0, nid1, reggedgedata in self.regg.edges(data=True):
            if reggedgedata['edgetype'] is 'transit':
                transitedges.append([self.gnodesplotpos[nid0][:2], self.gnodesplotpos[nid1][:2]])
            if reggedgedata['edgetype'] is 'transfer':
                transferedges.append([self.gnodesplotpos[nid0][:2], self.gnodesplotpos[nid1][:2]])
        transitec = mc.LineCollection(transitedges, colors=[0,1,1,1], linewidths=1)
        transferec = mc.LineCollection(transferedges, colors=[0,0,0,.1], linewidths=1)
        pltax.add_collection(transferec)
        pltax.add_collection(transitec)


class RegripTppAss(object):

    def __init__(self, objpath, nxtrobot, handpkg, gdb):

        self.graphtpp0 = GraphTpp(objpath, nxtrobot, handpkg, gdb, 'rgt')
        self.armname = armname

        self.gdb = gdb
        self.robot = nxtrobot
        self.hand = handpkg.newHandNM(hndcolor=[1, 0, 0, .1])

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

        # self.endnodeids is a dictionary where
        # self.endnodeids['rgt'] equals self.startnodeids
        # self.endnodeids['lft'] equals self.endnodeids # in right->left order
        self.endnodeids  = {}

        # load retraction distances
        self.rethandx, self.retworldz, self.retworlda, self.worldz = self.gdb.loadIKRet()
        # worlda is default for the general grasps on table top
        # for assembly at start and goal, worlda is computed by assembly planner
        self.worlda = Vec3(0,0,1)

        self.gnodesplotpos = {}

        self.freegripid = []
        self.freegripcontacts = []
        self.freegripnormals = []
        self.freegriprotmats = []
        self.freegripjawwidth = []

        # for start and goal grasps poses:
        radiusgrip = 1
        self.__xyzglobalgrippos_startgoal={}
        for k, globalgripid in enumerate(self.graphtpp.globalgripids):
            xypos = [radiusgrip * math.cos(2 * math.pi / len(self.graphtpp.globalgripids) * k),
                     radiusgrip * math.sin(2 * math.pi / len(self.graphtpp.globalgripids) * k)]
            self.__xyzglobalgrippos_startgoal[globalgripid] = [xypos[0],xypos[1],0]

        self.__loadFreeAirGrip()

    @property
    def dbobjname(self):
        # read-only property
        return self.graphtpp.dbobjname

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

    def addEnds(self, rotmat4, armname):
        # the node id of a globalgripid in end
        nodeidofglobalidinend = {}
        # the endnodeids is also for quick access
        self.endnodeids[armname] = []
        for j, rotmat in enumerate(self.freegriprotmats):
            assgsrotmat = rotmat * rotmat4
            # for collision detection, we move the object back to x=0,y=0
            assgsrotmatx0y0 = Mat4(rotmat4)
            assgsrotmatx0y0.setCell(3,0,0)
            assgsrotmatx0y0.setCell(3,1,0)
            assgsrotmatx0y0 = rotmat * assgsrotmatx0y0
            # check if the hand collide with tabletop
            tmphand = self.hand
            initmat = tmphand.getMat()
            initjawwidth = tmphand.jawwidth
            # set jawwidth to 80 to avoid collision with surrounding obstacles
            # set to gripping with is unnecessary
            # tmphand.setJawwidth(self.freegripjawwidth[j])
            tmphand.setJawwidth(tmphand.jawwidthopen)
            tmphand.setMat(assgsrotmatx0y0)
            # add hand model to bulletworld
            hndbullnode = cd.genCollisionMeshMultiNp(tmphand.handnp)
            result = self.bulletworld.contactTest(hndbullnode)
            if not result.getNumContacts():
                assgscct0=rotmat4.xformPoint(self.freegripcontacts[j][0])
                assgscct1=rotmat4.xformPoint(self.freegripcontacts[j][1])
                assgsfgrcenter = (assgscct0+assgscct1)/2
                handx = assgsrotmat.getRow3(0)
                assgsfgrcenterhandx = assgsfgrcenter + handx*self.rethandx
                # handxworldz is not necessary for start
                # assgsfgrcenterhandxworldz = assgsfgrcenterhandx + self.worldz*self.retworldz
                assgsfgrcenterworlda = assgsfgrcenter + self.worlda*self.retworlda
                assgsfgrcenterworldaworldz = assgsfgrcenterworlda+ self.worldz*self.retworldz
                assgsjawwidth = self.freegripjawwidth[j]
                assgsidfreeair = self.freegripid[j]
                assgsfgrcenternp = pg.v3ToNp(assgsfgrcenter)
                assgsfgrcenternp_handx = pg.v3ToNp(assgsfgrcenterhandx)
                # handxworldz is not necessary for start
                # assgsfgrcenternp_handxworldz = pg.v3ToNp(assgsfgrcenterhandxworldz)
                assgsfgrcenternp_worlda = pg.v3ToNp(assgsfgrcenterworlda)
                assgsfgrcenternp_worldaworldz = pg.v3ToNp(assgsfgrcenterworldaworldz)
                assgsrotmat3np = pg.mat3ToNp(assgsrotmat.getUpper3())
                ikc = self.robot.numikr(assgsfgrcenternp, assgsrotmat3np)
                ikcx = self.robot.numikr(assgsfgrcenternp_handx, assgsrotmat3np)
                ikca = self.robot.numikr(assgsfgrcenternp_worlda, assgsrotmat3np)
                ikcaz = self.robot.numikr(assgsfgrcenternp_worldaworldz, assgsrotmat3np)
                if (ikc is not None) and (ikcx is not None) and (ikca is not None) and (ikcaz is not None):
                    # note the tabletopposition here is not the contact for the intermediate states
                    # it is the zero pos
                    tabletopposition = rotmat4.getRow3(3)
                    startrotmat4worlda = Mat4(rotmat4)
                    startrotmat4worlda.setRow(3, rotmat4.getRow3(3)+self.worlda*self.retworlda)
                    startrotmat4worldaworldz = Mat4(startrotmat4worlda)
                    startrotmat4worldaworldz.setRow(3, startrotmat4worlda.getRow3(3)+self.worldz*self.retworldz)
                    self.graphtpp.regg.add_node('end'+armname+str(j), fgrcenter=assgsfgrcenternp,
                                       fgrcenterhandx = assgsfgrcenternp_handx,
                                       fgrcenterhandxworldz = 'na',
                                       fgrcenterworlda = assgsfgrcenternp_worlda,
                                       fgrcenterworldaworldz = assgsfgrcenternp_worldaworldz,
                                       jawwidth=assgsjawwidth, hndrotmat3np=assgsrotmat3np,
                                       globalgripid = assgsidfreeair, freetabletopplacementid = 'na',
                                       tabletopplacementrotmat = rotmat4,
                                       tabletopplacementrotmathandx = rotmat4,
                                       tabletopplacementrotmathandxworldz = 'na',
                                       tabletopplacementrotmatworlda = startrotmat4worlda,
                                       tabletopplacementrotmatworldaworldz = startrotmat4worldaworldz,
                                       angle = 'na', tabletopposition = tabletopposition)
                    nodeidofglobalidinend[assgsidfreeair]='start'+str(j)
                    self.endnodeids[armname].append('start'+str(j))
            tmphand.setMat(initmat)
            tmphand.setJawwidth(initjawwidth)

        if len(self.endnodeids[armname]) == 0:
            raise ValueError("No available end grip at " + armname)

        # add start transit edge
        for edge in list(itertools.combinations(self.endnodeids[armname], 2)):
            self.graphtpp.regg.add_edge(*edge, weight = 1, edgetype = 'endtransit')
        # add start transfer edge
        for reggnode, reggnodedata in self.graphtpp.regg.nodes(data=True):
            if reggnode.startswith(self.armname):
                globalgripid = reggnodedata['globalgripid']
                if globalgripid in nodeidofglobalidinend.keys():
                    endnodeid = nodeidofglobalidinend[globalgripid]
                    self.graphtpp.regg.add_edge(endnodeid, reggnode, weight=1, edgetype = 'endtransfer')

        for nid in self.graphtpp.regg.nodes():
            if nid.startswith('end'):
                ggid = self.graphtpp.regg.node[nid]['globalgripid']
                tabletopposition = self.graphtpp.regg.node[nid]['tabletopposition']
                xyzpos = map(add, self.__xyzglobalgrippos_startgoal[ggid],
                              [tabletopposition[0], tabletopposition[1], tabletopposition[2]])
                self.gnodesplotpos[nid] = xyzpos[:2]

    def plotGraph(self, pltax, endname = 'start', gtppoffset = [0,0]):
        """

        :param pltax:
        :param endname:
        :param gtppoffset: where to plot graphtpp, see the plotGraph function of GraphTpp class
        :return:
        """

        self.graphtpp.plotGraph(pltax, offset = gtppoffset)
        self.__plotEnds(pltax, endname)

    def __plotEnds(self, pltax, endname = 'end'):
        transitedges = []
        transferedges = []
        for nid0, nid1, reggedgedata in self.graphtpp.regg.edges(data=True):
            if nid0.startswith('end'):
                pos0 = self.gnodesplotpos[nid0][:2]
            else:
                pos0 = self.graphtpp.gnodesplotpos[nid0][:2]
            if nid1.startswith('end'):
                pos1 = self.gnodesplotpos[nid1][:2]
            else:
                pos1 = self.graphtpp.gnodesplotpos[nid1][:2]
            if (reggedgedata['edgetype'] == 'endtransit'):
                transitedges.append([pos0, pos1])
            elif (reggedgedata['edgetype'] is 'endtransfer'):
                transferedges.append([pos0, pos1])
        transitec = mc.LineCollection(transitedges, colors = [.5,0,0,.3], linewidths = 1)
        transferec = mc.LineCollection(transferedges, colors = [1,0,0,.3], linewidths = 1)
        pltax.add_collection(transferec)
        pltax.add_collection(transitec)


if __name__=='__main__':
    gdb = db.GraspDB()
    nxtrobot = nextage.NxtRobot()
    handpkg = rtq85nm

    base = pandactrl.World(camp=[700,300,600], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "planefrontstay.stl")
    armname = "rgt"
    regrip = RegripTppAss(objpath, nxtrobot, handpkg, gdb, armname)

    sprotmat4 = Mat4(1.0,0.0,0.0,0.0,\
                     0.0,1.0,0.0,0.0,\
                     0.0,0.0,1.0,0.0,\
                     350,-400,0.0,1.0)
    whrotmat4 = Mat4(1.0,0.0,0.0,0.0,\
                     0.0,1.0,0.0,0.0,\
                     0.0,0.0,1.0,0.0,\
                     350,400,0.0,1.0)

    regrip.addStart(sprotmat4)
    pltfig = plt.figure()
    ax = pltfig.add_subplot(111)
    regrip.plotGraph(ax, gtppoffset = [600,0])

    plt.axis("equal")
    plt.show()
    # regrip.plotgraphp3d(base)

    base.run()