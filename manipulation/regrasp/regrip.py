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

import networkx as nx
import math

class Regrip():

    def __init__(self, objpath):
        print objpath
        self.objtrimesh=trimesh.load_mesh(objpath)

        # regg = regrip graph
        self.regg = nx.Graph()

        self.ndiscreterot = 0
        self.nplacements = 0
        self.globalgripids = []

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

    def loadgripstobuildgraph(self):
        # connect to database
        db = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
        cursor = db.cursor()
        # get the global grip ids
        # and prepare the global edges
        globalidsedges = {}
        result = cursor.execute("SELECT idfreeairgrip FROM freeairgrip WHERE objname LIKE '%s'" % self.dbobjname)
        if result:
            ggrprows = np.array(list(cursor.fetchall()))
            for ggid in ggrprows:
                globalidsedges[str(ggid[0])] = []
                self.globalgripids.append(str(ggid[0]))
        else:
            print "cannot find the table of global grips"
        result = cursor.execute("SELECT idtabletopplacements, angleid, placementid, tabletopposition FROM tabletopplacements WHERE objname LIKE '%s'" % self.dbobjname)
        if result:
            tpsrows = np.array(list(cursor.fetchall()))
            # nubmer of discreted rotation
            self.ndiscreterot = len(set(tpsrows[:,1]))
            self.nplacements = len(set(tpsrows[:,2]))

            for i, tpsid in enumerate(tpsrows[:,0]):
                resultg = cursor.execute("SELECT idtabletopgrip, contactpnt0, contactpnt1, rotmat, jawwidth, idfreeairgrip FROM tabletopgrip WHERE idtabletopplacements = %d and ikfeasible like 'True'" % int(tpsid))
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

    def plotgraph(self):
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
            # if reggedgedata['edgetype'] is 'transit':
            #     transitedges.append([xyzpos0, xyzpos1])
            # if reggedgedata['edgetype'] is 'transfer':
            #     transferedges.append([xyzpos0, xyzpos1])
            #2d
            if reggedgedata['edgetype'] is 'transit':
                transitedges.append([xyzpos0[:2], xyzpos1[:2]])
            if reggedgedata['edgetype'] is 'transfer':
                transferedges.append([xyzpos0[:2], xyzpos1[:2]])
        #3d
        # transitec = mc3d.Line3DCollection(transitedges, colors=[0,1,1,1], linewidths=1)
        # transferec = mc3d.Line3DCollection(transferedges, colors=[0,0,0,.1], linewidths=1)
        #2d
        transitec = mc.LineCollection(transitedges, colors=[0,1,1,1], linewidths=1)
        transferec = mc.LineCollection(transferedges, colors=[0,0,0,.1], linewidths=1)
        fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        ax = fig.add_subplot(111)
        ax.add_collection(transferec)
        ax.add_collection(transitec)
        plt.axis('equal')

        # for reggnode, reggnodedata in self.regg.nodes(data=True):
        #     placementid =  reggnodedata['placementid']
        #     angleid = reggnodedata['angleid']
        #     globalgripid = reggnodedata['globalgripid']
        #    tabletopposition = reggnodedata['tabletopposition']
        #     xyzpos = map(add, xyzglobalgrippos[placementid][angleid][str(globalgripid)],[tabletopposition[0], tabletopposition[1], tabletopposition[2]])
        #     plt.plot(xyzpos[0], xyzpos[1], 'ro')

        plt.show()


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

    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0], "grip", "objects", "ttube.stl")
    regrip = Regrip(objpath)
    regrip.loadgripstobuildgraph()
    regrip.plotgraph()

    # base = pandactrl.World(camp=[700,300,300], lookatp=[0,0,0])
    # regrip.plotgraphp3d(base)

    # base.run()