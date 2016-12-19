#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from panda3d.bullet import BulletWorld
from panda3d.core import *

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
from manipulation.grip import tableplacements as tp
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from robotsim.nextage import nxtik
from robotsim.nextage import nextage


class TablePlacementsPosition(tp.TablePlacements):

    def __init__(self, objpath, robot):
        """
        initialization

        :param objpath: path of the object
        :param robot: the model implemented in nextage.py, etc

        author: weiwei
        date: 20161215, osaka
        """

        super(self.__class__, self).__init__(objpath)
        self.robot = robot

    def updateDBwithIK(self, doverh=.033, armid="rgt"):
        """
        remove the grasps that are ik infeasible
        this function should be called after saveToDB
        It updates the grasps in saveToDB

        :param doverh:
        :param armid: a string "rgt" or "lft" indicating the arm that will be solved
        :return:
        """

        ikavailability = {}

        # connect to database
        db = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
        cursor = db.cursor()

        result = cursor.execute(
            "SELECT idtabletopplacements, angleid, placementid, tabletopposition FROM tabletopplacements WHERE objname LIKE '%s'" % self.dbobjname)
        if result:
            tpsrows = np.array(list(cursor.fetchall()))
            for i, tpsid in enumerate(tpsrows[:, 0]):
                resultg = cursor.execute("SELECT idtabletopgrip, contactpnt0, contactpnt1, rotmat, jawwidth, idfreeairgrip FROM tabletopgrip WHERE idtabletopplacements = %d" % int(tpsid))
                if resultg:
                    ttgrows = np.array(list(cursor.fetchall()))
                    for ttgrow in ttgrows:
                        ttgid = int(ttgrow[0])
                        ttgcct0 = dc.strToV3(ttgrow[1])
                        ttgcct1 = dc.strToV3(ttgrow[2])
                        ttgrotmat = dc.strToMat4(ttgrow[3])
                        ttgjawwidth = float(ttgrow[4])
                        ttgidfreeair = int(ttgrow[5])
                        ttgfgrcenter = (ttgcct0 + ttgcct1) / 2
                        ttgfgrcenternp = pg.v3ToNp(ttgfgrcenter)
                        ttgrotmat3np = pg.mat3ToNp(ttgrotmat.getUpper3())
                        if nxtik.numikr(self.robot, ttgfgrcenternp, ttgrotmat3np):
                            ikavailability[ttgid] = 'True'
                        else:
                            ikavailability[ttgid] = 'False'

        for ikakey in ikavailability:
            ttgid = int(ikakey)
            ttgikf = ikavailability[ikakey]
            result = cursor.execute("update tabletopgrip set ikfeasible = '%s' where idtabletopgrip = %d" % (ttgikf, ttgid))

        db.commit()
        db.close()


    def saveToDB(self, positionlist, discretesize=8):
        """

        :param positionlist: a list of positions to place the object one the table
        :param discretesize:
        :return:

        author: weiwei
        date: 20161215, osaka
        """

        # save to database
        db = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
        cursor = db.cursor()
        for ttoppos in positionlist:
            for i in range(len(self.tpsmat4s)):
                ttoppos = Point3(ttoppos[0], ttoppos[1], ttoppos[2])
                for angleid in range(discretesize):
                    rotangle = 360.0 / discretesize * angleid
                    rotmat  = rm.rodrigues([0,0,1], rotangle)
                    rotmat4 = Mat4(rotmat[0][0], rotmat[0][1], rotmat[0][2], 0,
                                   rotmat[1][0], rotmat[1][1], rotmat[1][2], 0,
                                   rotmat[2][0], rotmat[2][1], rotmat[2][2], 0,
                                   ttoppos[0], ttoppos[1], ttoppos[2], 1)
                    varrotmat = self.tpsmat4s[i]*rotmat4
                    # tabletopplacements id includes both placement and rot
                    # placement id only indicates placements (different rotation belongs to the same placement)
                    # table top position is the position of the contact between the object and the table
                    sql = "INSERT INTO tabletopplacements(objname, rotmat, tabletopposition, angleid, placementid) \
                           VALUES('%s', '%s', '%s', %d, %d)" % \
                          (self.dbobjname, dc.mat4ToStr(varrotmat), dc.v3ToStr(ttoppos),angleid, i)
                    try:
                        cursor.execute(sql)
                    except mdb.Error as e:
                        print "MySQL Error [%d]: %s" % (e.args[0], e.args[1])
                        db.rollback()
                        raise mdb.Error

                    # get the id of the inserted item
                    lastid = 0
                    sql = "SELECT idtabletopplacements FROM tabletopplacements WHERE objname LIKE '%s'" % self.dbobjname
                    try:
                        cursor.execute(sql)
                        row = list(cursor.fetchall())
                        lastid = row[-1][0]
                    except mdb.Error as e:
                        print "MySQL Error [%d]: %s" % (e.args[0], e.args[1])
                        db.rollback()
                        raise mdb.Error

                    for j in range(len(self.tpsgriprotmats[i])):
                        cct0 = rotmat4.xformPoint(self.tpsgripcontacts[i][j][0])
                        cct1 = rotmat4.xformPoint(self.tpsgripcontacts[i][j][1])
                        cctn0 = rotmat4.xformVec(self.tpsgripnormals[i][j][0])
                        cctn1 = rotmat4.xformVec(self.tpsgripnormals[i][j][1])
                        tpsgriprotmat4 = self.tpsgriprotmats[i][j]*rotmat4
                        sql = "INSERT INTO tabletopgrip(idtabletopplacements, contactpnt0, contactpnt1, contactnormal0, contactnormal1, rotmat, jawwidth, idfreeairgrip) \
                        VALUES(%d, '%s', '%s', '%s', '%s', '%s', '%s', %d)" % \
                        (lastid, dc.v3ToStr(cct0), dc.v3ToStr(cct1), dc.v3ToStr(cctn0), dc.v3ToStr(cctn1), \
                         dc.mat4ToStr(tpsgriprotmat4), self.tpsgripjawwidth[i][j], self.tpsgripidfreeair[i][j])
                        try:
                            cursor.execute(sql)
                        except mdb.Error as e:
                            print "MySQL Error [%d]: %s" % (e.args[0], e.args[1])
                            db.rollback()
                            raise mdb.Error
        db.commit()
        db.close()

    def tpsgrpshow(self, base):

        # save to database
        db = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
        cursor = db.cursor()
        result = cursor.execute("SELECT * FROM tabletopplacements WHERE objname LIKE '%s' and angleid = %d" % (self.dbobjname, 3))
        if result:
            # ttp = tabletop placements
            ttprows = list(cursor.fetchall())
            nttp = len(ttprows)
            pg.plotAxis(base.render)

            # for i in range(nttp):
            for i in range(3,4):
                ttpid = ttprows[i][0]
                ttprotmat = dc.strToMat4(ttprows[i][2])
                ttppos = dc.strToV3(ttprows[i][3])
                pg.plotAxisSelf(base.render, spos=ttppos)

                # show obj
                transvert = []
                geom = pg.packpandageom(self.objtrimesh.vertices,
                                        self.objtrimesh.face_normals,
                                        self.objtrimesh.faces)
                node = GeomNode('obj')
                node.addGeom(geom)
                star = NodePath('obj')
                star.attachNewNode(node)
                star.setColor(Vec4(0, 1, 0, 1))
                star.setTransparency(TransparencyAttrib.MAlpha)
                star.setMat(ttprotmat)
                star.reparentTo(base.render)
                self.ttnodepath.setPos(ttppos)
                self.ttnodepath.setColor(Vec4(.5, .5, .5, .7))
                self.ttnodepath.setTransparency(TransparencyAttrib.MAlpha)
                self.ttnodepath.reparentTo(base.render)

                # get the id of the inserted item
                lastid = 0
                sql = "SELECT jawwidth, rotmat FROM tabletopgrip WHERE idtabletopplacements = %d" % ttpid
                try:
                    cursor.execute(sql)
                    ttgrows = list(cursor.fetchall())
                    nttgrows = len(ttgrows)
                    for j in range(nttgrows):
                        jawwidth = float(ttgrows[j][0])
                        rotmat = dc.strToMat4(ttgrows[j][1])

                        # show grasps
                        tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[0, 1, 0, .5])
                        tmprtq85.setMat(rotmat)
                        tmprtq85.setJawwidth(jawwidth)
                        tmprtq85.reparentTo(base.render)
                except mdb.Error as e:
                    print "MySQL Error [%d]: %s" % (e.args[0], e.args[1])
                    db.rollback()
                    raise mdb.Error
        else:
            print "The tabletopplacements of the object doesn't exsit"
        db.close()

if __name__ == '__main__':
    nxtrobot = nextage.NxtRobot()
    nxtrobot.goinitpose()

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(os.path.split(this_dir)[0]+os.sep, "grip", "objects", "ttube.stl")
    print objpath
    tps = TablePlacementsPosition(objpath, nxtrobot)

    # plot obj and its convexhull
    # geom = pandageom.packpandageom(tps.objtrimesh.vertices,
    #                                tps.objtrimesh.face_normals,
    #                                tps.objtrimesh.faces)
    # node = GeomNode('obj')
    # node.addGeom(geom)
    # star = NodePath('obj')
    # star.attachNewNode(node)
    # star.setColor(Vec4(1,0,0,1))
    # star.setTransparency(TransparencyAttrib.MAlpha)
    # star.reparentTo(base.render)
    #
    # geom = pandageom.packpandageom(tps.objtrimeshconv.vertices,
    #                                tps.objtrimeshconv.face_normals,
    #                                tps.objtrimeshconv.faces)
    # node = GeomNode('objconv')
    # node.addGeom(geom)
    # star = NodePath('objconv')
    # star.attachNewNode(node)
    # star.setColor(Vec4(0, 1, 0, .3))
    # star.setTransparency(TransparencyAttrib.MAlpha)
    # star.reparentTo(base.render)


    # def updateshow(task):
    #     # tps.ocfacetshow(base)
    #     tps.removebadfacetsshow(base, doverh=.033)
    #     return task.again
    #
    # taskMgr.doMethodLater(.1, updateshow, "tickTask")


    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont

    tps.removebadfacets(base, doverh=.033)
    # build grid space
    grids = []
    for x in range(300,501,100):
        for y in range(-600,601,200):
            grids.append([x,y,0])
    tps.saveToDB(grids)
    tps.updateDBwithIK()

    # bullcldrnp = base.render.attachNewNode("bulletcollider")
    # debugNode = BulletDebugNode('Debug')
    # debugNode.showWireframe(True)
    # debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()
    #
    # tps.bulletworldtable.setDebugNode(debugNP.node())
    #
    # taskMgr.add(updateworld, "updateworld", extraArgs=[tps.bulletworldtable], appendTask=True)

    # tps.tpsgrpshow(base)
    base.run()