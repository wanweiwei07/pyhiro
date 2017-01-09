#!/usr/bin/python

import os

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


class TablePlacements(object):

    def __init__(self, objpath):
        self.objtrimesh=trimesh.load_mesh(objpath)
        self.objcom = self.objtrimesh.center_mass
        self.objtrimeshconv=self.objtrimesh.convex_hull
        # oc means object convex
        self.ocfacets, self.ocfacetnormals = self.objtrimeshconv.facets_over(.9999)

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

        # use two bulletworld, one for the ray, the other for the tabletop
        self.bulletworldray = BulletWorld()
        self.bulletworldtable = BulletWorld()
        # add tabletop plane model to bulletworld
        # tt = tabletop
        this_dir, this_filename = os.path.split(__file__)
        ttpath = Filename.fromOsSpecific(os.path.join(this_dir, "supports", "tabletop.egg"))
        self.ttnodepath = NodePath("tabletop")
        ttl = loader.loadModel(ttpath)
        ttl.instanceTo(self.ttnodepath)
        self.ttbullnode = cd.genCollisionMeshNp(self.ttnodepath)
        self.bulletworldtable.attachRigidBody(self.ttbullnode)

        self.rtq85hnd = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, .1])

        # for dbsave
        # each tpsmat4 corresponds to a set of tpsgripcontacts/tpsgripnormals/tpsgripjawwidth list
        self.tpsmat4s = None
        self.tpsgripcontacts = None
        self.tpsgripnormals = None
        self.tpsgripjawwidth = None

        # for ocFacetShow
        self.counter = 0

    def removebadfacets(self, base, doverh=.1):
        """
        remove the facets that cannot support stable placements

        :param: doverh: d is the distance of mproj to supportfacet boundary, h is the height of com
                when fh>dmg, the object tends to fall over. setting doverh to 0.033 means
                when f>0.1mg, the object is judged to be unstable
        :return:

        author: weiwei
        date: 20161213
        """
        self.tpsmat4s = []
        self.tpsgripcontacts = []
        self.tpsgripnormals = []
        self.tpsgriprotmats = []
        self.tpsgripjawwidth = []
        # the id of the grip in freeair
        self.tpsgripidfreeair = []

        for i in range(len(self.ocfacets)):
            geom = pg.packpandageom(self.objtrimeshconv.vertices,
                                           self.objtrimeshconv.face_normals[self.ocfacets[i]],
                                           self.objtrimeshconv.faces[self.ocfacets[i]])
            geombullnode = cd.genCollisionMeshGeom(geom)
            self.bulletworldray.attachRigidBody(geombullnode)
            pFrom = Point3(self.objcom[0], self.objcom[1], self.objcom[2])
            pTo = self.objcom+self.ocfacetnormals[i]*99999
            pTo = Point3(pTo[0], pTo[1], pTo[2])
            result = self.bulletworldray.rayTestClosest(pFrom, pTo)
            self.bulletworldray.removeRigidBody(geombullnode)
            if result.hasHit():
                hitpos = result.getHitPos()

                facetinterpnt = np.array([hitpos[0],hitpos[1],hitpos[2]])
                facetnormal = np.array(self.ocfacetnormals[i])
                bdverts3d, bdverts2d, facetmat4 = pg.facetboundary(self.objtrimeshconv, self.ocfacets[i],
                                                                     facetinterpnt, facetnormal)
                facetp = Polygon(bdverts2d)
                facetinterpnt2d = rm.transformmat4(facetmat4, facetinterpnt)[:2]
                apntpnt = Point(facetinterpnt2d[0], facetinterpnt2d[1])
                dist2p = apntpnt.distance(facetp.exterior)
                dist2c = np.linalg.norm(np.array([hitpos[0],hitpos[1],hitpos[2]])-np.array([pFrom[0],pFrom[1],pFrom[2]]))
                if dist2p/dist2c > doverh:
                    # hit and stable
                    geom = pandageom.packpandageom(self.objtrimesh.vertices,
                                                   self.objtrimesh.face_normals,
                                                   self.objtrimesh.faces)
                    node = GeomNode('obj')
                    node.addGeom(geom)
                    star = NodePath('obj')
                    star.attachNewNode(node)
                    star.setColor(Vec4(0, 1, 0, 1))
                    star.setTransparency(TransparencyAttrib.MAlpha)
                    star.reparentTo(base.render)
                    star.setMat(pg.cvtMat4np4(facetmat4))
                    self.ttnodepath.setColor(Vec4(.5,.5,.5,.7))
                    self.ttnodepath.setTransparency(TransparencyAttrib.MAlpha)
                    self.ttnodepath.reparentTo(base.render)
                    # pg.plotAxisSelf(base.render, spos=Point3(0,0,0))

                    self.tpsmat4s.append(pg.cvtMat4np4(facetmat4))
                    self.tpsgripcontacts.append([])
                    self.tpsgripnormals.append([])
                    self.tpsgriprotmats.append([])
                    self.tpsgripjawwidth.append([])
                    self.tpsgripidfreeair.append([])
                    for j, rotmat in enumerate(self.freegriprotmats):
                        tpsgriprotmat = rotmat*self.tpsmat4s[-1]
                        # check if the hand collide with tabletop
                        tmprtq85 = self.rtq85hnd
                        # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
                        initmat = tmprtq85.getMat()
                        initjawwidth = tmprtq85.jawwidth
                        tmprtq85.setJawwidth(self.freegripjawwidth[j])
                        tmprtq85.setMat(tpsgriprotmat)
                        # add hand model to bulletworld
                        hndbullnode = cd.genCollisionMeshMultiNp(tmprtq85.rtq85np)
                        self.bulletworldtable.attachRigidBody(hndbullnode)
                        result = self.bulletworldtable.contactTest(self.ttbullnode)
                        print result.getNumContacts()
                        self.bulletworldtable.removeRigidBody(hndbullnode)
                        if not result.getNumContacts():
                            self.tpsgriprotmats[-1].append(tpsgriprotmat)
                            cct0 = self.tpsmat4s[-1].xformPoint(self.freegripcontacts[j][0])
                            cct1 = self.tpsmat4s[-1].xformPoint(self.freegripcontacts[j][1])
                            self.tpsgripcontacts[-1].append([cct0, cct1])
                            cctn0 = self.tpsmat4s[-1].xformVec(self.freegripnormals[j][0])
                            cctn1 = self.tpsmat4s[-1].xformVec(self.freegripnormals[j][1])
                            self.tpsgripnormals[-1].append([cctn0, cctn1])
                            self.tpsgripjawwidth[-1].append(self.freegripjawwidth[j])
                            self.tpsgripidfreeair[-1].append(self.freegripid[j])
                        #     tmprtq85.setColor([0,1,0,1])
                        #     tmprtq85.reparentTo(base.render)
                        # else:
                        #     tmprtq85.reparentTo(base.render)
                        tmprtq85.setMat(initmat)
                        tmprtq85.setJawwidth(initjawwidth)

    def saveToDB(self, discretesize=8):

        # save to database
        db = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
        cursor = db.cursor()
        result = cursor.execute("SELECT * FROM tabletopplacements WHERE objname LIKE '%s'" % self.dbobjname)
        if not result:
            for i in range(len(self.tpsmat4s)):
                for angleid in range(discretesize):
                    rotangle = 360.0 / discretesize * angleid
                    rotmat  = rm.rodrigues([0,0,1], rotangle)
                    rotmat4 = Mat4(rotmat[0][0], rotmat[0][1], rotmat[0][2], 0,
                                   rotmat[1][0], rotmat[1][1], rotmat[1][2], 0,
                                   rotmat[2][0], rotmat[2][1], rotmat[2][2], 0,
                                   0, 0, 0, 1)
                    varrotmat = self.tpsmat4s[i]*rotmat4
                    # tabletopplacements id includes both placement and rot
                    # placement id only indicates placements (different rotation belongs to the same placement)
                    # table top position is the position of the contact between the object and the table
                    sql = "INSERT INTO tabletopplacements(objname, rotmat, tabletopposition, angleid, placementid) \
                           VALUES('%s', '%s', '%s', %d, %d)" % \
                          (self.dbobjname, dc.mat4ToStr(varrotmat), dc.v3ToStr(Point3(0,0,0)),angleid, i)
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
        else:
            print "Grasps already saved or duplicated filename!"
        db.close()

    def removebadfacetsshow(self, base, doverh=.1):
        """
        remove the facets that cannot support stable placements

        :param: doverh: d is the distance of mproj to supportfacet boundary, h is the height of com
                when fh>dmg, the object tends to fall over. setting doverh to 0.033 means
                when f>0.1mg, the object is judged to be unstable
        :return:

        author: weiwei
        date: 20161213
        """

        plotoffsetfp = 10
        print self.counter

        if self.counter < len(self.ocfacets):
            i = self.counter
        # for i in range(len(self.ocfacets)):
            geom = pg.packpandageom(self.objtrimeshconv.vertices,
                                           self.objtrimeshconv.face_normals[self.ocfacets[i]],
                                           self.objtrimeshconv.faces[self.ocfacets[i]])
            geombullnode = cd.genCollisionMeshGeom(geom)
            self.bulletworld.attachRigidBody(geombullnode)
            pFrom = Point3(self.objcom[0], self.objcom[1], self.objcom[2])
            pTo = self.objcom+self.ocfacetnormals[i]*99999
            pTo = Point3(pTo[0], pTo[1], pTo[2])
            result = self.bulletworld.rayTestClosest(pFrom, pTo)
            self.bulletworld.removeRigidBody(geombullnode)
            if result.hasHit():
                hitpos = result.getHitPos()
                pg.plotArrow(base.render, spos=self.objcom,
                             epos = self.objcom+self.ocfacetnormals[i], length=100)

                facetinterpnt = np.array([hitpos[0],hitpos[1],hitpos[2]])
                facetnormal = np.array(self.ocfacetnormals[i])
                bdverts3d, bdverts2d, facetmat4 = pg.facetboundary(self.objtrimeshconv, self.ocfacets[i],
                                                                     facetinterpnt, facetnormal)
                for j in range(len(bdverts3d)-1):
                    spos = bdverts3d[j]
                    epos = bdverts3d[j+1]
                    pg.plotStick(base.render, spos, epos, thickness = 1, rgba=[.5,.5,.5,1])

                facetp = Polygon(bdverts2d)
                facetinterpnt2d = rm.transformmat4(facetmat4, facetinterpnt)[:2]
                apntpnt = Point(facetinterpnt2d[0], facetinterpnt2d[1])
                dist2p = apntpnt.distance(facetp.exterior)
                dist2c = np.linalg.norm(np.array([hitpos[0],hitpos[1],hitpos[2]])-np.array([pFrom[0],pFrom[1],pFrom[2]]))
                print dist2p
                print dist2c
                print dist2p/dist2c
                if dist2p/dist2c < doverh:
                    print "not stable"
                    # return
                else:
                    pol_ext = LinearRing(bdverts2d)
                    d = pol_ext.project(apntpnt)
                    p = pol_ext.interpolate(d)
                    closest_point_coords = list(p.coords)[0]
                    closep = np.array([closest_point_coords[0], closest_point_coords[1], 0])
                    closep3d = rm.transformmat4(rm.homoinverse(facetmat4), closep)[:3]
                    pg.plotDumbbell(base.render, spos=facetinterpnt, epos=closep3d, thickness=1.5, rgba=[0,0,1,1])

                    for j in range(len(bdverts3d)-1):
                        spos = bdverts3d[j]
                        epos = bdverts3d[j+1]
                        pg.plotStick(base.render, spos, epos, thickness = 1.5, rgba=[0,1,0,1])

                    # geomoff = pg.packpandageom(self.objtrimeshconv.vertices +
                    #                                np.tile(plotoffsetfp * self.ocfacetnormals[i],
                    #                                        [self.objtrimeshconv.vertices.shape[0], 1]),
                    #                         self.objtrimeshconv.face_normals[self.ocfacets[i]],
                    #                         self.objtrimeshconv.faces[self.ocfacets[i]])
                    #
                    # nodeoff = GeomNode('supportfacet')
                    # nodeoff.addGeom(geomoff)
                    # staroff = NodePath('supportfacet')
                    # staroff.attachNewNode(nodeoff)
                    # staroff.setColor(Vec4(1,0,1,1))
                    # staroff.setTransparency(TransparencyAttrib.MAlpha)
                    # staroff.setTwoSided(True)
                    # staroff.reparentTo(base.render)
            self.counter+=1
        else:
            self.counter=0

    def tpsgrpshow(self, base):

        # save to database
        db = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
        cursor = db.cursor()
        result = cursor.execute("SELECT * FROM tabletopplacements WHERE objname LIKE '%s' and angleid = %d" % (self.dbobjname, 3))
        if result:
            # ttp = tabletop placements
            ttprows = list(cursor.fetchall())
            nttp = len(ttprows)
            # for i in range(nttp):
            for i in range(20,21):
                ttpid = ttprows[i][0]
                ttprotmat = dc.strToMat4(ttprows[i][2])

                # show obj
                transvert = []
                geom = pandageom.packpandageom(self.objtrimesh.vertices,
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


    def ocfacetshow(self, base):
        print self.objcom

        npf = base.render.find("**/supportfacet")
        if npf:
            npf.removeNode()

        plotoffsetfp = 10

        print self.counter
        print len(self.ocfacets)
        if self.counter < len(self.ocfacets):
            geom = pandageom.packpandageom(self.objtrimeshconv.vertices+
                                           np.tile(plotoffsetfp*self.ocfacetnormals[self.counter],
                                                   [self.objtrimeshconv.vertices.shape[0],1]),
                                           self.objtrimeshconv.face_normals[self.ocfacets[self.counter]],
                                           self.objtrimeshconv.faces[self.ocfacets[self.counter]])
            # geom = pandageom.packpandageom(self.objtrimeshconv.vertices,
            #                                self.objtrimeshconv.face_normals,
            #                                self.objtrimeshconv.faces)
            node = GeomNode('supportfacet')
            node.addGeom(geom)
            star = NodePath('supportfacet')
            star.attachNewNode(node)
            star.setColor(Vec4(1,0,1,1))
            star.setTransparency(TransparencyAttrib.MAlpha)
            star.setTwoSided(True)
            star.reparentTo(base.render)
            self.counter+=1
        else:
            self.counter = 0

if __name__ == '__main__':

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "objects", "ttube.stl")
    print objpath
    tps = TablePlacements(objpath)

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


    def updateshow(task):
        # tps.ocfacetshow(base)
        tps.removebadfacetsshow(base, doverh=.033)
        return task.again

    taskMgr.doMethodLater(.1, updateshow, "tickTask")


    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont

    # tps.removebadfacets(base, doverh=.033)
    # tps.saveToDB()

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